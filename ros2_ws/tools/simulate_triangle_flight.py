#!/usr/bin/env python3
# Simule un vol en formation triangle sans PX4/Gazebo : pilote les
# parametres x/y/z des 3 swarm_node (le "mode position de secours" que
# test_election.py utilise pour tester l'election sans PX4 -- voir le
# commentaire pres de la declaration de x/y/z dans swarm_node.cpp) en
# rejouant la meme trajectoire/formation que le vrai controle
# (loiter_reference + rotate_formation_offset), et deconnecte/reconnecte
# drone_0 a mi-vol pour observer l'election reelle (les valeurs
# claimed_leader / peer_connected viennent des vrais swarm_node, pas d'une
# supposition de ce script).
#
# Important : l'election est "sticky" (vote majoritaire, voir elect_leader()
# dans swarm_node.cpp) -- quand drone_0 se reconnecte, le leader ne revient
# PAS automatiquement a drone_0. C'est voulu (evite de re-adopter par
# accident un leader qui vient de revenir), donc drone_0 rejoint la
# formation comme suiveur, pas comme leader.
#
# Lancer d'abord (pas besoin de run_swarm.sh / PX4) :
#   ros2 launch swarm_comm swarm_launch.py
# puis, dans un 2e terminal :
#   python3 tools/simulate_triangle_flight.py

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from swarm_msgs.msg import SwarmStatus

NUM_DRONES = 3
SPAWN_EAST_M = [0.0, 2.0, 4.0]
SPAWN_NORTH_M = [0.0, 0.0, 0.0]

# Doivent matcher les constantes de swarm_node.cpp pour que la simulation
# ressemble a ce que produirait un vrai vol.
LOITER_CENTER_EAST_M = 2.0
LOITER_CENTER_NORTH_M = 0.0
LOITER_RADIUS_M = 6.0
LOITER_ALT_M = 3.0
LOITER_PERIOD_S = 45.0
LOITER_OMEGA = 2.0 * math.pi / LOITER_PERIOD_S
FORMATION_TRAIL_M = 1.5
FORMATION_HALF_WIDTH_M = 1.0
MAX_RANGE_M = 10.0

TAKEOFF_S = 6.0
DISCONNECT_REL_S = 15.0   # apres le debut du vol
RECONNECT_REL_S = 35.0
REJOIN_RAMP_S = 5.0       # duree pour que drone_0 revienne en formation, pas de teleport
FLIGHT_DURATION_S = 55.0
LAND_S = 8.0
TICK_S = 0.2

PARK_EAST_M = LOITER_CENTER_EAST_M + 20.0   # >MAX_RANGE_M du reste de l'essaim
PARK_NORTH_M = LOITER_CENTER_NORTH_M


def lerp(a, b, frac):
    """Interpolation lineaire entre a et b, frac borne a [0, 1]."""
    frac = max(0.0, min(1.0, frac))
    return a + (b - a) * frac


def loiter_pos(t, phase_offset):
    """Position et vitesse (est/nord) sur le cercle de loiter a l'instant t.

    phase_offset permet de re-ancrer la phase (ex: au changement de leader).
    Retourne (e, n, ve, vn) : position puis vitesse est/nord.
    """
    phase = LOITER_OMEGA * t + phase_offset
    e = LOITER_CENTER_EAST_M + LOITER_RADIUS_M * math.cos(phase)
    n = LOITER_CENTER_NORTH_M + LOITER_RADIUS_M * math.sin(phase)
    ve = -LOITER_RADIUS_M * LOITER_OMEGA * math.sin(phase)
    vn = LOITER_RADIUS_M * LOITER_OMEGA * math.cos(phase)
    return e, n, ve, vn


def rotate_offset(forward, right, heading):
    """Convertit un decalage (avant, droite) relatif au cap en decalage est/nord absolu."""
    fwd_e, fwd_n = math.sin(heading), math.cos(heading)
    right_e, right_n = math.cos(heading), -math.sin(heading)
    return fwd_e * forward + right_e * right, fwd_n * forward + right_n * right


class FlightSimulator(Node):
    """Noeud ROS2 qui pilote directement les parametres x/y/z des 3 swarm_node
    pour simuler un vol (sans PX4/Gazebo) et observer l'election reelle."""

    def __init__(self):
        super().__init__('simulate_triangle_flight')
        # Un client de service SetParameters par drone, pour imposer sa position.
        self.param_clients = {
            i: self.create_client(SetParameters, f'/drone_{i}/set_parameters')
            for i in range(NUM_DRONES)
        }
        for i, client in self.param_clients.items():
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(
                    f'/drone_{i}/set_parameters indisponible -- '
                    f'lance d\'abord: ros2 launch swarm_comm swarm_launch.py')

        # Etat percu par le "vrai" systeme (rempli par les callbacks d'abonnement
        # ci-dessous), utilise seulement pour le log/verification -- pas pour piloter.
        self.claimed_leader = {i: -1 for i in range(NUM_DRONES)}
        self.peer_connected = {i: [True] * NUM_DRONES for i in range(NUM_DRONES)}
        for i in range(NUM_DRONES):
            self.create_subscription(
                SwarmStatus, f'/drone_{i}/swarm_status', self._make_cb(i), 10)

        # Position "physique" que ce script impose a chaque drone -- source
        # de verite ici puisqu'il n'y a pas de PX4 pour l'integrer soi-meme.
        self.pos = {i: [SPAWN_EAST_M[i], SPAWN_NORTH_M[i], 0.0] for i in range(NUM_DRONES)}

        self.phase_offset = 0.0        # ré-ancré à chaque changement de leader, comme elect_leader()
        self.current_leader = None
        self.confirmed = set()
        self.parked = False
        self.rejoin_start = None
        self.rejoin_from = None

    def _make_cb(self, i):
        """Cree le callback d'abonnement /swarm_status pour le drone i (capture i)."""
        def cb(msg):
            self.claimed_leader[i] = msg.claimed_leader
            self.peer_connected[i] = list(msg.peer_connected)
        return cb

    def set_pos(self, drone_id, x, y, z):
        """Impose la position (x, y, z) d'un drone via le service set_parameters."""
        self.pos[drone_id] = [x, y, z]
        req = SetParameters.Request()
        for name, val in (('x', x), ('y', y), ('z', z)):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))
            req.parameters.append(p)
        self.param_clients[drone_id].call_async(req)

    def formation_slots(self, leader_id):
        """Calcule les 2 positions laterales (gauche/droite) des suiveurs pour un leader donne."""
        followers = sorted(i for i in range(NUM_DRONES) if i != leader_id)
        # gauche = plus petit id, comme formation_side_is_left() dans swarm_node.cpp
        return {followers[0]: FORMATION_HALF_WIDTH_M, followers[1]: -FORMATION_HALF_WIDTH_M}

    def spin(self, seconds):
        """Traite les callbacks/futures ROS2 en attente pendant `seconds` (0 = non bloquant)."""
        rclpy.spin_once(self, timeout_sec=seconds)


def log(msg):
    """Affiche un message horodate (temps ecoule depuis START)."""
    print(f'[T+{time.monotonic() - START:6.1f}s] {msg}', flush=True)


def run():
    """Boucle principale : machine a etats decollage -> vol -> atterrissage -> fin.

    A chaque tick (TICK_S), calcule la position cible de chaque drone selon
    la phase courante et l'envoie via set_pos(). C'est ce script qui decide
    du scenario (deconnexion/reconnexion de drone_0) ; l'election reelle
    (claimed_leader) n'est lue que pour verifier/logger le comportement du
    vrai systeme, jamais pour piloter la simulation.
    """
    global START
    rclpy.init()
    sim = FlightSimulator()
    START = time.monotonic()

    log('Simulation demarree -- decollage.')
    last_report = -999.0
    t = 0.0
    while True:
        sim.spin(0.0)  # traite les callbacks/futures en attente sans bloquer

        if t < TAKEOFF_S:
            # Decollage : montee en ligne droite du sol vers la position de
            # formation initiale (t_flight=0), a altitude croissante.
            frac = t / TAKEOFF_S
            e0, n0, _, _ = loiter_pos(0.0, 0.0)
            targets = {0: (e0, n0)}
            slots = sim.formation_slots(0)
            for did, lateral in slots.items():
                oe, on = rotate_offset(-FORMATION_TRAIL_M, lateral, 0.0)
                targets[did] = (e0 + oe, n0 + on)
            for did in range(NUM_DRONES):
                te, tn = targets[did]
                x = lerp(SPAWN_EAST_M[did], te, frac)
                y = lerp(SPAWN_NORTH_M[did], tn, frac)
                z = lerp(0.0, LOITER_ALT_M, frac)
                sim.set_pos(did, x, y, z)

        elif t < TAKEOFF_S + FLIGHT_DURATION_S:
            flight_t = t - TAKEOFF_S

            # La trajectoire est pilotee par le scenario du script (sinon on
            # boucle sur nous-memes : tant que le vrai systeme n'a pas
            # detecte la deconnexion, il croit encore drone_0 leader, et une
            # boucle pilotee par sa croyance renverrait aussitot drone_0 sur
            # le cercle au lieu de le laisser "parque"). La confirmation du
            # vrai systeme (claimed_leader) sert seulement au log plus bas,
            # pour montrer le delai reel de detection (~TIMEOUT_S).
            intended_leader = 0 if flight_t < DISCONNECT_REL_S else 1
            if intended_leader != sim.current_leader:
                prev = sim.current_leader
                sim.current_leader = intended_leader
                if prev is not None:
                    log(f'Bascule de leader planifiee : drone_{prev} -> drone_{intended_leader} '
                        '(drone_0 sort de portee)')
                # Re-ancrage de la phase sur la position actuelle du nouveau
                # leader, comme elect_leader() le fait en vrai.
                cx, cy, _ = sim.pos[intended_leader]
                angle = math.atan2(cy - LOITER_CENTER_NORTH_M, cx - LOITER_CENTER_EAST_M)
                sim.phase_offset = angle - LOITER_OMEGA * flight_t

            leader_id = sim.current_leader
            le, ln, lve, lvn = loiter_pos(flight_t, sim.phase_offset)
            heading = math.atan2(lve, lvn)
            slots = sim.formation_slots(leader_id)

            if flight_t < DISCONNECT_REL_S:
                sim.set_pos(leader_id, le, ln, LOITER_ALT_M)
                for did, lateral in slots.items():
                    oe, on = rotate_offset(-FORMATION_TRAIL_M, lateral, heading)
                    sim.set_pos(did, le + oe, ln + on, LOITER_ALT_M)

            elif flight_t < RECONNECT_REL_S:
                if not sim.parked:
                    log('Deconnexion de drone_0 (sorti de la portee radio simulee, '
                        f'{MAX_RANGE_M:.0f}m).')
                    sim.set_pos(0, PARK_EAST_M, PARK_NORTH_M, LOITER_ALT_M)
                    sim.parked = True
                sim.set_pos(leader_id, le, ln, LOITER_ALT_M)
                for did, lateral in slots.items():
                    if did == 0:
                        continue  # hors de portee, ne participe plus a la formation
                    oe, on = rotate_offset(-FORMATION_TRAIL_M, lateral, heading)
                    sim.set_pos(did, le + oe, ln + on, LOITER_ALT_M)

            else:
                if sim.rejoin_start is None:
                    log('Reconnexion de drone_0 (retour en portee) -- '
                        'rejoint la formation en tant que suiveur.')
                    sim.rejoin_start = flight_t
                    sim.rejoin_from = tuple(sim.pos[0])

                sim.set_pos(leader_id, le, ln, LOITER_ALT_M)
                for did, lateral in slots.items():
                    oe, on = rotate_offset(-FORMATION_TRAIL_M, lateral, heading)
                    tx, ty, tz = le + oe, ln + on, LOITER_ALT_M
                    if did == 0:
                        frac = (flight_t - sim.rejoin_start) / REJOIN_RAMP_S
                        fx0, fy0, fz0 = sim.rejoin_from
                        sim.set_pos(did, lerp(fx0, tx, frac), lerp(fy0, ty, frac),
                                     lerp(fz0, tz, frac))
                    else:
                        sim.set_pos(did, tx, ty, tz)

            real_leader = sim.claimed_leader[1]
            if real_leader == leader_id and leader_id not in sim.confirmed:
                sim.confirmed.add(leader_id)
                log(f'Election reelle confirmee (claimed_leader vu par drone_1) : drone_{real_leader}')

            if t - last_report >= 5.0:
                connected0 = sim.peer_connected[1][0] if len(sim.peer_connected[1]) > 0 else None
                log(f'leader(scenario)=drone_{leader_id}  leader(reel, drone_1)=drone_{real_leader}  '
                    f'drone_0 connecte (vu par drone_1)={connected0}  '
                    f'pos0=({sim.pos[0][0]:.1f},{sim.pos[0][1]:.1f},{sim.pos[0][2]:.1f})')
                last_report = t

        elif t < TAKEOFF_S + FLIGHT_DURATION_S + LAND_S:
            if t - TICK_S < TAKEOFF_S + FLIGHT_DURATION_S:
                log('Fin du vol -- atterrissage, retour aux positions initiales.')
                sim.land_from = {i: tuple(sim.pos[i]) for i in range(NUM_DRONES)}
            frac = (t - (TAKEOFF_S + FLIGHT_DURATION_S)) / LAND_S
            for did in range(NUM_DRONES):
                fx, fy, fz = sim.land_from[did]
                sim.set_pos(did,
                             lerp(fx, SPAWN_EAST_M[did], frac),
                             lerp(fy, SPAWN_NORTH_M[did], frac),
                             lerp(fz, 0.0, frac))
        else:
            for did in range(NUM_DRONES):
                sim.set_pos(did, SPAWN_EAST_M[did], SPAWN_NORTH_M[did], 0.0)
            log('Simulation terminee -- les 3 drones sont revenus a leur position initiale.')
            break

        time.sleep(TICK_S)
        t += TICK_S

    sim.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        run()
    except (KeyboardInterrupt, RuntimeError) as e:
        print(f'Arret: {e}', file=sys.stderr)
        sys.exit(1)
