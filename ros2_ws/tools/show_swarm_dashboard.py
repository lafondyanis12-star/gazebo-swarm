#!/usr/bin/env python3
# Tableau de bord simplifie de l'essaim : affiche en continu, pour chaque
# drone, sa position, le leader qu'il croit avoir, et l'etat connecte/
# deconnecte de chaque pair -- rien d'autre. La distance minimale entre 2
# drones quelconques (risque de collision) est affichee en bas, en metres.
# Un journal d'evenements (changements de leader, connexion/deconnexion)
# fait aussi office de recap de vol : il reste affiche une fois le vol
# termine.
#
# A lancer dans un 2e terminal pendant que l'essaim tourne :
#   python3 tools/show_swarm_dashboard.py

import math
import sys
import time

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import SwarmStatus

NUM_DRONES = 3
SAFE_DIST_M = 1.0  # doit correspondre a SAFE_DIST_M dans swarm_node.cpp
REFRESH_S = 0.3
MAX_EVENTS = 14

GREEN = '\x1b[32m'
RED = '\x1b[31m'
BOLD = '\x1b[1m'
RESET = '\x1b[0m'


class DroneRow:
    """Derniere valeur connue (etat affiche) pour un drone du tableau de bord."""

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.claimed_leader = -1
        self.peer_connected = [False] * NUM_DRONES
        self.has_data = False  # True des qu'un premier message a ete recu


def distance(a, b):
    """Distance euclidienne 3D entre deux DroneRow."""
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


class SwarmDashboard(Node):
    """Noeud ROS2 qui s'abonne au /swarm_status de chaque drone et affiche
    un tableau de bord texte (rafraichi periodiquement) dans le terminal."""

    def __init__(self):
        super().__init__('swarm_dashboard')
        self.rows = {i: DroneRow() for i in range(NUM_DRONES)}
        self.events = []
        self.start_time = time.monotonic()
        for i in range(NUM_DRONES):
            self.create_subscription(
                SwarmStatus, f'/drone_{i}/swarm_status',
                self._make_callback(i), 10)
        self.create_timer(REFRESH_S, self._render)

    def _log_event(self, text):
        """Ajoute un evenement horodate au journal, en ne gardant que les MAX_EVENTS derniers."""
        elapsed = time.monotonic() - self.start_time
        self.events.append((elapsed, text))
        del self.events[:-MAX_EVENTS]

    def _make_callback(self, drone_id):
        """Cree le callback d'abonnement /swarm_status pour drone_id (capture drone_id).

        Detecte les changements (leader, connexions) par rapport a l'etat
        precedent pour alimenter le journal d'evenements, puis met a jour
        la ligne du tableau de bord correspondante.
        """
        def callback(msg):
            row = self.rows[drone_id]
            row.x, row.y, row.z = msg.x, msg.y, msg.z

            # Changement de leader percu par ce drone -> evenement journalise.
            if row.has_data and msg.claimed_leader != row.claimed_leader:
                if msg.claimed_leader == drone_id:
                    self._log_event(f'drone_{drone_id} became LEADER')
                else:
                    self._log_event(f'drone_{drone_id} now follows drone_{msg.claimed_leader}')
            row.claimed_leader = msg.claimed_leader

            # Changement de connectivite (par pair) -> evenement journalise.
            new_connected = list(msg.peer_connected)
            if row.has_data:
                for j in range(NUM_DRONES):
                    if j == drone_id or new_connected[j] == row.peer_connected[j]:
                        continue
                    verb = 'reconnected to' if new_connected[j] else 'lost connection to'
                    self._log_event(f'drone_{drone_id} {verb} drone_{j}')
            row.peer_connected = new_connected
            row.has_data = True
        return callback

    def _render(self):
        """Redessine tout l'ecran (appele periodiquement par le timer REFRESH_S)."""
        lines = [
            '\x1b[H\x1b[J',  # efface l'ecran, curseur en haut a gauche
            f'{BOLD}SWARM DASHBOARD{RESET}  (ctrl-c to quit)',
            '',
        ]
        for i in range(NUM_DRONES):
            row = self.rows[i]
            if not row.has_data:
                lines.append(f'{BOLD}drone_{i}{RESET}  no data yet')
                lines.append('')
                continue

            is_leader = row.claimed_leader == i
            leader_txt = (f'{BOLD}{GREEN}LEADER{RESET}' if is_leader
                          else f'follows drone_{row.claimed_leader}' if row.claimed_leader >= 0
                          else 'no leader')
            lines.append(
                f'{BOLD}drone_{i}{RESET}   '
                f'position=({row.x:.1f}, {row.y:.1f}, {row.z:.1f})   {leader_txt}'
            )
            for j in range(NUM_DRONES):
                if j == i:
                    continue
                connected = row.peer_connected[j]
                status = f'{GREEN}CONNECTED{RESET}' if connected else f'{RED}{BOLD}DISCONNECTED{RESET}'
                lines.append(f'   -> drone_{j} : {status}')
            lines.append('')

        # Distance minimale entre 2 drones parmi ceux ayant deja des donnees
        # -- indicateur simple de risque de collision.
        live_rows = [self.rows[i] for i in range(NUM_DRONES) if self.rows[i].has_data]
        if len(live_rows) >= 2:
            pair_dists = [
                distance(live_rows[a], live_rows[b])
                for a in range(len(live_rows)) for b in range(a + 1, len(live_rows))
            ]
            min_dist = min(pair_dists)
            min_txt = (f'{RED}{BOLD}{min_dist:.1f} m -- COLLISION RISK{RESET}'
                       if min_dist < SAFE_DIST_M else f'{GREEN}{BOLD}{min_dist:.1f} m{RESET}')
            lines.append(f'Minimum distance between any 2 drones: {min_txt}')

        lines.append('')
        lines.append(f'{BOLD}FLIGHT RECAP (events so far):{RESET}')
        if self.events:
            for t, text in self.events:
                lines.append(f'  [T+{t:5.1f}s] {text}')
        else:
            lines.append('  (nothing has happened yet)')

        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main():
    """Point d'entree : initialise ROS2, lance le tableau de bord jusqu'a ctrl-c."""
    rclpy.init()
    node = SwarmDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
