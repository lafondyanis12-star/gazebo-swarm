// ROS 2 port of ~/Documents/my_project/gazebo_swarm/swarm_node.cpp, now flying
// for real: each drone connects to its own PX4 SITL instance over MAVSDK,
// arms, takes off, and then flies a role driven by the swarm's existing
// comm/election layer (unchanged) -- leader loiters a scripted path,
// followers hold a triangle formation on the leader, and a separation rule
// keeps every pair >= SAFE_DIST_M apart.
//
// Frame note: MAVSDK's local NED position is relative to *each vehicle's
// own* EKF origin (its own spawn point), not a shared frame -- but
// worlds/swarm_persistent.sdf spawns x500_0/1/2 at world-ENU offsets
// (0,0,0), (2,0,0), (4,0,0) (see SPAWN_EAST_M below). Adding that fixed
// spawn offset to each vehicle's local NED position gives one shared
// East/North/Up frame all 3 drones can compare positions in directly --
// which is what SwarmStatus.x/y/z now carry (previously a fake parameter).
// Velocity is a free vector (no origin), so Offboard commands need no such
// correction.
//
// Threading note: the header comment on the original port explains why a
// single-threaded rclcpp executor needs no mutex across its callbacks.
// MAVSDK breaks that assumption -- its telemetry callbacks run on MAVSDK's
// own thread, concurrently with the rclcpp timer thread. Position/velocity
// are kept in std::atomic<double> for that reason; a full mutex isn't
// needed since each axis updates independently and a few-millisecond skew
// between x/y/z is imperceptible for a smoothly-flying setpoint.
//
// ros2 run swarm_comm swarm_node --ros-args -p id:=0/1/2
// (expects a PX4 SITL instance already up on udp 14540+id -- see
// run_swarm.sh)
//
// ============================================================================
// RÉSUMÉ EN FRANÇAIS -- rôle général de ce nœud ROS2
// ============================================================================
// Ce fichier est le nœud ROS2 principal ("swarm_node") exécuté par CHAQUE
// drone de l'essaim (un exemplaire par drone, lancé avec un paramètre --id
// différent : 0, 1 ou 2). Il fait à la fois :
//
//   1. Pilotage réel du drone : connexion à sa propre instance PX4 SITL via
//      MAVSDK (bibliothèque de contrôle de drones), armement, décollage,
//      puis passage en mode "Offboard" où ce nœud envoie 10 fois par
//      seconde une consigne de vitesse (setpoint) au pilote automatique.
//
//   2. Communication inter-drones : chaque drone publie en continu son état
//      (position, vitesse, qui il pense être le leader...) sur son propre
//      topic ROS2 "/drone_<id>/swarm_status" (type SwarmStatus, un message
//      personnalisé défini dans swarm_msgs), et s'abonne aux topics des
//      autres drones pour connaître leur état. Une portée radio simulée
//      (MAX_RANGE_M) limite la distance à laquelle deux drones peuvent
//      encore "s'entendre" -- au-delà, un message est ignoré comme s'il
//      n'avait jamais été reçu.
//
//   3. Élection de leader : à partir des messages reçus des pairs, chaque
//      drone déduit qui est actuellement "en vie" (signal reçu récemment,
//      avant expiration d'un délai TIMEOUT_S) et fait tourner un algorithme
//      de vote majoritaire (voir elect_leader()) pour désigner un leader
//      commun, de façon robuste aux déconnexions/reconnexions et aux
//      changements de topologie du réseau simulé.
//
//   4. Gestion de formation : le drone leader suit une trajectoire scriptée
//      (cercle de "loiter" ou aller-retour en ligne droite), et les drones
//      suiveurs (followers) maintiennent une position relative en triangle
//      derrière/à côté du leader, avec en plus une règle de séparation qui
//      repousse deux drones qui se rapprochent trop l'un de l'autre
//      (anticollision).
//
// Topics ROS2 publiés : "/drone_<id>/swarm_status" (SwarmStatus, 10 Hz).
// Topics ROS2 souscrits : "/drone_<peer>/swarm_status" pour chaque autre
// drone de l'essaim.
// Paramètres ROS2 : "id" (obligatoire, numéro du drone), "x"/"y"/"z"
// (position de secours pour tester sans PX4), "excursion" (déclenche une
// sortie temporaire de la formation, pour démonstration), "flight_pattern"
// ("loiter" ou "line", trajectoire du leader).
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <swarm_msgs/msg/swarm_status.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using swarm_msgs::msg::SwarmStatus;

namespace {

// --------------------------------------------------------------------------
// Constantes de configuration de l'essaim (namespace anonyme = usage interne
// à ce fichier uniquement). Regroupe les paramètres physiques, temporels et
// de sécurité qui pilotent le comportement de communication/élection/
// formation ci-dessous.
// --------------------------------------------------------------------------

constexpr int MAX_DRONES = 3;   // taille fixe de l'essaim (3 drones : id 0, 1, 2)
constexpr double TIMEOUT_S = 3.0;  // silence radio (s) au-delà duquel un pair est déclaré "perdu"
// Simulated radio range. Widened from an original 6m after live testing
// showed the triangle formation's own footprint (~1.8-2m) leaves too
// little slack at 6m: a follower doesn't need to drift far off-station
// before a transient (a hard turn, a moment of separation-avoidance) pushes
// a pair briefly past range and triggers an avoidable reconnect cycle.
// 10m keeps that margin comfortably above the formation's normal spread.
constexpr double MAX_RANGE_M = 10.0;  // portée radio simulée (m) : au-delà, un message pair est ignoré
constexpr int NO_LEADER = -1;  // valeur sentinelle : "aucun leader connu/élu"

constexpr int PX4_BASE_PORT = 14540;  // instance i: MAVSDK connects to udpin://0.0.0.0:14540+i
// -> chaque drone i se connecte à son propre PX4 SITL sur le port 14540+i.

// Décalages de spawn (position de départ) tirés de worlds/swarm_persistent.sdf
// (repère monde ENU : x=Est, y=Nord). Sert uniquement à convertir la position
// locale NED de chaque véhicule (relative à son propre point de spawn) vers
// le repère monde partagé -- voir la note "Frame" en en-tête du fichier.
constexpr double SPAWN_EAST_M[MAX_DRONES] = {0.0, 4.0, 8.0};
constexpr double SPAWN_NORTH_M[MAX_DRONES] = {0.0, 0.0, 0.0};

constexpr double TAKEOFF_WAIT_ALT_M = 1.5;   // altitude (m) au-delà de laquelle on considère le drone "en vol"
constexpr double CONTROL_PERIOD_S = 0.1;     // période (s) de la boucle de contrôle : 10Hz, diffusion + consigne Offboard
// Comfortably above the leader's loiter cruise speed (LOITER_RADIUS_M *
// LOITER_OMEGA =~ 0.84 m/s): a live flight test showed that with too little
// margin here, a follower that gets separation-repulsed away from the
// leader during the initial post-takeoff convergence chaos can lose enough
// ground that it never fully recovers -- the gap keeps growing until the
// pair exceeds MAX_RANGE_M and permanently splits off into its own
// leaderless sub-group. This margin plus the slower LOITER_PERIOD_S below
// give followers enough spare speed to reliably close a transient gap
// before it becomes unrecoverable.
constexpr double MAX_SPEED_M_S = 3.5;  // vitesse maximale (m/s) autorisée pour toute consigne de vol
constexpr double SEEK_KP = 0.8;              // gain proportionnel (loi de commande "seek") : écart de position -> vitesse

// Damping gain: pulls back on however much my *actual* velocity (real MAVSDK
// telemetry, not the setpoint just sent) exceeds the feedforward term. A
// live test with KP-only (+ feedforward) control caught a drone that had
// been pushed well off its target oscillating -- closing more than half the
// gap, then reversing and diverging *further* than where it started, with
// no further leader handoff or repulsion event to explain it (repulsion was
// 0 throughout) -- textbook underdamped feedback fighting real vehicle
// inertia/lag PX4's own velocity controller can't erase instantly. A pure
// P(+FF) term has no way to see that: it only reacts to the position gap,
// never to the fact that the vehicle is already carrying real velocity that
// the next setpoint hasn't caught up with yet. Damping directly against
// that real velocity turns this into a proper PD loop and removes the
// oscillation.
constexpr double SEEK_KD = 0.6;  // gain dérivé (amortissement) : contrôleur PD complet, évite les oscillations

// Formation-seeking (not repulsion -- see below) is speed-capped for this
// long right after becoming airborne, ramping linearly from
// TAKEOFF_SPEED_CAP_M_S up to the normal MAX_SPEED_M_S. Every drone
// independently converges on its formation slot the instant it's airborne
// -- at full MAX_SPEED_M_S closing speed on each side, a live test traced
// pairs closing to 0.18-0.29m (very likely an actual airframe collision in
// Gazebo, not just a logical proximity warning -- see
// separation_repulsion()'s comment) within about two 10Hz control ticks of
// entering the repulsion margin, faster than the control loop or a stronger
// repulsion ramp could react to. Slowing the deliberate, known-hazardous
// first few seconds gives both the loop and repulsion time to actually
// work. Repulsion itself is deliberately exempt -- it keeps full authority
// throughout, so an actual close encounter still gets maximum escape speed
// even during the slow-start window. Spawn spacing was also widened 2m ->
// 4m (see SPAWN_EAST_M / worlds/swarm_persistent.sdf) after this alone
// wasn't enough to stop near-collisions -- untested whether it's sufficient
// on its own.
constexpr double TAKEOFF_SPEED_CAP_M_S = 1.0;  // vitesse plafond (m/s) juste après le décollage
constexpr double TAKEOFF_SPEED_RAMP_S = 8.0;   // durée (s) de la rampe qui remonte de TAKEOFF_SPEED_CAP_M_S vers MAX_SPEED_M_S

// Finite "there and back" mission instead of an indefinite loiter/line loop:
// every drone flies its role (scripted leader path, or formation-following)
// for this long after becoming airborne, then independently breaks off --
// leader and followers alike -- and seeks its own spawn position directly
// (no more formation offset), landing once it arrives. Each drone times
// this off its own airborne_since_s_, so no extra coordination/messaging is
// needed for the whole swarm to head home at roughly the same time.
//
// 300s (5 min), not the original 40s: 40s was fine for an unattended
// automated test run, but live-testing demo_presentation.sh caught it
// landing the whole swarm out from under a live presentation -- the
// script's own ~40s of startup waits (PX4/Gazebo boot + node connect) ran
// right up against the mission clock, so by the time it handed control
// back the mission was already ending. 300s gives comfortable room to
// actually narrate the formation and trigger the excursion demo before the
// auto-return kicks in; RTL is still available on demand (Ctrl-C).
constexpr double MISSION_OUTBOUND_S = 300.0;  // durée (s) de la mission avant retour automatique au point de spawn
constexpr double HOME_ARRIVAL_RADIUS_M = 0.5; // distance horizontale (m) au spawn déclenchant l'atterrissage

constexpr double SAFE_DIST_M = 1.0;          // distance de sécurité minimale (m) imposée entre 2 drones quelconques

// On-demand "fly away and come back" excursion (triggered via `ros2 param
// set /drone_N excursion true`), for demoing the radio-range disconnect/
// reconnect path without the previous approach's problem: forcing an
// instant `gz set_pose` teleport on the physical vehicle. A teleport jumps
// position with no matching velocity, which the physics engine and PX4's
// EKF were never designed to absorb -- see MIN_ALT_HARD_M's comment thread
// for the resulting attitude-upset/freefall investigation, concluded as a
// Gazebo/PX4 SITL issue outside what this control loop can fix. Flying the
// same distance under normal Offboard velocity control instead never
// produces that discontinuity: same disconnect/re-election/reconnect
// behavior at the swarm-comm level (still driven purely by real distance
// vs. MAX_RANGE_M in on_peer_status()), reached by a continuous, physically
// ordinary flight path instead of a jump.
constexpr double EXCURSION_DURATION_S = 10.0;  // durée (s) de l'excursion volontaire hors formation (démo déconnexion/reconnexion)
// Clear of MAX_RANGE_M (10m) so the excursion reads as a real disconnect to
// peers, comfortably inside MAX_GEOFENCE_HARD_M (40m). Not just barely
// clear: the target is a fixed offset from the excursing drone's own
// position at trigger time, but the *peers* it needs to out-range keep
// moving too (the leader's loiter reaches up to
// LOITER_RADIUS_M*LOITER_OMEGA =~ 0.84 m/s).
//
// Honest result from repeated live trials: raising this from 15m to 20m to
// 25m did NOT cleanly raise the odds of an actual disconnect -- roughly 1
// run in 3 at each distance never dropped a peer at all (no "Signal lost"
// line, nothing to see), no better at 25m than at 20m. That's a sign the
// real culprit isn't distance margin at all: TIMEOUT_S (3s of silence
// needed to declare a peer lost) can outlast a real-distance excursion if
// the gap crosses back under MAX_RANGE_M even briefly before 3s of misses
// accumulate -- distance alone can't fix a timing coincidence like that.
// Left at 25m (no worse than 20m, and more margin against ordinary peer
// motion is still a reasonable thing to want), but don't trust a single
// trigger to reliably show a disconnect: if a live trigger doesn't print a
// "Signal lost" line within a few seconds, just trigger it again
// (`ros2 param set` is safe to repeat) -- worth fixing properly with more
// time than a demo-eve pass allows.
constexpr double EXCURSION_DISTANCE_M = 25.0;  // distance cible (m) de l'excursion, au-delà de MAX_RANGE_M pour provoquer une vraie déconnexion
// Same speed-ramp idea as TAKEOFF_SPEED_RAMP_S, applied at excursion start: without it,
// the position gap to a target 15m away saturates clamp_speed() immediately, snapping from
// whatever formation-seeking velocity was commanded the tick before straight to
// MAX_SPEED_M_S in one step. Ramping from TAKEOFF_SPEED_CAP_M_S over this many seconds
// gives a smooth departure instead of that step. The return leg needs no equivalent ramp:
// by the time it happens, the same distance gap drives normal formation-seeking, which
// already ramps down smoothly via seek_velocity's own P(D) term as the gap closes.
constexpr double EXCURSION_RAMP_S = 2.0;  // durée (s) de la rampe de vitesse en début d'excursion (évite un à-coup)

// Repulsion starts ramping at this distance, well before SAFE_DIST_M --
// anticipatory rather than purely reactive. Chosen below the formation's
// own normal spacing (~1.8m leader-follower, ~2.0m follower-follower, see
// FORMATION_TRAIL_M/HALF_WIDTH_M below) so steady-state formation flight
// doesn't sit inside the margin and fight itself; a live test showed the
// old margin (repulsion only starting exactly at SAFE_DIST_M) meant
// avoidance only ever kicked in once already in violation, one control
// tick from breaching it further on any real closing speed.
constexpr double AVOID_MARGIN_M = 1.4;  // distance (m) à partir de laquelle la répulsion anticollision commence à monter en puissance

// Nombre de cycles de contrôle consécutifs (à CONTROL_PERIOD_S chacun) qu'un
// pair peut passer sous SAFE_DIST_M avant d'être traité comme une proximité
// "chronique" plutôt qu'un simple frôlement passager -- voir l'usage de
// close_encounter_ticks_ dans control_step().
constexpr int CHRONIC_TICKS_THRESHOLD = 20; // 2s at the 10Hz control rate

// Formation en triangle, exprimée dans le repère avant/droite propre au leader.
constexpr double FORMATION_TRAIL_M = 1.5;    // distance (m) derrière le leader
constexpr double FORMATION_HALF_WIDTH_M = 1.0;  // décalage latéral (m) de chaque côté du leader

// Leader's scripted reference path: circular loiter, centered between the
// 3 spawn points, keyed off wall-clock time so a newly-elected leader
// continues the same curve instead of restarting it.
constexpr double LOITER_CENTER_EAST_M = 2.0;   // centre du cercle de loiter (Est, m)
constexpr double LOITER_CENTER_NORTH_M = 0.0;  // centre du cercle de loiter (Nord, m)
constexpr double LOITER_RADIUS_M = 6.0;        // rayon du cercle (m)
constexpr double LOITER_ALT_M = 3.0;           // altitude de croisière (m)
constexpr double LOITER_PERIOD_S = 45.0; // slower cruise = more margin for followers to keep pace
constexpr double LOITER_OMEGA = 2.0 * M_PI / LOITER_PERIOD_S;  // vitesse angulaire (rad/s) déduite de la période

// Alternative leader path: a straight line back and forth along the east
// axis (through the same center point as the loiter), selected via the
// `flight_pattern` parameter ("loiter", the default, or "line"). Same
// center/altitude as the loiter so both patterns stay comparable.
//
// Sinusoidal, not a constant-speed triangle wave: a live test with the
// triangle-wave version caught a follower flying to 41m from origin during
// a network split, traced to the leader's velocity reversing *instantly*
// at each end of the line -- a step discontinuity that snapped a
// follower's heading-based formation offset to the opposite side in one
// control tick. sin/cos gives continuous position *and* velocity
// everywhere, so there's no discontinuity for a follower to react badly
// to, matching how the circular loiter never had this problem.
constexpr double LINE_HALF_LENGTH_M = 8.0;  // demi-longueur (m) du segment parcouru par le motif "line"
constexpr double LINE_SPEED_M_S = 1.0; // peak speed, at the center of the traversal
constexpr double LINE_OMEGA = LINE_SPEED_M_S / LINE_HALF_LENGTH_M; // rad/s -- pulsation équivalente du mouvement sinusoïdal

// Safety bounds: any flight target (leader's own reference point, or a
// follower's formation slot) gets clamped into this box/cylinder before
// being pursued, so a bug in the control law can steer the *seek point*
// wherever it wants without ever commanding the vehicle itself outside a
// known-safe volume. MAX_ALT_HARD_M/MAX_GEOFENCE_HARD_M are harder
// backstops checked directly against *actual* position -- belt and
// suspenders, since clamping the target alone doesn't stop a drone that's
// already outside the fence from continuing to fly away from it (a live
// test caught exactly this: a follower isolated during a network split,
// still chasing a stale target at full speed, reached 41m out -- well
// past GEOFENCE_RADIUS_M -- before the split resolved and it turned back).
constexpr double GEOFENCE_RADIUS_M = 30.0;  // rayon (m) de la barrière géographique "souple" appliquée aux cibles visées
constexpr double MIN_ALT_M = 1.0;           // altitude minimale (m) autorisée pour une cible
constexpr double MAX_ALT_M = 6.0;           // altitude maximale (m) autorisée pour une cible
constexpr double MAX_ALT_HARD_M = 10.0;     // altitude réelle maximale (m) : au-delà, atterrissage d'urgence
constexpr double MAX_GEOFENCE_HARD_M = 40.0; // distance réelle maximale (m) à l'origine : au-delà, atterrissage d'urgence

// Low-altitude counterpart to MAX_ALT_HARD_M -- well below MIN_ALT_M's
// already-clamped target band, so normal flight (including post-takeoff
// convergence chaos) never comes near it. A live test traced a drone's
// *actual* altitude sagging to -4.83m -- i.e. well below the ground plane,
// something a target clamp alone can't explain -- immediately followed by a
// real MAVSDK-measured velocity spike over 15 m/s with no setpoint anywhere
// near that: Gazebo reacting to the vehicle skimming/clipping the ground
// with an unrealistic collision impulse. Landing here, the same response
// already used for the other hard backstops, catches the sag within one
// control tick of crossing this floor -- long before it's deep enough for
// that impulse to happen -- rather than continuing to fight it from inside
// Offboard while the discrepancy grows.
constexpr double MIN_ALT_HARD_M = 0.3;  // altitude réelle minimale (m) : en dessous, atterrissage d'urgence (garde-fou bas)

// Diagnostic instrumentation kept permanently for the still-open
// low-altitude physics anomaly (see MIN_ALT_HARD_M): once a drone strays
// this far from origin (well before MAX_GEOFENCE_HARD_M), dump full
// per-tick internal state so a horizontal excursion can be traced as it
// forms rather than only caught after the fact at 40m.
constexpr double DRIFT_TRACE_M = 12.0;  // distance (m) à l'origine déclenchant les logs de diagnostic détaillés

// Vecteur 3D générique utilisé partout dans ce fichier pour représenter une
// position ou une vitesse dans le repère monde partagé (Est/Nord/Haut).
struct Vec3 {
    double x = 0, y = 0, z = 0; // east, north, up
};

// Distance euclidienne (m) entre deux points 3D quelconques.
double distance_m(double x1, double y1, double z1, double x2, double y2, double z2) {
    double dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Clamps horizontal and vertical speed *separately*, not the 3D vector as a
// single magnitude. A live test traced a drone's altitude sagging to
// negative (below the ground plane) during a large horizontal excursion --
// with a single combined clamp, a big horizontal correction (seen well past
// 10-20 m/s during an active divergence) forces the same scale-down factor
// onto the altitude term too, crushing whatever climb correction was needed
// right when the vehicle most needed it, in favor of horizontal chase speed
// it can't even use productively past MAX_SPEED_M_S anyway. Gazebo then
// reacted to the vehicle skimming/clipping the ground with an unrealistic
// collision impulse -- a real velocity spike (over 15 m/s actual, measured
// via telemetry) with no matching setpoint anywhere near that. Giving
// altitude its own speed budget means it's never starved by horizontal
// chaos.
// Limite la vitesse d'un vecteur à max_speed, horizontalement et
// verticalement DE FAÇON SÉPARÉE (composante horizontale x/y réduite en
// gardant sa direction, composante verticale z bornée indépendamment) --
// voir le commentaire au-dessus pour la raison (éviter qu'une grosse
// correction horizontale n'écrase la correction d'altitude).
Vec3 clamp_speed(Vec3 v, double max_speed) {
    double horiz = std::hypot(v.x, v.y);
    if (horiz > max_speed && horiz > 1e-6) {
        double s = max_speed / horiz;
        v.x *= s; v.y *= s;
    }
    v.z = std::clamp(v.z, -max_speed, max_speed);
    return v;
}

// Pulls a flight target back inside the geofence cylinder / altitude band
// before it's ever pursued -- see GEOFENCE_RADIUS_M etc.
// Ramène une cible (point que le drone cherche à atteindre) à l'intérieur
// du cylindre de barrière géographique (GEOFENCE_RADIUS_M) et de la bande
// d'altitude autorisée (MIN_ALT_M..MAX_ALT_M), avant même qu'elle soit
// poursuivie par la loi de commande.
Vec3 clamp_to_geofence(Vec3 target) {
    double horiz = std::hypot(target.x, target.y);
    if (horiz > GEOFENCE_RADIUS_M) {
        double s = GEOFENCE_RADIUS_M / horiz;
        target.x *= s; target.y *= s;
    }
    target.z = std::clamp(target.z, MIN_ALT_M, MAX_ALT_M);
    return target;
}

// Leader's reference point on the loiter circle at time t (seconds, any
// consistent epoch -- we use wall-clock seconds so a leader handoff picks
// up the same curve seamlessly), plus its feedforward velocity.
// phase_offset shifts the circle so it passes near wherever the current
// leader actually was when it took over (see loiter_phase_offset_): without
// it, phase = OMEGA*wall_clock_seconds lands on an arbitrary point on the
// circle, and a leader starting far from that point would dash toward it
// at full speed the moment it goes airborne, dragging followers (which
// have no such shortcut) into a losing tail chase.
// Point de référence renvoyé par les trajectoires scriptées du leader :
// position visée (pos) et sa vitesse feedforward associée (vel).
struct ReferencePoint { Vec3 pos, vel; };

// Calcule le point de la trajectoire circulaire ("loiter") du leader à
// l'instant t (secondes), plus sa vitesse feedforward (tangente au cercle).
// phase_offset recale le cercle pour qu'un nouveau leader reprenne la
// courbe là où il se trouve, sans "sauter" vers un point arbitraire.
ReferencePoint loiter_reference(double t, double phase_offset) {
    double phase = LOITER_OMEGA * t + phase_offset;
    ReferencePoint r;
    r.pos.x = LOITER_CENTER_EAST_M + LOITER_RADIUS_M * std::cos(phase);
    r.pos.y = LOITER_CENTER_NORTH_M + LOITER_RADIUS_M * std::sin(phase);
    r.pos.z = LOITER_ALT_M;
    r.vel.x = -LOITER_RADIUS_M * LOITER_OMEGA * std::sin(phase);
    r.vel.y = LOITER_RADIUS_M * LOITER_OMEGA * std::cos(phase);
    r.vel.z = 0.0;
    return r;
}

// Straight-line counterpart to loiter_reference(): back and forth along
// the east axis. phase_offset plays the same re-anchoring role it does
// for the loiter -- an angle, since this is sinusoidal too now (see the
// comment on LINE_OMEGA for why it isn't a constant-speed triangle wave).
// Équivalent de loiter_reference() pour le motif "line" : trajectoire en
// aller-retour sinusoïdal le long de l'axe Est, avec la même logique de
// phase_offset pour recaler la position au moment d'une prise de leadership.
ReferencePoint line_reference(double t, double phase_offset) {
    double phase = LINE_OMEGA * t + phase_offset;
    ReferencePoint r;
    r.pos.x = LOITER_CENTER_EAST_M + LINE_HALF_LENGTH_M * std::sin(phase);
    r.pos.y = LOITER_CENTER_NORTH_M;
    r.pos.z = LOITER_ALT_M;
    r.vel.x = LINE_HALF_LENGTH_M * LINE_OMEGA * std::cos(phase);
    r.vel.y = 0.0;
    r.vel.z = 0.0;
    return r;
}

// Rotates a (forward, right) offset in the leader's heading frame into the
// shared world (east, north) frame. heading_rad is a compass bearing
// (0 = North, increasing toward East), matching NED yaw.
// Fait pivoter un décalage (avant, droite) exprimé dans le repère "cap du
// leader" vers le repère monde partagé (Est, Nord). heading_rad est un cap
// façon boussole (0 = Nord, augmente vers l'Est), cohérent avec le lacet NED.
Vec3 rotate_formation_offset(double forward_m, double right_m, double heading_rad, double up_m) {
    double fwd_e = std::sin(heading_rad), fwd_n = std::cos(heading_rad);
    double right_e = std::cos(heading_rad), right_n = -std::sin(heading_rad);
    Vec3 v;
    v.x = fwd_e * forward_m + right_e * right_m;
    v.y = fwd_n * forward_m + right_n * right_m;
    v.z = up_m;
    return v;
}

// Pure control law: given my role, current world position, the reference
// data it needs (leader loiter reference, or the leader's live tracked
// position/heading), returns a desired world-frame velocity (pre-collision
// -avoidance, pre-clamp). Kept free of ROS/MAVSDK types so the formation
// math is exercised the same way whether driven by real telemetry or a
// kinematic stand-in.
// Loi de commande PD (proportionnelle-dérivée) + feedforward : calcule la
// vitesse à commander pour rejoindre `target` depuis `my_pos`, en tenant
// compte de la vitesse actuelle `my_vel` (terme dérivé, amortissement) et
// d'une vitesse feedforward optionnelle (ex : vitesse du leader à suivre).
// Ne dépend d'aucun type ROS/MAVSDK -- testable indépendamment de la
// simulation réelle.
Vec3 seek_velocity(const Vec3& target, const Vec3& my_pos, const Vec3& my_vel,
                    const Vec3& feedforward = Vec3{}) {
    Vec3 v;
    v.x = feedforward.x + SEEK_KP * (target.x - my_pos.x) - SEEK_KD * (my_vel.x - feedforward.x);
    v.y = feedforward.y + SEEK_KP * (target.y - my_pos.y) - SEEK_KD * (my_vel.y - feedforward.y);
    v.z = feedforward.z + SEEK_KP * (target.z - my_pos.z) - SEEK_KD * (my_vel.z - feedforward.z);
    return v;
}

// Repulsion vector keeping every tracked peer >= SAFE_DIST_M away.
// Starts ramping at AVOID_MARGIN_M (well outside the hard floor) rather
// than exactly at SAFE_DIST_M, and ramps quadratically rather than
// linearly: a gentle early nudge that only becomes sharply repulsive
// right near the floor, not a switch that's off right up until the
// instant a violation is already happening.
//
// The ramp reaches *full* strength (matching MAX_SPEED_M_S, same budget as
// formation-seeking, via the priority blend in control_step()) exactly at
// SAFE_DIST_M, not only as d approaches 0 -- a live test caught a pair
// closing at up to MAX_SPEED_M_S each (7 m/s combined) blow straight through
// the floor to 0.18m apart in about two 10Hz control ticks, because
// normalizing the ramp against the *full* AVOID_MARGIN_M interval meant
// strength was still only ~8% at the floor itself (it wouldn't reach 100%
// until d neared 0) -- weakest exactly where it's supposed to be a hard
// limit. 0.18m between two multirotor airframes is almost certainly an
// actual physical collision in Gazebo, not just a logical proximity
// warning, and no control-law correction after that point can undo the loss
// of authority a real collision causes (this is the same mechanism behind
// the sudden real-telemetry velocity spikes traced to MIN_ALT_HARD_M).
// Normalizing against (AVOID_MARGIN_M - SAFE_DIST_M) instead means the
// early-warning zone above the floor keeps the same gentle quadratic
// character, but nothing between the floor and total collision is ever
// weaker than full authority.
//
// Horizontal distance/push only -- deliberately never touches z. The
// triangle formation is flat by design (every offset's up_m is 0; leader and
// both followers all target the same altitude), so two drones ever needing
// separation are, by construction, close in altitude too, not stacked. A 3D
// push used to add a vertical component sized off of whichever tiny altitude
// difference happened to exist at that instant -- and a live test found that
// during the post-takeoff convergence chaos (drones still closing on their
// formation slots, near cruise altitudes already close to the floor),
// landing on the "push down" side of that coin was enough on its own to sag
// a drone toward/through the ground and into an unrealistic Gazebo collision
// impulse (see MIN_ALT_HARD_M). Since vertical separation was never doing
// real collision-avoidance work here (the formation has none to preserve),
// dropping it removes the risk without giving up anything.
// Calcule un vecteur de répulsion (anticollision) qui écarte le drone de
// chaque pair tracké situé à moins de AVOID_MARGIN_M, avec une montée en
// puissance quadratique (douce au début, pleine puissance dès SAFE_DIST_M
// atteint). Uniquement horizontal (x/y) -- jamais de composante verticale,
// puisque la formation est plate par construction (voir commentaire ci-dessus).
Vec3 separation_repulsion(const Vec3& my_pos, const std::vector<Vec3>& peer_positions) {
    Vec3 push;
    for (const auto& p : peer_positions) {
        double d = std::hypot(my_pos.x - p.x, my_pos.y - p.y);
        if (d < AVOID_MARGIN_M && d > 1e-3) {
            double t = std::clamp((AVOID_MARGIN_M - d) / (AVOID_MARGIN_M - SAFE_DIST_M), 0.0, 1.0);
            double strength = t * t; // quadratic: gentle early, full strength by the floor
            push.x += (my_pos.x - p.x) / d * strength * MAX_SPEED_M_S;
            push.y += (my_pos.y - p.y) / d * strength * MAX_SPEED_M_S;
        }
    }
    return push;
}

} // namespace

// État suivi localement pour chaque AUTRE drone de l'essaim (un tracker par
// pair, stocké dans SwarmNode::trackers_). Mis à jour à chaque réception
// d'un message SwarmStatus de ce pair (voir on_peer_status()) et par le
// contrôle de timeout (voir check_and_elect()).
struct DroneTracker {
    rclcpp::Time last_seen;           // horodatage ROS du dernier message reçu de ce pair
    bool has_seen = false;            // au moins un message a déjà été reçu de ce pair
    bool connected = false;           // pair actuellement "en vie" (pas de timeout dépassé)
    int last_claimed_leader = NO_LEADER;  // dernier leader que CE pair prétendait suivre (pour le vote)
    Vec3 pos, vel;                    // dernière position/vitesse connues de ce pair (repère monde)
    Vec3 ref_vel; // analytical scripted-path velocity, only meaningful while this peer is leader
    bool ready = false; // this peer's own offboard_ready_ -- see SwarmStatus.msg's `ready` field
};

// Nœud ROS2 principal exécuté par chaque drone de l'essaim. Combine :
// - la connexion/pilotage MAVSDK vers PX4 (thread dédié, voir connect_and_fly()) ;
// - la communication et le suivi des autres drones (publication/souscription
//   du topic SwarmStatus, voir on_peer_status()) ;
// - l'élection de leader (voir elect_leader()) ;
// - la boucle de contrôle 10Hz qui calcule et envoie les consignes de
//   vitesse Offboard selon le rôle (leader ou follower), voir control_step().
class SwarmNode : public rclcpp::Node {
public:
    // Constructeur : lit le paramètre "id" (numéro du drone, 0 à MAX_DRONES-1,
    // obligatoire), initialise les trackers pour les autres drones, crée le
    // publisher/les souscriptions ROS2, déclare les paramètres dynamiques
    // (x/y/z, excursion, flight_pattern), démarre les timers de contrôle et
    // d'élection, puis lance la connexion MAVSDK sur un thread séparé.
    SwarmNode() : Node("swarm_node") {
        my_id_ = declare_parameter<int>("id", -1);
        if (my_id_ < 0 || my_id_ >= MAX_DRONES) {
            RCLCPP_ERROR(get_logger(), "Usage: ros2 run swarm_comm swarm_node --ros-args -p id:=0|1|2");
            throw std::runtime_error("invalid --id");
        }
        my_name_ = "drone_" + std::to_string(my_id_);
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (i != my_id_) trackers_[i]; // default-constructs via map::operator[]
        }

        // Publie mon propre statut sur mon topic dédié, et m'abonne au topic
        // de chaque autre drone pour recevoir le leur (communication inter-drones).
        pub_ = create_publisher<SwarmStatus>("/drone_" + std::to_string(my_id_) + "/swarm_status", 10);
        for (int peer = 0; peer < MAX_DRONES; ++peer) {
            if (peer == my_id_) continue;
            std::string topic = "/drone_" + std::to_string(peer) + "/swarm_status";
            subs_.push_back(create_subscription<SwarmStatus>(
                topic, 10, [this, peer](SwarmStatus::SharedPtr msg) { on_peer_status(peer, msg); }));
        }

        // x/y/z parameters: a fallback position source for comm/election
        // testing without PX4 (test_election.py's `ros2 param set x <val>`
        // trick to fake a range disconnect). Once MAVSDK telemetry starts
        // flowing during real flight it overwrites these atomics 10x/sec,
        // so a stray `ros2 param set` during real flight is harmless --
        // it's stale again before the next control tick.
        declare_parameter<double>("x", 0.0);
        declare_parameter<double>("y", 0.0);
        declare_parameter<double>("z", 0.0);
        // Excursion trigger -- see EXCURSION_DURATION_S's comment. Edge-triggered on each
        // `true` set while not already mid-excursion, so it can be re-armed by setting it
        // true again after a previous excursion completes (excursion_active_ resets to
        // false on its own in control_step(), independent of this parameter's stored value).
        declare_parameter<bool>("excursion", false);
        // Which scripted path the leader flies: "loiter" (circular, default)
        // or "line" (straight back-and-forth). Read once at startup --
        // whoever holds leadership when this is checked flies that pattern;
        // it's not meant to be swapped live mid-flight.
        flight_pattern_ = declare_parameter<std::string>("flight_pattern", "loiter");
        my_x_.store(get_parameter("x").as_double());
        my_y_.store(get_parameter("y").as_double());
        my_z_.store(get_parameter("z").as_double());
        // Callback appelé à chaque `ros2 param set` sur ce nœud : met à jour
        // x/y/z (position de secours) et déclenche une excursion si le
        // paramètre "excursion" passe à true (front montant uniquement).
        param_cb_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if (p.get_name() == "x") my_x_.store(p.as_double());
                    else if (p.get_name() == "y") my_y_.store(p.as_double());
                    else if (p.get_name() == "z") my_z_.store(p.as_double());
                    else if (p.get_name() == "excursion" && p.as_bool() && !excursion_active_) {
                        excursion_active_ = true;
                        excursion_started_s_ = now().seconds();
                        excursion_until_s_ = excursion_started_s_ + EXCURSION_DURATION_S;
                        Vec3 my_pos{my_x_.load(), my_y_.load(), my_z_.load()};
                        excursion_target_ = Vec3{my_pos.x, my_pos.y - EXCURSION_DISTANCE_M, my_pos.z};
                        RCLCPP_INFO(get_logger(),
                            "[%s] Excursion started -- heading to (%.1f, %.1f, %.1f) for %.0fs.",
                            my_name_.c_str(), excursion_target_.x, excursion_target_.y,
                            excursion_target_.z, EXCURSION_DURATION_S);
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        // Timer 1 (10Hz) : boucle de contrôle/pilotage -- voir control_step().
        // Timer 2 (2Hz) : détection des pairs perdus (timeout) + élection du leader.
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(CONTROL_PERIOD_S), [this]() { control_step(); });
        check_timer_ = create_wall_timer(500ms, [this]() { check_and_elect(); });

        RCLCPP_INFO(get_logger(), "[%s] Starting. Connecting to PX4 on udp %d...",
                    my_name_.c_str(), PX4_BASE_PORT + my_id_);
        std::thread(&SwarmNode::connect_and_fly, this).detach();

        // Demo/orchestration convenience: a clean SIGINT (ros2 launch's own
        // Ctrl-C handling, which calls rclcpp::shutdown() before this runs --
        // not a raw OS signal handler, so it's safe to make a blocking MAVSDK
        // call here) commands RTL instead of just dropping the offboard link
        // and leaving PX4 to its own failsafe. No effect if flight never
        // reached Offboard (action_ still null).
        rclcpp::on_shutdown([this]() {
            if (action_) {
                RCLCPP_INFO(get_logger(), "[%s] Shutdown requested -- returning to launch.",
                            my_name_.c_str());
                action_->return_to_launch();
            }
        });
    }

private:
    // --- MAVSDK setup + arm/takeoff/offboard sequence, run on its own
    // thread so it never blocks the ROS executor (this involves several
    // blocking MAVSDK calls and a 30s connection wait). ---
    // Séquence complète de connexion au pilote automatique et de mise en
    // vol : connexion MAVSDK à PX4, abonnement à la télémétrie (position,
    // vitesse, mode de vol, santé...), armement, décollage, puis bascule
    // en mode Offboard (mode où ce nœud contrôle directement la vitesse du
    // drone). Tourne sur un thread séparé pour ne jamais bloquer
    // l'exécuteur ROS2 pendant les appels MAVSDK bloquants.
    void connect_and_fly() {
        mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
            mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});
        std::string url = "udpin://0.0.0.0:" + std::to_string(PX4_BASE_PORT + my_id_);
        if (mavsdk_->add_any_connection(url) != mavsdk::ConnectionResult::Success) {
            RCLCPP_ERROR(get_logger(), "[%s] MAVSDK connection failed on %s", my_name_.c_str(), url.c_str());
            return;
        }

        // 90s, not the more obvious ~30s: this needs to comfortably outlast
        // a cold PX4+Gazebo boot (run_swarm.sh's own stagger already means
        // drone_1/2 don't even start booting until ~15s after drone_0, on
        // top of each instance's own Gazebo-attach + EKF wait), for the
        // case where swarm_node is started at the same time as the
        // simulator rather than against an already-warm one -- see
        // test_formation.py, which now launches both itself.
        auto system = mavsdk_->first_autopilot(90.0);
        if (!system) {
            RCLCPP_ERROR(get_logger(), "[%s] No autopilot found after 90s", my_name_.c_str());
            return;
        }

        telemetry_ = std::make_unique<mavsdk::Telemetry>(system.value());
        action_ = std::make_unique<mavsdk::Action>(system.value());
        offboard_ = std::make_unique<mavsdk::Offboard>(system.value());

        // Callback télémétrie principal : convertit la position/vitesse NED
        // locale (propre à ce véhicule) en repère monde partagé Est/Nord/Haut
        // en ajoutant le décalage de spawn (voir SPAWN_EAST_M/SPAWN_NORTH_M et
        // la note "Frame" en en-tête du fichier). Écrit dans des std::atomic
        // car ce callback tourne sur le thread MAVSDK, concurremment avec le
        // thread des timers ROS2.
        telemetry_->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed pv) {
            my_x_.store(SPAWN_EAST_M[my_id_] + pv.position.east_m);
            my_y_.store(SPAWN_NORTH_M[my_id_] + pv.position.north_m);
            my_z_.store(-pv.position.down_m);
            my_vx_.store(pv.velocity.east_m_s);
            my_vy_.store(pv.velocity.north_m_s);
            my_vz_.store(-pv.velocity.down_m_s);
        });

        // PX4's *actual* flight mode -- separate from offboard_ready_, which
        // only reflects whether our offboard_->start() call was accepted,
        // not whether PX4 stayed in Offboard afterward. Investigating a
        // drone that sat frozen at its spawn point with no error logged
        // anywhere: this is the one thing that could explain velocity
        // setpoints being sent successfully (Offboard::Result::Success)
        // while PX4 silently ignores them because it's not actually in
        // Offboard mode.
        // Suit le mode de vol RÉEL de PX4 (pas seulement si notre appel
        // offboard_->start() a été accepté) : journalise chaque changement.
        telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode mode) {
            auto m = static_cast<int>(mode);
            current_flight_mode_.store(m);
            if (m != last_logged_flight_mode_.exchange(m)) {
                std::ostringstream oss;
                oss << mode;
                RCLCPP_WARN(get_logger(), "[%s] PX4 flight mode changed: %s",
                            my_name_.c_str(), oss.str().c_str());
            }
        });

        // Diagnostic instrumentation for the still-open freefall investigation
        // (see MIN_ALT_HARD_M / README's "What's left"): PX4 flags crash
        // detection, failsafes, and landed-state transitions through these
        // channels well before/independent of anything swarm_node itself
        // computes -- if this is a real PX4-side event (crash detector,
        // EKF reset, a disarm) rather than a pure Gazebo physics glitch, this
        // is where it would show up.
        // Callbacks de diagnostic (texte de statut PX4, état armé/désarmé,
        // état "posé"/en vol) : journalisent uniquement les CHANGEMENTS
        // d'état, utiles pour comprendre un comportement anormal côté PX4.
        telemetry_->subscribe_status_text([this](mavsdk::Telemetry::StatusText st) {
            RCLCPP_WARN(get_logger(), "[%s] PX4 STATUSTEXT: %s", my_name_.c_str(), st.text.c_str());
        });
        telemetry_->subscribe_armed([this](bool armed) {
            bool a = armed;
            if (a != last_logged_armed_.exchange(a)) {
                RCLCPP_WARN(get_logger(), "[%s] PX4 armed state changed: %s",
                            my_name_.c_str(), a ? "ARMED" : "DISARMED");
            }
        });
        telemetry_->subscribe_landed_state([this](mavsdk::Telemetry::LandedState state) {
            auto s = static_cast<int>(state);
            if (s != last_logged_landed_state_.exchange(s)) {
                std::ostringstream oss;
                oss << state;
                RCLCPP_WARN(get_logger(), "[%s] PX4 landed_state changed: %s",
                            my_name_.c_str(), oss.str().c_str());
            }
        });
        // Diagnostic-only (freefall investigation): if a real physical
        // collision knocked the vehicle's attitude off, the outer velocity
        // loop commanding a climb wouldn't show it directly -- most of the
        // thrust vector could be pointing sideways rather than up, which
        // would explain climbing hard while still losing altitude with no
        // PX4-side failsafe/crash-detect message (see STATUSTEXT above).
        telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle angle) {
            my_roll_deg_.store(angle.roll_deg);
            my_pitch_deg_.store(angle.pitch_deg);
        });

        // Attend que le GPS (position globale) et le point "home" soient
        // valides côté PX4 avant de continuer -- max 30s d'attente.
        RCLCPP_INFO(get_logger(), "[%s] Waiting for global + home position...", my_name_.c_str());
        // Captured by value (shared_ptr) rather than by reference: this
        // subscription is never explicitly cancelled, so a by-reference
        // capture of these stack locals would dangle the moment this
        // function returns -- and MAVSDK keeps delivering health updates
        // long after that, straight into freed stack memory. A crash
        // that took down all 3 drones right after "Airborne, in Offboard"
        // came from exactly this.
        auto ready_promise = std::make_shared<std::promise<void>>();
        auto ready_future = ready_promise->get_future();
        auto ready_fired = std::make_shared<std::atomic<bool>>(false);
        telemetry_->subscribe_health([ready_promise, ready_fired](mavsdk::Telemetry::Health health) {
            if (!ready_fired->exchange(true) && health.is_global_position_ok && health.is_home_position_ok) {
                ready_promise->set_value();
            }
        });
        if (ready_future.wait_for(30s) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "[%s] Timed out waiting for position lock", my_name_.c_str());
            return;
        }

        // Armement puis décollage (commandes MAVSDK Action), puis attente
        // (max 30s) que l'altitude réelle dépasse TAKEOFF_WAIT_ALT_M avant
        // de considérer le drone "en vol".
        RCLCPP_INFO(get_logger(), "[%s] Arming...", my_name_.c_str());
        if (auto res = action_->arm(); res != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(get_logger(), "[%s] Arm failed", my_name_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "[%s] Taking off...", my_name_.c_str());
        if (auto res = action_->takeoff(); res != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(get_logger(), "[%s] Takeoff failed", my_name_.c_str());
            return;
        }
        for (int i = 0; i < 30 && my_z_.load() < TAKEOFF_WAIT_ALT_M; ++i) {
            std::this_thread::sleep_for(1s);
        }

        // Offboard requires an existing setpoint stream before it'll accept
        // PX4's mode switch -- seed a couple, then start. Offboard::start()
        // returning Success only means PX4 acked the mode-switch command,
        // not that it stayed there: live testing caught PX4 accepting the
        // switch and then reverting straight to Hold under a second later
        // (visible as a Takeoff -> Hold transition in the flight-mode log,
        // with no Offboard in between) -- presumably because it judged the
        // setpoint stream not yet "established" going into the switch.
        // Confirm against the real flight mode rather than trusting the
        // single start() call, and retry if it didn't stick.
        // Bascule en mode Offboard : envoie d'abord des consignes nulles
        // pour amorcer le flux (requis par PX4), puis demande le mode
        // Offboard et VÉRIFIE contre le mode de vol réel (pas seulement le
        // retour de l'appel) que PX4 y reste effectivement -- jusqu'à 5
        // tentatives en cas de rejet ou de retour immédiat à Hold.
        bool entered_offboard = false;
        for (int attempt = 0; attempt < 5 && !entered_offboard; ++attempt) {
            offboard_->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
            std::this_thread::sleep_for(100ms);
            offboard_->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
            if (auto res = offboard_->start(); res != mavsdk::Offboard::Result::Success) {
                RCLCPP_WARN(get_logger(), "[%s] Offboard start attempt %d rejected, retrying...",
                            my_name_.c_str(), attempt);
                continue;
            }
            std::this_thread::sleep_for(300ms);
            if (current_flight_mode_.load() == static_cast<int>(mavsdk::Telemetry::FlightMode::Offboard)) {
                entered_offboard = true;
            } else {
                RCLCPP_WARN(get_logger(), "[%s] Offboard reverted immediately (attempt %d), retrying...",
                            my_name_.c_str(), attempt);
            }
        }
        if (!entered_offboard) {
            RCLCPP_ERROR(get_logger(), "[%s] Could not establish Offboard mode after retries", my_name_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "[%s] Airborne, in Offboard.", my_name_.c_str());
        airborne_since_s_.store(now().seconds());
        offboard_ready_.store(true);
    }

    // --- 10Hz loop: publish real telemetry, and (once airborne) send the
    // role-appropriate Offboard velocity setpoint. ---
    // Cœur du nœud, appelé 10 fois par seconde (CONTROL_PERIOD_S) :
    //   1) construit et publie le message SwarmStatus (ma position/vitesse,
    //      qui je pense être le leader, l'état de mes pairs connectés) ;
    //   2) vérifie les garde-fous de sécurité durs (altitude, géo-barrière,
    //      retour au point de spawn) -- atterrissage d'urgence si dépassés ;
    //   3) calcule la vitesse désirée selon mon rôle (leader/follower),
    //      applique la rampe de vitesse au décollage/en excursion, mélange
    //      avec la répulsion anticollision, puis envoie la consigne de
    //      vitesse Offboard réelle à PX4.
    void control_step() {
        double now_s = now().seconds();
        Vec3 my_pos{my_x_.load(), my_y_.load(), my_z_.load()};
        Vec3 my_vel{my_vx_.load(), my_vy_.load(), my_vz_.load()};

        // Construction du message d'état diffusé aux autres drones.
        SwarmStatus msg;
        msg.sender_id = my_id_;
        msg.claimed_leader = leader_id_;
        msg.x = my_pos.x; msg.y = my_pos.y; msg.z = my_pos.z;
        msg.vx = my_vel.x; msg.vy = my_vel.y; msg.vz = my_vel.z;
        msg.timestamp = now_s;
        for (auto& [id, tracker] : trackers_) msg.peer_connected[id] = tracker.connected;
        msg.ready = offboard_ready_.load();
        // Only publish a reference velocity once actually flying it -- a drone that has
        // claimed leadership over the radio link but never reached Offboard (e.g. its own
        // MAVSDK connection never came up) would otherwise still broadcast a fully-formed,
        // plausible-looking ref_vx/ref_vy from the scripted path it isn't really flying,
        // with x/y/z frozen at 0. See `ready`'s comment in SwarmStatus.msg for what that
        // caused a follower to do.
        if (leader_id_ == my_id_ && offboard_ready_.load()) {
            auto ref = flight_pattern_ == "line"
                ? line_reference(now_s, line_phase_offset_)
                : loiter_reference(now_s, loiter_phase_offset_);
            msg.ref_vx = ref.vel.x;
            msg.ref_vy = ref.vel.y;
        }
        pub_->publish(msg);

        if (!offboard_ready_.load()) return;  // pas encore en vol Offboard : rien d'autre à faire ce tick

        // Fin automatique de l'excursion une fois sa durée écoulée.
        if (excursion_active_ && now_s > excursion_until_s_) {
            excursion_active_ = false;
            RCLCPP_INFO(get_logger(), "[%s] Excursion complete -- resuming formation.",
                        my_name_.c_str());
        }

        // --- Garde-fous de sécurité durs : altitude haute, altitude basse,
        // géo-barrière, retour au point de spawn -- chacun déclenche un
        // atterrissage immédiat (action_->land()) et coupe le mode Offboard. ---
        if (my_pos.z > MAX_ALT_HARD_M) {
            // Hard backstop on actual altitude, independent of and in
            // addition to clamp_to_geofence() on *targets*: catches a bug
            // in the control law itself steering the vehicle out of bounds,
            // not just a bug in where it's aiming.
            RCLCPP_ERROR(get_logger(), "[%s] Altitude %.1fm over hard limit -- landing.",
                         my_name_.c_str(), my_pos.z);
            offboard_ready_.store(false);
            if (action_) action_->land();
            return;
        }
        if (my_pos.z < MIN_ALT_HARD_M) {
            // See MIN_ALT_HARD_M's comment: catches a dangerous sag toward/through the
            // ground early, before Gazebo's physics reacts to it.
            RCLCPP_ERROR(get_logger(), "[%s] Altitude %.1fm under hard low-altitude limit -- landing.",
                         my_name_.c_str(), my_pos.z);
            offboard_ready_.store(false);
            if (action_) action_->land();
            return;
        }
        if (std::hypot(my_pos.x, my_pos.y) > MAX_GEOFENCE_HARD_M) {
            // Same idea, horizontally: clamp_to_geofence() only bounds
            // where a target is aimed, not how far a drone already
            // chasing a bad one (e.g. stale/isolated during a network
            // split) can travel before catching up to a corrected target.
            RCLCPP_ERROR(get_logger(), "[%s] %.1fm from origin, over hard geofence -- landing.",
                         my_name_.c_str(), std::hypot(my_pos.x, my_pos.y));
            offboard_ready_.store(false);
            if (action_) action_->land();
            return;
        }
        if (returning_home() &&
            std::hypot(my_pos.x - SPAWN_EAST_M[my_id_], my_pos.y - SPAWN_NORTH_M[my_id_]) < HOME_ARRIVAL_RADIUS_M) {
            RCLCPP_INFO(get_logger(), "[%s] Back at spawn position -- landing.", my_name_.c_str());
            offboard_ready_.store(false);
            if (action_) action_->land();
            return;
        }

        // Vitesse "désirée" selon mon rôle actuel (leader/follower/retour au spawn/excursion).
        Vec3 desired = compute_role_velocity(my_pos, my_vel);
        // Speed-cap formation-seeking only (not repulsion, computed below with its own
        // full-strength authority), ramping up from TAKEOFF_SPEED_CAP_M_S right after
        // becoming airborne (see TAKEOFF_SPEED_RAMP_S) and again right after an excursion
        // starts (see EXCURSION_RAMP_S) -- whichever ramp is still active wins, since both
        // exist to smooth out the same kind of sudden-large-position-gap step.
        double airborne_elapsed_s = now_s - airborne_since_s_.load();
        double speed_cap = airborne_elapsed_s < TAKEOFF_SPEED_RAMP_S
            ? TAKEOFF_SPEED_CAP_M_S +
                  (MAX_SPEED_M_S - TAKEOFF_SPEED_CAP_M_S) * (airborne_elapsed_s / TAKEOFF_SPEED_RAMP_S)
            : MAX_SPEED_M_S;
        if (excursion_active_) {
            double excursion_elapsed_s = now_s - excursion_started_s_;
            double excursion_speed_cap = excursion_elapsed_s < EXCURSION_RAMP_S
                ? TAKEOFF_SPEED_CAP_M_S +
                      (MAX_SPEED_M_S - TAKEOFF_SPEED_CAP_M_S) * (excursion_elapsed_s / EXCURSION_RAMP_S)
                : MAX_SPEED_M_S;
            speed_cap = std::min(speed_cap, excursion_speed_cap);
        }
        desired = clamp_speed(desired, speed_cap);
        // Ne considère que les pairs actuellement connectés pour la répulsion.
        std::vector<Vec3> peer_positions;
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected) peer_positions.push_back(tracker.pos);
        }
        Vec3 repulsion = separation_repulsion(my_pos, peer_positions);
        double repulsion_mag = std::sqrt(
            repulsion.x * repulsion.x + repulsion.y * repulsion.y + repulsion.z * repulsion.z);

        // Distance au pair connecté le plus proche, pour détecter/journaliser
        // une violation de la distance de sécurité (SAFE_DIST_M).
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& p : peer_positions) {
            min_dist = std::min(min_dist, distance_m(my_pos.x, my_pos.y, my_pos.z, p.x, p.y, p.z));
        }
        if (min_dist < SAFE_DIST_M) {
            RCLCPP_WARN(get_logger(), "[%s] SEPARATION: %.2fm from a peer, inside the %.1fm floor",
                        my_name_.c_str(), min_dist, SAFE_DIST_M);
            ++close_encounter_ticks_;
        } else {
            close_encounter_ticks_ = 0;
        }

        // A momentary close call and a *chronic* one need different
        // responses. Live testing traced ~half of all flights to a hard
        // geofence trigger (40m+ drift) with no single dramatic cause --
        // instead, a peer staying inside the 1m floor for minutes at a
        // time kept the priority blend below permanently diverting *some*
        // of the speed budget away from pure escape and into
        // formation-seeking, which (if that seeking is itself part of
        // what's causing the conflict) never lets the gap fully close --
        // just enough net bias, sustained long enough, to add up to a
        // large drift with no single moment that looks like a bug. Past
        // CHRONIC_TICKS_THRESHOLD, drop formation-seeking entirely and
        // spend the whole speed budget on escaping -- once clear,
        // close_encounter_ticks_ resets and normal blending resumes.
        // Proximité "chronique" (dépasse CHRONIC_TICKS_THRESHOLD ticks) : on
        // abandonne totalement la poursuite de formation pour consacrer
        // toute la vitesse disponible à l'évitement -- voir le long
        // commentaire ci-dessus pour le raisonnement détaillé.
        bool chronic = close_encounter_ticks_ > CHRONIC_TICKS_THRESHOLD;
        if (chronic && close_encounter_ticks_ % 20 == 0) {
            RCLCPP_ERROR(get_logger(),
                "[%s] CHRONIC proximity for %.1fs -- pure escape, formation-seeking suspended",
                my_name_.c_str(), close_encounter_ticks_ * CONTROL_PERIOD_S);
        }

        // Priority blend, not a plain vector sum: as repulsion strengthens,
        // it takes progressively more of the speed budget from the
        // formation-seeking component instead of the two being added
        // together and the final clamp_speed() diluting whichever
        // direction loses out. A live test showed the old additive
        // approach let a strong seek command water down avoidance right
        // when a peer was closest -- exactly the wrong moment for that.
        // Mélange par PRIORITÉ (pas une simple somme) : plus la répulsion est
        // forte, plus elle prend de poids sur la composante "formation" au
        // lieu de simplement s'additionner puis d'être écrêtée.
        double repulsion_weight = chronic ? 1.0 : std::clamp(repulsion_mag / MAX_SPEED_M_S, 0.0, 1.0);
        Vec3 blended{
            desired.x * (1.0 - repulsion_weight) + repulsion.x,
            desired.y * (1.0 - repulsion_weight) + repulsion.y,
            // Not blended by repulsion_weight: repulsion is horizontal-only (see
            // separation_repulsion()'s comment), so repulsion.z is always 0 -- blending it
            // here the same way as x/y would crush altitude correction toward 0 any time
            // horizontal repulsion is strong or chronic, exactly the case a live test
            // caught: desired.z=3.5 (climbing hard, correctly) but command.z=0.00 at
            // w=1.00, while real telemetry showed the drone in freefall at -7 m/s. Altitude
            // has nothing to do with horizontal avoidance, so it keeps full authority always.
            desired.z,
        };
        Vec3 command = clamp_speed(blended, MAX_SPEED_M_S);  // consigne finale, écrêtée à MAX_SPEED_M_S

        // Log de diagnostic détaillé si le drone s'éloigne anormalement de
        // l'origine (dérive) -- aide à comprendre après coup ce qui s'est passé.
        if (std::hypot(my_pos.x, my_pos.y) > DRIFT_TRACE_M) {
            RCLCPP_WARN(get_logger(),
                "[%s] DRIFT_TRACE role=%s pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f) min_dist=%.2f "
                "desired=(%.2f,%.2f) repulsion=(%.2f,%.2f) w=%.2f chronic=%d "
                "command=(%.2f,%.2f)",
                my_name_.c_str(), (leader_id_ == my_id_ ? "leader" : "follower"),
                my_pos.x, my_pos.y, my_pos.z, my_vel.x, my_vel.y, min_dist,
                desired.x, desired.y, repulsion.x, repulsion.y,
                repulsion_weight, chronic ? 1 : 0, command.x, command.y);
        }
        // Same idea as DRIFT_TRACE, kept permanently: traces the still-open
        // low-altitude physics anomaly, firing before MIN_ALT_HARD_M does.
        if (my_pos.z < 1.2) {
            RCLCPP_WARN(get_logger(),
                "[%s] ALT_TRACE role=%s z=%.2f vz=%.2f desired.z=%.2f repulsion.z=%.2f "
                "w=%.2f command.z=%.2f roll=%.1f pitch=%.1f",
                my_name_.c_str(), (leader_id_ == my_id_ ? "leader" : "follower"),
                my_pos.z, my_vel.z, desired.z, repulsion.z, repulsion_weight, command.z,
                my_roll_deg_.load(), my_pitch_deg_.load());
        }

        // Envoi de la consigne réelle à PX4 : conversion du repère interne
        // (est, nord, haut) vers le repère attendu par MAVSDK
        // NED = (nord, est, bas). En cas d'échec répété, voir handle_offboard_failure().
        // NED = (north, east, down); command is stored (east, north, up).
        auto res = offboard_->set_velocity_ned(
            {static_cast<float>(command.y), static_cast<float>(command.x),
             static_cast<float>(-command.z), 0.0f});
        if (res != mavsdk::Offboard::Result::Success) {
            handle_offboard_failure();
        }
    }

    // Past MISSION_OUTBOUND_S airborne: every drone heads home on its own,
    // regardless of role -- see MISSION_OUTBOUND_S's comment.
    // Vrai une fois la mission terminée (MISSION_OUTBOUND_S écoulées depuis
    // le décollage) : chaque drone rentre alors seul à son point de spawn,
    // sans coordination supplémentaire nécessaire.
    bool returning_home() const {
        return now().seconds() - airborne_since_s_.load() > MISSION_OUTBOUND_S;
    }

    // Leader: pursue the scripted loiter reference (feedforward + feedback).
    // Follower: hold a triangle-formation offset on the live leader.
    // Neither role known yet, or leader's position not seen yet: hover.
    // Past MISSION_OUTBOUND_S: role/formation ignored, straight home instead.
    // Calcule la vitesse "désirée" (avant répulsion/écrêtage) selon la
    // situation actuelle, par ordre de priorité :
    //   1) excursion en cours -> cap vers la cible d'excursion ;
    //   2) mission terminée -> cap vers le point de spawn (atterrissage) ;
    //   3) je suis le leader -> je suis ma trajectoire scriptée (loiter/line) ;
    //   4) pas de leader connu/prêt -> vol stationnaire (hover) ;
    //   5) sinon (follower normal) -> tenir ma place dans la formation
    //      triangle par rapport au leader suivi.
    Vec3 compute_role_velocity(const Vec3& my_pos, const Vec3& my_vel) {
        if (excursion_active_) {
            // Overrides role/formation entirely while active -- the point is to actually
            // leave the formation, not seek it while also being pulled elsewhere. Hard
            // safety backstops (altitude/geofence, checked earlier in control_step()) still
            // apply unconditionally.
            return seek_velocity(excursion_target_, my_pos, my_vel, Vec3{});
        }
        if (returning_home()) {
            Vec3 home{SPAWN_EAST_M[my_id_], SPAWN_NORTH_M[my_id_], LOITER_ALT_M};
            return seek_velocity(home, my_pos, my_vel, Vec3{});
        }
        if (leader_id_ == my_id_) {
            auto ref = flight_pattern_ == "line"
                ? line_reference(now().seconds(), line_phase_offset_)
                : loiter_reference(now().seconds(), loiter_phase_offset_);
            ref.pos = clamp_to_geofence(ref.pos);
            return seek_velocity(ref.pos, my_pos, my_vel, ref.vel);
        }
        if (leader_id_ == NO_LEADER || !trackers_[leader_id_].has_seen ||
            !trackers_[leader_id_].ready) {
            // Nothing to follow yet -- either no leader, or the elected leader hasn't
            // actually reached Offboard (its claim is over the radio link alone, not
            // contingent on flying). Chasing it before then means chasing x/y/z frozen at
            // 0 plus a ref_vel that no longer even gets sent (see the `ready` gate on
            // publish, above) -- hover in place instead.
            return Vec3{};
        }
        const auto& leader = trackers_[leader_id_];
        // Threshold for "trust this velocity sample enough to update
        // heading" -- was 0.2 (fine for the old constant 1.0 m/s
        // triangle-wave line, which was below 0.2 only for a literal
        // instant at each reversal). The sinusoidal line/loiter spend a
        // real fraction of each cycle below that near their slow points
        // (cos(phase) < 0.2 for ~13% of each quarter-cycle at peak speed
        // 1.0 m/s), so 0.2 was freezing the heading estimate for seconds
        // at a time near every turnaround -- a live test traced recurring
        // SEPARATION bursts throughout an otherwise-smooth flight, and an
        // eventual 40m+ excursion caught only by the hard geofence, back
        // to exactly this: a follower holding a stale heading while the
        // leader's true direction kept changing under it. Lower epsilon
        // trusts the velocity sample down near where noise alone would
        // matter, not before.
        // Use the leader's *analytical* reference velocity (ref_vel, what
        // its scripted path says it should be doing right now) rather than
        // its telemetry-derived vel: vel carries the leader's own tracking
        // lag and real MAVSDK noise, which only gets worse riding through
        // a second seek loop on top. ref_vel is exactly what the leader is
        // trying to fly, clean, and it's what actually flips sign at the
        // sinusoid's zero-crossings without the noise that made the old
        // 0.2 freeze-threshold necessary in the first place.
        double leader_speed = std::hypot(leader.ref_vel.x, leader.ref_vel.y);
        if (leader_speed > 0.03) leader_heading_rad_ = std::atan2(leader.ref_vel.x, leader.ref_vel.y);

        // Calcule ma position cible dans la formation triangle : décalage
        // (arrière, gauche/droite) selon mon côté assigné, tourné selon le
        // cap actuel du leader, puis ajouté à sa position -- borné par la géo-barrière.
        double lateral = formation_side_is_left() ? FORMATION_HALF_WIDTH_M : -FORMATION_HALF_WIDTH_M;
        Vec3 offset = rotate_formation_offset(-FORMATION_TRAIL_M, lateral, leader_heading_rad_, 0.0);
        Vec3 target = clamp_to_geofence(Vec3{leader.pos.x + offset.x, leader.pos.y + offset.y, leader.pos.z});
        // Feedforward on the leader's reference velocity, not just
        // proportional feedback on the position gap: a follower purely
        // chasing the leader's *position* can never close the gap once
        // behind, since both are capped at the same MAX_SPEED_M_S (a
        // losing tail chase). Matching the leader's intended velocity and
        // correcting the offset error on top of that is standard
        // formation-control practice, and is what makes catching up to /
        // holding station on a moving leader possible.
        return seek_velocity(target, my_pos, my_vel, leader.ref_vel);
    }

    // Deterministic slot assignment among the (up to) 2 non-leader drones:
    // the smaller surviving id takes the left slot.
    // Attribution déterministe du côté (gauche/droite) dans la formation
    // parmi les (au plus 2) drones non-leaders : le plus petit id restant
    // prend systématiquement la gauche.
    bool formation_side_is_left() {
        std::vector<int> followers;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (i != leader_id_) followers.push_back(i);
        }
        std::sort(followers.begin(), followers.end());
        return !followers.empty() && followers.front() == my_id_;
    }

    // Compte les échecs consécutifs d'envoi de consigne Offboard ; après 5
    // échecs de suite, atterrissage de sécurité (le lien Offboard est jugé rompu).
    void handle_offboard_failure() {
        if (++offboard_failure_count_ < 5) return;
        RCLCPP_ERROR(get_logger(), "[%s] Offboard setpoint rejected repeatedly -- landing.", my_name_.c_str());
        offboard_ready_.store(false);
        if (action_) action_->land();
        offboard_failure_count_ = 0;
    }

    // Callback de réception d'un message SwarmStatus d'un pair (souscription
    // ROS2). Simule la portée radio : un message venant d'un pair trop
    // éloigné (> MAX_RANGE_M) est purement ignoré, comme s'il n'était jamais
    // arrivé. Sinon, met à jour le tracker de ce pair (position, vitesse,
    // leader qu'il revendique, état "ready") et journalise la première
    // détection d'un signal.
    void on_peer_status(int peer, SwarmStatus::SharedPtr msg) {
        double range = distance_m(my_x_.load(), my_y_.load(), my_z_.load(), msg->x, msg->y, msg->z);
        if (range > MAX_RANGE_M) return; // out of simulated radio range: same as not receiving anything

        auto& tracker = trackers_[peer];
        tracker.last_seen = now();
        tracker.has_seen = true;
        tracker.last_claimed_leader = msg->claimed_leader;
        tracker.pos = Vec3{msg->x, msg->y, msg->z};
        tracker.vel = Vec3{msg->vx, msg->vy, msg->vz};
        tracker.ref_vel = Vec3{msg->ref_vx, msg->ref_vy, 0.0};
        tracker.ready = msg->ready;
        if (!tracker.connected) {
            tracker.connected = true;
            RCLCPP_INFO(get_logger(), "[%s] SUCCESS: signal detected from drone_%d",
                        my_name_.c_str(), peer);
        }
    }

    // Appelé toutes les 500ms (check_timer_) : marque comme déconnecté tout
    // pair dont on n'a plus reçu de message depuis plus de TIMEOUT_S
    // secondes (gestion du timeout/heartbeat), puis relance l'élection.
    void check_and_elect() {
        rclcpp::Time now_t = now();
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected && tracker.has_seen &&
                (now_t - tracker.last_seen).seconds() > TIMEOUT_S) {
                tracker.connected = false;
                RCLCPP_INFO(get_logger(), "[%s] Signal lost with drone_%d!", my_name_.c_str(), id);
            }
        }
        elect_leader();
    }

    // Sticky, majority-vote election: sticky leadership (a leader that
    // comes back doesn't reclaim the role -- it defers to whoever took
    // over) and never inventing a new belief while completely alone (that
    // first UDP version, tested end to end, is what caught and fixed an
    // infinite-oscillation bug -- see swarm_node.cpp's comments).
    //
    // Earlier version of this adopted the first differing `claimed_leader`
    // gossiped by any peer, in ID order. That broke when the old leader
    // reconnects: right after reconnecting, it and its peers are *each*
    // relaying whatever they believed a moment ago (the ex-leader's belief
    // was frozen at itself the whole time it was alone), and depending on
    // which stale message happens to arrive first, the group could
    // re-adopt the ex-leader by accident. A majority vote among everyone
    // currently alive (self included) fixes that: the 2 drones that
    // already agree on the real leader always outvote the 1 reconnecting
    // drone still relaying its stale belief, regardless of message timing.
    // ALGORITHME D'ÉLECTION DE LEADER : vote majoritaire parmi les drones
    // actuellement "en vie" (connectés), avec repli déterministe sur le plus
    // petit id en cas d'égalité ou d'absence de vote exploitable. Points clés :
    //   - "Sticky" : un ancien leader qui revient ne reprend pas le rôle
    //     automatiquement -- il faut que le vote le redésigne.
    //   - Chaque drone vote pour le leader qu'il croit actuellement valide
    //     (sa propre croyance + la dernière revendication connue de chaque
    //     pair vivant), ce qui permet à la majorité de 2 drones déjà
    //     d'accord de l'emporter sur 1 drone qui vient de se reconnecter et
    //     relaie encore une croyance périmée (voir le long commentaire
    //     ci-dessus pour l'historique du bug corrigé par cette approche).
    void elect_leader() {
        // Liste des drones actuellement joignables (moi + pairs connectés).
        std::vector<int> alive = {my_id_};
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected) alive.push_back(id);
        }
        if (alive.size() == 1) return;  // seul en vie : rien à élire, on garde la croyance actuelle

        // Comptage des votes : un vote = une croyance sur le leader actuel,
        // émise soit par moi-même, soit par un pair vivant (sa dernière
        // revendication reçue). Une croyance pointant vers un drone qui
        // n'est plus vivant est ignorée (croyance périmée).
        std::map<int, int> votes;
        // My own current belief is one vote, but only if it's still someone
        // alive -- otherwise it's a stale belief about to be corrected, and
        // should carry no more weight than a stale peer claim would.
        if (leader_id_ != NO_LEADER &&
            std::find(alive.begin(), alive.end(), leader_id_) != alive.end()) {
            votes[leader_id_]++;
        }
        for (int id : alive) {
            if (id == my_id_) continue;
            int claim = trackers_[id].last_claimed_leader;
            if (claim >= 0 && std::find(alive.begin(), alive.end(), claim) != alive.end()) {
                votes[claim]++;
            }
        }

        // Recherche du candidat ayant le plus de votes ; détecte aussi une
        // éventuelle égalité (tied_with > 0) pour ne jamais choisir
        // arbitrairement entre deux candidats à égalité.
        int winner = NO_LEADER, winner_votes = 0, tied_with = 0;
        for (auto& [candidate, count] : votes) {
            if (count > winner_votes) {
                winner = candidate;
                winner_votes = count;
                tied_with = 0;
            } else if (count == winner_votes) {
                ++tied_with;
            }
        }
        // No usable votes yet, or a tie nobody breaks (e.g. fresh boot,
        // everyone still claims NO_LEADER): fall back to smallest alive ID.
        // -> Aucun vote exploitable, ou égalité non tranchée (ex : démarrage à
        // froid où tout le monde revendique encore NO_LEADER) : repli
        // déterministe sur le plus petit id parmi les drones vivants.
        int new_leader = (winner == NO_LEADER || tied_with > 0)
            ? *std::min_element(alive.begin(), alive.end())
            : winner;

        int old_leader = leader_id_;
        if (new_leader == old_leader) return;  // pas de changement : rien à faire
        leader_id_ = new_leader;
        if (new_leader == my_id_) {
            // Je viens de devenir/rester leader : je dois recaler la phase de
            // ma trajectoire scriptée (cercle ou ligne) sur ma position
            // actuelle, pour ne pas "sauter" brutalement vers un point
            // arbitraire de la courbe dicté par l'horloge murale.
            // Re-anchor the loiter circle's phase to wherever I actually am
            // right now, so I pick up the path from here instead of
            // dashing toward whatever point wall-clock time happens to put
            // on the circle -- matters both on first becoming leader and on
            // taking over mid-flight from a previous leader.
            double angle = std::atan2(my_y_.load() - LOITER_CENTER_NORTH_M,
                                       my_x_.load() - LOITER_CENTER_EAST_M);
            loiter_phase_offset_ = angle - LOITER_OMEGA * now().seconds();

            // Same idea for the line pattern: pick the phase whose position
            // matches where I am along the east axis right now, continuing
            // in whichever direction I'm already moving (or +east if
            // roughly stationary), so taking over doesn't teleport the
            // reference point to the opposite end of the line. asin gives
            // the phase in [-pi/2, pi/2] where cos (velocity) is already
            // >= 0 -- moving forward; its mirror pi-asin(frac) is the
            // point on the sine curve at the same position moving the
            // other way.
            //
            // A live test caught this handoff landing on a bad instant: an
            // unplanned takeover (leader lost to a network split) happened
            // moments after a near-collision, so the taking-over drone's
            // instantaneous velocity was dominated by collision-avoidance
            // repulsion, not its actual direction of travel along the
            // line -- a noisy sign right when it mattered, sending it
            // reanchoring onto the wrong leg and off toward the far end at
            // speed. A deadband means only a velocity clearly indicating
            // "moving the other way" flips the branch; noise near zero
            // defaults to continuing outbound instead of being trusted.
            constexpr double DIRECTION_DEADBAND_M_S = 0.15;
            double frac = std::clamp((my_x_.load() - LOITER_CENTER_EAST_M) / LINE_HALF_LENGTH_M, -1.0, 1.0);
            double base_phase = std::asin(frac);
            double phase = (my_vx_.load() >= -DIRECTION_DEADBAND_M_S) ? base_phase : (M_PI - base_phase);
            line_phase_offset_ = phase - LINE_OMEGA * now().seconds();
        }
        if (old_leader == NO_LEADER) {
            RCLCPP_INFO(get_logger(), "[%s] New leader: drone_%d", my_name_.c_str(), new_leader);
        } else {
            RCLCPP_INFO(get_logger(), "[%s] New leader: drone_%d (was drone_%d)",
                        my_name_.c_str(), new_leader, old_leader);
        }
    }

    int my_id_ = -1;
    std::string my_name_;
    int leader_id_ = NO_LEADER;
    double leader_heading_rad_ = 0.0; // last-known good heading, held when leader is ~stationary
    double loiter_phase_offset_ = 0.0; // recomputed each time I become leader; see loiter_reference()
    double line_phase_offset_ = 0.0; // same idea for line_reference(), see elect_leader()
    std::string flight_pattern_ = "loiter"; // "loiter" or "line", set from the flight_pattern param
    std::map<int, DroneTracker> trackers_;
    rclcpp::Publisher<SwarmStatus>::SharedPtr pub_;
    std::vector<rclcpp::Subscription<SwarmStatus>::SharedPtr> subs_;
    rclcpp::TimerBase::SharedPtr control_timer_, check_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    std::atomic<bool> offboard_ready_{false};
    std::atomic<double> airborne_since_s_{0.0}; // now().seconds() when Offboard was entered
    std::atomic<int> last_logged_flight_mode_{-1}; // -1: nothing logged yet
    std::atomic<bool> last_logged_armed_{false};
    std::atomic<int> last_logged_landed_state_{-1};
    std::atomic<int> current_flight_mode_{-1}; // mavsdk::Telemetry::FlightMode, cast to int
    int offboard_failure_count_ = 0;
    int close_encounter_ticks_ = 0; // consecutive ticks with a peer inside SAFE_DIST_M
    // Excursion state -- see EXCURSION_DURATION_S's comment. Only touched from the ROS
    // executor thread (parameter callback + timers), same as leader_id_ etc., so plain
    // members are fine here -- no atomics needed.
    bool excursion_active_ = false;
    double excursion_started_s_ = 0.0;
    double excursion_until_s_ = 0.0;
    Vec3 excursion_target_;

    // Written by the MAVSDK telemetry thread, read by the ROS timer thread.
    std::atomic<double> my_x_{0}, my_y_{0}, my_z_{0};
    std::atomic<double> my_vx_{0}, my_vy_{0}, my_vz_{0};
    // Diagnostic-only (freefall investigation): real attitude, degrees.
    std::atomic<double> my_roll_deg_{0}, my_pitch_deg_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmNode>());
    } catch (const std::exception&) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
