# Test d'integration : verifie l'algorithme d'election de leader de
# l'essaim (swarm_node.cpp) sur des positions FICTIVES (parametres x/y/z
# imposes via `ros2 param set`, pas de vol PX4 reel -- voir test_formation.py
# pour la version avec vol reel). Automatise les 3 scenarios qui etaient
# verifies a la main auparavant : lancer l'essaim, lire `docker logs`,
# executer `ros2 param set` depuis un 2e terminal, relire les logs. Memes
# verifications, memes lignes de log, juste scripte.
#
# ros2 launch_test src/swarm_comm/test/test_election.py  (ou via colcon test)

import os
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest

# Le launch_testing de colcon/ament (via ctest) retire DYLD_LIBRARY_PATH
# avant d'exec'er ce processus de test -- le meme mecanisme que celui
# documente dans swarm_comm/CMakeLists.txt pour swarm_node lui-meme, sauf
# que ce correctif-la (RPATH integre au binaire) ne couvre que le binaire
# swarm_node, pas les sous-processus `ros2` CLI lances ici. Sans cette
# variable, `ros2 topic echo`/`param set` ne peuvent pas charger (dlopen)
# la bibliotheque .dylib de typesupport de swarm_msgs et echouent
# immediatement avec "invalid allocator" -- ce n'est pas une course de
# decouverte DDS, donc reessayer seul ne resout pas le probleme.
_INSTALL_LIB_DIRS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install', pkg, 'lib')
    for pkg in ('swarm_msgs', 'swarm_comm')
]
_ROS2_ENV = {
    **os.environ,
    'DYLD_LIBRARY_PATH': ':'.join(
        [*_INSTALL_LIB_DIRS, os.environ.get('DYLD_LIBRARY_PATH', '')]),
}


@pytest.mark.launch_test
def generate_test_description():
    """Decrit le lancement des 3 noeuds swarm_node (drone_0/1/2) pour le test.

    Retourne le tuple (LaunchDescription, drones) attendu par launch_testing :
    la description sert a demarrer les noeuds, le dict `drones` est injecte
    tel quel dans les methodes de test (fixtures drone_0/drone_1/drone_2).
    """
    drones = {
        f'drone_{i}': launch_ros.actions.Node(
            package='swarm_comm',
            executable='swarm_node',
            name=f'drone_{i}',
            parameters=[{'id': i}],
            output='screen',
        )
        for i in range(3)
    }
    return launch.LaunchDescription([
        *drones.values(),
        launch_testing.actions.ReadyToTest(),
    ]), drones


def set_x(drone_name, value):
    """Impose la coordonnee x (metres) d'un drone via `ros2 param set` -- simule son deplacement."""
    subprocess.run(
        ['ros2', 'param', 'set', f'/{drone_name}', 'x', str(value)],
        check=True, capture_output=True, env=_ROS2_ENV,
    )


def get_claimed_leader(drone_name):
    """Lit le leader que ce drone croit actuellement (champ claimed_leader de /swarm_status)."""
    # Reessaie avec un court delai de grace -- `ros2 topic echo --once` peut
    # echouer si la decouverte DDS n'a pas encore trouve le publisher,
    # comportement instable sous forte charge systeme (voir README "What's left").
    last_err = None
    for _ in range(5):
        try:
            out = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', f'/{drone_name}/swarm_status'],
                check=True, capture_output=True, text=True, timeout=10, env=_ROS2_ENV,
            ).stdout
            for line in out.splitlines():
                if line.startswith('claimed_leader:'):
                    return int(line.split(':')[1].strip())
            raise RuntimeError(f'no claimed_leader in output from {drone_name}: {out!r}')
        except subprocess.CalledProcessError as err:
            last_err = err
            time.sleep(1)
    raise last_err


class TestSwarmElection(unittest.TestCase):
    """
    Scenarios d'election, executes dans l'ordre de definition sur les memes
    3 noeuds (les 3 drones ne sont pas relances entre les tests).

    test_1_, test_2_, ... sont tries par ordre alphabetique, ce qui reproduit
    le deroule de la session manuelle : election normale d'abord, puis
    deconnexion par portee, puis isolation totale.
    """

    def test_1_initial_election(self, proc_output, drone_0, drone_1, drone_2):
        """Verifie qu'au demarrage, les 3 drones convergent vers UN SEUL leader."""
        # Lequel des 3 gagne est une course de timing (chaque noeud choisit
        # l'ID minimum de *son propre* ensemble de pairs vivants au moment
        # ou son minuteur d'election se declenche, et avec une decouverte
        # quasi simultanee il n'y a aucune garantie d'ordre strict -- voir
        # les commentaires d'elect_leader() dans swarm_node.cpp). Ce qui
        # doit tenir, c'est le consensus, pas un gagnant precis : les 3
        # doivent s'accorder sur le meme leader.
        for name, proc in (('drone_0', drone_0), ('drone_1', drone_1), ('drone_2', drone_2)):
            proc_output.assertWaitFor('New leader: drone_', process=proc, timeout=10)
        leaders = {name: get_claimed_leader(name) for name in ('drone_0', 'drone_1', 'drone_2')}
        self.assertEqual(len(set(leaders.values())), 1, f'no consensus: {leaders}')

    def test_2_range_disconnect_and_reconnect(self, proc_output, drone_0, drone_1, drone_2):
        """Verifie la detection de perte de signal (hors portee) puis la reconnexion."""
        set_x('drone_2', 30.0)  # 30m > MAX_RANGE_M (10m) -> hors de portee
        proc_output.assertWaitFor('Signal lost with drone_2!', process=drone_0, timeout=6)
        proc_output.assertWaitFor('Signal lost with drone_2!', process=drone_1, timeout=6)

        set_x('drone_2', 0.0)  # retour en portee
        signal_2 = 'SUCCESS: signal detected from drone_2'
        proc_output.assertWaitFor(signal_2, process=drone_0, timeout=6)
        proc_output.assertWaitFor(signal_2, process=drone_1, timeout=6)

    def test_3_total_isolation_freezes_leader_belief(self, proc_output, drone_0, drone_1, drone_2):
        """Isole completement les 3 drones et verifie qu'aucun ne s'auto-elit leader (anti split-brain)."""
        # Re-verifie le leader actuel plutot que de se fier au resultat de
        # test_1 : la deconnexion/reconnexion de drone_2 dans test_2 aurait
        # en principe pu changer le leader entre-temps (ex: si drone_2
        # lui-meme etait leader) -- on veut l'etat courant, pas un etat
        # memorise.
        current_leader = f'drone_{get_claimed_leader("drone_0")}'

        # Disperse les 3 drones pour que chaque distance deux-a-deux depasse MAX_RANGE_M.
        set_x('drone_0', 0.0)
        set_x('drone_1', 100.0)
        set_x('drone_2', 200.0)

        for proc in (drone_0, drone_1, drone_2):
            proc_output.assertWaitFor('Signal lost with', process=proc, timeout=6)

        # Laisse au minuteur d'election (500ms) plusieurs cycles pour bien
        # prouver le point, puis verifie que les 2 drones non-leaders ne se
        # sont PAS auto-declares leader alors qu'ils sont completement
        # isoles -- c'est le comportement "split-brain" que le garde-fou
        # `if (alive.size() == 1) return;` est cense empecher. (Quel drone
        # mene est une course de timing tranchee dans test_1 -- voir ses
        # commentaires -- donc les suiveurs a verifier sont deduits de ce
        # resultat, pas codes en dur.)
        time.sleep(2)
        procs = {'drone_0': drone_0, 'drone_1': drone_1, 'drone_2': drone_2}
        followers = [(name, proc) for name, proc in procs.items() if name != current_leader]
        for name, proc in followers:
            with self.assertRaises(AssertionError, msg=f'{name} elected itself while isolated'):
                proc_output.assertWaitFor(f'New leader: {name}', process=proc, timeout=1)

        # Rapproche les 3 drones et verifie qu'ils reconvergent proprement.
        set_x('drone_1', 0.0)
        set_x('drone_2', 0.0)
        signal_from = 'SUCCESS: signal detected from drone_{}'
        proc_output.assertWaitFor(signal_from.format(2), process=drone_0, timeout=6)
        proc_output.assertWaitFor(signal_from.format(1), process=drone_0, timeout=6)
