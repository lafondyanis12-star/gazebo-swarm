# Verifications de formation + de bascule de leader en vol contre un VRAI
# vol PX4 SITL -- contrairement a test_election.py, ceci pilote de vrais
# drones commandes par MAVSDK, pas seulement la couche communication/
# election sur des positions fictives.
#
# Precondition : run_swarm.sh doit deja tourner (3 instances PX4 SITL +
# Gazebo sur les ports 14540-14542) -- idealement via
# `HEADLESS=1 ./run_swarm.sh` (voir "What's left" du README : sinon le
# client GUI de Gazebo entre en concurrence avec les 3 instances physiques
# pour le CPU/GPU et degrade mesurablement la gigue de connectivite).
# A lancer a la main, une commande par terminal :
#
#   HEADLESS=1 ./run_swarm.sh                                   # terminal 1
#   ros2 launch_test src/swarm_comm/test/test_formation.py      # terminal 2
#
# `drone_0.terminate()` n'a jamais ete valide -- launch_ros.actions.Node n'a
# pas une telle methode (seulement une config sigterm_timeout/sigkill_timeout,
# pas une action) ; l'appeler levait toujours une AttributeError. Ce bug etait
# latent depuis la premiere ecriture de ce fichier : l'objet fixture que les
# methodes de test recoivent ici est l'*action* de lancement, pas un handle
# de processus en cours d'execution, et rien n'avait encore exerce ce chemin
# de code precis -- la bascule de leader n'avait jusque-la ete verifiee qu'a
# la main (kill -SIGINT sur un pid trouve via pgrep), ce que ce test fait
# maintenant de facon programmatique.
#
# Une version totalement autonome de ce fichier (lancant run_swarm.sh
# lui-meme, pour qu'une seule commande suffise sans precondition lancee
# separement) a aussi ete tentee pendant cette investigation puis
# abandonnee -- utile de noter pourquoi :
#   1. Un launch.actions.TimerAction retardant les 3 actions Node de
#      swarm_node le temps que PX4 demarre faisait *ressembler* l'
#      AttributeError ci-dessus a un probleme specifique au TimerAction
#      (ce n'etait pas le cas -- voir ci-dessus).
#   2. En retirant ce delai (le demarrage de swarm_node entre alors en
#      course avec run_swarm.sh, en comptant sur une attente de connexion
#      MAVSDK allongee a 90s -- voir connect_and_fly() dans swarm_node.cpp
#      -- pour survivre a un demarrage a froid), un vrai bug independant
#      est apparu : le decalage fixe de 15s de run_swarm.sh entre drone_0
#      et drone_1/2 n'etait pas suffisant pour que Gazebo soit pret sous
#      la charge concurrente *supplementaire* de 3 processus ROS2 qui
#      demarrent aussi, et les instances PX4 de drone_1/2 mouraient avec
#      "Timed out waiting for Gazebo world". Corrige en elargissant ce
#      decalage a 25s (un vrai correctif, conserve -- voir run_swarm.sh).
#   3. Une fois ce point corrige, 2 des 3 drones restaient quand meme
#      bloques silencieusement -- aucune progression loguee au-dela du
#      message d'election initial, ni passage en Offboard ni timeout avec
#      erreur. Cause racine non identifiee dans le temps disponible.
# Bilan : deux vrais bugs independants trouves et corriges dans les deux
# cas (conserves), mais l'automatisation complete en une commande n'est
# pas encore fiable -- livrer la version plus instable serait pire que la
# precondition manuelle en deux etapes.

import subprocess
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Decrit le lancement des 3 noeuds swarm_node (drone_0/1/2) pour le test.

    Suppose que run_swarm.sh (PX4 SITL + Gazebo) tourne deja separement --
    voir la precondition dans le commentaire de module. Retourne le tuple
    (LaunchDescription, drones) attendu par launch_testing.
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


def kill_node(drone_name):
    """Tue un noeud swarm_node par SIGINT (simule une deconnexion/panne pour tester la bascule de leader)."""
    # launch_ros.actions.Node n'a pas de .terminate() -- voir la docstring
    # de module. C'est le meme mecanisme utilise a la main tout au long du
    # developpement (kill -SIGINT sur le pid, trouve en filtrant sur
    # l'argument de nom de noeud ROS dans la ligne de commande).
    subprocess.run(
        ['pkill', '-SIGINT', '-f', f'__node:={drone_name}'],
        check=True,
    )


class TestSwarmFormation(unittest.TestCase):
    """
    Vol reel arm/decollage/Offboard contre un PX4 SITL deja en cours
    d'execution, donc les timeouts sont plus genereux que dans la version
    a positions fictives de test_election.py (une vraie montee plus la
    poignee de main Offboard prend environ 10-15s).
    """

    def test_1_all_drones_reach_offboard(self, proc_output, drone_0, drone_1, drone_2):
        """Verifie que les 3 drones decollent bien et passent en mode Offboard."""
        for name, proc in (('drone_0', drone_0), ('drone_1', drone_1), ('drone_2', drone_2)):
            proc_output.assertWaitFor('Airborne, in Offboard', process=proc, timeout=60)

    def test_2_leader_failover_in_flight(self, proc_output, drone_0, drone_1, drone_2):
        """Tue drone_0 (leader) en plein vol et verifie que drone_1/2 basculent vers un nouveau leader."""
        # Reproduit la bascule deja couverte au sol par test_election.py,
        # mais cette fois l'essaim est vraiment en vol : confirme que la
        # passation d'election et la loi de commande de formation (qui
        # re-ancre la phase de loiter sur la position reelle du nouveau
        # leader -- voir elect_leader() dans swarm_node.cpp) survivent
        # toutes deux au contact d'un vrai vol, pas seulement de positions
        # simulees.
        kill_node('drone_0')
        for name, proc in (('drone_1', drone_1), ('drone_2', drone_2)):
            proc_output.assertWaitFor('Signal lost with drone_0', process=proc, timeout=8)
            proc_output.assertWaitFor('New leader: drone_1', process=proc, timeout=8)
