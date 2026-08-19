#!/usr/bin/env bash
# Déclenche la démo de déconnexion/reconnexion en direct sur un drone : il
# s'éloigne pendant 10s (hors de la portée radio simulée), puis revient et
# se reconnecte tout seul. Usage :
#   ./trigger_excursion.sh          (drone_1 par défaut)
#   ./trigger_excursion.sh 0        (drone_0)
#   ./trigger_excursion.sh 2        (drone_2)
set -eo pipefail

# ID du drone ciblé, passé en 1er argument (drone_1 si absent)
ID="${1:-1}"

# Préfère un ROS2 installé nativement via apt (standard sous Linux) ;
# sinon se rabat sur un environnement RoboStack/micromamba (utilisé sur
# macOS, où ROS2 n'a pas de paquet natif).
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f "$HOME/micromamba/envs/ros_env/setup.bash" ]]; then
  export PATH="$HOME/micromamba/envs/ros_env/bin:$PATH"
  source "$HOME/micromamba/envs/ros_env/setup.bash"
else
  echo "Aucun environnement ROS2 Jazzy trouve (ni /opt/ros/jazzy, ni ~/micromamba/envs/ros_env)." >&2
  exit 1
fi
# Charge l'espace de travail ROS2 du projet (noeuds swarm_comm)
source "$HOME/Documents/my_project/gazebo_swarm/ros2_ws/install/setup.bash"
# Limite la découverte ROS2 à la machine locale (évite d'interférer avec
# d'autres machines sur le même réseau)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "Declenchement de l'excursion sur drone_$ID..."
# Active le paramètre "excursion" du noeud du drone visé : c'est ce
# paramètre que swarm_node.cpp surveille pour déclencher l'éloignement
ros2 param set "/drone_$ID" excursion true

echo ""
echo "Regarde le dashboard/terminal ROS2 : tu dois voir 'Excursion started' puis,"
echo "quelques secondes plus tard, 'Signal lost'. Si rien ne bouge apres 5-10s,"
echo "relance simplement ce script (la reconnexion n'est pas garantie a 100% du"
echo "premier coup, voir swarm_node.cpp)."
