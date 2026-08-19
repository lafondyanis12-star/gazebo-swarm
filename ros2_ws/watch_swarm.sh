#!/usr/bin/env bash
# Tableau de bord en direct des 3 drones (position + qui chacun croit etre
# le leader + connecte/LOST), a lancer dans un 2e terminal pendant que
# ./run_swarm.sh (ou demo_presentation.sh) tourne.
set -euo pipefail

# Répertoire du script (racine du workspace ROS2), quel que soit l'endroit
# d'où le script est appelé
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
# Charge l'environnement du workspace ROS2 du projet (noeuds swarm_comm)
source "$DIR/install/setup.bash"
# Limite la découverte ROS2 à la machine locale (évite d'interférer avec
# d'autres machines sur le même réseau)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Lance le script Python qui affiche le tableau de bord en direct
python3 "$DIR/tools/show_swarm_dashboard.py"
