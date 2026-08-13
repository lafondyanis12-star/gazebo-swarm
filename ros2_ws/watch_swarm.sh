#!/usr/bin/env bash
# Tableau de bord en direct des 3 drones (position + qui chacun croit etre
# le leader + connecte/LOST), a lancer dans un 2e terminal pendant que
# ./run_swarm.sh (ou demo_presentation.sh) tourne.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Prefer a native apt-installed ROS2 (standard on Linux); fall back to a
# RoboStack/micromamba env (used on macOS, where ROS2 has no native package).
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f "$HOME/micromamba/envs/ros_env/setup.bash" ]]; then
  export PATH="$HOME/micromamba/envs/ros_env/bin:$PATH"
  source "$HOME/micromamba/envs/ros_env/setup.bash"
else
  echo "Aucun environnement ROS2 Jazzy trouve (ni /opt/ros/jazzy, ni ~/micromamba/envs/ros_env)." >&2
  exit 1
fi
source "$DIR/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

python3 "$DIR/tools/show_swarm_dashboard.py"
