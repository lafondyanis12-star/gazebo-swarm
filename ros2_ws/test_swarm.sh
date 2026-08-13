#!/usr/bin/env bash
# Rejoue automatiquement, via launch_testing, les 3 scenarios verifies a la
# main pendant la session : election initiale, deconnexion par portee, puis
# isolement total des 3 drones (voir test/test_election.py). Pas besoin de
# 3 terminaux ni de taper les ros2 param set soi-meme.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f "$HOME/micromamba/envs/ros_env/setup.bash" ]]; then
  export PATH="$HOME/micromamba/envs/ros_env/bin:$PATH"
  source "$HOME/micromamba/envs/ros_env/setup.bash"
else
  echo "Aucun environnement ROS2 Jazzy trouve (ni /opt/ros/jazzy, ni ~/micromamba/envs/ros_env)." >&2
  exit 1
fi

cd "$DIR"
colcon build --packages-select swarm_comm swarm_msgs
source install/setup.bash
colcon test --packages-select swarm_comm --event-handlers console_direct+
colcon test-result --verbose
