#!/usr/bin/env bash
# Triggers the live disconnect/reconnect demo on one drone: it flies away
# for 10s (out of simulated radio range), then comes back and reconnects
# on its own. Usage:
#   ./trigger_excursion.sh          (defaults to drone_1)
#   ./trigger_excursion.sh 0        (drone_0)
#   ./trigger_excursion.sh 2        (drone_2)
set -eo pipefail

ID="${1:-1}"

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
source "$HOME/Documents/my_project/gazebo_swarm/ros2_ws/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "Declenchement de l'excursion sur drone_$ID..."
ros2 param set "/drone_$ID" excursion true

echo ""
echo "Regarde le dashboard/terminal ROS2 : tu dois voir 'Excursion started' puis,"
echo "quelques secondes plus tard, 'Signal lost'. Si rien ne bouge apres 5-10s,"
echo "relance simplement ce script (la reconnexion n'est pas garantie a 100% du"
echo "premier coup, voir swarm_node.cpp)."
