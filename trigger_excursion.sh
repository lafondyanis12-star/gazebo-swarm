#!/usr/bin/env bash
# Triggers the live disconnect/reconnect demo on one drone: it flies away
# for 10s (out of simulated radio range), then comes back and reconnects
# on its own. Usage:
#   ./trigger_excursion.sh          (defaults to drone_1)
#   ./trigger_excursion.sh 0        (drone_0)
#   ./trigger_excursion.sh 2        (drone_2)
set -eo pipefail

ID="${1:-1}"

ROSENV="$HOME/micromamba/envs/ros_env"
export PATH="$ROSENV/bin:$PATH"
source "$ROSENV/setup.bash"
source "$HOME/Documents/my_project/gazebo_swarm/ros2_ws/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "Declenchement de l'excursion sur drone_$ID..."
ros2 param set "/drone_$ID" excursion true

echo ""
echo "Regarde le dashboard/terminal ROS2 : tu dois voir 'Excursion started' puis,"
echo "quelques secondes plus tard, 'Signal lost'. Si rien ne bouge apres 5-10s,"
echo "relance simplement ce script (la reconnexion n'est pas garantie a 100% du"
echo "premier coup, voir swarm_node.cpp)."
