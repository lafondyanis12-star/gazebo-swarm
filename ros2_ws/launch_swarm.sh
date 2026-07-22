#!/usr/bin/env bash
# Equivalent ROS 2 de launch_all.sh : build (si besoin) + lance les 3 drones
# d'un coup dans le conteneur, via le launch file swarm_launch.py.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker run -it --rm \
  -v "$DIR:/root/ros2_ws" \
  -w /root/ros2_ws \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && ros2 launch swarm_comm swarm_launch.py"
