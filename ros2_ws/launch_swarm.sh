#!/usr/bin/env bash
# Equivalent ROS 2 de launch_all.sh : build (si besoin) + lance les 3 drones
# d'un coup dans le conteneur, via le launch file swarm_launch.py.
#
# Nom de conteneur fixe (swarm_dev) pour pouvoir lui envoyer des commandes
# depuis un autre terminal pendant que ça tourne, ex:
#   docker exec swarm_dev bash -c "source /opt/ros/jazzy/setup.bash && ros2 param set /drone_2 x 30.0"
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker rm -f swarm_dev >/dev/null 2>&1 || true

docker run -it --rm \
  --name swarm_dev \
  -v "$DIR:/root/ros2_ws" \
  -w /root/ros2_ws \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && ros2 launch swarm_comm swarm_launch.py"
