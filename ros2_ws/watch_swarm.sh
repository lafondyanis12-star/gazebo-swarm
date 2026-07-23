#!/usr/bin/env bash
# Tableau de bord en direct des 3 drones (position + qui chacun croit etre
# le leader + connecte/LOST), a lancer dans un 2e terminal pendant que
# ./launch_swarm.sh tourne dans le premier (le conteneur swarm_dev doit
# deja etre en cours).
set -euo pipefail

docker exec -it swarm_dev bash -c \
  "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && python3 /root/ros2_ws/tools/swarm_dashboard.py"
