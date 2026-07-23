#!/usr/bin/env bash
# Rejoue automatiquement, via launch_testing, les 3 scenarios verifies a la
# main pendant la session : election initiale, deconnexion par portee, puis
# isolement total des 3 drones (voir test/test_election.py). Pas besoin de
# 3 terminaux ni de taper les ros2 param set soi-meme.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker run -it --rm \
  -v "$DIR:/root/ros2_ws" \
  -w /root/ros2_ws \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && colcon test --packages-select swarm_comm --event-handlers console_direct+ && colcon test-result --verbose"
