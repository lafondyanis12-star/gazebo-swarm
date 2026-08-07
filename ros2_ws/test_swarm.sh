#!/usr/bin/env bash
# Rejoue automatiquement, via launch_testing, les 3 scenarios verifies a la
# main pendant la session : election initiale, deconnexion par portee, puis
# isolement total des 3 drones (voir test/test_election.py). Pas besoin de
# 3 terminaux ni de taper les ros2 param set soi-meme.
#
# build/install/log restent partages sur disque avec les builds natifs
# macOS (RoboStack, utilises pour les vols reels) -- --build-base/
# --install-base pointent vers des repertoires *_docker dedies pour que
# le CMakeCache du conteneur (chemin /root/ros2_ws) n'entre jamais en
# collision avec celui du build natif (chemin /Users/...).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker run --rm \
  -v "$DIR:/root/ros2_ws" \
  -w /root/ros2_ws \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build --build-base build_docker --install-base install_docker && source install_docker/setup.bash && colcon test --packages-select swarm_comm --build-base build_docker --event-handlers console_direct+ && colcon test-result --test-result-base build_docker --verbose"
