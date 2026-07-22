#!/usr/bin/env bash
# Ouvre un shell interactif dans le conteneur ROS2 Jazzy, avec ce workspace
# monté dedans -- les fichiers dans src/ sont partagés avec le Mac, donc on
# édite avec son éditeur habituel et on compile/lance dans le conteneur.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker run -it --rm \
  -v "$DIR:/root/ros2_ws" \
  -w /root/ros2_ws \
  ros:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && exec bash"
