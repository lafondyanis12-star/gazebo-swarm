#!/usr/bin/env bash
# Ouvre tout d'un coup : une fenêtre pour Gazebo/PX4 (run_swarm.sh), puis
# 3 fenêtres pour les noeuds de communication, une fois PX4 pret.
set -euo pipefail

DIR="$HOME/Documents/my_project/gazebo_swarm"
BIN="$DIR/build/swarm_node"

if [[ ! -x "$BIN" ]]; then
  echo "swarm_node n'est pas compile, compilation..."
  cmake -S "$DIR" -B "$DIR/build"
  cmake --build "$DIR/build"
fi

osascript <<OSA
tell application "Terminal"
  activate
  do script "$DIR/run_swarm.sh"
end tell
OSA

echo "Gazebo + PX4 lances. Attente d'environ 25s avant de demarrer la communication..."
sleep 25

osascript <<OSA
tell application "Terminal"
  do script "cd $DIR && $BIN --id 0"
  do script "cd $DIR && $BIN --id 1"
  do script "cd $DIR && $BIN --id 2"
end tell
OSA

echo "Tout est lance : 1 fenetre Gazebo/PX4 + 3 fenetres swarm_node."
