#!/usr/bin/env bash
# One-command launch for the professor demo: cleans up any leftover
# processes from a previous run, then opens the 3 terminal windows in
# order (Gazebo/PX4, ROS2 comm, live dashboard), positioned on the
# extended/Sidecar display. Prints the command to trigger the live
# disconnect/reconnect demo once the formation looks stable.
set -euo pipefail

DIR="$HOME/Documents/my_project/gazebo_swarm"

echo "=== Nettoyage d'une eventuelle simulation precedente ==="
pkill -f "swarm_comm/swarm_node" 2>/dev/null || true
pkill -f "ros2 launch swarm_comm" 2>/dev/null || true
pkill -f "swarm_dashboard.py" 2>/dev/null || true
sleep 1
pkill -x px4 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
sleep 2

echo "=== 1/3 : Gazebo + PX4 ==="
osascript <<'EOF'
tell application "Terminal"
  activate
  do script "cd ~/Documents/my_project/gazebo_swarm && ./run_swarm.sh"
end tell
delay 0.5
tell application "System Events"
  tell process "Terminal"
    set position of window 1 to {1512, 40}
    set size of window 1 to {868, 650}
  end tell
end tell
EOF

echo "Attente ~30s (demarrage PX4/Gazebo)..."
sleep 30

echo "=== 2/3 : ROS2 (communication + vol) ==="
osascript <<'EOF'
tell application "Terminal"
  activate
  do script "cd ~/Documents/my_project/gazebo_swarm/ros2_ws && bash -lc 'ROSENV=~/micromamba/envs/ros_env && export PATH=\"$ROSENV/bin:$PATH\" && source \"$ROSENV/setup.bash\" && source install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && ros2 launch swarm_comm swarm_launch.py'"
end tell
delay 0.5
tell application "System Events"
  tell process "Terminal"
    set position of window 1 to {2380, 40}
    set size of window 1 to {420, 314}
  end tell
end tell
EOF

echo "Attente ~10s (connexion des noeuds a PX4)..."
sleep 10

echo "=== 3/3 : Dashboard live ==="
osascript <<'EOF'
tell application "Terminal"
  activate
  do script "cd ~/Documents/my_project/gazebo_swarm/ros2_ws && bash -lc 'ROSENV=~/micromamba/envs/ros_env && export PATH=\"$ROSENV/bin:$PATH\" && source \"$ROSENV/setup.bash\" && source install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && python3 tools/swarm_dashboard.py'"
end tell
delay 0.5
tell application "System Events"
  tell process "Terminal"
    set position of window 1 to {1512, 716}
    set size of window 1 to {1288, 296}
  end tell
end tell
EOF

echo ""
echo "=== Tout est lance ==="
echo ""
echo "Laisser la formation se stabiliser a l'ecran (~15-20s : triangle forme, les 3 drones CONNECTED)."
echo ""
echo "Pour declencher la demo en direct (drone_1 s'eloigne 10s hors de portee radio puis revient) :"
echo ""
echo "  cd $DIR/ros2_ws && bash -lc 'ROSENV=~/micromamba/envs/ros_env && export PATH=\"\$ROSENV/bin:\$PATH\" && source \"\$ROSENV/setup.bash\" && source install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && ros2 param set /drone_1 excursion true'"
echo ""
echo "Pour tout arreter en fin de demo :"
echo ""
echo "  pkill -f swarm_comm/swarm_node; pkill -f 'ros2 launch swarm_comm'; pkill -f swarm_dashboard.py; pkill -x px4; pkill -f 'gz sim'"
echo ""
