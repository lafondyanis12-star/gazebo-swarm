# 🐝 Gazebo Swarm — Drone Communication & Leader Election

Drone swarm communication and leader election, built on Gazebo Sim + PX4 SITL (the simulator recommended for this internship, after an initial Webots prototype — see the report). Three simulated drones detect each other, notice when one drops off (either by process failure or by going out of radio range), and elect a new leader automatically.

📄 Full write-up: [`RAPPORT_EN.md`](./RAPPORT_EN.md) — the research process, the problems hit along the way and how they got fixed.

## 🗺️ Where things stand

1. ✅ **Protocol research.** Compared raw Gazebo Transport, ROS 2 and PX4 SITL + MAVLink/MAVSDK as ways to get the drones talking. Went with PX4 SITL + MAVSDK since PX4-Autopilot was already on the machine and it's closer to how a real fleet would be set up.
2. ✅ **Environment setup.** Got PX4 SITL building for the modern `gz-sim` target — had to work around a broken Homebrew Python and an OpenCV version mismatch on an unrelated plugin.
3. ✅ **Communication + leader election.** `swarm_node.cpp`: each drone reads its own GPS position through MAVSDK (C++) and broadcasts it to the other two over a small UDP mesh. First design tried to have every drone listen to every PX4 instance directly and that just doesn't work (port conflicts) — see the report for why. Timeout-based disconnection detection and leader election (lowest surviving ID) both tested and working.
4. ✅ **Persistent Gazebo world.** Drones used to be spawned dynamically and vanished on a simulation reset. Now they live in the world file itself.
5. ✅ **Range-based disconnection + split-brain fix.** Added a simulated 6m radio range so you can disconnect a drone by moving it instead of killing its process. That surfaced a real bug — an isolated drone would elect itself leader and never give it back — which is now fixed.
6. ✅ **Repo set up.** This one.

## 🔜 Still to do

- ⬜ Port the V-formation flight logic from the Webots reference so the elected leader actually flies differently from the followers, instead of just holding a logical role.
- ⬜ Think through what happens if two groups of drones never reconnect (each keeps its own leader forever — expected, but worth a dedicated test).

## 📁 Files

- `launch_all.sh` — builds if needed and opens all 4 terminal windows automatically (the fastest way to run everything).
- `run_swarm.sh` — starts the 3 PX4 SITL instances in the persistent world.
- `swarm_node.cpp` / `CMakeLists.txt` — communication, disconnection detection, leader election (`./build/swarm_node --id 0|1|2`).
- `worlds/swarm_persistent.sdf` — the Gazebo world with the 3 drones already placed in it.

## 🛠️ Requirements

- PX4-Autopilot built for `gz-sim` (`make px4_sitl`).
- MAVSDK C++ (`brew install mavsdk` on macOS).
- CMake ≥ 3.16 and a C++17 compiler.
