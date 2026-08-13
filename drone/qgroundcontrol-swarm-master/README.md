# QGroundControl — SwarmDashboard v2
**Multi-UAV Swarm Management Extension for QGroundControl**

Internship project — USTH Hanoi 2026

Student: Yacine Abdi (ITS2, EPISEN/UPEC)

Supervisor: Prof. Pham Xuan Tung

GitHub: github.com/OEOEOEDZ/qgroundcontrol-swarm

---

## What This Project Does

This project extends QGroundControl (QGC) with a SwarmDashboard — a custom QML overlay enabling real-time supervision and control of a drone swarm. Validated with PX4 SITL and Gazebo simulation.

Features:
- Per-drone selection with ALL / NONE / LEADER shortcuts
- Real-time telemetry: LAT, LON, ALT, SPD, HDG, BAT% per drone
- Leader election algorithm with automatic failover every 2 seconds
- CAM button per drone to switch QGC video feed
- Configurable takeoff altitude (1-50m) and max speed (1-20 m/s)
- In-flight altitude change
- All commands apply only to selected drones
- Pure QML — no C++ modifications required

---

## Repository Contents

| File | Purpose |
|------|---------|
| SwarmDashboard.qml | Main dashboard — all UI and logic |
| CMakeLists.txt | Registers SwarmDashboard.qml as QML resource |
| FlyViewWidgetLayer.qml | Instantiates SwarmDashboard in QGC FlyView |
| install.sh | Automatic installer |
| swarm_camera_stream.py | Python synthetic camera streamer (fallback) |

---

## Exact Changes Made to QGC Source

Only 2 files were modified. Everything else is new.

### Change 1 — src/FlyView/CMakeLists.txt

Add one line after MultiVehicleList.qml:

        MultiVehicleList.qml
+       SwarmDashboard.qml
        ObstacleDistanceOverlay.qml

### Change 2 — src/FlyView/FlyViewWidgetLayer.qml

Add this block before the last closing brace of the file:

+   SwarmDashboard {
+       id:                 swarmDashboard
+       anchors.left:       toolStrip.right
+       anchors.leftMargin: _toolsMargin
+       anchors.top:        toolStrip.bottom
+       anchors.topMargin:  _toolsMargin
+       z:                  QGroundControl.zOrderWidgets
+       visible:            QGroundControl.multiVehicleManager.vehicles.count > 0
+   }

---

## System Requirements

| Component | Version | Notes |
|-----------|---------|-------|
| OS | Ubuntu 22.04 LTS | Or WSL2 on Windows 11 |
| Qt | 6.10.3 | Via aqtinstall — do NOT use system Qt |
| cmake | 3.28+ | May need symlink fix |
| ninja | any | Build system |
| PX4-Autopilot | main 2026 | For SITL |
| Gazebo Classic | 11.10.2 | For video streaming |
| Gazebo Harmonic | 8.12.0 | For 3-drone simulation |
| QGroundControl | Qt6 daily | Cloned from source |

---

## STEP 0 — Find Your WSL2 Gateway IP

This IP changes every time you reboot. Always check before running.

    cat /etc/resolv.conf | grep nameserver | awk '{print $2}'

Replace YOUR_GATEWAY_IP in all mavlink commands with this value (example: 172.27.80.1)

---

## STEP 1 — Install System Dependencies

    sudo apt update && sudo apt upgrade -y

    sudo apt install -y git cmake ninja-build build-essential python3-pip \
        libsecret-1-dev libxkbcommon-dev libgl1-mesa-dev libglu1-mesa-dev \
        x11-apps gazebo libgazebo-dev gz-harmonic

    sudo chmod 700 /run/user/1000

    echo "net.core.rmem_max=134217728" | sudo tee -a /etc/sysctl.conf
    echo "net.core.rmem_default=134217728" | sudo tee -a /etc/sysctl.conf
    echo "net.core.wmem_max=134217728" | sudo tee -a /etc/sysctl.conf
    sudo sysctl -p

---

## STEP 2 — Install Qt 6.10.3

    pip3 install aqtinstall

    ~/.local/bin/aqt install-qt linux desktop 6.10.3 linux_gcc_64 \
      -m qt5compat qtcharts qtimageformats qtlocation qtpositioning \
      qtsensors qtserialport qtmultimedia qtshadertools qtwebsockets \
      qtwebchannel qtwebengine qtspeech qtscxml qtremoteobjects \
      qtnetworkauth qtserialbus qtconnectivity qtvirtualkeyboard \
      qtdatavis3d qtgraphs qtquick3d qtquick3dphysics qthttpserver \
      qtlottie -O ~/Qt

    ~/Qt/6.10.3/gcc_64/bin/qmake --version
    # Expected: QMake version 3.1 / Using Qt version 6.10.3

---

## STEP 3 — Clone and Build QGroundControl

CRITICAL: always use --recursive

    git clone https://github.com/mavlink/qgroundcontrol.git --recursive
    cd qgroundcontrol

Fix cmake if needed:

    sudo ln -sf /usr/local/bin/cmake /usr/bin/cmake

    mkdir build && cd build
    cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=~/Qt/6.10.3/gcc_64
    ninja -j4

If ninja fails with SDL3 PCH error:

    touch ~/qgroundcontrol/build/_deps/sdl3-build/CMakeFiles/SDL3-static.dir/cmake_pch.hxx.gch
    ninja -j4

---

## STEP 4 — Install SwarmDashboard

    cd ~
    git clone https://github.com/OEOEOEDZ/qgroundcontrol-swarm.git
    bash qgroundcontrol-swarm/install.sh ~/qgroundcontrol
    cd ~/qgroundcontrol/build && ninja -j4

---

## STEP 5 — Install PX4-Autopilot

    cd ~
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd PX4-Autopilot
    bash ./Tools/setup/ubuntu.sh

Reopen terminal after setup. Validate environment:

    HEADLESS=1 make px4_sitl gz_x500

When pxh> appears type quit. If build fails: rm -rf build then retry.

---

## PART A — 3-Drone Simulation with Gazebo Harmonic (no video)

Open 5 terminals. Run in order. Wait for each to be ready.

### Terminal 1 — Gazebo GUI

    sudo chmod 700 /run/user/1000
    export DISPLAY=:0
    export LIBGL_ALWAYS_SOFTWARE=1
    gz sim -g

Wait for the Gazebo window to open.

### Terminal 2 — Drone 1

    export DISPLAY=:0
    export LIBGL_ALWAYS_SOFTWARE=1
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
    cd ~/PX4-Autopilot
    PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_GZ_MODEL=x500_mono_cam HEADLESS=0 ./build/px4_sitl_default/bin/px4 -i 0

When pxh> appears:

    mavlink start -x -u 14560 -r 4000000 -t YOUR_GATEWAY_IP -o 14550

### Terminal 3 — Drone 2

    export DISPLAY=:0
    export LIBGL_ALWAYS_SOFTWARE=1
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
    cd ~/PX4-Autopilot
    PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="2,0,0,0,0,0" PX4_GZ_MODEL=x500_mono_cam HEADLESS=0 ./build/px4_sitl_default/bin/px4 -i 1

When pxh> appears:

    mavlink start -x -u 14561 -r 4000000 -t YOUR_GATEWAY_IP -o 14551

### Terminal 4 — Drone 3

    export DISPLAY=:0
    export LIBGL_ALWAYS_SOFTWARE=1
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
    cd ~/PX4-Autopilot
    PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="4,0,0,0,0,0" PX4_GZ_MODEL=x500_mono_cam HEADLESS=0 ./build/px4_sitl_default/bin/px4 -i 2

When pxh> appears:

    mavlink start -x -u 14562 -r 4000000 -t YOUR_GATEWAY_IP -o 14552

### Terminal 5 — QGroundControl

    cd ~/qgroundcontrol/build
    LIBGL_ALWAYS_SOFTWARE=1 QSG_RENDER_LOOP=basic ./Debug/QGroundControl

Expected: SwarmDashboard shows Drones: 3

---

## PART B — Video Streaming with Gazebo Classic (1 drone)

IMPORTANT: Disconnect any VPN before starting. VPN blocks the video stream.

### Terminal 1 — PX4 + Gazebo Classic

    export DISPLAY=:0
    export LIBGL_ALWAYS_SOFTWARE=1
    export GAZEBO_PLUGIN_PATH=~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
    cd ~/PX4-Autopilot
    make px4_sitl gazebo-classic_typhoon_h480

Wait for Gazebo Classic to open with the Typhoon H480 drone.
Click the green VIDEO: ON button in the top left of Gazebo.

When pxh> appears:

    mavlink start -x -u 14560 -r 4000000 -t YOUR_GATEWAY_IP -o 14550

### Terminal 2 — QGroundControl (launch AFTER clicking Video ON)

    cd ~/qgroundcontrol/build
    LIBGL_ALWAYS_SOFTWARE=1 QSG_RENDER_LOOP=basic ./Debug/QGroundControl

In QGC: Settings > Video > Source: UDP h.264 Video Stream > UDP URL: 0.0.0.0:5600

The camera feed from Gazebo Classic appears in QGC FlyView.

---

## SwarmDashboard Usage

### Telemetry

| Field | Description |
|-------|-------------|
| LAT | Latitude in decimal degrees |
| LON | Longitude in decimal degrees |
| ALT (m) | Altitude Above Ground Level |
| SPD | Ground speed in m/s |
| HDG | Heading in degrees |
| BAT (%) | Battery — green >50%, orange 20-50%, red <20% |

### Selection

- Click any drone card to select or deselect
- ALL = select all drones
- NONE = clear selection
- LEADER = select only the current leader
- Orange border = current leader
- Blue border = selected
- Green CAM = active camera feed

### Commands

- ARM / DISARM
- TAKEOFF Xm = takeoff to altitude set by slider
- CHANGE ALT = change altitude in flight (must be airborne)
- LAND
- CAM = switch QGC video to this drone

---

## Leader Election Algorithm

Runs every 2 seconds. Scoring:

    if drone is actively flying  -> score 2  (best candidate)
    if drone is armed on ground  -> score 1
    if drone is disarmed         -> score 0
    if drone connection is lost  -> excluded

Highest score wins. Tie = lowest System ID wins.
Failover is automatic and immediate when leader loses connection.

---

## Altitude Change Explained

guidedModeChangeAltitude takes a DELTA not an absolute value.

    delta = target_AGL - current_AGL
    v.guidedModeChangeAltitude(delta, false)

false = pauseVehicle: drone keeps flying during altitude change.
Only works when drone is airborne (ALT > 0.5m).

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Gazebo does not open | sudo chmod 700 /run/user/1000 |
| cmake not found | sudo ln -sf /usr/local/bin/cmake /usr/bin/cmake |
| ninja fails SDL3 PCH | touch .../SDL3-static.dir/cmake_pch.hxx.gch |
| gz_x500 unknown target | rm -rf build then rebuild |
| Arming denied | Restart PX4, wait for Ready for takeoff |
| SwarmDashboard not visible | Run ninja -j4, connect drones |
| WAITING FOR VIDEO | Click Video ON in Gazebo BEFORE launching QGC |
| Video disappears after 3s | Disconnect VPN, reduce CPU load |
| Black Gazebo screen | Disconnect VPN then relaunch |
| Drones 2 and 3 not detected | Add Comm Links for ports 14551/14552 in QGC settings |
| CHANGE ALT does nothing | Must be airborne first |

---

## Clean Shutdown

    1. Close QGC
    2. Ctrl+C in all PX4 terminals, type quit if pxh> still active
    3. Close Gazebo

If next launch fails:

    rm ~/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/dataman

---

## Architecture

    QGroundControl (custom build)
        src/FlyView/FlyViewWidgetLayer.qml  <- MODIFIED
        src/FlyView/SwarmDashboard.qml      <- NEW
        src/FlyView/CMakeLists.txt          <- MODIFIED

    Option A — Gazebo Harmonic (3 drones, no video)
        PX4 SITL x3 -> x500_mono_cam -> Gazebo Harmonic 8.12.0
        MAVLink UDP: 14550 / 14551 / 14552

    Option B — Gazebo Classic (1 drone, with video)
        PX4 SITL x1 -> typhoon_h480 -> Gazebo Classic 11.10.2
        GStreamer H264 -> UDP 5600 -> QGC VideoReceiver
        MAVLink UDP: 14550

---

## References

| Resource | URL |
|----------|-----|
| QGroundControl | github.com/mavlink/qgroundcontrol |
| PX4 Autopilot | github.com/PX4/PX4-Autopilot |
| PX4 Gazebo Classic video | docs.px4.io/main/en/sim_gazebo_classic |
| Gazebo Harmonic | gazebosim.org/docs/harmonic |
| MAVLink | mavlink.io/en |
| Qt QML | doc.qt.io/qt-6/qmlapplications.html |
| aqtinstall | github.com/miurahr/aqtinstall |
ENDREADME
echo "README written"
