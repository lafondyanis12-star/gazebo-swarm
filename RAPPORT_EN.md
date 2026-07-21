# Report — Communication and Leader Election for a Drone Swarm (Gazebo + PX4)

This report walks through the work, roughly in the order it happened, of setting up communication and leader election for a drone swarm under Gazebo and PX4.

## Context

The internship focuses on a swarm of 3 drones that need to communicate with each other, detect and report when a drone disconnects, and elect a new leader when the leader (`drone_0`) fails — without a human stepping in.

The project initially started under Webots — a starting choice that turned out not to be the right one. A first version had been written there (`~/mavic2pro.c`, kept outside this repo as a reference): GPS position broadcast every 0.2s via `emitter`/`receiver`, a 3s timeout for detection, and V-formation flight with `drone_0` hard-coded as leader. After discussion, and taking into account the fundamentals of Yassine's project, it became clear that Gazebo made more sense — the simulator recommended by the professor for this internship, and a chance to actually learn the tool expected for it rather than stick with an initial personal choice.

The goal of this phase was therefore to take the logic already written under Webots (communication, detection, election — not the flight formation yet) and redo it with the tools actually used for the internship: Gazebo Sim and PX4 SITL. What was already missing in the Webots version, and what remains the core of the work presented here, was the election part: if the leader disappears, nothing takes its place, the followers just keep trying to follow a drone that no longer exists.

## Deciding how the drones would talk

Before writing any code, the first thing to figure out was how the drones would actually talk to each other under Gazebo/PX4, since there's no ready-made equivalent to Webots' `emitter`/`receiver` in that environment. Three options were on the table.

The first was writing a Gazebo Transport (`gz-transport`) plugin that publishes and listens to messages directly inside the simulator, without PX4 or ROS involved. That's the closest thing conceptually to what Webots did, but it means writing and compiling a C++ plugin, which is a heavier lift.

The second was ROS 2: each drone publishing its position on ROS 2 topics. The catch is there was no ROS 2 workspace set up on the machine at all, so that option got dropped for this phase.

The third, and the one that got used, was running PX4 in SITL (Software In The Loop — PX4 runs on the computer instead of on real hardware) and reading/driving each instance through MAVSDK, the official C++ library that speaks MAVLink. PX4-Autopilot was already cloned on the machine, and this setup is the closest to what an actual multi-drone deployment would look like.

MAVLink and MAVSDK only handle getting each drone's real position — the "tell the others where I am and elect a leader" part doesn't map to any standard protocol, so that had to be written from scratch (details below).

### The protocols involved

A protocol is just a rule two programs agree to follow so they can understand each other. From lowest to highest level, here's everything actually at play in this project:

- **IP** — the foundation of any network communication, even between two programs on the same machine (address `127.0.0.1`, "localhost"). Never touched directly, but everything else sits on top of it.
- **UDP** — sends a packet with no prior connection and no delivery guarantee, like dropping a letter in a mailbox. Used both for MAVLink and for the custom swarm protocol — a good fit here since messages repeat very often (five times a second), so losing one occasionally doesn't matter.
- **TCP** — guarantees ordered delivery, at the cost of an actual connection set up beforehand. Not used directly in the code, but Gazebo Transport (below) relies on it internally.
- **MAVLink** — the standardized message format for the drone world (position, speed, battery...), understood by nearly every autopilot on the market. This is what PX4 speaks to announce its position, carried over UDP. The program (`swarm_node.cpp`) speaks MAVLink directly to PX4 through the MAVSDK library, with no middleman: the full chain to read a position is simply `swarm_node → MAVLink/UDP → PX4`.
- **Gazebo Transport (gz-transport)** — Gazebo's internal publish/subscribe system: one program publishes on a named channel, another subscribes to it. This is what's behind every `gz topic`/`gz service` command used here (listing the drones present, moving one, triggering a reset).
- **The custom swarm protocol** — covered below.

## Getting PX4 and Gazebo working

**Planned:** build PX4 for the modern Gazebo target (gz-sim, not gazebo-classic) and install MAVSDK, in order to start coding the communication logic. This step ended up taking longer than expected, because of three problems that had nothing to do with the internship's actual subject but still had to be solved before moving on.

**Problem encountered (1/3):** the Homebrew-installed Python (3.14) was broken — an internal library (`expat`) didn't match what Python expected, which made `pip` completely unusable.
**Solution:** use Apple's system Python (`/usr/bin/python3`) everywhere instead, which worked fine.

**Problem encountered (2/3):** PX4 had its own isolated Python environment (`.venv`), built on that same broken Python. `pip` didn't even exist inside it, so there was no way to install build dependencies like `kconfiglib`.
**Solution:** rebuild that `.venv` with the system Python, then reinstall everything in `Tools/setup/requirements.txt` into it.

**Problem encountered (3/3):** once the build was retried, it failed on a Gazebo plugin for camera-based motion detection (`optical_flow`), which isn't used for anything here but still gets compiled by default. That plugin needs an old OpenCV API (`opencv2/core/types_c.h`) that's been removed from the version installed on the machine (5.0, too recent).
**Solution:** rather than replace OpenCV and risk breaking something else, install the older version (`opencv@4`) alongside it, and reconfigure CMake to target that specific version just for that one module.

With those three sorted out, `make px4_sitl` builds cleanly and produces the binary at `build/px4_sitl_default/bin/px4`.

## Communication and leader election

**Planned:** each drone reads its real position, shares it with the other two, detects a disconnection (3s timeout, matching Webots), and elects a new leader when the leader disappears.

The design comes down to two pieces, run concurrently as three `std::thread`s per drone (one to broadcast, one to receive, one to check timeouts and run the election). First, each drone connects via MAVSDK's `Telemetry` plugin to its own PX4 instance and reads its real GPS position, computed by Gazebo's physics. Second, each drone broadcasts that position to the other two, five times a second, over a small UDP mesh built directly with POSIX sockets (`socket`/`bind`/`sendto`/`recvfrom`) — the direct equivalent of Webots' `emitter`/`receiver`. Each drone keeps track of when it last heard from each other drone; past 3 seconds of silence (matching the Webots threshold), it's marked as disconnected.

The election follows a simple rule each drone applies on its own, with no coordination: the leader should always be the lowest surviving ID among the ones it can see. This rule is recomputed every 500ms by the timeout-checking thread, not just when a disconnection happens (see the split-brain fix below for why that distinction matters). Since all three drones see roughly the same network, they all land on the same answer.

### The custom protocol's wire format

Each message is 36 bytes:

| Field | Size | Content |
|---|---|---|
| `sender_id` | 4 bytes | the sending drone's ID (0, 1 or 2) |
| `x`, `y`, `z` | 8 bytes each | latitude, longitude, altitude |
| `timestamp` | 8 bytes | time the message was sent |

Defined in `swarm_node.cpp` as a `SwarmPacket` struct (`sender_id` as an `int32_t`, the rest as `double`, with `#pragma pack(1)` so the compiler doesn't insert padding between fields) — directly modeled on the `Packet` struct from the Webots code.

### What didn't work on the first try

**Problem encountered (1/3):** the very first version had every drone connect via MAVSDK to all three PX4 instances at once — its own plus the other two. It seemed simpler, no need to invent a protocol. But when tested, no detection worked at all. The reason: PX4 sends its data to a fixed target port (14540+id), and only one program can listen on a given port at a time. If `drone_1` and `drone_2` both try to listen to `drone_0` on that same port, only one of them actually gets anything.
**Solution:** go back to the two-piece design above — each drone only listens to its own instance, and passes the information along to the others itself through the custom UDP mesh.

**Problem encountered (2/3):** MAVSDK printed a deprecation warning on the connection URL (`udp://` being phased out).
**Solution:** replaced with `udpin://0.0.0.0:PORT` (same behavior, more explicit syntax).

**Problem encountered (3/3):** while testing the election itself, stopping only the leader's PX4 process wasn't enough — its communication process kept running and kept rebroadcasting its last known position on a loop, so the other drones still thought they were hearing from it.
**Solution:** to simulate a real failure, stop both programs for the affected drone, PX4 and the communication process — just like in real life, a failure takes everything down at once, not just part of it.

**Problem encountered (4/4):** once the program was compiled and launched, nothing was printed at all, even though MAVSDK's own internal logs showed the connection to PX4 had succeeded. The program wasn't crashing, just silent.
**Solution:** when standard output (`std::cout`) isn't connected to an actual terminal (for example redirected to a file), it gets buffered and only flushed once the buffer fills up or the program exits. Adding `std::cout.setf(std::ios::unitbuf)` at the start of the program forces an immediate flush on every line.

**Test performed:** all three drones detect each other correctly, and killing `drone_0` (both processes) leads `drone_1` and `drone_2` to detect the loss within about 3 seconds and both elect `drone_1` as the new leader.

## Making the Gazebo world persistent

**Planned:** make sure a simulation reset (the button that zeroes out the simulation clock) does not make the three drones disappear.

At first, the three drones were spawned dynamically by PX4 on every launch — never written into the world file, just added in memory. That meant a reset reloaded the world from its original file, and the drones disappeared.

**General fix:** write them directly into the world file (`worlds/swarm_persistent.sdf`) at fixed positions, then tell PX4 to stop spawning drones and instead attach to the ones already there (the `PX4_GZ_MODEL_NAME` variable instead of `PX4_SIM_MODEL`). The world file lives in this project, but PX4 looks for its worlds in its own folder (`Tools/simulation/gz/worlds/`) — rather than keeping two copies in sync, a symbolic link was created in PX4's folder pointing back to the real file kept here.

**Problem encountered:** once that was in place, `drone_0` attached to the world fine, but `drone_1` and `drone_2` stayed stuck indefinitely repeating "waiting for Gazebo." The cause turned out to be a bit subtle: the `x500` model's config file defaults the Gazebo world name to `"default"` if nothing else is specified, and the real name (`swarm_persistent`) had only been given to `drone_0`.
**Solution:** explicitly set the world name for all three instances, not just the first.

**Test performed:** triggered a reset through Gazebo's control service — all three drones stay put, PX4 keeps running normally.

**Caveat noted:** triggering a reset manually while a PX4 instance has been running for a while can desync its internal clock (the "lockstep" mode it shares with Gazebo) and silently stall it — the process stays alive but stops responding. A freshly started instance doesn't have that problem.

## Simulated radio range and the split-brain bug

**Planned:** be able to test a disconnection by physically moving a drone away, not just by killing its process, to get closer to a realistic scenario.

**Solution put in place:** a simulated 6-meter radio range, computed from the real GPS distance between two drones using a flat-Earth approximation (converting degrees of latitude/longitude to meters locally, since the drones never move far enough apart for the Earth's curvature to matter) — a message received from further away is simply ignored, the same way a real radio would lose signal. Alongside that, real terminal windows were opened for each drone (macOS's `osascript`/AppleScript, one `do script` call per window) so connection/disconnection/election events would actually be visible live, instead of running silently in the background.

**Test performed:** `drone_2` teleported 30 meters away using Gazebo's `set_pose` service, then brought back to its starting position.

**Problem encountered:** that test surfaced a real bug. An isolated drone that can't see anyone else logically elects itself leader — but the original election rule never gave leadership back once a drone had taken it, even after the real leader became reachable again. The result was that after reconnecting, two drones could both stay convinced they were the leader: a classic split-brain situation in distributed systems.

**Solution:** have the rule continuously recompute who *should* be leader (always the lowest alive ID), instead of only reacting when the current leader disappears.

**Validation test:** `drone_2`, after being isolated and reconnected, correctly hands leadership back to `drone_0` as soon as it sees it again.

## Version control

**Planned:** keep a versioned record of the work. Installed and authenticated `gh` (browser login with a one-time code), configured Git, then initialized the repo, made the first commit and created a private GitHub repository. No problems encountered at this stage.

## Testing it yourself

**Shortcut:** `~/Documents/my_project/gazebo_swarm/launch_all.sh` does all of this in one go — builds `swarm_node` if needed, opens the Gazebo/PX4 window, waits for it to be ready, then opens the 3 `swarm_node` windows. The step-by-step breakdown below is still useful for understanding what's happening, or troubleshooting if something gets stuck.

Open 4 terminal windows.

In the first, run `~/Documents/my_project/gazebo_swarm/run_swarm.sh` and wait for Gazebo to open with the three drones sitting on the ground. The Gazebo "reset" button can be tried at this point — the drones should stay put.

First, build `swarm_node` (once, or again after any code change):
```bash
cd ~/Documents/my_project/gazebo_swarm
cmake -S . -B build
cmake --build build
```

In the three following terminals, run `./build/swarm_node --id 0`, `--id 1` and `--id 2` respectively. Each terminal should print "SUCCESS: signal detected" for the other two drones within a few seconds.

To simulate the leader failing:
```bash
pkill -f "bin/px4 -i 0"
pkill -f "swarm_node --id 0"
```
Terminals 3 and 4 should show the signal loss and `drone_1`'s election after about 3 seconds.

To simulate a distance-based disconnection without killing anything:
```bash
gz service -s /world/swarm_persistent/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'x500_2', position: {x: 30, y: 0, z: 0}"
```
then the same command with `x: 4` to bring it back. (Also doable by hand in the Gazebo window with the translate tool.)

To stop everything:
```bash
pkill -f "swarm_node"
pkill -f "bin/px4 -i"
pkill -f "gz sim"
```

Connection/disconnection/leader information shows up in the three `swarm_node` terminal windows, not in Gazebo itself — Gazebo only handles the 3D simulation and knows nothing about the communication layer.

## What's next

What's left is porting the V-formation flight logic from `mavic2pro.c` to MAVSDK offboard control (in C++, alongside `swarm_node.cpp`), so the elected leader actually flies differently from the followers instead of just holding a logical role. It would also be worth explicitly testing the case of a permanent network split: two groups that never reconnect each keep their own leader forever, which is the expected behavior, but hasn't been tested directly yet.
