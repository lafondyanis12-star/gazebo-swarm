# Weekly Report — Week of 2026-08-03 to 2026-08-08

## 2026-08-07 — Takeoff collision fix + finite mission

Closes the "takeoff is not always clean" issue flagged as next week's work in
the previous report (`reports/2026-07-31/weekly_report.md`).

- **Root cause found and fixed**: the 3 drones spawning only ~2m apart and
  converging on their formation slots at full speed right after takeoff was
  causing real airframe collisions in Gazebo (separation down to 0.18–0.4m),
  tipping a drone's attitude enough to lose lift. Every control-law-level
  mitigation had already been tried (damping, repulsion ramp, speed clamps —
  see `collision_avoidance.md`); the one untried lever was spawn spacing
  itself. Widened `SPAWN_EAST_M` 2m → 4m in `worlds/swarm_persistent.sdf`.
  Verified over a 6+ minute flight with zero SEPARATION events during
  takeoff.
- **Added a finite "there and back" mission** instead of the indefinite
  loiter/line loop: each drone now flies its role for `MISSION_OUTBOUND_S`
  (40s) after becoming airborne, then independently breaks off, seeks its own
  spawn position, and lands on arrival — timed off its own
  `airborne_since_s_`, so the whole swarm heads home together with no extra
  coordination needed. Verified end-to-end: all 3 drones return to spawn and
  land cleanly.
- Bundled several macOS build fixes that had been accumulating: Homebrew's
  Python framework breaking CMake's ament tooling, `swarm_node` crashing on
  `swarm_msgs`' Python bindings needing libpython in-process,
  `DYLD_LIBRARY_PATH` being stripped for `ros2` CLI subprocesses in
  `test_election.py`, Docker test build outputs colliding with native
  RoboStack builds, and a shutdown handler that now sends RTL instead of just
  dropping the Offboard link.

## 2026-08-08 (today) — Real flown excursion replaces the teleport-based disconnect demo

The existing demo procedure (`COMMANDES_DEMO.txt`) simulated a radio-range
disconnect by teleporting a drone's physical pose in Gazebo (`gz service
.../set_pose`). Re-running that demo today, live, produced exactly the
failure mode the freefall investigation in `collision_avoidance.md` had
already flagged as an open, unsolved issue: an instant position jump with no
matching velocity is a discontinuity neither the physics engine nor PX4's EKF
is designed to absorb. The teleported drone's attitude was knocked into an
uncontrolled drift — measured live drifting to **265m off course** before the
hard geofence backstop (`MAX_GEOFENCE_HARD_M`) caught it and forced a landing.

**Fix**: replaced the teleport with a genuine flown maneuver.

- Added an `excursion` ROS 2 parameter (`ros2 param set /drone_N excursion
  true`): on trigger, the drone flies itself — under its own normal Offboard
  velocity control, no teleporting involved — to a point `EXCURSION_DISTANCE_M`
  (15m) away for `EXCURSION_DURATION_S` (10s), then automatically resumes its
  normal role (leader loiter or formation-following).
- Because the target is chosen to clear `MAX_RANGE_M` (10m), the existing
  radio-range logic in `on_peer_status()` disconnects/reconnects exactly as
  before — same leader re-election and reconnection behavior, just reached by
  a continuous flight path instead of a physics-breaking jump.
- Added a short speed ramp (`EXCURSION_RAMP_S`, 2s) into the excursion, the
  same idea already used for the takeoff speed ramp: without it, the 15m
  position-gap step would saturate the speed clamp instantly and snap
  straight to `MAX_SPEED_M_S` in one control tick.

**Verified live, twice**, watching real telemetry (`ros2 topic echo`) and the
`swarm_node` logs, not just the outcome:

- Triggered on the **leader** (`drone_0`): altitude held stable at ~2.9–3.1m
  for the entire 10s excursion (no freefall). Followers, actively chasing the
  leader's live position, kept pace and the whole formation relocated
  together rather than losing contact — an expected, if not originally
  anticipated, consequence of triggering it on the leader specifically.
- Triggered on a **follower** (`drone_1`): clean `Excursion started` →
  `Signal lost with drone_0` / `drone_2` (~6s in, once past the 10m range) →
  `Excursion complete` at exactly 10.0s → reconnected ~11s later → leader
  re-election back to `drone_0`. Full disconnect/reconnect/re-election cycle,
  zero freefall, altitude stable throughout.

**Next step**: `COMMANDES_DEMO.txt` still documents the old `set_pose`
procedure for the live demo — worth updating it to the `excursion` parameter
before the next presentation, and triggering it on a follower rather than the
leader for a cleaner "disconnect" visual.
