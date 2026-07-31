# Collision Avoidance: Constraints and Operating Principle

This document covers exclusively the collision-avoidance layer of `swarm_node.cpp`
(`ros2_ws/src/swarm_comm/src/swarm_node.cpp`) — the mechanism that keeps every pair
of drones apart during formation flight. It does not cover communication, leader
election, or the formation-seeking law itself except where they interact with
avoidance.

## Operating principle

Collision avoidance is a **reactive repulsion field layered on top of
formation-seeking**, blended by priority rather than summed:

1. Every drone independently tracks the live position of its peers (received over
   its own `SwarmStatus` broadcast/subscribe mesh, at 10 Hz).
2. On every control tick, each drone computes two candidate velocities:
   - `desired`: the formation-seeking velocity (PD controller chasing its
     triangle-formation slot behind the leader).
   - `repulsion`: a horizontal-only push away from any peer closer than
     `AVOID_MARGIN_M`.
3. Instead of adding these two vectors together, the controller **blends them by
   priority**: as repulsion strengthens, it takes progressively more of the
   available speed budget away from formation-seeking, rather than the two
   fighting each other and being diluted by a final clamp. At full repulsion
   strength, formation-seeking is dropped entirely — escaping takes 100% of the
   command.
4. The blended velocity is clamped (horizontal and vertical speed clamped
   *separately*, not as one 3D magnitude — see below) and sent to PX4 as an
   Offboard velocity setpoint.

This runs identically for the leader and both followers; there is no
special-cased "avoidance-only" drone. Repulsion is symmetric — if two drones
close on each other, both push apart independently, each unaware of the other's
own repulsion output.

## Constraints (control loop, 10 Hz — `CONTROL_PERIOD_S = 0.1 s`)

| Constant | Value | Meaning |
|---|---|---|
| `SAFE_DIST_M` | 1.0 m | Hard separation floor between any two drones. Crossing it logs a `SEPARATION` warning. |
| `AVOID_MARGIN_M` | 1.4 m | Distance at which repulsion *starts* ramping — anticipatory, not reactive. Chosen below the formation's own steady-state spacing (~1.8–2.0 m) so normal flight never sits inside the margin and fights itself. |
| `MAX_SPEED_M_S` | 3.5 m/s | Shared speed budget for both formation-seeking and repulsion (post-blend). |
| `CHRONIC_TICKS_THRESHOLD` | 20 ticks (2.0 s at 10 Hz) | Consecutive ticks a peer can stay inside `SAFE_DIST_M` before the encounter is treated as chronic rather than momentary. |
| `TAKEOFF_SPEED_CAP_M_S` | 1.0 m/s | Formation-seeking speed cap immediately after becoming airborne. |
| `TAKEOFF_SPEED_RAMP_S` | 8.0 s | Duration of the linear ramp from `TAKEOFF_SPEED_CAP_M_S` back up to `MAX_SPEED_M_S`. Repulsion is exempt — it keeps full authority throughout, even during this window. |
| `MIN_ALT_HARD_M` | 0.3 m | Absolute low-altitude backstop — independent of avoidance, but the usual trigger for it is a repulsion-caused vertical/attitude disturbance (see "Known limitation" below). |

## The repulsion field

```cpp
Vec3 separation_repulsion(const Vec3& my_pos, const std::vector<Vec3>& peer_positions) {
    Vec3 push;
    for (const auto& p : peer_positions) {
        double d = std::hypot(my_pos.x - p.x, my_pos.y - p.y);
        if (d < AVOID_MARGIN_M && d > 1e-3) {
            double t = std::clamp((AVOID_MARGIN_M - d) / (AVOID_MARGIN_M - SAFE_DIST_M), 0.0, 1.0);
            double strength = t * t;   // quadratic: gentle early, full strength by the floor
            push.x += (my_pos.x - p.x) / d * strength * MAX_SPEED_M_S;
            push.y += (my_pos.y - p.y) / d * strength * MAX_SPEED_M_S;
        }
    }
    return push;
}
```

Key properties, each traced to a specific failure mode found in live testing:

- **Quadratic ramp, not linear.** Strength grows as `t²` from 0 at `AVOID_MARGIN_M`
  to 1 at `SAFE_DIST_M`. A gentle early nudge that only becomes sharply repulsive
  right near the floor — not a hard switch that stays off until a violation is
  already happening.
- **Ramp reaches full strength exactly at `SAFE_DIST_M`, not only as distance
  approaches zero.** Normalizing against the full `AVOID_MARGIN_M` interval
  (instead of `AVOID_MARGIN_M − SAFE_DIST_M`) left repulsion at only ~8% of
  maximum right at the floor — weakest exactly where it needs to be a hard
  limit. A pair closing at up to `MAX_SPEED_M_S` each (7 m/s combined) blew
  straight through the old margin before the ramp caught up.
- **Horizontal-only.** The formation is flat by design (no vertical offsets), so
  an earlier 3D version of this function added a vertical push sized off
  whatever small altitude difference happened to exist between two drones —
  landing on the "push down" side during post-takeoff convergence chaos was
  enough on its own to sag a drone toward the ground. Repulsion now only ever
  acts in x/y; `repulsion.z` is always `0`.

## Priority blend (not a vector sum)

```cpp
double repulsion_weight = chronic ? 1.0 : std::clamp(repulsion_mag / MAX_SPEED_M_S, 0.0, 1.0);
Vec3 blended{
    desired.x * (1.0 - repulsion_weight) + repulsion.x,
    desired.y * (1.0 - repulsion_weight) + repulsion.y,
    desired.z,   // altitude always keeps full authority, see below
};
Vec3 command = clamp_speed(blended, MAX_SPEED_M_S);
```

An earlier additive version (`desired + repulsion`, then clamp) let a strong
formation-seeking command dilute the avoidance direction right when a peer was
closest — exactly the wrong moment. The priority blend instead hands repulsion
an increasing *share* of the speed budget as it strengthens, so a close
encounter always gets escape speed proportional to how close it actually is.

**Altitude is never blended by `repulsion_weight`.** Since repulsion is
horizontal-only, blending `desired.z` by the same weight as x/y would crush
altitude correction toward zero any time horizontal avoidance is strong or
chronic — this was caught live: `desired.z = 3.5` (climbing hard, correctly)
but `command.z = 0.00` at `w = 1.00`, while real telemetry showed the drone in
freefall. Altitude keeps full authority regardless of what avoidance is doing
horizontally.

## Chronic vs. momentary proximity

A single close encounter and a *sustained* one need different responses.
`close_encounter_ticks_` counts consecutive control ticks with a peer inside
`SAFE_DIST_M`; past `CHRONIC_TICKS_THRESHOLD` (2 s), the encounter is flagged
`chronic` and `repulsion_weight` is forced to `1.0` — formation-seeking is
dropped entirely and the full speed budget goes to escaping, until the peer
clears the floor and the counter resets.

This exists because live testing traced roughly half of all test flights
hitting the hard geofence (40 m+ drift) with no single dramatic cause: a peer
staying just inside the floor for an extended period kept the normal priority
blend permanently diverting *some* of the speed budget back into
formation-seeking, which — if that seeking was itself contributing to the
conflict — never let the gap fully close. Small bias, sustained long enough,
added up to a large drift with no single moment that looked like a bug.

## Independent speed clamping (horizontal vs. vertical)

`clamp_speed()` scales horizontal speed and clamps vertical speed
*separately*, rather than normalizing the combined 3D vector to one magnitude.
A single combined clamp meant a large horizontal correction (well past
10–20 m/s during an active divergence) forced the same scale-down factor onto
the altitude term too, crushing whatever climb correction was most needed at
that exact moment.

## Known limitation — not solved by the control law alone

Every cause above was found and fixed at the control-law level (ghost-leader
feedforward, P-only oscillation, combined speed clamp, 3D repulsion, ramp
curve) — see the project README for the full trace of each. After all of
them, drones can still occasionally end up in real freefall, confirmed via
live MAVSDK attitude telemetry: roll/pitch measured at 41–56° during a fall
(over half the airframe's thrust pointing sideways instead of up), correlated
with `SEPARATION` events logged down to 0.16–0.4 m — consistent with an actual
airframe knock in Gazebo during a close pass, not a logical bug in this file.
`MIN_ALT_HARD_M` reliably catches every instance and lands the drone safely
rather than letting it crash or drift, but this is a physical near-miss
problem the reactive repulsion field cannot fully prevent by construction — it
reacts to *position*, not to an already-in-progress physical contact. See the
session report in this folder for the current open investigation into how
often this triggers at takeoff specifically.
