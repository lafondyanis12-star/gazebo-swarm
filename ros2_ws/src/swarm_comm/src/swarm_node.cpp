// ROS 2 port of ~/Documents/my_project/gazebo_swarm/swarm_node.cpp, now flying
// for real: each drone connects to its own PX4 SITL instance over MAVSDK,
// arms, takes off, and then flies a role driven by the swarm's existing
// comm/election layer (unchanged) -- leader loiters a scripted path,
// followers hold a triangle formation on the leader, and a separation rule
// keeps every pair >= SAFE_DIST_M apart.
//
// Frame note: MAVSDK's local NED position is relative to *each vehicle's
// own* EKF origin (its own spawn point), not a shared frame -- but
// worlds/swarm_persistent.sdf spawns x500_0/1/2 at world-ENU offsets
// (0,0,0), (2,0,0), (4,0,0) (see SPAWN_EAST_M below). Adding that fixed
// spawn offset to each vehicle's local NED position gives one shared
// East/North/Up frame all 3 drones can compare positions in directly --
// which is what SwarmStatus.x/y/z now carry (previously a fake parameter).
// Velocity is a free vector (no origin), so Offboard commands need no such
// correction.
//
// Threading note: the header comment on the original port explains why a
// single-threaded rclcpp executor needs no mutex across its callbacks.
// MAVSDK breaks that assumption -- its telemetry callbacks run on MAVSDK's
// own thread, concurrently with the rclcpp timer thread. Position/velocity
// are kept in std::atomic<double> for that reason; a full mutex isn't
// needed since each axis updates independently and a few-millisecond skew
// between x/y/z is imperceptible for a smoothly-flying setpoint.
//
// ros2 run swarm_comm swarm_node --ros-args -p id:=0/1/2
// (expects a PX4 SITL instance already up on udp 14540+id -- see
// run_swarm.sh)

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <swarm_msgs/msg/swarm_status.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using swarm_msgs::msg::SwarmStatus;

namespace {

constexpr int MAX_DRONES = 3;
constexpr double TIMEOUT_S = 3.0;
// Simulated radio range. Widened from an original 6m after live testing
// showed the triangle formation's own footprint (~1.8-2m) leaves too
// little slack at 6m: a follower doesn't need to drift far off-station
// before a transient (a hard turn, a moment of separation-avoidance) pushes
// a pair briefly past range and triggers an avoidable reconnect cycle.
// 10m keeps that margin comfortably above the formation's normal spread.
constexpr double MAX_RANGE_M = 10.0;
constexpr int NO_LEADER = -1;

constexpr int PX4_BASE_PORT = 14540;  // instance i: MAVSDK connects to udpin://0.0.0.0:14540+i

// Spawn offsets from worlds/swarm_persistent.sdf (world-ENU: x=East,
// y=North). Only needed to translate each vehicle's own-origin local NED
// into the shared world frame -- see file header.
constexpr double SPAWN_EAST_M[MAX_DRONES] = {0.0, 2.0, 4.0};
constexpr double SPAWN_NORTH_M[MAX_DRONES] = {0.0, 0.0, 0.0};

constexpr double TAKEOFF_WAIT_ALT_M = 1.5;   // consider "airborne" past this altitude
constexpr double CONTROL_PERIOD_S = 0.1;     // 10Hz: broadcast + Offboard setpoint stream
// Comfortably above the leader's loiter cruise speed (LOITER_RADIUS_M *
// LOITER_OMEGA =~ 0.84 m/s): a live flight test showed that with too little
// margin here, a follower that gets separation-repulsed away from the
// leader during the initial post-takeoff convergence chaos can lose enough
// ground that it never fully recovers -- the gap keeps growing until the
// pair exceeds MAX_RANGE_M and permanently splits off into its own
// leaderless sub-group. This margin plus the slower LOITER_PERIOD_S below
// give followers enough spare speed to reliably close a transient gap
// before it becomes unrecoverable.
constexpr double MAX_SPEED_M_S = 3.5;
constexpr double SEEK_KP = 0.8;              // proportional gain, target position -> velocity

constexpr double SAFE_DIST_M = 1.0;          // hard separation floor between any 2 drones

// Repulsion starts ramping at this distance, well before SAFE_DIST_M --
// anticipatory rather than purely reactive. Chosen below the formation's
// own normal spacing (~1.8m leader-follower, ~2.0m follower-follower, see
// FORMATION_TRAIL_M/HALF_WIDTH_M below) so steady-state formation flight
// doesn't sit inside the margin and fight itself; a live test showed the
// old margin (repulsion only starting exactly at SAFE_DIST_M) meant
// avoidance only ever kicked in once already in violation, one control
// tick from breaching it further on any real closing speed.
constexpr double AVOID_MARGIN_M = 1.4;

// Consecutive control ticks (at CONTROL_PERIOD_S each) a peer can stay
// inside SAFE_DIST_M before it's treated as chronic rather than a
// momentary close call -- see control_step()'s use of close_encounter_ticks_.
constexpr int CHRONIC_TICKS_THRESHOLD = 20; // 2s at the 10Hz control rate

// Triangle formation, in the leader's own forward/right frame.
constexpr double FORMATION_TRAIL_M = 1.5;    // behind the leader
constexpr double FORMATION_HALF_WIDTH_M = 1.0;

// Leader's scripted reference path: circular loiter, centered between the
// 3 spawn points, keyed off wall-clock time so a newly-elected leader
// continues the same curve instead of restarting it.
constexpr double LOITER_CENTER_EAST_M = 2.0;
constexpr double LOITER_CENTER_NORTH_M = 0.0;
constexpr double LOITER_RADIUS_M = 6.0;
constexpr double LOITER_ALT_M = 3.0;
constexpr double LOITER_PERIOD_S = 45.0; // slower cruise = more margin for followers to keep pace
constexpr double LOITER_OMEGA = 2.0 * M_PI / LOITER_PERIOD_S;

// Alternative leader path: a straight line back and forth along the east
// axis (through the same center point as the loiter), selected via the
// `flight_pattern` parameter ("loiter", the default, or "line"). Same
// center/altitude as the loiter so both patterns stay comparable.
//
// Sinusoidal, not a constant-speed triangle wave: a live test with the
// triangle-wave version caught a follower flying to 41m from origin during
// a network split, traced to the leader's velocity reversing *instantly*
// at each end of the line -- a step discontinuity that snapped a
// follower's heading-based formation offset to the opposite side in one
// control tick. sin/cos gives continuous position *and* velocity
// everywhere, so there's no discontinuity for a follower to react badly
// to, matching how the circular loiter never had this problem.
constexpr double LINE_HALF_LENGTH_M = 8.0;
constexpr double LINE_SPEED_M_S = 1.0; // peak speed, at the center of the traversal
constexpr double LINE_OMEGA = LINE_SPEED_M_S / LINE_HALF_LENGTH_M; // rad/s

// Safety bounds: any flight target (leader's own reference point, or a
// follower's formation slot) gets clamped into this box/cylinder before
// being pursued, so a bug in the control law can steer the *seek point*
// wherever it wants without ever commanding the vehicle itself outside a
// known-safe volume. MAX_ALT_HARD_M/MAX_GEOFENCE_HARD_M are harder
// backstops checked directly against *actual* position -- belt and
// suspenders, since clamping the target alone doesn't stop a drone that's
// already outside the fence from continuing to fly away from it (a live
// test caught exactly this: a follower isolated during a network split,
// still chasing a stale target at full speed, reached 41m out -- well
// past GEOFENCE_RADIUS_M -- before the split resolved and it turned back).
constexpr double GEOFENCE_RADIUS_M = 30.0;
constexpr double MIN_ALT_M = 1.0;
constexpr double MAX_ALT_M = 6.0;
constexpr double MAX_ALT_HARD_M = 10.0;
constexpr double MAX_GEOFENCE_HARD_M = 40.0;

struct Vec3 {
    double x = 0, y = 0, z = 0; // east, north, up
};

double distance_m(double x1, double y1, double z1, double x2, double y2, double z2) {
    double dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Vec3 clamp_speed(Vec3 v, double max_speed) {
    double mag = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (mag > max_speed && mag > 1e-6) {
        double s = max_speed / mag;
        v.x *= s; v.y *= s; v.z *= s;
    }
    return v;
}

// Pulls a flight target back inside the geofence cylinder / altitude band
// before it's ever pursued -- see GEOFENCE_RADIUS_M etc.
Vec3 clamp_to_geofence(Vec3 target) {
    double horiz = std::hypot(target.x, target.y);
    if (horiz > GEOFENCE_RADIUS_M) {
        double s = GEOFENCE_RADIUS_M / horiz;
        target.x *= s; target.y *= s;
    }
    target.z = std::clamp(target.z, MIN_ALT_M, MAX_ALT_M);
    return target;
}

// Leader's reference point on the loiter circle at time t (seconds, any
// consistent epoch -- we use wall-clock seconds so a leader handoff picks
// up the same curve seamlessly), plus its feedforward velocity.
// phase_offset shifts the circle so it passes near wherever the current
// leader actually was when it took over (see loiter_phase_offset_): without
// it, phase = OMEGA*wall_clock_seconds lands on an arbitrary point on the
// circle, and a leader starting far from that point would dash toward it
// at full speed the moment it goes airborne, dragging followers (which
// have no such shortcut) into a losing tail chase.
struct ReferencePoint { Vec3 pos, vel; };
ReferencePoint loiter_reference(double t, double phase_offset) {
    double phase = LOITER_OMEGA * t + phase_offset;
    ReferencePoint r;
    r.pos.x = LOITER_CENTER_EAST_M + LOITER_RADIUS_M * std::cos(phase);
    r.pos.y = LOITER_CENTER_NORTH_M + LOITER_RADIUS_M * std::sin(phase);
    r.pos.z = LOITER_ALT_M;
    r.vel.x = -LOITER_RADIUS_M * LOITER_OMEGA * std::sin(phase);
    r.vel.y = LOITER_RADIUS_M * LOITER_OMEGA * std::cos(phase);
    r.vel.z = 0.0;
    return r;
}

// Straight-line counterpart to loiter_reference(): back and forth along
// the east axis. phase_offset plays the same re-anchoring role it does
// for the loiter -- an angle, since this is sinusoidal too now (see the
// comment on LINE_OMEGA for why it isn't a constant-speed triangle wave).
ReferencePoint line_reference(double t, double phase_offset) {
    double phase = LINE_OMEGA * t + phase_offset;
    ReferencePoint r;
    r.pos.x = LOITER_CENTER_EAST_M + LINE_HALF_LENGTH_M * std::sin(phase);
    r.pos.y = LOITER_CENTER_NORTH_M;
    r.pos.z = LOITER_ALT_M;
    r.vel.x = LINE_HALF_LENGTH_M * LINE_OMEGA * std::cos(phase);
    r.vel.y = 0.0;
    r.vel.z = 0.0;
    return r;
}

// Rotates a (forward, right) offset in the leader's heading frame into the
// shared world (east, north) frame. heading_rad is a compass bearing
// (0 = North, increasing toward East), matching NED yaw.
Vec3 rotate_formation_offset(double forward_m, double right_m, double heading_rad, double up_m) {
    double fwd_e = std::sin(heading_rad), fwd_n = std::cos(heading_rad);
    double right_e = std::cos(heading_rad), right_n = -std::sin(heading_rad);
    Vec3 v;
    v.x = fwd_e * forward_m + right_e * right_m;
    v.y = fwd_n * forward_m + right_n * right_m;
    v.z = up_m;
    return v;
}

// Pure control law: given my role, current world position, the reference
// data it needs (leader loiter reference, or the leader's live tracked
// position/heading), returns a desired world-frame velocity (pre-collision
// -avoidance, pre-clamp). Kept free of ROS/MAVSDK types so the formation
// math is exercised the same way whether driven by real telemetry or a
// kinematic stand-in.
Vec3 seek_velocity(const Vec3& target, const Vec3& my_pos, const Vec3& feedforward = Vec3{}) {
    Vec3 v;
    v.x = feedforward.x + SEEK_KP * (target.x - my_pos.x);
    v.y = feedforward.y + SEEK_KP * (target.y - my_pos.y);
    v.z = feedforward.z + SEEK_KP * (target.z - my_pos.z);
    return v;
}

// Repulsion vector keeping every tracked peer >= SAFE_DIST_M away.
// Starts ramping at AVOID_MARGIN_M (well outside the hard floor) rather
// than exactly at SAFE_DIST_M, and ramps quadratically rather than
// linearly: a gentle early nudge that only becomes sharply repulsive
// right near the floor, not a switch that's off right up until the
// instant a violation is already happening.
Vec3 separation_repulsion(const Vec3& my_pos, const std::vector<Vec3>& peer_positions) {
    Vec3 push;
    for (const auto& p : peer_positions) {
        double d = distance_m(my_pos.x, my_pos.y, my_pos.z, p.x, p.y, p.z);
        if (d < AVOID_MARGIN_M && d > 1e-3) {
            double t = (AVOID_MARGIN_M - d) / AVOID_MARGIN_M; // 0..1
            double strength = t * t; // quadratic: gentle early, sharp near the floor
            push.x += (my_pos.x - p.x) / d * strength * MAX_SPEED_M_S;
            push.y += (my_pos.y - p.y) / d * strength * MAX_SPEED_M_S;
            push.z += (my_pos.z - p.z) / d * strength * MAX_SPEED_M_S;
        }
    }
    return push;
}

} // namespace

struct DroneTracker {
    rclcpp::Time last_seen;
    bool has_seen = false;
    bool connected = false;
    int last_claimed_leader = NO_LEADER;
    Vec3 pos, vel;
    Vec3 ref_vel; // analytical scripted-path velocity, only meaningful while this peer is leader
};

class SwarmNode : public rclcpp::Node {
public:
    SwarmNode() : Node("swarm_node") {
        my_id_ = declare_parameter<int>("id", -1);
        if (my_id_ < 0 || my_id_ >= MAX_DRONES) {
            RCLCPP_ERROR(get_logger(), "Usage: ros2 run swarm_comm swarm_node --ros-args -p id:=0|1|2");
            throw std::runtime_error("invalid --id");
        }
        my_name_ = "drone_" + std::to_string(my_id_);
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (i != my_id_) trackers_[i]; // default-constructs via map::operator[]
        }

        pub_ = create_publisher<SwarmStatus>("/drone_" + std::to_string(my_id_) + "/swarm_status", 10);
        for (int peer = 0; peer < MAX_DRONES; ++peer) {
            if (peer == my_id_) continue;
            std::string topic = "/drone_" + std::to_string(peer) + "/swarm_status";
            subs_.push_back(create_subscription<SwarmStatus>(
                topic, 10, [this, peer](SwarmStatus::SharedPtr msg) { on_peer_status(peer, msg); }));
        }

        // x/y/z parameters: a fallback position source for comm/election
        // testing without PX4 (test_election.py's `ros2 param set x <val>`
        // trick to fake a range disconnect). Once MAVSDK telemetry starts
        // flowing during real flight it overwrites these atomics 10x/sec,
        // so a stray `ros2 param set` during real flight is harmless --
        // it's stale again before the next control tick.
        declare_parameter<double>("x", 0.0);
        declare_parameter<double>("y", 0.0);
        declare_parameter<double>("z", 0.0);
        // Which scripted path the leader flies: "loiter" (circular, default)
        // or "line" (straight back-and-forth). Read once at startup --
        // whoever holds leadership when this is checked flies that pattern;
        // it's not meant to be swapped live mid-flight.
        flight_pattern_ = declare_parameter<std::string>("flight_pattern", "loiter");
        my_x_.store(get_parameter("x").as_double());
        my_y_.store(get_parameter("y").as_double());
        my_z_.store(get_parameter("z").as_double());
        param_cb_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if (p.get_name() == "x") my_x_.store(p.as_double());
                    else if (p.get_name() == "y") my_y_.store(p.as_double());
                    else if (p.get_name() == "z") my_z_.store(p.as_double());
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(CONTROL_PERIOD_S), [this]() { control_step(); });
        check_timer_ = create_wall_timer(500ms, [this]() { check_and_elect(); });

        RCLCPP_INFO(get_logger(), "[%s] Starting. Connecting to PX4 on udp %d...",
                    my_name_.c_str(), PX4_BASE_PORT + my_id_);
        std::thread(&SwarmNode::connect_and_fly, this).detach();
    }

private:
    // --- MAVSDK setup + arm/takeoff/offboard sequence, run on its own
    // thread so it never blocks the ROS executor (this involves several
    // blocking MAVSDK calls and a 30s connection wait). ---
    void connect_and_fly() {
        mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
            mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});
        std::string url = "udpin://0.0.0.0:" + std::to_string(PX4_BASE_PORT + my_id_);
        if (mavsdk_->add_any_connection(url) != mavsdk::ConnectionResult::Success) {
            RCLCPP_ERROR(get_logger(), "[%s] MAVSDK connection failed on %s", my_name_.c_str(), url.c_str());
            return;
        }

        // 90s, not the more obvious ~30s: this needs to comfortably outlast
        // a cold PX4+Gazebo boot (run_swarm.sh's own stagger already means
        // drone_1/2 don't even start booting until ~15s after drone_0, on
        // top of each instance's own Gazebo-attach + EKF wait), for the
        // case where swarm_node is started at the same time as the
        // simulator rather than against an already-warm one -- see
        // test_formation.py, which now launches both itself.
        auto system = mavsdk_->first_autopilot(90.0);
        if (!system) {
            RCLCPP_ERROR(get_logger(), "[%s] No autopilot found after 90s", my_name_.c_str());
            return;
        }

        telemetry_ = std::make_unique<mavsdk::Telemetry>(system.value());
        action_ = std::make_unique<mavsdk::Action>(system.value());
        offboard_ = std::make_unique<mavsdk::Offboard>(system.value());

        telemetry_->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed pv) {
            my_x_.store(SPAWN_EAST_M[my_id_] + pv.position.east_m);
            my_y_.store(SPAWN_NORTH_M[my_id_] + pv.position.north_m);
            my_z_.store(-pv.position.down_m);
            my_vx_.store(pv.velocity.east_m_s);
            my_vy_.store(pv.velocity.north_m_s);
            my_vz_.store(-pv.velocity.down_m_s);
        });

        // PX4's *actual* flight mode -- separate from offboard_ready_, which
        // only reflects whether our offboard_->start() call was accepted,
        // not whether PX4 stayed in Offboard afterward. Investigating a
        // drone that sat frozen at its spawn point with no error logged
        // anywhere: this is the one thing that could explain velocity
        // setpoints being sent successfully (Offboard::Result::Success)
        // while PX4 silently ignores them because it's not actually in
        // Offboard mode.
        telemetry_->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode mode) {
            auto m = static_cast<int>(mode);
            current_flight_mode_.store(m);
            if (m != last_logged_flight_mode_.exchange(m)) {
                std::ostringstream oss;
                oss << mode;
                RCLCPP_WARN(get_logger(), "[%s] PX4 flight mode changed: %s",
                            my_name_.c_str(), oss.str().c_str());
            }
        });

        RCLCPP_INFO(get_logger(), "[%s] Waiting for global + home position...", my_name_.c_str());
        // Captured by value (shared_ptr) rather than by reference: this
        // subscription is never explicitly cancelled, so a by-reference
        // capture of these stack locals would dangle the moment this
        // function returns -- and MAVSDK keeps delivering health updates
        // long after that, straight into freed stack memory. A crash
        // that took down all 3 drones right after "Airborne, in Offboard"
        // came from exactly this.
        auto ready_promise = std::make_shared<std::promise<void>>();
        auto ready_future = ready_promise->get_future();
        auto ready_fired = std::make_shared<std::atomic<bool>>(false);
        telemetry_->subscribe_health([ready_promise, ready_fired](mavsdk::Telemetry::Health health) {
            if (!ready_fired->exchange(true) && health.is_global_position_ok && health.is_home_position_ok) {
                ready_promise->set_value();
            }
        });
        if (ready_future.wait_for(30s) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "[%s] Timed out waiting for position lock", my_name_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "[%s] Arming...", my_name_.c_str());
        if (auto res = action_->arm(); res != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(get_logger(), "[%s] Arm failed", my_name_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "[%s] Taking off...", my_name_.c_str());
        if (auto res = action_->takeoff(); res != mavsdk::Action::Result::Success) {
            RCLCPP_ERROR(get_logger(), "[%s] Takeoff failed", my_name_.c_str());
            return;
        }
        for (int i = 0; i < 30 && my_z_.load() < TAKEOFF_WAIT_ALT_M; ++i) {
            std::this_thread::sleep_for(1s);
        }

        // Offboard requires an existing setpoint stream before it'll accept
        // PX4's mode switch -- seed a couple, then start. Offboard::start()
        // returning Success only means PX4 acked the mode-switch command,
        // not that it stayed there: live testing caught PX4 accepting the
        // switch and then reverting straight to Hold under a second later
        // (visible as a Takeoff -> Hold transition in the flight-mode log,
        // with no Offboard in between) -- presumably because it judged the
        // setpoint stream not yet "established" going into the switch.
        // Confirm against the real flight mode rather than trusting the
        // single start() call, and retry if it didn't stick.
        bool entered_offboard = false;
        for (int attempt = 0; attempt < 5 && !entered_offboard; ++attempt) {
            offboard_->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
            std::this_thread::sleep_for(100ms);
            offboard_->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
            if (auto res = offboard_->start(); res != mavsdk::Offboard::Result::Success) {
                RCLCPP_WARN(get_logger(), "[%s] Offboard start attempt %d rejected, retrying...",
                            my_name_.c_str(), attempt);
                continue;
            }
            std::this_thread::sleep_for(300ms);
            if (current_flight_mode_.load() == static_cast<int>(mavsdk::Telemetry::FlightMode::Offboard)) {
                entered_offboard = true;
            } else {
                RCLCPP_WARN(get_logger(), "[%s] Offboard reverted immediately (attempt %d), retrying...",
                            my_name_.c_str(), attempt);
            }
        }
        if (!entered_offboard) {
            RCLCPP_ERROR(get_logger(), "[%s] Could not establish Offboard mode after retries", my_name_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "[%s] Airborne, in Offboard.", my_name_.c_str());
        offboard_ready_.store(true);
    }

    // --- 10Hz loop: publish real telemetry, and (once airborne) send the
    // role-appropriate Offboard velocity setpoint. ---
    void control_step() {
        Vec3 my_pos{my_x_.load(), my_y_.load(), my_z_.load()};
        Vec3 my_vel{my_vx_.load(), my_vy_.load(), my_vz_.load()};

        SwarmStatus msg;
        msg.sender_id = my_id_;
        msg.claimed_leader = leader_id_;
        msg.x = my_pos.x; msg.y = my_pos.y; msg.z = my_pos.z;
        msg.vx = my_vel.x; msg.vy = my_vel.y; msg.vz = my_vel.z;
        msg.timestamp = now().seconds();
        for (auto& [id, tracker] : trackers_) msg.peer_connected[id] = tracker.connected;
        if (leader_id_ == my_id_) {
            auto ref = flight_pattern_ == "line"
                ? line_reference(now().seconds(), line_phase_offset_)
                : loiter_reference(now().seconds(), loiter_phase_offset_);
            msg.ref_vx = ref.vel.x;
            msg.ref_vy = ref.vel.y;
        }
        pub_->publish(msg);

        if (!offboard_ready_.load()) return;

        if (my_pos.z > MAX_ALT_HARD_M) {
            // Hard backstop on actual altitude, independent of and in
            // addition to clamp_to_geofence() on *targets*: catches a bug
            // in the control law itself steering the vehicle out of bounds,
            // not just a bug in where it's aiming.
            RCLCPP_ERROR(get_logger(), "[%s] Altitude %.1fm over hard limit -- landing.",
                         my_name_.c_str(), my_pos.z);
            offboard_ready_.store(false);
            if (action_) action_->land();
            return;
        }
        if (std::hypot(my_pos.x, my_pos.y) > MAX_GEOFENCE_HARD_M) {
            // Same idea, horizontally: clamp_to_geofence() only bounds
            // where a target is aimed, not how far a drone already
            // chasing a bad one (e.g. stale/isolated during a network
            // split) can travel before catching up to a corrected target.
            RCLCPP_ERROR(get_logger(), "[%s] %.1fm from origin, over hard geofence -- landing.",
                         my_name_.c_str(), std::hypot(my_pos.x, my_pos.y));
            offboard_ready_.store(false);
            if (action_) action_->land();
            return;
        }

        Vec3 desired = compute_role_velocity(my_pos);
        std::vector<Vec3> peer_positions;
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected) peer_positions.push_back(tracker.pos);
        }
        Vec3 repulsion = separation_repulsion(my_pos, peer_positions);
        double repulsion_mag = std::sqrt(
            repulsion.x * repulsion.x + repulsion.y * repulsion.y + repulsion.z * repulsion.z);

        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& p : peer_positions) {
            min_dist = std::min(min_dist, distance_m(my_pos.x, my_pos.y, my_pos.z, p.x, p.y, p.z));
        }
        if (min_dist < SAFE_DIST_M) {
            RCLCPP_WARN(get_logger(), "[%s] SEPARATION: %.2fm from a peer, inside the %.1fm floor",
                        my_name_.c_str(), min_dist, SAFE_DIST_M);
            ++close_encounter_ticks_;
        } else {
            close_encounter_ticks_ = 0;
        }

        // A momentary close call and a *chronic* one need different
        // responses. Live testing traced ~half of all flights to a hard
        // geofence trigger (40m+ drift) with no single dramatic cause --
        // instead, a peer staying inside the 1m floor for minutes at a
        // time kept the priority blend below permanently diverting *some*
        // of the speed budget away from pure escape and into
        // formation-seeking, which (if that seeking is itself part of
        // what's causing the conflict) never lets the gap fully close --
        // just enough net bias, sustained long enough, to add up to a
        // large drift with no single moment that looks like a bug. Past
        // CHRONIC_TICKS_THRESHOLD, drop formation-seeking entirely and
        // spend the whole speed budget on escaping -- once clear,
        // close_encounter_ticks_ resets and normal blending resumes.
        bool chronic = close_encounter_ticks_ > CHRONIC_TICKS_THRESHOLD;
        if (chronic && close_encounter_ticks_ % 20 == 0) {
            RCLCPP_ERROR(get_logger(),
                "[%s] CHRONIC proximity for %.1fs -- pure escape, formation-seeking suspended",
                my_name_.c_str(), close_encounter_ticks_ * CONTROL_PERIOD_S);
        }

        // Priority blend, not a plain vector sum: as repulsion strengthens,
        // it takes progressively more of the speed budget from the
        // formation-seeking component instead of the two being added
        // together and the final clamp_speed() diluting whichever
        // direction loses out. A live test showed the old additive
        // approach let a strong seek command water down avoidance right
        // when a peer was closest -- exactly the wrong moment for that.
        double repulsion_weight = chronic ? 1.0 : std::clamp(repulsion_mag / MAX_SPEED_M_S, 0.0, 1.0);
        Vec3 blended{
            desired.x * (1.0 - repulsion_weight) + repulsion.x,
            desired.y * (1.0 - repulsion_weight) + repulsion.y,
            desired.z * (1.0 - repulsion_weight) + repulsion.z,
        };
        Vec3 command = clamp_speed(blended, MAX_SPEED_M_S);

        // NED = (north, east, down); command is stored (east, north, up).
        auto res = offboard_->set_velocity_ned(
            {static_cast<float>(command.y), static_cast<float>(command.x),
             static_cast<float>(-command.z), 0.0f});
        if (res != mavsdk::Offboard::Result::Success) {
            handle_offboard_failure();
        }
    }

    // Leader: pursue the scripted loiter reference (feedforward + feedback).
    // Follower: hold a triangle-formation offset on the live leader.
    // Neither role known yet, or leader's position not seen yet: hover.
    Vec3 compute_role_velocity(const Vec3& my_pos) {
        if (leader_id_ == my_id_) {
            auto ref = flight_pattern_ == "line"
                ? line_reference(now().seconds(), line_phase_offset_)
                : loiter_reference(now().seconds(), loiter_phase_offset_);
            ref.pos = clamp_to_geofence(ref.pos);
            return seek_velocity(ref.pos, my_pos, ref.vel);
        }
        if (leader_id_ == NO_LEADER || !trackers_[leader_id_].has_seen) {
            return Vec3{}; // hover in place, nothing to follow yet
        }
        const auto& leader = trackers_[leader_id_];
        // Threshold for "trust this velocity sample enough to update
        // heading" -- was 0.2 (fine for the old constant 1.0 m/s
        // triangle-wave line, which was below 0.2 only for a literal
        // instant at each reversal). The sinusoidal line/loiter spend a
        // real fraction of each cycle below that near their slow points
        // (cos(phase) < 0.2 for ~13% of each quarter-cycle at peak speed
        // 1.0 m/s), so 0.2 was freezing the heading estimate for seconds
        // at a time near every turnaround -- a live test traced recurring
        // SEPARATION bursts throughout an otherwise-smooth flight, and an
        // eventual 40m+ excursion caught only by the hard geofence, back
        // to exactly this: a follower holding a stale heading while the
        // leader's true direction kept changing under it. Lower epsilon
        // trusts the velocity sample down near where noise alone would
        // matter, not before.
        // Use the leader's *analytical* reference velocity (ref_vel, what
        // its scripted path says it should be doing right now) rather than
        // its telemetry-derived vel: vel carries the leader's own tracking
        // lag and real MAVSDK noise, which only gets worse riding through
        // a second seek loop on top. ref_vel is exactly what the leader is
        // trying to fly, clean, and it's what actually flips sign at the
        // sinusoid's zero-crossings without the noise that made the old
        // 0.2 freeze-threshold necessary in the first place.
        double leader_speed = std::hypot(leader.ref_vel.x, leader.ref_vel.y);
        if (leader_speed > 0.03) leader_heading_rad_ = std::atan2(leader.ref_vel.x, leader.ref_vel.y);

        double lateral = formation_side_is_left() ? FORMATION_HALF_WIDTH_M : -FORMATION_HALF_WIDTH_M;
        Vec3 offset = rotate_formation_offset(-FORMATION_TRAIL_M, lateral, leader_heading_rad_, 0.0);
        Vec3 target = clamp_to_geofence(Vec3{leader.pos.x + offset.x, leader.pos.y + offset.y, leader.pos.z});
        // Feedforward on the leader's reference velocity, not just
        // proportional feedback on the position gap: a follower purely
        // chasing the leader's *position* can never close the gap once
        // behind, since both are capped at the same MAX_SPEED_M_S (a
        // losing tail chase). Matching the leader's intended velocity and
        // correcting the offset error on top of that is standard
        // formation-control practice, and is what makes catching up to /
        // holding station on a moving leader possible.
        return seek_velocity(target, my_pos, leader.ref_vel);
    }

    // Deterministic slot assignment among the (up to) 2 non-leader drones:
    // the smaller surviving id takes the left slot.
    bool formation_side_is_left() {
        std::vector<int> followers;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (i != leader_id_) followers.push_back(i);
        }
        std::sort(followers.begin(), followers.end());
        return !followers.empty() && followers.front() == my_id_;
    }

    void handle_offboard_failure() {
        if (++offboard_failure_count_ < 5) return;
        RCLCPP_ERROR(get_logger(), "[%s] Offboard setpoint rejected repeatedly -- landing.", my_name_.c_str());
        offboard_ready_.store(false);
        if (action_) action_->land();
        offboard_failure_count_ = 0;
    }

    void on_peer_status(int peer, SwarmStatus::SharedPtr msg) {
        double range = distance_m(my_x_.load(), my_y_.load(), my_z_.load(), msg->x, msg->y, msg->z);
        if (range > MAX_RANGE_M) return; // out of simulated radio range: same as not receiving anything

        auto& tracker = trackers_[peer];
        tracker.last_seen = now();
        tracker.has_seen = true;
        tracker.last_claimed_leader = msg->claimed_leader;
        tracker.pos = Vec3{msg->x, msg->y, msg->z};
        tracker.vel = Vec3{msg->vx, msg->vy, msg->vz};
        tracker.ref_vel = Vec3{msg->ref_vx, msg->ref_vy, 0.0};
        if (!tracker.connected) {
            tracker.connected = true;
            RCLCPP_INFO(get_logger(), "[%s] SUCCESS: signal detected from drone_%d",
                        my_name_.c_str(), peer);
        }
    }

    void check_and_elect() {
        rclcpp::Time now_t = now();
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected && tracker.has_seen &&
                (now_t - tracker.last_seen).seconds() > TIMEOUT_S) {
                tracker.connected = false;
                RCLCPP_INFO(get_logger(), "[%s] Signal lost with drone_%d!", my_name_.c_str(), id);
            }
        }
        elect_leader();
    }

    // Sticky, majority-vote election: sticky leadership (a leader that
    // comes back doesn't reclaim the role -- it defers to whoever took
    // over) and never inventing a new belief while completely alone (that
    // first UDP version, tested end to end, is what caught and fixed an
    // infinite-oscillation bug -- see swarm_node.cpp's comments).
    //
    // Earlier version of this adopted the first differing `claimed_leader`
    // gossiped by any peer, in ID order. That broke when the old leader
    // reconnects: right after reconnecting, it and its peers are *each*
    // relaying whatever they believed a moment ago (the ex-leader's belief
    // was frozen at itself the whole time it was alone), and depending on
    // which stale message happens to arrive first, the group could
    // re-adopt the ex-leader by accident. A majority vote among everyone
    // currently alive (self included) fixes that: the 2 drones that
    // already agree on the real leader always outvote the 1 reconnecting
    // drone still relaying its stale belief, regardless of message timing.
    void elect_leader() {
        std::vector<int> alive = {my_id_};
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected) alive.push_back(id);
        }
        if (alive.size() == 1) return;

        std::map<int, int> votes;
        // My own current belief is one vote, but only if it's still someone
        // alive -- otherwise it's a stale belief about to be corrected, and
        // should carry no more weight than a stale peer claim would.
        if (leader_id_ != NO_LEADER &&
            std::find(alive.begin(), alive.end(), leader_id_) != alive.end()) {
            votes[leader_id_]++;
        }
        for (int id : alive) {
            if (id == my_id_) continue;
            int claim = trackers_[id].last_claimed_leader;
            if (claim >= 0 && std::find(alive.begin(), alive.end(), claim) != alive.end()) {
                votes[claim]++;
            }
        }

        int winner = NO_LEADER, winner_votes = 0, tied_with = 0;
        for (auto& [candidate, count] : votes) {
            if (count > winner_votes) {
                winner = candidate;
                winner_votes = count;
                tied_with = 0;
            } else if (count == winner_votes) {
                ++tied_with;
            }
        }
        // No usable votes yet, or a tie nobody breaks (e.g. fresh boot,
        // everyone still claims NO_LEADER): fall back to smallest alive ID.
        int new_leader = (winner == NO_LEADER || tied_with > 0)
            ? *std::min_element(alive.begin(), alive.end())
            : winner;

        int old_leader = leader_id_;
        if (new_leader == old_leader) return;
        leader_id_ = new_leader;
        if (new_leader == my_id_) {
            // Re-anchor the loiter circle's phase to wherever I actually am
            // right now, so I pick up the path from here instead of
            // dashing toward whatever point wall-clock time happens to put
            // on the circle -- matters both on first becoming leader and on
            // taking over mid-flight from a previous leader.
            double angle = std::atan2(my_y_.load() - LOITER_CENTER_NORTH_M,
                                       my_x_.load() - LOITER_CENTER_EAST_M);
            loiter_phase_offset_ = angle - LOITER_OMEGA * now().seconds();

            // Same idea for the line pattern: pick the phase whose position
            // matches where I am along the east axis right now, continuing
            // in whichever direction I'm already moving (or +east if
            // roughly stationary), so taking over doesn't teleport the
            // reference point to the opposite end of the line. asin gives
            // the phase in [-pi/2, pi/2] where cos (velocity) is already
            // >= 0 -- moving forward; its mirror pi-asin(frac) is the
            // point on the sine curve at the same position moving the
            // other way.
            //
            // A live test caught this handoff landing on a bad instant: an
            // unplanned takeover (leader lost to a network split) happened
            // moments after a near-collision, so the taking-over drone's
            // instantaneous velocity was dominated by collision-avoidance
            // repulsion, not its actual direction of travel along the
            // line -- a noisy sign right when it mattered, sending it
            // reanchoring onto the wrong leg and off toward the far end at
            // speed. A deadband means only a velocity clearly indicating
            // "moving the other way" flips the branch; noise near zero
            // defaults to continuing outbound instead of being trusted.
            constexpr double DIRECTION_DEADBAND_M_S = 0.15;
            double frac = std::clamp((my_x_.load() - LOITER_CENTER_EAST_M) / LINE_HALF_LENGTH_M, -1.0, 1.0);
            double base_phase = std::asin(frac);
            double phase = (my_vx_.load() >= -DIRECTION_DEADBAND_M_S) ? base_phase : (M_PI - base_phase);
            line_phase_offset_ = phase - LINE_OMEGA * now().seconds();
        }
        if (old_leader == NO_LEADER) {
            RCLCPP_INFO(get_logger(), "[%s] New leader: drone_%d", my_name_.c_str(), new_leader);
        } else {
            RCLCPP_INFO(get_logger(), "[%s] New leader: drone_%d (was drone_%d)",
                        my_name_.c_str(), new_leader, old_leader);
        }
    }

    int my_id_ = -1;
    std::string my_name_;
    int leader_id_ = NO_LEADER;
    double leader_heading_rad_ = 0.0; // last-known good heading, held when leader is ~stationary
    double loiter_phase_offset_ = 0.0; // recomputed each time I become leader; see loiter_reference()
    double line_phase_offset_ = 0.0; // same idea for line_reference(), see elect_leader()
    std::string flight_pattern_ = "loiter"; // "loiter" or "line", set from the flight_pattern param
    std::map<int, DroneTracker> trackers_;
    rclcpp::Publisher<SwarmStatus>::SharedPtr pub_;
    std::vector<rclcpp::Subscription<SwarmStatus>::SharedPtr> subs_;
    rclcpp::TimerBase::SharedPtr control_timer_, check_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    std::atomic<bool> offboard_ready_{false};
    std::atomic<int> last_logged_flight_mode_{-1}; // -1: nothing logged yet
    std::atomic<int> current_flight_mode_{-1}; // mavsdk::Telemetry::FlightMode, cast to int
    int offboard_failure_count_ = 0;
    int close_encounter_ticks_ = 0; // consecutive ticks with a peer inside SAFE_DIST_M

    // Written by the MAVSDK telemetry thread, read by the ROS timer thread.
    std::atomic<double> my_x_{0}, my_y_{0}, my_z_{0};
    std::atomic<double> my_vx_{0}, my_vy_{0}, my_vz_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmNode>());
    } catch (const std::exception&) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
