// ROS 2 port of ~/Documents/my_project/gazebo_swarm/swarm_node.cpp.
// Same detection/timeout/election logic (already validated there), but
// swapped from hand-rolled UDP sockets + threads to topics + a single-
// threaded executor -- rclcpp::spin() runs all callbacks (timers,
// subscriptions) one at a time on the same thread, so there's no need for
// the mutex the UDP version needed across its 3 threads.
//
// No PX4 yet: position is a fake, distinct value per drone, just to prove
// the 3 nodes see each other and agree on a leader over real ROS 2 topics.
//
// ros2 run swarm_comm swarm_node --ros-args -p id:=0/1/2

#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/swarm_status.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using swarm_msgs::msg::SwarmStatus;

namespace {

constexpr int MAX_DRONES = 3;
constexpr double TIMEOUT_S = 3.0;
constexpr int NO_LEADER = -1; // "I don't know who's leader yet"

struct DroneTracker {
    rclcpp::Time last_seen;
    bool has_seen = false;
    bool connected = false;
    int last_claimed_leader = NO_LEADER;
};

} // namespace

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

        broadcast_timer_ = create_wall_timer(200ms, [this]() { broadcast(); });
        check_timer_ = create_wall_timer(500ms, [this]() { check_and_elect(); });

        RCLCPP_INFO(get_logger(), "[%s] Starting.", my_name_.c_str());
    }

private:
    void broadcast() {
        SwarmStatus msg;
        msg.sender_id = my_id_;
        msg.claimed_leader = leader_id_;
        msg.x = my_id_ * 10.0; // fake position, no PX4 in this prototype
        msg.y = 0.0;
        msg.z = 0.0;
        msg.timestamp = now().seconds();
        pub_->publish(msg);
    }

    void on_peer_status(int peer, SwarmStatus::SharedPtr msg) {
        auto& tracker = trackers_[peer];
        tracker.last_seen = now();
        tracker.has_seen = true;
        tracker.last_claimed_leader = msg->claimed_leader;
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

    // Ported as-is from the UDP version's elect_leader(): sticky leadership
    // (a leader that comes back doesn't reclaim the role -- it defers to
    // whoever took over), learned from peers via `claimed_leader` gossiped
    // in every message, and never inventing a new belief while completely
    // alone (that first version, tested end to end, is what caught and
    // fixed an infinite-oscillation bug -- see swarm_node.cpp's comments).
    void elect_leader() {
        std::vector<int> alive = {my_id_};
        for (auto& [id, tracker] : trackers_) {
            if (tracker.connected) alive.push_back(id);
        }
        if (alive.size() == 1) return;

        for (int id : alive) {
            if (id == my_id_) continue;
            int claim = trackers_[id].last_claimed_leader;
            if (claim < 0 || claim == leader_id_) continue;
            if (std::find(alive.begin(), alive.end(), claim) != alive.end()) {
                RCLCPP_INFO(get_logger(), "[%s] Adopting drone_%d as leader (learned from drone_%d)",
                            my_name_.c_str(), claim, id);
                leader_id_ = claim;
                return;
            }
        }

        if (leader_id_ != NO_LEADER &&
            std::find(alive.begin(), alive.end(), leader_id_) != alive.end()) {
            return;
        }

        int old_leader = leader_id_;
        int new_leader = *std::min_element(alive.begin(), alive.end());
        if (new_leader == old_leader) return;
        leader_id_ = new_leader;
        if (old_leader == NO_LEADER) {
            RCLCPP_INFO(get_logger(), "[%s] New leader: drone_%d", my_name_.c_str(), new_leader);
        } else {
            RCLCPP_INFO(get_logger(), "[%s] New leader: drone_%d (drone_%d unreachable)",
                        my_name_.c_str(), new_leader, old_leader);
        }
    }

    int my_id_ = -1;
    std::string my_name_;
    int leader_id_ = NO_LEADER;
    std::map<int, DroneTracker> trackers_;
    rclcpp::Publisher<SwarmStatus>::SharedPtr pub_;
    std::vector<rclcpp::Subscription<SwarmStatus>::SharedPtr> subs_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_, check_timer_;
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
