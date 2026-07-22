// Each drone reads its own position via MAVSDK and sends it to the other 2
// over a small custom UDP mesh (one listening port per drone).
//
// Why not just listen to all 3 PX4 streams from every node? Tried it,
// doesn't work: PX4 sends to a fixed port (14540+id) and only one process
// can bind that port. Two nodes both trying to listen to drone_0 step on
// each other.
//
// ./swarm_node --id 0/1/2

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono;

namespace {

constexpr int MAX_DRONES = 3;
constexpr int PX4_BASE_PORT = 14540;      // instance i: mavsdk connects to udp://:14540+i
constexpr int SWARM_BASE_PORT = 16000;    // instance i: listens on the mesh on 16000+i
constexpr double BROADCAST_INTERVAL_S = 0.2;
constexpr double TIMEOUT_S = 3.0;
constexpr double MAX_RANGE_M = 6.0;       // simulated radio range
constexpr double METERS_PER_DEG_LAT = 111320.0;
constexpr int NO_LEADER = -1;             // "I don't know who's leader yet"

#pragma pack(push, 1)
struct SwarmPacket {
    int32_t sender_id;
    int32_t claimed_leader; // who the sender currently believes is leader
    double x, y, z;
    double timestamp;
};
#pragma pack(pop)
static_assert(sizeof(SwarmPacket) == 40, "packet size mismatch");

double local_distance_m(double lat_a, double lon_a, double alt_a, double lat_b, double lon_b,
                         double alt_b) {
    double meters_per_deg_lon = METERS_PER_DEG_LAT * std::cos(lat_a * M_PI / 180.0);
    double dy = (lat_a - lat_b) * METERS_PER_DEG_LAT;
    double dx = (lon_a - lon_b) * meters_per_deg_lon;
    double dz = alt_a - alt_b;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double now_s() {
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

struct DroneTracker {
    double last_seen = -1.0;
    bool connected = false;
    int last_claimed_leader = NO_LEADER; // last leader_id this peer said it believed in
};

struct SharedState {
    std::mutex mutex;
    double my_x = 0.0, my_y = 0.0, my_z = 0.0;
    std::map<int, DroneTracker> trackers;
    int leader_id = NO_LEADER; // learned from peers or elected below, not assumed
};

// Continuously recomputes who should be leader, not just when the current
// leader drops -- otherwise an isolated drone that elected itself never
// hands leadership back (split-brain). Call with state.mutex held.
//
// Leadership is sticky: as long as the current leader is still alive, it
// keeps the role, even if a drone with a lower ID reappears. A leader that
// drops out and comes back doesn't get to instantly reclaim it -- it just
// rejoins as an ordinary follower, the same spot the new leader was in
// before taking over. If the leader genuinely disappears, the normal rule
// kicks back in (lowest surviving ID), so a former leader can end up
// leading again later if it's really its turn.
//
// A drone that just restarted (or reconnected) has no memory of any of
// that, so it learns the current leader from `claimed_leader`, gossiped in
// every packet: if a reachable peer already believes in a leader that's
// still alive and different from mine, I defer to it.
//
// First version of this trusted *any* differing claim from *any* reachable
// peer unconditionally, every tick -- including from a peer that only
// believes in itself because it was isolated a moment ago. Two drones each
// convinced they're the rightful leader would then flip-flop forever,
// each "correcting" the other back and forth. Fix: never invent a new
// belief (self-elect or otherwise) while completely alone -- with no one
// around to notice, that's more likely to be me getting cut off than a
// real failure. A lone drone just keeps repeating its last known belief
// (possibly still NO_LEADER) until it can see someone again, so it never
// broadcasts a bogus claim in the first place.
void elect_leader(SharedState& state, int my_id, const std::string& my_name) {
    std::vector<int> alive = {my_id};
    for (auto& [id, tracker] : state.trackers) {
        if (tracker.connected) alive.push_back(id);
    }
    if (alive.size() == 1) return; // no one to corroborate anything with

    for (int id : alive) {
        if (id == my_id) continue;
        int claim = state.trackers[id].last_claimed_leader;
        if (claim < 0 || claim == state.leader_id) continue;
        if (std::find(alive.begin(), alive.end(), claim) != alive.end()) {
            std::cout << "[" << my_name << "] Adopting drone_" << claim
                      << " as leader (learned from drone_" << id << ")\n";
            state.leader_id = claim;
            return;
        }
    }

    if (state.leader_id != NO_LEADER &&
        std::find(alive.begin(), alive.end(), state.leader_id) != alive.end()) {
        return;
    }

    int old_leader = state.leader_id;
    int new_leader = *std::min_element(alive.begin(), alive.end());
    if (new_leader == old_leader) return;
    state.leader_id = new_leader;
    if (old_leader == NO_LEADER) {
        std::cout << "[" << my_name << "] New leader: drone_" << new_leader << "\n";
    } else {
        std::cout << "[" << my_name << "] New leader: drone_" << new_leader << " (drone_"
                  << old_leader << " unreachable)\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    int my_id = -1;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--id" && i + 1 < argc) {
            my_id = std::stoi(argv[i + 1]);
        }
    }
    if (my_id < 0 || my_id >= MAX_DRONES) {
        std::cerr << "Usage: " << argv[0] << " --id 0|1|2\n";
        return 1;
    }

    std::cout.setf(std::ios::unitbuf); // flush every line, otherwise nothing shows
                                        // up until the process exits

    std::string my_name = "drone_" + std::to_string(my_id);
    SharedState state;
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (i != my_id) state.trackers[i] = DroneTracker{};
    }

    std::cout << "[" << my_name << "] Starting (" << (my_id == 0 ? "LEADER" : "FOLLOWER")
              << ").\n";

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    if (mavsdk.add_any_connection("udpin://0.0.0.0:" + std::to_string(PX4_BASE_PORT + my_id)) !=
        ConnectionResult::Success) {
        std::cerr << "[" << my_name << "] MAVSDK connection failed\n";
        return 1;
    }

    auto system = mavsdk.first_autopilot(10.0);
    if (!system) {
        std::cerr << "[" << my_name << "] No PX4 system found\n";
        return 1;
    }
    std::cout << "[" << my_name << "] Connecting to its own PX4 instance (port "
              << (PX4_BASE_PORT + my_id) << ")...\n";

    Telemetry telemetry{system.value()};
    telemetry.subscribe_position([&](Telemetry::Position position) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.my_x = position.latitude_deg;
        state.my_y = position.longitude_deg;
        state.my_z = position.relative_altitude_m;
    });

    int recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    recv_addr.sin_port = htons(SWARM_BASE_PORT + my_id);
    if (bind(recv_sock, reinterpret_cast<sockaddr*>(&recv_addr), sizeof(recv_addr)) < 0) {
        std::cerr << "[" << my_name << "] UDP bind failed\n";
        return 1;
    }

    int send_sock = socket(AF_INET, SOCK_DGRAM, 0);

    std::thread receiver([&]() {
        while (true) {
            SwarmPacket packet{};
            sockaddr_in from{};
            socklen_t from_len = sizeof(from);
            ssize_t n = recvfrom(recv_sock, &packet, sizeof(packet), 0,
                                  reinterpret_cast<sockaddr*>(&from), &from_len);
            if (n != static_cast<ssize_t>(sizeof(packet)) || packet.sender_id == my_id) continue;

            std::lock_guard<std::mutex> lock(state.mutex);
            auto it = state.trackers.find(packet.sender_id);
            if (it == state.trackers.end()) continue;

            // simulated radio range: a signal received from too far away is ignored
            double dist = local_distance_m(state.my_x, state.my_y, state.my_z, packet.x,
                                            packet.y, packet.z);
            if (dist > MAX_RANGE_M) continue;

            it->second.last_seen = now_s();
            it->second.last_claimed_leader = packet.claimed_leader;
            if (!it->second.connected) {
                it->second.connected = true;
                std::cout << "[" << my_name << "] SUCCESS: signal detected from drone_"
                          << packet.sender_id << "\n";
            }
        }
    });

    std::thread broadcaster([&]() {
        while (true) {
            SwarmPacket packet{};
            {
                std::lock_guard<std::mutex> lock(state.mutex);
                packet.sender_id = my_id;
                packet.claimed_leader = state.leader_id;
                packet.x = state.my_x;
                packet.y = state.my_y;
                packet.z = state.my_z;
            }
            packet.timestamp = now_s();
            for (int peer = 0; peer < MAX_DRONES; ++peer) {
                if (peer == my_id) continue;
                sockaddr_in dest{};
                dest.sin_family = AF_INET;
                dest.sin_addr.s_addr = inet_addr("127.0.0.1");
                dest.sin_port = htons(SWARM_BASE_PORT + peer);
                sendto(send_sock, &packet, sizeof(packet), 0, reinterpret_cast<sockaddr*>(&dest),
                       sizeof(dest));
            }
            std::this_thread::sleep_for(duration<double>(BROADCAST_INTERVAL_S));
        }
    });

    std::thread checker([&]() {
        while (true) {
            std::this_thread::sleep_for(milliseconds(500));
            double now = now_s();
            std::lock_guard<std::mutex> lock(state.mutex);
            for (auto& [id, tracker] : state.trackers) {
                if (tracker.connected && tracker.last_seen >= 0 &&
                    (now - tracker.last_seen) > TIMEOUT_S) {
                    tracker.connected = false;
                    std::cout << "[" << my_name << "] Signal lost with drone_" << id
                              << "!\n";
                }
            }
            elect_leader(state, my_id, my_name);
        }
    });

    receiver.join();
    broadcaster.join();
    checker.join();
    return 0;
}
