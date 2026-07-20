// Chaque drone lit sa propre position via MAVSDK et la balance aux 2 autres
// par un petit reseau UDP maison (un port d'ecoute par drone).
//
// Pourquoi pas juste ecouter les 3 flux PX4 depuis chaque noeud ? Teste,
// ca ne marche pas : PX4 envoie vers un port fixe (14540+id) et un seul
// process peut bind ce port. Deux noeuds qui veulent ecouter drone_0 en
// meme temps se marchent dessus.
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
constexpr int PX4_BASE_PORT = 14540;      // instance i : mavsdk se connecte a udp://:14540+i
constexpr int SWARM_BASE_PORT = 16000;    // instance i : ecoute le maillage sur 16000+i
constexpr double BROADCAST_INTERVAL_S = 0.2;
constexpr double TIMEOUT_S = 3.0;
constexpr double MAX_RANGE_M = 6.0;       // portee radio simulee
constexpr double METERS_PER_DEG_LAT = 111320.0;

#pragma pack(push, 1)
struct SwarmPacket {
    int32_t sender_id;
    double x, y, z;
    double timestamp;
};
#pragma pack(pop)
static_assert(sizeof(SwarmPacket) == 36, "packet size mismatch");

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
};

struct SharedState {
    std::mutex mutex;
    double my_x = 0.0, my_y = 0.0, my_z = 0.0;
    std::map<int, DroneTracker> trackers;
    int leader_id = 0; // drone_0 est le leader initial
};

// recalcule en continu qui devrait etre leader, pas seulement quand le
// leader courant tombe -- sinon un drone isole qui s'est elu tout seul
// ne rend jamais la main (split-brain). A appeler avec state.mutex tenu.
void elect_leader(SharedState& state, int my_id, const std::string& my_name) {
    std::vector<int> alive = {my_id};
    for (auto& [id, tracker] : state.trackers) {
        if (tracker.connected) alive.push_back(id);
    }
    int new_leader = *std::min_element(alive.begin(), alive.end());
    if (new_leader == state.leader_id) return;

    int old_leader = state.leader_id;
    state.leader_id = new_leader;
    bool old_still_alive = std::find(alive.begin(), alive.end(), old_leader) != alive.end();
    if (!old_still_alive) {
        std::cout << "[" << my_name << "] Nouveau leader : drone_" << new_leader << " (drone_"
                  << old_leader << " injoignable)\n";
    } else {
        std::cout << "[" << my_name << "] Nouveau leader : drone_" << new_leader << " (drone_"
                  << old_leader << " reste joignable mais drone_" << new_leader
                  << " a priorite)\n";
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

    std::cout.setf(std::ios::unitbuf); // flush a chaque ligne, sinon rien ne s'affiche
                                        // tant que le process ne se termine pas

    std::string my_name = "drone_" + std::to_string(my_id);
    SharedState state;
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (i != my_id) state.trackers[i] = DroneTracker{};
    }

    std::cout << "[" << my_name << "] Demarrage (" << (my_id == 0 ? "LEADER" : "SUIVEUR")
              << ").\n";

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    if (mavsdk.add_any_connection("udpin://0.0.0.0:" + std::to_string(PX4_BASE_PORT + my_id)) !=
        ConnectionResult::Success) {
        std::cerr << "[" << my_name << "] Echec de connexion MAVSDK\n";
        return 1;
    }

    auto system = mavsdk.first_autopilot(10.0);
    if (!system) {
        std::cerr << "[" << my_name << "] Aucun systeme PX4 trouve\n";
        return 1;
    }
    std::cout << "[" << my_name << "] Connexion a sa propre instance PX4 (port "
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
        std::cerr << "[" << my_name << "] Echec du bind UDP\n";
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

            // portee radio simulee : un signal recu de trop loin est ignore
            double dist = local_distance_m(state.my_x, state.my_y, state.my_z, packet.x,
                                            packet.y, packet.z);
            if (dist > MAX_RANGE_M) continue;

            it->second.last_seen = now_s();
            if (!it->second.connected) {
                it->second.connected = true;
                std::cout << "[" << my_name << "] SUCCES : signal detecte de drone_"
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
                    std::cout << "[" << my_name << "] Perte de signal avec drone_" << id
                              << " !\n";
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
