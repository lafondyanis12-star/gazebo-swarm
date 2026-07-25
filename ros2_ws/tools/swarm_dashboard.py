#!/usr/bin/env python3
# Live terminal dashboard for the 3 drones: real position, who each one
# thinks the leader is, and -- per peer -- both the live geometric range
# (computed here from raw positions) and whether that peer is actually
# CONNECTED (the drone's own belief, published in peer_connected --
# message-timeout based, same state the election uses). These can
# disagree: a peer well within radio range can still show DISCONNECTED if
# its updates stopped arriving for another reason (a real in-flight
# failure scenario, not just "too far away"), and conversely a peer just
# past MAX_RANGE_M stays CONNECTED for a few seconds until TIMEOUT_S
# elapses (hysteresis, not instantaneous). Run in a second terminal while
# the swarm is up (./launch_swarm.sh in the first one) -- see
# watch_swarm.sh for the one-command version.

import math
import sys

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import SwarmStatus

NUM_DRONES = 3
MAX_RANGE_M = 10.0  # must match swarm_node.cpp
SAFE_DIST_M = 1.0  # must match SAFE_DIST_M in swarm_node.cpp
REFRESH_S = 0.3

GREEN = '\x1b[32m'
RED = '\x1b[31m'
BOLD = '\x1b[1m'
RESET = '\x1b[0m'


class DroneRow:

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.claimed_leader = -1
        self.peer_connected = [False] * NUM_DRONES
        self.has_data = False


def distance(a, b):
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


class SwarmDashboard(Node):

    def __init__(self):
        super().__init__('swarm_dashboard')
        self.rows = {i: DroneRow() for i in range(NUM_DRONES)}
        for i in range(NUM_DRONES):
            self.create_subscription(
                SwarmStatus, f'/drone_{i}/swarm_status',
                self._make_callback(i), 10)
        self.create_timer(REFRESH_S, self._render)

    def _make_callback(self, drone_id):
        def callback(msg):
            row = self.rows[drone_id]
            row.x, row.y, row.z = msg.x, msg.y, msg.z
            row.claimed_leader = msg.claimed_leader
            row.peer_connected = list(msg.peer_connected)
            row.has_data = True
        return callback

    def _peer_link_txt(self, row, i, j):
        if not self.rows[j].has_data:
            return f'drone_{j} {"no data":>9}'
        dist = distance(row, self.rows[j])
        connected = row.peer_connected[j]
        status = f'{GREEN}CONNECTED{RESET}' if connected else f'{RED}{BOLD}DISCONNECTED{RESET}'
        # Flag the mismatch the column exists for: within range but the
        # drone itself doesn't currently believe it's connected.
        flag = f' {RED}{BOLD}(in range!){RESET}' if (dist <= MAX_RANGE_M and not connected) else ''
        return f'drone_{j} {dist:>6.2f}m {status}{flag}'

    def _render(self):
        lines = [
            '\x1b[H\x1b[J',  # clear screen, cursor home
            'SWARM DASHBOARD  (ctrl-c to quit)',
            f'radio range: {MAX_RANGE_M:.0f}m   min separation: {SAFE_DIST_M:.0f}m',
            '-' * 96,
            f'{"DRONE":<8}{"X":>7}{"Y":>7}{"Z":>7}{"LEADER":>10}{"DIST-LEAD":>11}   PEER RANGE / LINK STATUS',
        ]
        for i in range(NUM_DRONES):
            row = self.rows[i]
            if not row.has_data:
                lines.append(f'drone_{i:<2}{"no data yet":>31}')
                continue

            leader = f'drone_{row.claimed_leader}' if row.claimed_leader >= 0 else '-'
            marker = '  *' if row.claimed_leader == i else ''
            if row.claimed_leader >= 0 and row.claimed_leader != i and self.rows[row.claimed_leader].has_data:
                dist_lead = f'{distance(row, self.rows[row.claimed_leader]):>10.2f}'
            else:
                dist_lead = f'{"-":>10}'
            peer_links = '   '.join(
                self._peer_link_txt(row, i, j) for j in range(NUM_DRONES) if j != i
            )
            lines.append(
                f'drone_{i:<2}{row.x:>7.1f}{row.y:>7.1f}{row.z:>7.1f}'
                f'{leader:>10}{dist_lead}   {peer_links}{marker}'
            )

        live_rows = [self.rows[i] for i in range(NUM_DRONES) if self.rows[i].has_data]
        if len(live_rows) >= 2:
            pair_dists = [
                distance(live_rows[a], live_rows[b])
                for a in range(len(live_rows)) for b in range(a + 1, len(live_rows))
            ]
            min_dist = min(pair_dists)
            min_txt = (f'{RED}{BOLD}{min_dist:.2f}m -- COLLISION RISK{RESET}'
                       if min_dist < SAFE_DIST_M else f'{GREEN}{min_dist:.2f}m{RESET}')
            lines.append('-' * 78)
            lines.append(f'min pairwise distance: {min_txt}')

        lines.append('-' * 96)
        lines.append('* believes it is the leader  --  range is live geometric distance;')
        lines.append('CONNECTED/DISCONNECTED is the drone\'s own belief (message-timeout based, not just distance)')
        lines.append('(in range!) flags a peer that\'s geometrically close but still shows disconnected -- a real link problem, not distance')
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = SwarmDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
