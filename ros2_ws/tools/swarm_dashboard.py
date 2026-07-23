#!/usr/bin/env python3
# Live terminal dashboard for the 3 drones: real position, who each one
# thinks the leader is, and -- computed the same way swarm_node.cpp does it
# (pairwise distance vs MAX_RANGE_M) -- which peers it can actually still
# hear. A drone kept broadcasting after being moved out of range still
# shows up on its own topic, so "in range" has to be computed here rather
# than read off message freshness. Run in a second terminal while the
# swarm is up (./launch_swarm.sh in the first one) -- see watch_swarm.sh
# for the one-command version.

import math
import sys

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import SwarmStatus

NUM_DRONES = 3
MAX_RANGE_M = 6.0  # must match swarm_node.cpp
REFRESH_S = 0.3

GREEN = '\x1b[32m'
RED = '\x1b[31m'
BOLD = '\x1b[1m'
RESET = '\x1b[0m'


class DroneRow:

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.claimed_leader = -1
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
            row.has_data = True
        return callback

    def _render(self):
        lines = [
            '\x1b[H\x1b[J',  # clear screen, cursor home
            'SWARM DASHBOARD  (ctrl-c to quit)',
            f'radio range: {MAX_RANGE_M:.0f}m',
            '-' * 66,
            f'{"DRONE":<8}{"X":>7}{"Y":>7}{"Z":>7}{"LEADER":>10}   IN RANGE OF',
        ]
        for i in range(NUM_DRONES):
            row = self.rows[i]
            if not row.has_data:
                lines.append(f'drone_{i:<2}{"no data yet":>31}')
                continue

            in_range = [
                j for j in range(NUM_DRONES)
                if j != i and self.rows[j].has_data
                and distance(row, self.rows[j]) <= MAX_RANGE_M
            ]
            if in_range:
                peers = ', '.join(f'drone_{j}' for j in in_range)
                range_txt = f'{GREEN}{peers}{RESET}'
            else:
                range_txt = f'{RED}{BOLD}ISOLATED{RESET}'

            leader = f'drone_{row.claimed_leader}' if row.claimed_leader >= 0 else '-'
            marker = '  *' if row.claimed_leader == i else ''
            lines.append(
                f'drone_{i:<2}{row.x:>7.1f}{row.y:>7.1f}{row.z:>7.1f}'
                f'{leader:>10}   {range_txt}{marker}'
            )
        lines.append('-' * 66)
        lines.append('* believes it is the leader  --  IN RANGE OF: <= 6m, live distance')
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
