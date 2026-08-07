# Automates the 3 scenarios that were previously checked by hand: launch the
# swarm, read `docker logs`, run `ros2 param set` from a second terminal,
# read the logs again. Same checks, same log lines, just scripted.
#
# ros2 launch_test src/swarm_comm/test/test_election.py  (or via colcon test)

import os
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest

# colcon/ament's launch_testing (via ctest) strips DYLD_LIBRARY_PATH before
# exec'ing this test process -- the same mechanism documented in
# swarm_comm/CMakeLists.txt for swarm_node itself, but that fix (baked-in
# RPATH) only covers swarm_node's own binary, not the `ros2` CLI subprocesses
# spawned here. Without it, `ros2 topic echo`/`param set` can't dlopen
# swarm_msgs' typesupport .dylib and fail immediately with "invalid
# allocator" -- not a DDS discovery race, so retrying alone doesn't fix it.
_INSTALL_LIB_DIRS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install', pkg, 'lib')
    for pkg in ('swarm_msgs', 'swarm_comm')
]
_ROS2_ENV = {
    **os.environ,
    'DYLD_LIBRARY_PATH': ':'.join(
        [*_INSTALL_LIB_DIRS, os.environ.get('DYLD_LIBRARY_PATH', '')]),
}


@pytest.mark.launch_test
def generate_test_description():
    drones = {
        f'drone_{i}': launch_ros.actions.Node(
            package='swarm_comm',
            executable='swarm_node',
            name=f'drone_{i}',
            parameters=[{'id': i}],
            output='screen',
        )
        for i in range(3)
    }
    return launch.LaunchDescription([
        *drones.values(),
        launch_testing.actions.ReadyToTest(),
    ]), drones


def set_x(drone_name, value):
    subprocess.run(
        ['ros2', 'param', 'set', f'/{drone_name}', 'x', str(value)],
        check=True, capture_output=True, env=_ROS2_ENV,
    )


def get_claimed_leader(drone_name):
    # Retry with a short discovery grace period -- `ros2 topic echo --once`
    # can still fail if DDS discovery hasn't found the publisher yet, flaky
    # under heavy system load (see README "What's left").
    last_err = None
    for _ in range(5):
        try:
            out = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', f'/{drone_name}/swarm_status'],
                check=True, capture_output=True, text=True, timeout=10, env=_ROS2_ENV,
            ).stdout
            for line in out.splitlines():
                if line.startswith('claimed_leader:'):
                    return int(line.split(':')[1].strip())
            raise RuntimeError(f'no claimed_leader in output from {drone_name}: {out!r}')
        except subprocess.CalledProcessError as err:
            last_err = err
            time.sleep(1)
    raise last_err


class TestSwarmElection(unittest.TestCase):
    """
    Election scenarios, run in definition order against the same 3 nodes.

    test_1_, test_2_, ... sort alphabetically, mirroring how the manual
    session went: normal election first, then range disconnect, then full
    isolation.
    """

    def test_1_initial_election(self, proc_output, drone_0, drone_1, drone_2):
        # Which of the 3 wins is a timing race (each node picks the min ID
        # of *its own* alive set the instant its election timer fires, and
        # with near-simultaneous discovery there's no strict ordering
        # guarantee -- see elect_leader()'s comments in swarm_node.cpp).
        # What must hold is consensus, not a specific winner: all 3 settle
        # on the same leader.
        for name, proc in (('drone_0', drone_0), ('drone_1', drone_1), ('drone_2', drone_2)):
            proc_output.assertWaitFor('New leader: drone_', process=proc, timeout=10)
        leaders = {name: get_claimed_leader(name) for name in ('drone_0', 'drone_1', 'drone_2')}
        self.assertEqual(len(set(leaders.values())), 1, f'no consensus: {leaders}')

    def test_2_range_disconnect_and_reconnect(self, proc_output, drone_0, drone_1, drone_2):
        set_x('drone_2', 30.0)  # 30m > MAX_RANGE_M (10m)
        proc_output.assertWaitFor('Signal lost with drone_2!', process=drone_0, timeout=6)
        proc_output.assertWaitFor('Signal lost with drone_2!', process=drone_1, timeout=6)

        set_x('drone_2', 0.0)  # back in range
        signal_2 = 'SUCCESS: signal detected from drone_2'
        proc_output.assertWaitFor(signal_2, process=drone_0, timeout=6)
        proc_output.assertWaitFor(signal_2, process=drone_1, timeout=6)

    def test_3_total_isolation_freezes_leader_belief(self, proc_output, drone_0, drone_1, drone_2):
        # Re-check who's leading right now rather than trusting test_1's
        # result: test_2's disconnect/reconnect of drone_2 could in
        # principle have flipped leadership in between (e.g. if drone_2
        # itself had been leader) -- current, not remembered, state.
        current_leader = f'drone_{get_claimed_leader("drone_0")}'

        # Scatter all 3 so every pairwise distance is > MAX_RANGE_M.
        set_x('drone_0', 0.0)
        set_x('drone_1', 100.0)
        set_x('drone_2', 200.0)

        for proc in (drone_0, drone_1, drone_2):
            proc_output.assertWaitFor('Signal lost with', process=proc, timeout=6)

        # Give the 500ms election timer several ticks to prove the point,
        # then confirm the 2 non-leader drones did NOT invent themselves as
        # leader while completely alone -- the split-brain behavior the
        # `if (alive.size() == 1) return;` guard exists to prevent. (Which
        # drone leads is a timing race resolved in test_1 -- see its
        # comments -- so the followers to check are derived from that,
        # not hardcoded.)
        time.sleep(2)
        procs = {'drone_0': drone_0, 'drone_1': drone_1, 'drone_2': drone_2}
        followers = [(name, proc) for name, proc in procs.items() if name != current_leader]
        for name, proc in followers:
            with self.assertRaises(AssertionError, msg=f'{name} elected itself while isolated'):
                proc_output.assertWaitFor(f'New leader: {name}', process=proc, timeout=1)

        # Bring them back together and confirm they reconverge cleanly.
        set_x('drone_1', 0.0)
        set_x('drone_2', 0.0)
        signal_from = 'SUCCESS: signal detected from drone_{}'
        proc_output.assertWaitFor(signal_from.format(2), process=drone_0, timeout=6)
        proc_output.assertWaitFor(signal_from.format(1), process=drone_0, timeout=6)
