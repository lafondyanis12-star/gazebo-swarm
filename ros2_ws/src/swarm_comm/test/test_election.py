# Automates the 3 scenarios that were previously checked by hand: launch the
# swarm, read `docker logs`, run `ros2 param set` from a second terminal,
# read the logs again. Same checks, same log lines, just scripted.
#
# ros2 launch_test src/swarm_comm/test/test_election.py  (or via colcon test)

import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


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
        check=True, capture_output=True,
    )


class TestSwarmElection(unittest.TestCase):
    """
    Election scenarios, run in definition order against the same 3 nodes.

    test_1_, test_2_, ... sort alphabetically, mirroring how the manual
    session went: normal election first, then range disconnect, then full
    isolation.
    """

    def test_1_initial_election(self, proc_output, drone_0, drone_1, drone_2):
        for name, proc in (('drone_0', drone_0), ('drone_1', drone_1), ('drone_2', drone_2)):
            proc_output.assertWaitFor('New leader: drone_0', process=proc, timeout=10)

    def test_2_range_disconnect_and_reconnect(self, proc_output, drone_0, drone_1, drone_2):
        set_x('drone_2', 30.0)  # 30m > MAX_RANGE_M (6m)
        proc_output.assertWaitFor('Signal lost with drone_2!', process=drone_0, timeout=6)
        proc_output.assertWaitFor('Signal lost with drone_2!', process=drone_1, timeout=6)

        set_x('drone_2', 0.0)  # back in range
        signal_2 = 'SUCCESS: signal detected from drone_2'
        proc_output.assertWaitFor(signal_2, process=drone_0, timeout=6)
        proc_output.assertWaitFor(signal_2, process=drone_1, timeout=6)

    def test_3_total_isolation_freezes_leader_belief(self, proc_output, drone_0, drone_1, drone_2):
        # Scatter all 3 so every pairwise distance is > MAX_RANGE_M.
        set_x('drone_0', 0.0)
        set_x('drone_1', 100.0)
        set_x('drone_2', 200.0)

        for proc in (drone_0, drone_1, drone_2):
            proc_output.assertWaitFor('Signal lost with', process=proc, timeout=6)

        # Give the 500ms election timer several ticks to prove the point,
        # then confirm drone_1 and drone_2 did NOT invent themselves as
        # leader while completely alone -- the split-brain behavior the
        # `if (alive.size() == 1) return;` guard exists to prevent.
        time.sleep(2)
        for name, proc in (('drone_1', drone_1), ('drone_2', drone_2)):
            with self.assertRaises(AssertionError, msg=f'{name} elected itself while isolated'):
                proc_output.assertWaitFor(f'New leader: {name}', process=proc, timeout=1)

        # Bring them back together and confirm they reconverge cleanly.
        set_x('drone_1', 0.0)
        set_x('drone_2', 0.0)
        signal_from = 'SUCCESS: signal detected from drone_{}'
        proc_output.assertWaitFor(signal_from.format(2), process=drone_0, timeout=6)
        proc_output.assertWaitFor(signal_from.format(1), process=drone_0, timeout=6)
