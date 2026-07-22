# Equivalent of launch_all.sh's 3 terminal windows, but the ROS 2 way:
# one launch file starts all 3 drones, logs interleaved and prefixed with
# each node's name instead of needing separate terminal windows.
#
# ros2 launch swarm_comm swarm_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_comm',
            executable='swarm_node',
            name=f'drone_{i}',
            parameters=[{'id': i}],
            output='screen',
        )
        for i in range(3)
    ])
