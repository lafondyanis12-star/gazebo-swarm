# Equivalent of launch_all.sh's 3 terminal windows, but the ROS 2 way:
# one launch file starts all 3 drones, logs interleaved and prefixed with
# each node's name instead of needing separate terminal windows.
#
# ros2 launch swarm_comm swarm_launch.py
# ros2 launch swarm_comm swarm_launch.py flight_pattern:=line   # straight-line leader path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    flight_pattern_arg = DeclareLaunchArgument(
        'flight_pattern', default_value='loiter',
        description="Leader's scripted path: 'loiter' (circular) or 'line' (straight, back and forth)",
    )
    flight_pattern = LaunchConfiguration('flight_pattern')
    return LaunchDescription([
        flight_pattern_arg,
        *(
            Node(
                package='swarm_comm',
                executable='swarm_node',
                name=f'drone_{i}',
                parameters=[{'id': i, 'flight_pattern': flight_pattern}],
                output='screen',
            )
            for i in range(3)
        ),
    ])
