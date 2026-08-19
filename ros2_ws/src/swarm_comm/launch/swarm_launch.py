# Fichier de lancement ROS2 de l'essaim : demarre les 3 noeuds swarm_node
# (drone_0, drone_1, drone_2) qui implementent la communication, l'election
# de leader et le controle de formation. Equivalent des 3 fenetres de
# terminal de launch_all.sh, mais a la maniere ROS2 : un seul fichier de
# lancement demarre les 3 drones, avec les logs entrelaces et prefixes par
# le nom de chaque noeud, au lieu de fenetres de terminal separees.
#
# ros2 launch swarm_comm swarm_launch.py
# ros2 launch swarm_comm swarm_launch.py flight_pattern:=line   # trajectoire du leader en ligne droite

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Construit la description de lancement ROS2 pour les 3 drones de l'essaim.

    Declare l'argument de lancement `flight_pattern` (trajectoire du
    leader : 'loiter' pour un cercle, 'line' pour un aller-retour en ligne
    droite) puis cree un noeud swarm_node par drone (id 0, 1, 2), chacun
    recevant ce meme parametre.
    """
    flight_pattern_arg = DeclareLaunchArgument(
        'flight_pattern', default_value='loiter',
        description="Leader's scripted path: 'loiter' (circular) or 'line' (straight, back and forth)",
    )
    flight_pattern = LaunchConfiguration('flight_pattern')
    return LaunchDescription([
        flight_pattern_arg,
        *(
            # Un noeud swarm_node par drone, nomme drone_0/drone_1/drone_2,
            # avec son id et la trajectoire (flight_pattern) partagee.
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
