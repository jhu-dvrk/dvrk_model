import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch import LaunchContext, LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    arm = LaunchConfiguration('arm')
    generation = LaunchConfiguration('generation')
    simulated = LaunchConfiguration('simulated', default = 'true')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    rate = LaunchConfiguration('rate', default = 50.0)  # Hz, default is 10 so we're increasing that a bit.

    system_json = [
        PathJoinSubstitution([FindPackageShare('dvrk_config'), 'system', '']),
        '/system-', arm, '_', generation, '_KIN_SIMULATED.json',
    ]

    rviz_config_file = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'), 'rviz', generation, '']),
        '/', arm, '.rviz',
    ]

    # Declare nodes
    dvrk_node = Node(
        package = 'dvrk_robot',
        executable = 'dvrk_system',
        condition = IfCondition(simulated),
        arguments = ['-j', system_json],
        output = 'both',
    )

    publisher_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dvrk_model'),
                'launch',
                'arm_state_publishers.launch.py')),
        launch_arguments = {
            'arm': arm,
            'generation': generation,
            'use_sim_time': use_sim_time,
            'rate': rate,
            'suj': 'false'
        }.items()
    )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        arguments = ['-d', rviz_config_file],
        output = 'both',
    )

    ld = LaunchDescription([
        dvrk_node,
        publisher_nodes,
        rviz_node,
    ])

    return ld
