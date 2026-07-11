import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch import LaunchContext, LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

SUPPORTED_PSM_INSTRUMENTS = [
    '400001', '400003', '400006', '400007', '400033', '400035',
    '400036', '400048', '400049', '400093', '400127', '400172',
    '400178', '400179', '400181', '400189', '400205', '400207',
    '400208', '400230', '400296',
    '420001', '420006', '420007', '420033', '420036', '420048',
    '420049', '420093', '420172', '420178', '420179', '420181',
    '420205', '420230', '420296', '420327',
]

def generate_launch_description():
    arm = LaunchConfiguration('arm')
    generation = LaunchConfiguration('generation')
    instrument = LaunchConfiguration('instrument', default='')
    endoscope = LaunchConfiguration('endoscope', default='')
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
            'instrument': instrument,
            'endoscope': endoscope,
            'use_sim_time': use_sim_time,
            'rate': rate,
            'suj': 'false',
            'show_rcm': LaunchConfiguration('show_rcm')
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
        DeclareLaunchArgument('arm'),
        DeclareLaunchArgument('generation'),
        DeclareLaunchArgument(
            'instrument',
            default_value='',
            description='PSM instrument model. Supported: {}'.format(
                ', '.join(SUPPORTED_PSM_INSTRUMENTS))),
        DeclareLaunchArgument('endoscope', default_value=''),
        DeclareLaunchArgument('simulated', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rate', default_value='50.0'),
        DeclareLaunchArgument('show_rcm', default_value='true'),
        dvrk_node,
        publisher_nodes,
        rviz_node,
    ])

    return ld
