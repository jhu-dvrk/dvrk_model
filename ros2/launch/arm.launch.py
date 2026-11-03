import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch import LaunchContext, LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument

def psm_instruments():
    yaml_file = os.path.join(
        get_package_share_directory('dvrk_model'),
        'urdf', 'common', 'instruments', 'instruments.yaml')
    with open(yaml_file, 'r') as stream:
        return yaml.safe_load(stream)['instruments']

def supported_psm_instruments_message():
    instruments = psm_instruments()
    return ', '.join(
        '{} ({})'.format(model, instruments[model]['name'])
        for model in sorted(instruments.keys()))

def create_simulated_robot(context):
    arm = LaunchConfiguration('arm').perform(context)
    generation = LaunchConfiguration('generation').perform(context)
    system_generation = 'Si' if generation == 'Virtual' else generation
    system_file = os.path.join(
        get_package_share_directory('dvrk_config'), 'system',
        'system-{}_{}_KIN_SIMULATED.json'.format(arm, system_generation))
    arguments = ['-j', system_file]

    if arm.startswith('PSM'):
        instrument = LaunchConfiguration('instrument').perform(context).strip()
        if instrument == '':
            instrument = '420006' if generation in ['Si', 'Virtual'] else '400006'
        if instrument not in psm_instruments():
            raise RuntimeError('Unsupported PSM instrument: {}'.format(instrument))
        # The tool index accepts a full model without a name via :MODEL.
        # Revision 1 retains the existing 420006 default (before geared jaws).
        arguments.extend(['--tool', '{}=:{}[1]'.format(arm, instrument)])

    return [Node(
        package='dvrk_robot',
        executable='dvrk_system',
        arguments=arguments,
        output='log',
    )]


def generate_launch_description():
    arm = LaunchConfiguration('arm')
    generation = LaunchConfiguration('generation')
    instrument = LaunchConfiguration('instrument', default='')
    endoscope = LaunchConfiguration('endoscope', default='')
    simulated = LaunchConfiguration('simulated', default = 'true')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    rate = LaunchConfiguration('rate', default = 50.0)  # Hz, default is 10 so we're increasing that a bit.

    # Virtual uses the Virtual URDF, but its simulated PSM system configuration
    # is the same as the Si configuration.
    system_generation = PythonExpression([
        "'Si' if '", generation, "' == 'Virtual' else '", generation, "'"
    ])
    rviz_generation = system_generation

    rviz_config_file = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'), 'rviz', rviz_generation, '']),
        '/', arm, '.rviz',
    ]

    # Declare nodes
    dvrk_node = OpaqueFunction(
        function=create_simulated_robot,
        condition=IfCondition(simulated),
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
        output = 'log',
    )

    ld = LaunchDescription([
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        DeclareLaunchArgument('arm'),
        DeclareLaunchArgument(
            'generation',
            choices=['Classic', 'Si', 'Virtual'],
            description='dVRK system generation'
        ),
        DeclareLaunchArgument(
            'instrument',
            default_value='',
            description='Six-digit PSM instrument model ID. Supported: {}'.format(
                supported_psm_instruments_message())),
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
