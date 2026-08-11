import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchContext, LaunchDescription, Substitution
from typing import Text
import xacro

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

VALID_GENERATIONS = ['Classic', 'Si', 'Virtual']
VALID_ENDOSCOPES = ['Classic_SD_straight', 'Si_straight']


def psm_instruments():
    yaml_file = os.path.join(
        get_package_share_directory('dvrk_model'),
        'urdf', 'common', 'instruments', 'instruments.yaml')
    with open(yaml_file, 'r') as stream:
        return yaml.safe_load(stream)['instruments']


PSM_INSTRUMENTS = psm_instruments()
VALID_INSTRUMENTS = sorted(PSM_INSTRUMENTS.keys())
VALID_INSTRUMENT_OPTIONS = ', '.join(
    '{} ({})'.format(model, PSM_INSTRUMENTS[model]['name'])
    for model in VALID_INSTRUMENTS)


def valid_arm_names(package_share, generation):
    urdf_dir = os.path.join(package_share, 'urdf', generation)
    return sorted(
        os.path.splitext(os.path.splitext(name)[0])[0]
        for name in os.listdir(urdf_dir)
        if name.endswith('.urdf.xacro')
        and not name.endswith('_base.urdf.xacro')
        and not name.endswith('_base_virtual.urdf.xacro')
        and not name.endswith('_zero_check.urdf.xacro')
        and name not in ['SUJ.urdf.xacro']
    )


def valid_options_message(options):
    return ', '.join(options)


def require_choice(name, value, options, options_message=None):
    if value not in options:
        raise RuntimeError(
            "Invalid {} '{}'. Valid options: {}".format(
                name, value, options_message or valid_options_message(options)))


# Create a list of ROS topics that will provide all the joint states
# from the dVRK main node to the joint_state_publisher
class ArmSourceListSubstitution(Substitution):
    """Generate source list for a given arm"""

    def __init__(self,
                 arm: LaunchConfiguration):
        super().__init__()
        self.__arm = arm

    def describe(self) -> Text:
        return 'ArmSourceList({})'.format(self.__arm.describe())

    def perform(self, context: LaunchContext) -> Text:
        arm = self.__arm.perform(context)
        source_list = ['measured_js']
        if arm.startswith('PSM'):
            source_list.append('jaw/measured_js')
        elif arm.startswith('MTM'):
            source_list.append('gripper/measured_js')

        # launch_ros parses values from yaml, so source list needs to be yaml
        sources = ','.join(source for source in source_list)
        yaml =  '[{}]'.format(sources)
        return yaml


def create_robot_description(context):
    full_name = context.launch_configurations['arm']
    generation = context.launch_configurations['generation']
    require_choice('generation', generation, VALID_GENERATIONS)

    if full_name.startswith('MTM') and generation != 'Classic':
        raise RuntimeError(
            "MTM arms (MTML/MTMR) are only supported with generation:=Classic; "
            "generation '{}' is not supported for {}.".format(generation, full_name))

    dvrk_model_share = get_package_share_directory('dvrk_model')
    valid_arms = valid_arm_names(dvrk_model_share, generation)
    require_choice('arm', full_name, valid_arms)

    xacro_file = os.path.join(dvrk_model_share,
                              'urdf',
                              generation,
                              full_name + '.urdf.xacro')
    instrument = context.launch_configurations.get('instrument', '').strip()
    endoscope = context.launch_configurations.get('endoscope', '').strip()
    generation_prefix = '420' if generation in ['Si', 'Virtual'] else '400'
    if instrument == '':
        instrument = generation_prefix + '006'
    elif instrument.isdigit() and len(instrument) == 3:
        instrument = generation_prefix + instrument

    if full_name == 'ECM':
        if endoscope != '':
            require_choice('endoscope', endoscope, VALID_ENDOSCOPES)
    elif endoscope != '':
        raise RuntimeError(
            "Invalid endoscope '{}' for arm '{}'. The endoscope argument is only valid for ECM. Valid ECM endoscopes: {}".format(
                endoscope, full_name, valid_options_message(VALID_ENDOSCOPES)))

    if full_name.startswith('PSM'):
        require_choice('instrument',
                       instrument,
                       VALID_INSTRUMENTS,
                       VALID_INSTRUMENT_OPTIONS)
    elif context.launch_configurations.get('instrument', '').strip() != '':
        raise RuntimeError(
            "Invalid instrument '{}' for arm '{}'. The instrument argument is only valid for PSM arms. Valid PSM instruments: {}".format(
                instrument, full_name, VALID_INSTRUMENT_OPTIONS))

    show_rcm = context.launch_configurations.get('show_rcm', 'true').strip()
    mappings = {'arm': full_name, 'instrument': instrument, 'show_rcm': show_rcm}
    if full_name == 'ECM' and endoscope != '':
        mappings['endoscope'] = endoscope
    if context.launch_configurations.get('suj') == 'true':
        if generation == 'Virtual':
            mappings['parent_link_'] = full_name + '_base'
        else:
            mappings['parent_link_'] = 'SUJ_' + full_name + '_RCM'

    robot_description_config = xacro.process_file(xacro_file,
                                                  mappings = mappings)
    robot_description_xml = robot_description_config.toxml()
    # joint_state_publisher (ROS 2 Jazzy package) initializes from a file path
    # argument, or waits for /robot_description topic. Provide a temp URDF file
    # to avoid startup races and topic-mode waiting.
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix='_' + full_name + '.urdf',
        prefix='dvrk_model_',
        delete=False,
    )
    with tmp:
        tmp.write(robot_description_xml)

    return [
        SetLaunchConfiguration(name = 'robot_description',
                               value = robot_description_xml),
        SetLaunchConfiguration(name = 'robot_description_file',
                               value = tmp.name)
    ]


create_robot_description_arg = OpaqueFunction(function = create_robot_description)


def generate_launch_description():
    arm = LaunchConfiguration('arm')
    generation = LaunchConfiguration('generation')
    instrument = LaunchConfiguration('instrument', default='')
    endoscope = LaunchConfiguration('endoscope', default='')
    direct_control = LaunchConfiguration('direct_control', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    suj = LaunchConfiguration('suj', default='false')
    rate = LaunchConfiguration('rate', default = 50.0)  # Hz, default is 10 so we're increasing that a bit.

    # Declare nodes
    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        namespace = arm,
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        condition = UnlessCondition(direct_control),
        arguments = [LaunchConfiguration('robot_description_file')],
        parameters = [{'use_sim_time': use_sim_time,
                       'use_robot_description_topic': False,
                       'robot_description': launch_ros.descriptions.ParameterValue(
                           LaunchConfiguration('robot_description'),
                           value_type = str),
                       'source_list': ArmSourceListSubstitution(arm),
                       'rate': rate}],
        output = "log",
    )

    joint_state_publisher_gui_node = Node(
        package = 'joint_state_publisher_gui',
        namespace = arm,
        executable = 'joint_state_publisher_gui',
        name = 'joint_state_publisher',
        condition = IfCondition(direct_control),
        arguments = [LaunchConfiguration('robot_description_file')],
        parameters = [{'use_sim_time': use_sim_time,
                       'use_robot_description_topic': False,
                       'robot_description': launch_ros.descriptions.ParameterValue(
                           LaunchConfiguration('robot_description'),
                           value_type = str),
                       'rate': rate}],
        output = "log",
    )

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = arm,
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'robot_description': launch_ros.descriptions.ParameterValue(
                           LaunchConfiguration('robot_description'),
                           value_type = str),
                       'publish_robot_description': True,
                       'publish_frequency': rate}],
        output = "log",
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('arm'),
        DeclareLaunchArgument(
            'generation',
            choices=['Classic', 'Si', 'Virtual'],
            description='dVRK system generation'
        ),
        DeclareLaunchArgument('instrument', default_value=''),
        DeclareLaunchArgument('endoscope', default_value=''),
        DeclareLaunchArgument(
            'direct_control',
            default_value='false',
            description='Use joint_state_publisher_gui for manual joint control instead of mirroring incoming joint-state topics'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('suj', default_value='false'),
        DeclareLaunchArgument('rate', default_value='50.0'),
        DeclareLaunchArgument('show_rcm', default_value='true'),
        create_robot_description_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])

    return ld
