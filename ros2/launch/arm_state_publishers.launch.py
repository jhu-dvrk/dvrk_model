import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch import LaunchContext, LaunchDescription, Substitution
from typing import Text
import xacro

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

# Create a list of ROS topics that will provide all the joint states
# from the dVRK main node to the joint_state_publisher
class ArmSourceListSubstitution(Substitution):
    """Generate source list for a given arm"""

    def __init__(self, arm: LaunchConfiguration):
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
    if full_name.startswith('PSM'):
        urdf_prefix = 'psm'
    elif full_name == 'ECM':
        urdf_prefix = 'ecm'
    else:
        urdf_prefix = 'to do for mtm. arm_state_publishers.launch.py'
    xacro_file = os.path.join(get_package_share_directory('dvrk_model'),
                              'urdf',
                              context.launch_configurations['generation'],
                              urdf_prefix + '.urdf.xacro')
    assert os.path.exists(xacro_file), 'The urdf file doesnt exist: ' + str(xacro_file)
    mappings = {'arm': full_name}
    if context.launch_configurations['suj'] == 'true':
        mappings['parent_link_'] = context.launch_configurations['arm'] + '_mounting_point'

    print(mappings)
    robot_description_config = xacro.process_file(xacro_file,
                                                  mappings = mappings)
    robot_description_xml = robot_description_config.toxml()
    print(robot_description_xml)
    return [SetLaunchConfiguration(name = 'robot_description',
                                   value = robot_description_xml)]

create_robot_description_arg = OpaqueFunction(function = create_robot_description)


def generate_launch_description():
    arm = LaunchConfiguration('arm')
    generation = LaunchConfiguration('generation')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    suj = LaunchConfiguration('suj', default = 'false')
    rate = LaunchConfiguration('rate', default = 50.0)  # Hz, default is 10 so we're increasing that a bit.

    # Declare nodes
    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        namespace = arm,
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'source_list': ArmSourceListSubstitution(arm),
                       'rate': rate}],
        output="both",
    )

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = arm,
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'robot_description': LaunchConfiguration('robot_description'),
                       'publish_frequency': rate}],
        output="both",
    )

    ld = LaunchDescription([
        create_robot_description_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])

    return ld
