# Add flag to check if the first message is recieved and if not do nothing
from launch import LaunchContext, LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration,TextSubstitution, PathJoinSubstitution

import os
from typing import Text
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

class ArmSourceListSubstitution(Substitution):
    """Generate source list for a given arm"""

    def __init__(self, arm_name: LaunchConfiguration):
        super().__init__()
        self.__arm_name = arm_name

    def describe(self) -> Text:
        return 'ArmSourceList({})'.format(self.__arm_name.describe())

    def perform(self, context: LaunchContext) -> Text:
        arm_name = self.__arm_name.perform(context)
        source_list = ['measured_js']
        if arm_name.startswith('PSM'):
            source_list.append('jaw/measured_js')
        elif arm_name.startswith('MTM'):
            source_list.append('gripper/measured_js')

        # launch_ros parses values from yaml, so source list needs to be yaml
        sources = ','.join(source for source in source_list)
        yaml =  '[{}]'.format(sources)
        return yaml
        
        
def generate_launch_description():
    declared_arguments=[]
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="dvrk_joint_trajectory_config.yaml",
            description="the configuration file for the controllers",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="dvrk_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="dvrk_model",
            description="Description package with robot URDF/xacro files.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="PSM1",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm",
            default_value="PSM1",
            description="arm name.",
        )
    )
    controllers_file = LaunchConfiguration("controllers_file")
    robot_controller = LaunchConfiguration("robot_controller")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_name = LaunchConfiguration('arm')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    rate = LaunchConfiguration('rate', default = 50.0)

    robot_description_content = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), 'model', '']),
            arm_name,
            '.urdf.xacro'
        ]),value_type=str)
        
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            controllers_file
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "PSM1.rviz"]
    )   
    console_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "../../../sawIntuitiveResearchKitAll","share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json"]
    )   
    
    #############################
    #Defining Nodes from here on.
    
    #console_node=Node(
    #    package="dvrk_robot",
    #    executable="dvrk_console_json",
    #    name="dvrk_console",
    #    output="screen",
    #    arguments=["-j",console_file]
    #)  
    # CHANGE THE PATH HERE TO THE JSON FILE

    joint_state_publisher_node = Node(
    package = 'joint_state_publisher',
    namespace = arm_name,
    executable = 'joint_state_publisher',
    name = 'joint_state_publisher',
    parameters = [{'use_sim_time': use_sim_time,
                   'source_list': ArmSourceListSubstitution(arm_name),
                   'rate': rate}],
    )

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = arm_name,
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       "robot_description":robot_description_content,
                       'publish_frequency': rate}],
        output="both",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        # prefix=['xterm -e gdb -ex run --args']
        # remappings=[('/joint_states','/broadcast_joint_states'),]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        # prefix=['xterm -e gdb -ex run --args']
        # remappings=[('/joint_states','/broadcast_joint_states'),]        
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller,"--controller-manager",  "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

        # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes =[
            #console_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
            #rviz_node,
            control_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
            ]
    return LaunchDescription(declared_arguments+nodes)
