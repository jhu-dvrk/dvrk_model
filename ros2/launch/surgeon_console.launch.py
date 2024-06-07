from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch import LaunchContext, LaunchDescription, Substitution
from typing import Text


from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    simulated = LaunchConfiguration('simulated', default = 'true')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    rate = LaunchConfiguration('rate', default = 50.0)  # Hz, default is 10 so we're increasing that a bit.  Funny enough joint and robot state publishers don't have the same name for that parameter :-(

    console_json = [
        PathJoinSubstitution([FindPackageShare('dvrk_config'),
                              'console/console-surgeon-console-simulated.json'])
    ]

    MTML_model = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'),
                              'urdf/Classic/MTML.urdf.xacro'])
    ]

    MTMR_model = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'),
                              'urdf/Classic/MTMR.urdf.xacro'])
    ]

    rviz_config_file = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'),
                              'rviz/Classic/surgeon-console.rviz'])
    ]

    # Use xacro to process robot model at substitution time
    # Can't happen until substitution time when we know robot_model
    MTML_description = ParameterValue(
        Command(
            [FindExecutable(name = 'xacro'), ' ', *MTML_model]
        ),
        value_type = str,
    )

    MTMR_description = ParameterValue(
        Command(
            [FindExecutable(name = 'xacro'), ' ', *MTMR_model]
        ),
        value_type = str,
    )

    # Declare nodes
    dvrk_node = Node(
        package = 'dvrk_robot',
        executable = 'dvrk_console_json',
        condition = IfCondition(simulated),
        arguments = ['-j', console_json],
        output = 'both',
    )

    MTML_joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        namespace = 'MTML',
        executable = 'joint_state_publisher',
        name = 'MTML_joint_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'source_list': ['measured_js', 'jaw/measured_js'],
                       'rate': rate}],
        output = 'both',
    )

    MTML_robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = 'MTML',
        executable = 'robot_state_publisher',
        name = 'MTML_robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'robot_description': MTML_description,
                       'publish_frequency': rate}],
        output = 'both',
    )

    MTMR_joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        namespace = 'MTMR',
        executable = 'joint_state_publisher',
        name = 'MTMR_joint_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'source_list': ['measured_js', 'jaw/measured_js'],
                       'rate': rate}],
        output = 'both',
    )

    MTMR_robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        namespace = 'MTMR',
        executable = 'robot_state_publisher',
        name = 'MTML_robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,
                       'robot_description': MTMR_description,
                       'publish_frequency': rate}],
        output = 'both',
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
        MTML_joint_state_publisher_node,
        MTML_robot_state_publisher_node,
        MTMR_joint_state_publisher_node,
        MTMR_robot_state_publisher_node,
        rviz_node,
    ])

    return ld
