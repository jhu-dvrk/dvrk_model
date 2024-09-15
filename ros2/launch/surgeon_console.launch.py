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

    ld = LaunchDescription()

    # dVRK console
    console_json = [
        PathJoinSubstitution([FindPackageShare('dvrk_config'),
                              'console/console-surgeon-console-simulated.json'])
    ]
    dvrk_node = Node(
        package = 'dvrk_robot',
        executable = 'dvrk_console_json',
        condition = IfCondition(simulated),
        arguments = ['-j', console_json],
        output = 'both',
    )
    ld.add_action(dvrk_node)

    # Arm joint/robot state publishers
    for arm in ['MTML', 'MTMR']:
        model = [
            PathJoinSubstitution([FindPackageShare('dvrk_model'),
                                  'urdf/Classic/'])
            , arm,
            '.urdf.xacro'
        ]
        # Use xacro to process robot model at substitution time
        # Can't happen until substitution time when we know robot_model
        description = ParameterValue(
            Command(
                [FindExecutable(name = 'xacro'), ' ', *model]
            ),
            value_type = str,
        )
        joint_state_publisher_node = Node(
            package = 'joint_state_publisher',
            namespace = arm,
            executable = 'joint_state_publisher',
            name = arm + '_joint_state_publisher',
            parameters = [{'use_sim_time': use_sim_time,
                           'source_list': ['measured_js', 'jaw/measured_js'],
                           'rate': rate}],
            output = 'both',
        )
        robot_state_publisher_node = Node(
            package = 'robot_state_publisher',
            namespace = arm,
            executable = 'robot_state_publisher',
            name = arm + '_robot_state_publisher',
            parameters = [{'use_sim_time': use_sim_time,
                           'robot_description': description,
                           'publish_frequency': rate}],
            output = 'both',
        )
        ld.add_action(joint_state_publisher_node)
        ld.add_action(robot_state_publisher_node)

    # RViz
    rviz_config_file = [
        PathJoinSubstitution([FindPackageShare('dvrk_model'),
                              'rviz/Classic/surgeon-console.rviz'])
    ]
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2_surgeon_console',
        arguments = ['-d', rviz_config_file],
        output = 'both',
    )
    ld.add_action(rviz_node)

    return ld
