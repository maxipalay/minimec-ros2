import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration,\
    PythonExpression


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_jsp',
            default_value='false',
            description="controls whether the joint_state_publisher is used\
                  to publish default joint states.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            'use_rsp',
            default_value='false',
            description="controls whether the robot_state_publisher is used\
                  to publish a robot description.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description="controls whether rviz is launched.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='basic_config.rviz',
            description='selects the rviz configuration file loaded.'
            ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_rsp'), '\'', '== \'true\''])),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description":
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                         PathJoinSubstitution(
                    [FindPackageShare("minimec_description"), "urdf",
                     "minimec.urdf.xacro"])])}
            ]
        ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_jsp'), '\'', '== \'true\''])),
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_rviz'), '\'', '== \'true\''])),
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("minimec_description"), "config/",
                     LaunchConfiguration('rviz_config')]),
                     "-f", "/base_footprint"],
            on_exit=launch.actions.Shutdown()
        )
    ])