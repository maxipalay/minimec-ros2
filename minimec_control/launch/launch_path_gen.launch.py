import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description="controls whether rviz is launched.",
            choices=["true", "false"]),
        SetLaunchConfiguration(
            'rviz_config',
            value='view_path.rviz'
            ),
        Node(
            package='minimec_control',
            executable='path_generator'
            ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_rviz'), '\'', '== \'true\''])),
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("minimec_control"), "config/",
                     LaunchConfiguration('rviz_config')]),
                     "-f", "/odom"],
            on_exit=launch.actions.Shutdown()
        )
    ])