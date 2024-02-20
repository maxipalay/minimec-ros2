import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration,\
        PythonExpression
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        SetLaunchConfiguration(
            'robot_params',
            value='config/params.yaml'
            ),
        IncludeLaunchDescription(
            YAMLLaunchDescriptionSource(
                [FindPackageShare("odrive_can"), '/launch',
                    '/example_launch.yaml']
            )
        ),
        Node(
            package='minimec_ros',
            executable='velocity_parser',
            parameters=[ParameterFile(PathJoinSubstitution(
                            [FindPackageShare("minimec_ros"), LaunchConfiguration('robot_params')])),
                        ],
            ),
    ])
