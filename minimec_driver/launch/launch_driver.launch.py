import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        SetLaunchConfiguration(
            'params',
            value='config/params.yaml'
            ),
        Node(
            package='minimec_driver',
            executable='minimec_driver',
            parameters=[ParameterFile(PathJoinSubstitution(
                            [FindPackageShare("minimec_driver"), LaunchConfiguration('params')])),
                        ],
            )
    ]) 