from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            default_value='teleop',
            description="controls the source of the command velocity commands.",
            choices=["teleop", "path"]),
        SetLaunchConfiguration(
            'robot_params',
            value='config/params.yaml'
            ),
        Node(
            package='minimec_control',
            executable='kinematics',
            parameters=[ParameterFile(PathJoinSubstitution(
                            [FindPackageShare("minimec_control"), LaunchConfiguration('robot_params')])),
                        ],
            ),
        Node(
            package='minimec_control',
            executable='odometry',
            parameters=[ParameterFile(PathJoinSubstitution(
                            [FindPackageShare("minimec_control"), LaunchConfiguration('robot_params')])),
                        ],
            ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'path\''])),
            package='minimec_control',
            executable='commands'
            ),
    ])
