from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions.declare_launch_argument import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            default_value='teleop',
            description="controls the source of the command velocity commands.",
            choices=["teleop", "path"]),
        IncludeLaunchDescription(
            YAMLLaunchDescriptionSource(
                [FindPackageShare("odrive_can"), '/launch',
                    '/example_launch.yaml']
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("minimec_control"), '/launch',
                 '/launch_control.launch.py']
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("minimec_driver"), '/launch',
                 '/launch_driver.launch.py']
            ),
            launch_arguments=[['cmd_src', LaunchConfiguration('cmd_src')]]
        ),
        Node(
            package='minimec_lights',
            executable='lights'
        ),
        Node(
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
    ])