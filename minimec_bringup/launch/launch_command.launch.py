import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            default_value='teleop',
            description="controls the source of the command velocity commands.",
            choices=["teleop", "path"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("minimec_control"), '/launch',
                 '/launch_path_gen.launch.py']
            ),
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'path\'']))
        ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'teleop\''])),
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('cmd_vel','cmd_vel_raw')]
        ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'teleop\''])),
            package='teleop-smoother',
            executable='smoother',
            parameters=[{"linear_acceleration": 0.75,
                         "angular_acceleration":1.0,
                         "input_vel_topic":"cmd_vel_raw",
                         "output_vel_topic":"cmd_vel",
                         "loop_frequency":50.0}]
        )

        
    ])