from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='3'),
        Node(
            package='explorer',
            executable='explorer_node',
            name='explorer',
            output='screen',
            parameters=[{'n_robots': ParameterValue(LaunchConfiguration('n_robots'), value_type=int)}],
        ),
    ])
