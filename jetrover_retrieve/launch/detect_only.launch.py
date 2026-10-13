from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('jetrover_retrieve'), 'config', 'retrieve.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('target_color', default_value='red'),
        DeclareLaunchArgument('config_file', default_value=config_path),
        DeclareLaunchArgument('show_debug_image', default_value='false'),
        Node(
            package='jetrover_retrieve',
            executable='object_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'target_color': LaunchConfiguration('target_color'),
                    'show_debug_image': ParameterValue(
                        LaunchConfiguration('show_debug_image'), value_type=bool),
                },
            ],
        ),
    ])
