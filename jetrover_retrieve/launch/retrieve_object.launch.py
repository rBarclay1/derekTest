"""Start the retrieval mission, camera, and controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    actions = []
    if LaunchConfiguration('start_bringup').perform(context).lower() == 'true':
        if os.environ.get('need_compile', 'False') == 'True':
            peripherals_path = get_package_share_directory('peripherals')
            controller_path = get_package_share_directory('controller')
        else:
            peripherals_path = '/home/ubuntu/ros2_ws/src/peripherals'
            controller_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        for path, filename in ((peripherals_path, 'depth_camera.launch.py'),
                               (controller_path, 'controller.launch.py')):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(path, 'launch', filename))))
    actions.append(Node(
        package='jetrover_retrieve', executable='retrieve_object', output='screen',
        parameters=[LaunchConfiguration('config_file'), {
            'target_color': LaunchConfiguration('target_color'),
            'mission_mode': LaunchConfiguration('mission_mode'),
            'show_debug_image': ParameterValue(LaunchConfiguration('show_debug_image'), value_type=bool),
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
            'allow_unverified_pick': ParameterValue(LaunchConfiguration('allow_unverified_pick'), value_type=bool),
        }]))
    return actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory('jetrover_retrieve'), 'config', 'retrieve.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('target_color', default_value='red'),
        DeclareLaunchArgument('mission_mode', default_value='full'),
        DeclareLaunchArgument('show_debug_image', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('allow_unverified_pick', default_value='false'),
        DeclareLaunchArgument('start_bringup', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
