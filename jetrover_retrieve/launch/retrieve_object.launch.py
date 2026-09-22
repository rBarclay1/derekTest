import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    start_bringup = LaunchConfiguration('start_bringup').perform(context)
    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'

    example_package_path = get_package_share_directory('jetrover_retrieve')
    config_path = os.path.join(example_package_path, 'config', 'retrieve.yaml')

    actions = []
    if start_bringup.lower() == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(controller_package_path, 'launch/controller.launch.py')),
            )
        )

    actions.append(
        Node(
            package='jetrover_retrieve',
            executable='retrieve_object',
            output='screen',
            parameters=[
                config_path,
                {
                    'target_color': LaunchConfiguration('target_color'),
                    'show_debug_image': LaunchConfiguration('show_debug_image'),
                },
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_color', default_value='red'),
        DeclareLaunchArgument('show_debug_image', default_value='true'),
        DeclareLaunchArgument(
            'start_bringup',
            default_value='true',
            description='Launch depth camera and chassis/arm controllers'),
        OpaqueFunction(function=launch_setup),
    ])
