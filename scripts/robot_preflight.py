#!/usr/bin/env python3
"""Check installed dependencies on the robot without starting nodes or motion."""

import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    failures = []

    def report(name, ok, detail):
        print(f'{"PASS" if ok else "FAIL"} {name}: {detail}')
        if not ok:
            failures.append(name)

    for command in ('ros2', 'colcon'):
        path = shutil.which(command)
        report(command, bool(path), path or 'not on PATH; source the robot ROS environment')
    for module in ('rclpy', 'cv2', 'numpy', 'yaml', 'cv_bridge',
                   'sensor_msgs.msg', 'nav_msgs.msg', 'geometry_msgs.msg',
                   'std_srvs.srv', 'servo_controller_msgs.msg', 'launch_ros'):
        try:
            importlib.import_module(module)
            report(module, True, 'imported')
        except Exception as exc:
            report(module, False, str(exc))
    chassis = os.environ.get('MACHINE_TYPE', '')
    report('MACHINE_TYPE', chassis in ('JetRover_Mecanum', 'JetRover_Tank'), chassis or 'unset')
    report('need_compile', os.environ.get('need_compile') in ('True', 'False'),
           os.environ.get('need_compile', 'unset'))
    camera = os.environ.get('DEPTH_CAMERA_TYPE', '')
    report('DEPTH_CAMERA_TYPE', camera == 'Dabai',
           camera + ' (bundled bringup requires Dabai for depth; custom bringup is possible)')
    print('INFO ROS_DISTRO:', os.environ.get('ROS_DISTRO', 'unset'))
    print('INFO ROS_DOMAIN_ID:', os.environ.get('ROS_DOMAIN_ID', '0 (default)'))
    if shutil.which('ros2'):
        for package in ('controller', 'peripherals', 'servo_controller',
                        'ros_robot_controller', 'jetrover_description',
                        'robot_localization', 'nav2_common', 'orbbec_camera',
                        'imu_filter_madgwick', 'imu_calib'):
            try:
                result = subprocess.run(['ros2', 'pkg', 'prefix', package],
                                        capture_output=True, text=True, timeout=10)
                report(package, result.returncode == 0,
                       (result.stdout + result.stderr).strip())
            except Exception as exc:
                report(package, False, str(exc))
    if os.environ.get('need_compile') == 'False':
        for relative in ('peripherals/launch/depth_camera.launch.py',
                         'driver/controller/launch/controller.launch.py',
                         'driver/servo_controller/launch/servo_controller.launch.py',
                         'driver/ros_robot_controller/launch/ros_robot_controller.launch.py',
                         'calibration/config/imu_calib.yaml',
                         'simulations/jetrover_description/launch/robot_description.launch.py'):
            path = Path('/home/ubuntu/ros2_ws/src') / relative
            report(str(path), path.is_file(), 'required by source-path bringup')
    elif os.environ.get('need_compile') == 'True' and shutil.which('ros2'):
        result = subprocess.run(['ros2', 'pkg', 'prefix', 'calibration'],
                                capture_output=True, text=True, timeout=10)
        report('calibration', result.returncode == 0,
               (result.stdout + result.stderr).strip())
    print(f'\n{len(failures)} failed check(s). No nodes started or motion commanded.')
    print('This checks dependencies only. Sensor streams, watchdog, arm and driving need hardware checks.')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
