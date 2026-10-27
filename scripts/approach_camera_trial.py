#!/usr/bin/env python3
"""Short supervised camera-tilt/approach experiment, not a pickup mission.

Run on Derek in its ROS environment. Uses the experimentally observed joint-4
direction. Does not change other arm joints or search when the target is lost.
"""
import json
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from ros_robot_controller_msgs.msg import GetBusServoCmd, ServoPosition, ServosPosition
from ros_robot_controller_msgs.srv import GetBusServoState
from jetrover_retrieve.color_detect import detect_largest_object, depth_to_meters


def main():
    rclpy.init()
    node = rclpy.create_node('supervised_camera_approach_trial')
    bridge = CvBridge()
    state = {'rgb': None, 'depth': None, 'battery': None}
    events = []
    stop_requested = False

    def request_stop(*_):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    subscriptions = [
        node.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                 lambda m: state.update(rgb=m), qos_profile_sensor_data),
        node.create_subscription(Image, '/depth_cam/depth/image_raw',
                                 lambda m: state.update(depth=m), qos_profile_sensor_data),
        node.create_subscription(UInt16, '/ros_robot_controller/battery',
                                 lambda m: state.update(battery=(time.monotonic(), m.data)), 10),
    ]
    velocity = node.create_publisher(Twist, '/controller/cmd_vel', 10)
    servo = node.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
    client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
    output = Path('/home/ubuntu/camera-check')
    output.mkdir(exist_ok=True)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def joint_position():
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('Servo feedback service missing')
        request = GetBusServoState.Request()
        command = GetBusServoCmd()
        command.id, command.get_position = 4, 1
        request.cmd = [command]
        future = client.call_async(request)
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not future.done() or not future.result().success:
            raise RuntimeError('No joint feedback')
        return future.result().state[0].position[0]

    start = None
    reason = 'not_started'
    forward_seconds = 0.0
    initial_joint = final_joint = None
    try:
        spin(3)
        if node.count_subscribers('/controller/cmd_vel') != 1:
            raise RuntimeError('Expected one chassis command subscriber')
        if node.count_subscribers('/ros_robot_controller/bus_servo/set_position') != 1:
            raise RuntimeError('Expected one servo command subscriber')
        initial_joint = joint_position()
        if not 149 <= initial_joint <= 300:
            raise RuntimeError('Joint 4 is outside the tested trial range')
        target = initial_joint
        previous_center = None
        stable = 0
        last_frame = None
        last_tilt = 0.0
        last_log = 0.0
        last_detection = time.monotonic()
        start = previous_tick = time.monotonic()
        last_forward = 0.0
        reason = '12_second_trial_limit'
        print(json.dumps({'event': 'START', 'joint4': initial_joint,
                          'max_forward_seconds': 4, 'speed_m_s': 0.04}), flush=True)
        while time.monotonic() - start < 12 and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            forward_seconds += last_forward * (now - previous_tick)
            previous_tick = now
            last_forward = 0.0
            command = Twist()
            battery = state['battery']
            if battery is None or now - battery[0] > 2 or battery[1] < 10000:
                reason = 'battery_missing_stale_or_below_10V_trial_limit'
                break
            rgb, depth = state['rgb'], state['depth']
            ros_now = node.get_clock().now().nanoseconds / 1e9
            if (rgb is None or depth is None
                    or not -0.1 <= ros_now - stamp(rgb) <= 0.4
                    or not -0.1 <= ros_now - stamp(depth) <= 0.4
                    or abs(stamp(rgb) - stamp(depth)) > 0.15):
                reason = 'camera_or_depth_stale'
                break
            if forward_seconds >= 4:
                reason = 'four_seconds_forward_limit'
                break
            frame = bridge.imgmsg_to_cv2(rgb, 'bgr8')
            depth_image = bridge.imgmsg_to_cv2(depth, 'passthrough')
            if depth_image.shape != frame.shape[:2]:
                reason = 'image_depth_size_mismatch'
                break
            result = detect_largest_object(frame, 'red', 20,
                                            previous_center, max_jump_px=45)
            new_frame = stamp(rgb) != last_frame
            last_frame = stamp(rgb)
            if result is None:
                stable = 0
                velocity.publish(command)
                if now - last_detection > 0.6:
                    reason = 'target_lost'
                    break
                continue
            last_detection = now
            x, y, area, color, marked = result
            previous_center = (x, y)
            if new_frame:
                stable += 1
            distance = depth_to_meters(depth_image, x, y, depth.encoding)
            if distance is None:
                reason = 'invalid_target_depth'
                break
            if distance <= 0.75:
                reason = 'camera_depth_0.75m_stop'
                break
            vertical_error = frame.shape[0] / 2 - y
            horizontal_error = x - frame.shape[1] / 2
            if stable >= 5 and abs(vertical_error) > 10 and now - last_tilt >= 0.4:
                change = int(round(np.clip(0.45 * vertical_error / 2.2, -3, 3)))
                next_target = int(np.clip(target + change, 149, 300))
                if next_target == target:
                    reason = 'camera_tilt_trial_limit'
                    break
                target = next_target
                message = ServosPosition()
                message.duration = 0.35
                position = ServoPosition()
                position.id, position.position = 4, target
                message.position = [position]
                servo.publish(message)
                last_tilt = now
            if stable >= 5:
                if abs(horizontal_error) > 12:
                    command.angular.z = float(np.clip(-0.003 * horizontal_error, -0.08, 0.08))
                if abs(horizontal_error) <= 35 and abs(vertical_error) <= 25:
                    command.linear.x = 0.04
                    last_forward = 1.0
            velocity.publish(command)
            if now - last_log >= 0.5:
                event = {'elapsed': round(now-start, 2), 'x': x, 'y': y,
                         'depth_m': distance, 'area': area, 'joint4_target': target,
                         'forward_m_s': command.linear.x, 'yaw_rad_s': command.angular.z,
                         'forward_seconds': round(forward_seconds, 2)}
                events.append(event)
                print(json.dumps(event), flush=True)
                cv2.line(marked, (0, 180), (639, 180), (255, 255, 0), 1)
                cv2.imwrite(str(output/'approach-trial.png'), marked)
                last_log = now
        if stop_requested:
            reason = 'interrupted'
    finally:
        for _ in range(15):
            velocity.publish(Twist())
            spin(0.03)
        try:
            final_joint = joint_position()
        except Exception:
            pass
        summary = {'reason': reason, 'forward_seconds': forward_seconds,
                   'commanded_distance_upper_estimate_m': forward_seconds * 0.04,
                   'initial_joint4': initial_joint, 'final_joint4': final_joint,
                   'events': events, 'physical_motion_requires_confirmation': True}
        (output/'approach-trial.json').write_text(json.dumps(summary, indent=2))
        print('SUMMARY ' + json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
