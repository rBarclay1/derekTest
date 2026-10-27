#!/usr/bin/env python3
"""Repeat the manually calibrated release, grip, lift; record every stage.

Only valid from the ground-level pose used in this calibration session.
Run through run_approach_trial.run_ros_script for the robot's ROS environment.
"""
from datetime import datetime, timezone
import argparse
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from ros_robot_controller_msgs.msg import GetBusServoCmd, ServoPosition, ServosPosition
from ros_robot_controller_msgs.srv import GetBusServoState
from jetrover_retrieve.color_detect import (
    color_mask, depth_to_meters, detect_largest_object, largest_blob)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--from-air', action='store_true',
                        help='Start holding the cube, lower joint 2 to 112, then release/regrasp/lift')
    args = parser.parse_args(argv)
    rclpy.init()
    node = rclpy.create_node('repeat_calibrated_pickup_trial')
    bridge = CvBridge()
    latest = {}
    subscriptions = [
        node.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                 lambda m: latest.update(rgb=m), qos_profile_sensor_data),
        node.create_subscription(Image, '/depth_cam/depth/image_raw',
                                 lambda m: latest.update(depth=m), qos_profile_sensor_data),
        node.create_subscription(UInt16, '/ros_robot_controller/battery',
                                 lambda m: latest.update(battery=(time.monotonic(), m.data)), 10),
    ]
    publisher = node.create_publisher(ServosPosition,
                                     '/ros_robot_controller/bus_servo/set_position', 10)
    client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
    ids = [1, 2, 3, 4, 5, 10]
    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8]
    folder = Path.home()/'grip-data'/'repeat-pickup'/name
    folder.mkdir(parents=True)
    report = {'trial': name, 'status': 'started', 'stages': [],
              'lift_success': 'awaiting_user_confirmation', 'wheel_commands_sent': False}
    report['from_air'] = args.from_air
    report['joint2_ground_target'] = 112 if args.from_air else 92

    def save():
        (folder/'trial.json').write_text(json.dumps(report, indent=2)+'\n')

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)

    def read():
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('Servo read service missing')
        request = GetBusServoState.Request()
        for sid in ids:
            command = GetBusServoCmd()
            command.id, command.get_position = sid, 1
            request.cmd.append(command)
        future = client.call_async(request)
        end = time.monotonic()+3
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        if not future.done() or not future.result().success:
            raise RuntimeError('Missing joint feedback')
        states = future.result().state
        if len(states) != len(ids) or any(len(s.position) != 1 for s in states):
            raise RuntimeError('Incomplete joint feedback')
        return {str(sid): s.position[0] for sid, s in zip(ids, states)}

    def fresh_camera():
        rgb, depth = latest.get('rgb'), latest.get('depth')
        stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec/1e9
        now = node.get_clock().now().nanoseconds/1e9
        if rgb is None or depth is None:
            raise RuntimeError('Camera streams missing')
        if any(not -0.1 <= now-stamp(m) <= 0.5 for m in (rgb, depth)):
            raise RuntimeError('Camera streams stale')
        if abs(stamp(rgb)-stamp(depth)) > 0.15:
            raise RuntimeError('RGB/depth timestamp mismatch')
        return rgb, depth

    def move(sid, target, duration):
        fresh_camera()
        battery = latest.get('battery')
        if battery is None or time.monotonic()-battery[0] > 2:
            raise RuntimeError('Battery telemetry missing or stale')
        if publisher.get_subscription_count() != 1:
            raise RuntimeError('Expected one hardware servo subscriber')
        report['next_command'] = {'id': sid, 'target': target, 'duration': duration}
        save()
        message = ServosPosition()
        position = ServoPosition()
        position.id, position.position = sid, target
        message.duration, message.position = float(duration), [position]
        publisher.publish(message)
        spin(duration+1)

    def capture(stage):
        positions = read()
        rgb, depth = fresh_camera()
        frame = bridge.imgmsg_to_cv2(rgb, 'bgr8')
        raw = bridge.imgmsg_to_cv2(depth, 'passthrough')
        stage_folder = folder/stage
        stage_folder.mkdir()
        cv2.imwrite(str(stage_folder/'rgb.png'), frame)
        np.save(stage_folder/'depth_raw.npy', raw)
        result = detect_largest_object(frame, 'red', 20)
        detection = {'detected': result is not None}
        if result:
            x, y, area, color, marked = result
            blob = largest_blob(color_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), 'red'), 20)
            bx, by, bw, bh = cv2.boundingRect(blob[3])
            detection.update(center_px={'x': x, 'y': y}, contour_area_px=area,
                             bounding_box_px=[bx, by, bw, bh],
                             touches_image_edge=bx == 0 or by == 0 or
                             bx+bw >= rgb.width or by+bh >= rgb.height,
                             depth_at_visible_center_m=depth_to_meters(raw, x, y, depth.encoding))
            cv2.imwrite(str(stage_folder/'detection.png'), marked)
        entry = {'stage': stage, 'captured_utc': datetime.now(timezone.utc).isoformat(),
                 'servo_positions': positions, 'cube_detection': detection,
                 'image_size_px': [rgb.width, rgb.height], 'depth_encoding': depth.encoding,
                 'battery_mV': latest['battery'][1]}
        report['stages'].append(entry)
        (stage_folder/'metadata.json').write_text(json.dumps(entry, indent=2)+'\n')
        save()
        print(json.dumps(entry), flush=True)
        return positions

    try:
        spin(3)
        initial = capture('00-held-start' if args.from_air else '00-ground-closed')
        bounds = {'1': (480, 520), '2': (150, 230) if args.from_air else (70, 120), '3': (380, 420),
                  '4': (395, 435), '5': (480, 520), '10': (450, 520)}
        if any(not low <= initial[joint] <= high for joint, (low, high) in bounds.items()):
            raise RuntimeError('Current pose differs from expected starting pose; no movement')
        if args.from_air:
            move(2, 112, 4)
            capture('01-lowered-target112')
        opened = max(200, initial['10']-200)
        move(10, opened, 2)
        capture('02-open' if args.from_air else '01-open')
        move(10, 515, 2)
        capture('03-closed' if args.from_air else '02-closed')
        move(2, 198, 4)
        capture('04-lifted' if args.from_air else '03-lifted')
        report['status'] = 'sequence_completed'
    except BaseException as exc:
        report['status'] = 'stopped'
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        save()
        print('SAVED '+str(folder), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
