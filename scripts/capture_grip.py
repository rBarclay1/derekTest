#!/usr/bin/env python3
"""Save one stationary grip sample. Run on the robot with its ROS environment sourced."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

# Change these if the robot uses different topic names.
RGB_TOPIC = '/depth_cam/rgb/image_raw'
DEPTH_TOPIC = '/depth_cam/depth/image_raw'
RGB_INFO_TOPIC = '/depth_cam/rgb/camera_info'
DEPTH_INFO_TOPIC = '/depth_cam/depth/camera_info'
SERVO_SERVICE = '/ros_robot_controller/bus_servo/get_state'
SERVO_IDS = [1, 2, 3, 4, 5, 10]
MAX_FRAME_AGE = 1.0
MAX_PAIR_GAP = 0.10
CAMERA_WAIT = 10.0
SERVO_WAIT = 3.0


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def fresh(message, now):
    return stamp(message) > 0 and -0.1 <= now - stamp(message) <= MAX_FRAME_AGE


def choose_images(colors, depths, now):
    """Find a recent RGB frame and the closest depth timestamp in the buffer."""
    for color in reversed(colors):
        if not fresh(color, now):
            continue
        closest = None
        gap = MAX_PAIR_GAP
        for depth in depths:
            difference = abs(stamp(color) - stamp(depth))
            if fresh(depth, now) and difference <= gap:
                closest = depth
                gap = difference
        if closest is not None:
            return color, closest
    return None, None


def servo_positions(response):
    if not response.success or len(response.state) != len(SERVO_IDS):
        raise ValueError('Servo readback failed or returned the wrong number of entries')
    positions = {}
    for servo_id, state in zip(SERVO_IDS, response.state):
        if len(state.position) != 1 or not 0 <= state.position[0] <= 1000:
            raise ValueError(f'Missing or invalid position for servo {servo_id}')
        positions[str(servo_id)] = int(state.position[0])
    return positions


def image_details(message):
    return {'stamp_sec': stamp(message), 'frame_id': message.header.frame_id,
            'encoding': message.encoding, 'width': message.width, 'height': message.height}


def camera_details(message):
    if message is None:
        return None
    return {'stamp_sec': stamp(message), 'frame_id': message.header.frame_id,
            'width': message.width, 'height': message.height,
            'distortion_model': message.distortion_model, 'd': list(message.d),
            'k': list(message.k), 'r': list(message.r), 'p': list(message.p),
            'binning_x': message.binning_x, 'binning_y': message.binning_y,
            'roi': {'x_offset': message.roi.x_offset, 'y_offset': message.roi.y_offset,
                    'width': message.roi.width, 'height': message.roi.height,
                    'do_rectify': message.roi.do_rectify}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', required=True, help='Group samples, for example trial01')
    parser.add_argument('--stage', required=True, choices=['ready', 'grasp', 'held', 'empty'])
    parser.add_argument('--object', default='unspecified', help='Object description')
    parser.add_argument('--notes', default='', help='Lighting, distance, success/failure, etc.')
    parser.add_argument('--output', default='~/grip-data')
    parser.add_argument('--skip-servos', action='store_true', help='Save images without querying servos')
    args = parser.parse_args()
    if not args.trial or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.trial):
        parser.error('--trial must use letters, numbers, underscores, or hyphens')

    # Imports stay here so --help and the timing tests work without ROS installed.
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CameraInfo
    from ros_robot_controller_msgs.msg import GetBusServoCmd
    from ros_robot_controller_msgs.srv import GetBusServoState

    rclpy.init()
    node = Node('capture_grip')
    bridge = CvBridge()
    colors, depths = [], []
    info = {'rgb': None, 'depth': None}

    def color_callback(message):
        colors.append(message)
        del colors[:-20]

    def depth_callback(message):
        depths.append(message)
        del depths[:-20]

    def color_info_callback(message):
        info['rgb'] = message

    def depth_info_callback(message):
        info['depth'] = message

    node.create_subscription(Image, RGB_TOPIC, color_callback, qos_profile_sensor_data)
    node.create_subscription(Image, DEPTH_TOPIC, depth_callback, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, RGB_INFO_TOPIC, color_info_callback, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, depth_info_callback, qos_profile_sensor_data)
    warnings = []
    try:
        print('Keep the robot and object still. Waiting for fresh camera frames...', flush=True)
        end = time.monotonic() + CAMERA_WAIT
        color = depth = None
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds / 1e9
            color, depth = choose_images(colors, depths, now)
            if color is not None and info['rgb'] is not None and info['depth'] is not None:
                break
        now = node.get_clock().now().nanoseconds / 1e9
        if color is None:
            for candidate in reversed(colors):
                if fresh(candidate, now):
                    color = candidate
                    break
        if color is None:
            raise RuntimeError('No fresh RGB frame. Check the camera launch and topic names.')
        if depth is None:
            warnings.append('No fresh matching depth frame; saved RGB only.')

        folder_name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
        folder_name += '-' + args.stage + '-' + uuid.uuid4().hex[:8]
        folder = Path(args.output).expanduser() / args.trial / folder_name
        folder.mkdir(parents=True, exist_ok=False)
        rgb = bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
        if not cv2.imwrite(str(folder / 'rgb.png'), rgb):
            raise RuntimeError('Could not write rgb.png')
        metadata = {'trial': args.trial, 'stage': args.stage, 'object': args.object,
                    'notes': args.notes, 'captured_utc': datetime.now(timezone.utc).isoformat(),
                    'rgb': image_details(color), 'depth': None,
                    'topics': {'rgb': RGB_TOPIC, 'depth': DEPTH_TOPIC},
                    'rgb_age_sec_at_selection': now - stamp(color),
                    'camera_info': {'rgb': camera_details(info['rgb']),
                                    'depth': camera_details(info['depth'])},
                    'registration_verified': False, 'servo_positions': None,
                    'servo_status': 'skipped', 'warnings': warnings}
        if info['rgb'] is None or info['depth'] is None:
            warnings.append('Some camera calibration messages were unavailable.')
        if depth is not None:
            try:
                raw = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
                if raw.ndim != 2 or depth.encoding.upper() not in ('16UC1', '32FC1'):
                    raise ValueError('Expected 16UC1 millimeters or 32FC1 meters')
                meters = raw.astype(np.float32)
                if depth.encoding.upper() == '16UC1':
                    meters /= 1000.0
                valid = np.isfinite(meters) & (meters > 0)
                meters[~valid] = np.nan
                np.save(folder / 'depth_raw.npy', raw)
                np.save(folder / 'depth_meters.npy', meters)
                # Fixed preview range: near is bright, far is dark; missing is black.
                preview = np.zeros(raw.shape, dtype=np.uint8)
                preview[valid] = (255 - 254 * np.clip((meters[valid] - 0.05) / 1.95, 0, 1)).astype(np.uint8)
                if not cv2.imwrite(str(folder / 'depth_preview.png'), preview):
                    raise RuntimeError('Could not write depth preview')
                metadata['depth'] = image_details(depth)
                metadata['pair_gap_sec'] = abs(stamp(color) - stamp(depth))
                metadata['depth_valid_fraction'] = float(valid.mean())
                metadata['same_image_size'] = raw.shape == rgb.shape[:2]
                if not metadata['same_image_size']:
                    warnings.append('RGB/depth dimensions differ; do not match their pixels directly.')
                if not valid.any():
                    warnings.append('Depth frame has no valid distances; RGB is still saved.')
            except Exception as exc:
                warnings.append(f'Depth save failed: {exc}')

        if not args.skip_servos:
            client = node.create_client(GetBusServoState, SERVO_SERVICE)
            metadata['servo_status'] = 'unavailable'
            if client.wait_for_service(timeout_sec=1.0):
                request = GetBusServoState.Request()
                for servo_id in SERVO_IDS:
                    command = GetBusServoCmd()
                    command.id = servo_id
                    command.get_position = 1
                    request.cmd.append(command)
                metadata['servo_request_ros_sec'] = node.get_clock().now().nanoseconds / 1e9
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=SERVO_WAIT)
                if future.done():
                    try:
                        metadata['servo_positions'] = servo_positions(future.result())
                        metadata['servo_status'] = 'measured_after_images'
                    except Exception as exc:
                        metadata['servo_status'] = 'invalid_response'
                        warnings.append(str(exc))
                else:
                    future.cancel()
                    metadata['servo_status'] = 'timeout'
                    warnings.append('Servo readback timed out. The driver may be blocked; check it before another query or move.')
                metadata['servo_read_finished_ros_sec'] = node.get_clock().now().nanoseconds / 1e9
            else:
                warnings.append('Servo service unavailable; images saved without joint readings.')
        (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n')
        print(f'Saved: {folder}')
        print(f"Servo readings: {metadata['servo_positions']}")
        for warning in warnings:
            print('NOTE:', warning)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        raise SystemExit(str(exc))
