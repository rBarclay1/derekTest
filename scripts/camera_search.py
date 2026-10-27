#!/usr/bin/env python3
"""Scan joint 4 for a red cube, center vertically, and stop. No wheel commands."""
import argparse
import json
from pathlib import Path
import signal
import time


class SearchPolicy:
    """Pulse limits and image-feedback decisions, independent of ROS."""
    def __init__(self, position, now, lower=149, upper=300):
        if not 149 <= lower < upper <= 300 or not lower <= position <= upper:
            raise ValueError('Joint 4 must start within limits inside 149..300')
        self.lower, self.upper = lower, upper
        self.target = position
        self.direction = 1
        self.last_move = now
        self.last_seen = None
        self.confirmed_frames = self.centered_frames = 0
        self.mode = 'SEARCH'

    def update(self, now, y, height=360):
        """Call once per distinct fresh image. Return (new_target, stop_reason)."""
        if y is None:
            self.confirmed_frames = self.centered_frames = 0
            if self.last_seen is not None and now - self.last_seen < 0.6:
                return None, None
            self.mode = 'SEARCH'
            if now - self.last_move < 0.9:
                return None, None
            if self.target >= self.upper:
                self.direction = -1
            elif self.target <= self.lower:
                self.direction = 1
            self.target = max(self.lower, min(self.upper, self.target + 10*self.direction))
            self.last_move = now
            return self.target, None
        self.last_seen = now
        self.confirmed_frames += 1
        self.mode = 'CONFIRM' if self.confirmed_frames < 5 else 'CENTER'
        if self.confirmed_frames < 5:
            return None, None
        error = height/2 - y
        if abs(error) <= 10:
            if now - self.last_move >= 0.6:
                self.centered_frames += 1
            if self.centered_frames >= 8:
                return None, 'centered_vertically'
            return None, None
        self.centered_frames = 0
        if now - self.last_move < 0.45:
            return None, None
        delta = max(-3, min(3, round(0.55*error/2.2)))
        target = max(self.lower, min(self.upper, self.target + delta))
        if target == self.target:
            return None, 'tilt_limit_before_center'
        self.target, self.last_move = target, now
        return target, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--min-area', type=float, default=20)
    args = parser.parse_args()
    if not 0 < args.timeout <= 60 or not 0 < args.min_area:
        parser.error('timeout must be in (0,60]; min-area must be positive')

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import UInt16
    from ros_robot_controller_msgs.msg import (
        BusServoState, GetBusServoCmd, SetBusServoState, ServoPosition, ServosPosition)
    from ros_robot_controller_msgs.srv import GetBusServoState
    from jetrover_retrieve.color_detect import detect_largest_object

    rclpy.init()
    node = rclpy.create_node('stationary_cube_camera_search')
    bridge = CvBridge()
    latest = {'image': None, 'battery': None}
    interrupted = False

    def stop(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    subscriptions = [
        node.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                 lambda m: latest.update(image=m), qos_profile_sensor_data),
        node.create_subscription(UInt16, '/ros_robot_controller/battery',
                                 lambda m: latest.update(battery=(time.monotonic(), m.data)), 10),
    ]
    publisher = node.create_publisher(ServosPosition,
                                     '/ros_robot_controller/bus_servo/set_position', 10)
    torque = node.create_publisher(SetBusServoState,
                                  '/ros_robot_controller/bus_servo/set_state', 10)
    client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
    folder = Path('/home/ubuntu/camera-check')
    folder.mkdir(exist_ok=True)
    events = []
    reason = 'preflight_failed'
    initial = final = None
    last_result = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not interrupted:
            rclpy.spin_once(node, timeout_sec=0.03)

    def read_position():
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('Servo read service missing')
        request = GetBusServoState.Request()
        command = GetBusServoCmd()
        command.id, command.get_position = 4, 1
        request.cmd = [command]
        future = client.call_async(request)
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        if not future.done() or not future.result().success:
            raise RuntimeError('Servo readback timed out or failed')
        return future.result().state[0].position[0]

    try:
        spin(3)
        if interrupted:
            reason = 'interrupted'
            return
        if node.count_subscribers('/ros_robot_controller/bus_servo/set_position') != 1:
            raise RuntimeError('Expected one hardware servo subscriber')
        initial = read_position()
        policy = SearchPolicy(initial, time.monotonic())
        # Load the current position before enabling torque to avoid an old target.
        message = ServosPosition()
        position = ServoPosition()
        position.id, position.position = 4, initial
        message.duration, message.position = 0.5, [position]
        publisher.publish(message)
        spin(0.1)
        state = BusServoState()
        state.present_id, state.enable_torque = [1, 4], [1, 1]
        message = SetBusServoState()
        message.state = [state]
        torque.publish(message)
        print(json.dumps({'event': 'START', 'joint4': initial, 'limits': [149, 300],
                          'timeout': args.timeout, 'min_area': args.min_area}), flush=True)
        started = time.monotonic()
        last_stamp = None
        previous_center = None
        reason = 'search_timeout'
        while time.monotonic()-started < args.timeout and not interrupted:
            rclpy.spin_once(node, timeout_sec=0.03)
            now = time.monotonic()
            battery, image = latest['battery'], latest['image']
            if battery is None or now-battery[0] > 2 or battery[1] < 10000:
                reason = 'battery_missing_stale_or_below_10V_trial_limit'
                break
            if image is None:
                reason = 'no_camera_image'
                break
            stamp = image.header.stamp.sec + image.header.stamp.nanosec/1e9
            age = node.get_clock().now().nanoseconds/1e9 - stamp
            if not -0.1 <= age <= 0.5:
                reason = 'stale_camera_image'
                break
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            frame = bridge.imgmsg_to_cv2(image, 'bgr8')
            result = detect_largest_object(frame, 'red', args.min_area,
                                            previous_center, max_jump_px=45)
            y = None
            marked = frame.copy()
            if result:
                x, y, area, color, marked = result
                previous_center = (x, y)
                last_result = {'x': x, 'y': y, 'area': area}
            else:
                previous_center = None
                last_result = None
            target, done = policy.update(now, y, image.height)
            if target is not None:
                message = ServosPosition()
                position = ServoPosition()
                position.id, position.position = 4, target
                message.duration = 0.65 if policy.mode == 'SEARCH' else 0.35
                message.position = [position]
                publisher.publish(message)
                event = {'elapsed': round(now-started, 2), 'mode': policy.mode,
                         'target': target, 'cube': last_result}
                events.append(event)
                print(json.dumps(event), flush=True)
            cv2.line(marked, (0, image.height//2), (image.width-1, image.height//2),
                     (255, 255, 0), 1)
            cv2.putText(marked, policy.mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 1)
            cv2.imwrite(str(folder/'camera-search.png'), marked)
            if done:
                reason = done
                break
        if interrupted:
            reason = 'interrupted'
    finally:
        # No new targets after stopping. Let the last short move finish before feedback.
        spin(0.7)
        try:
            final = read_position()
        except Exception:
            pass
        summary = {'reason': reason, 'initial_joint4': initial, 'final_joint4': final,
                   'last_detection': last_result, 'events': events,
                   'wheel_commands_sent': False}
        (folder/'camera-search.json').write_text(json.dumps(summary, indent=2))
        print('SUMMARY '+json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
