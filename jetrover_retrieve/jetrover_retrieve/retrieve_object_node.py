#!/usr/bin/env python3
"""One object mission: save home, search, approach, pick, return, and place."""

import json
import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from jetrover_retrieve.arm_control import ArmController
from jetrover_retrieve.color_detect import detect_largest_object, depth_to_meters
from jetrover_retrieve.navigation import clamp, pose_from_odometry, return_command


ACTIVE_STATES = {'PREPARE', 'SEARCH', 'ALIGN', 'APPROACH', 'PICK',
                 'VERIFY_PICK', 'RETURN_HOME', 'PLACE'}
VISION_STATES = {'SEARCH', 'ALIGN', 'APPROACH'}


class RetrieveObjectNode(Node):
    def __init__(self):
        super().__init__('retrieve_object')
        defaults = {
            'target_color': 'red', 'show_debug_image': False,
            'auto_start': False, 'allow_unverified_pick': False,
            'min_object_area': 500.0, 'center_tolerance_px': 40.0,
            'grasp_distance_min': 0.16, 'grasp_distance_max': 0.28,
            'search_yaw_speed': 0.25, 'max_linear_speed': 0.08,
            'max_yaw_speed': 0.4, 'return_linear_speed': 0.06,
            'confirm_frames': 12, 'detection_confirm_frames': 5,
            'lost_frames_limit': 20, 'max_target_jump_px': 100.0,
            'sensor_timeout_sec': 0.5, 'max_rgb_depth_skew_sec': 0.15,
            'startup_timeout_sec': 20.0, 'search_timeout_sec': 60.0,
            'approach_timeout_sec': 60.0, 'return_timeout_sec': 90.0,
            'arm_timeout_sec': 30.0, 'verify_timeout_sec': 60.0,
            'mission_timeout_sec': 240.0, 'max_home_distance_m': 3.0,
            'home_position_tolerance_m': 0.08, 'home_heading_tolerance_rad': 0.12,
            'return_turn_tolerance_rad': 0.25, 'home_settle_sec': 0.5,
            'min_valid_depth_m': 0.05, 'max_valid_depth_m': 5.0,
            'color_topic': '/depth_cam/rgb/image_raw',
            'depth_topic': '/depth_cam/depth/image_raw',
            'odom_topic': '/odom', 'cmd_vel_topic': '/controller/cmd_vel',
            'servo_topic': '/servo_controller',
            'controller_ready_service': '/controller_manager/init_finish',
            'action_group_path': '/home/ubuntu/software/arm_pc/ActionGroups',
            'chassis_type': os.environ.get('MACHINE_TYPE', 'JetRover_Mecanum'),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.config = {name: self.get_parameter(name).value for name in defaults}
        self._validate_config()
        self.bridge = CvBridge()
        self.state = 'INIT'
        self.detail = 'Waiting for controller, RGB, depth, and odometry'
        self.state_started = time.monotonic()
        self.mission_started = None
        self.approach_started = None
        self.home = None
        self.pose = None
        self.odom_frame = None
        self.home_frame = None
        self.received = {}
        self.latest_depth = None
        self.depth_encoding = None
        self.depth_stamp = None
        self.last_stamps = {}
        self.observation = None
        self.observation_id = 0
        self.processed_id = -1
        self.target_color = None
        self.target_center = None
        self.target_count = 0
        self.grasp_count = 0
        self.grasp_depth_stamp = None
        self.lost_count = 0
        self.settle_started = None
        self.last_status_time = 0.0
        self.controller_ready = False
        self.ready_future = None
        self.shutting_down = False
        self.auto_start_used = False
        self.pick_verified = False
        self.cmd_pub = self.create_publisher(Twist, self.config['cmd_vel_topic'], 1)
        self.status_pub = self.create_publisher(
            String, '~/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.arm = ArmController(self, self.config['servo_topic'], self.config['action_group_path'])
        self.create_subscription(Image, self.config['color_topic'], self.color_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.config['depth_topic'], self.depth_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.config['odom_topic'], self.odom_callback,
                                 qos_profile_sensor_data)
        self.ready_client = self.create_client(Trigger, self.config['controller_ready_service'])
        self.create_service(Trigger, '~/start', self.start_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(Trigger, '~/confirm_pick', self.confirm_pick_callback)
        self.create_timer(0.05, self.control_tick)
        self.get_logger().info(self.detail)

    def _validate_config(self):
        c = self.config
        if c['target_color'] not in ('red', 'green', 'blue', 'any'):
            raise ValueError('target_color must be red, green, blue, or any')
        if c['chassis_type'] not in ('JetRover_Mecanum', 'JetRover_Tank'):
            raise ValueError('This mission requires a mecanum or tank chassis that can turn in place')
        for name, value in c.items():
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{name} must be finite and positive')
        if not c['min_valid_depth_m'] <= c['grasp_distance_min'] < c['grasp_distance_max'] <= c['max_valid_depth_m']:
            raise ValueError('Grasp range must lie inside the valid depth range')
        if c['return_linear_speed'] > c['max_linear_speed']:
            raise ValueError('return_linear_speed cannot exceed max_linear_speed')
        if c['search_yaw_speed'] > c['max_yaw_speed']:
            raise ValueError('search_yaw_speed cannot exceed max_yaw_speed')
        if c['max_rgb_depth_skew_sec'] > c['sensor_timeout_sec']:
            raise ValueError('RGB/depth skew cannot exceed the sensor timeout')
        if c['home_position_tolerance_m'] >= c['max_home_distance_m']:
            raise ValueError('Home tolerance must be smaller than the mission radius')
        for name in ('home_heading_tolerance_rad', 'return_turn_tolerance_rad'):
            if c[name] >= math.pi / 2:
                raise ValueError(f'{name} must be smaller than pi/2')

    def _transition(self, state, detail):
        self.state, self.detail = state, detail
        self.state_started = time.monotonic()
        self.get_logger().info(f'{state}: {detail}')
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state, 'detail': self.detail,
            'home': vars(self.home) if self.home else None,
            'pose': vars(self.pose) if self.pose else None,
            'frame': self.home_frame, 'target_color': self.target_color,
            'pick_verified': self.pick_verified,
        })
        self.status_pub.publish(msg)
        self.last_status_time = time.monotonic()

    def _message_stamp(self, msg, stream):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if stamp <= 0 or age > self.config['sensor_timeout_sec'] or age < -0.1:
            return None
        # Queued duplicate or out-of-order frames must not refresh a watchdog
        # or count toward detection/grasp confirmation.
        if stamp <= self.last_stamps.get(stream, -1.0):
            return None
        self.last_stamps[stream] = stamp
        return stamp

    def _fresh(self, stream, now):
        return now - self.received.get(stream, -math.inf) <= self.config['sensor_timeout_sec']

    def _sensor_problem(self, now, streams):
        missing = [name for name in streams if not self._fresh(name, now)]
        return 'Missing or stale: ' + ', '.join(missing) if missing else None

    def depth_callback(self, msg):
        stamp = self._message_stamp(msg, 'depth')
        if stamp is None:
            return
        try:
            if msg.encoding.upper() not in ('16UC1', '32FC1'):
                raise ValueError(f'Unsupported depth encoding {msg.encoding}')
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if image.ndim != 2:
                raise ValueError('Expected a single-channel depth image')
            self.latest_depth, self.depth_encoding, self.depth_stamp = image, msg.encoding, stamp
            self.received['depth'] = time.monotonic()
        except Exception as exc:
            self._callback_failure('Depth', exc)

    def odom_callback(self, msg):
        if self._message_stamp(msg, 'odom') is None:
            return
        try:
            if not msg.header.frame_id or not msg.child_frame_id:
                raise ValueError('Odometry must name its parent and child frames')
            if self.home_frame and self.home_frame != (msg.header.frame_id, msg.child_frame_id):
                raise ValueError('Odometry frame changed during the mission')
            self.pose = pose_from_odometry(msg)
            self.odom_frame = (msg.header.frame_id, msg.child_frame_id)
            self.received['odom'] = time.monotonic()
        except Exception as exc:
            self._callback_failure('Odometry', exc)

    def color_callback(self, msg):
        stamp = self._message_stamp(msg, 'rgb')
        if stamp is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.received['rgb'] = time.monotonic()
            if self.state not in VISION_STATES:
                return
            result = detect_largest_object(
                frame, self.target_color or self.config['target_color'],
                self.config['min_object_area'], self.target_center,
                self.config['max_target_jump_px'])
            self.observation_id += 1
            self.observation = None
            vis = frame
            if result is not None:
                cx, cy, area, color, vis = result
                depth = None
                synchronized = (self.depth_stamp is not None and
                                abs(stamp - self.depth_stamp) <= self.config['max_rgb_depth_skew_sec'])
                if synchronized and self.latest_depth.shape == frame.shape[:2]:
                    depth = depth_to_meters(
                        self.latest_depth, cx, cy, self.depth_encoding,
                        self.config['min_valid_depth_m'], self.config['max_valid_depth_m'])
                self.observation = (cx, cy, frame.shape[1], color, depth, self.depth_stamp)
            if self.config['show_debug_image']:
                cv2.putText(vis, self.state, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)
                cv2.imshow('retrieve', vis)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    self._stop('STOPPED', 'Stopped from the debug window')
        except Exception as exc:
            self._callback_failure('RGB', exc)

    def _callback_failure(self, name, exc):
        if self.state in ACTIVE_STATES:
            self._stop('ERROR', f'{name} input failed: {exc}')
        else:
            self.get_logger().error(f'{name} input failed: {exc}')

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def _stop(self, state, detail):
        self.arm.cancel()
        self.stop_base()
        self._transition(state, detail)

    def start_callback(self, request, response):
        if self.state not in ('IDLE', 'COMPLETE'):
            response.success = False
            response.message = f'Cannot start from {self.state}. Restart after STOPPED or ERROR.'
            return response
        problem = self._sensor_problem(time.monotonic(), ('rgb', 'depth', 'odom'))
        if problem or not self.controller_ready or not self.ready_client.service_is_ready():
            response.success = False
            response.message = problem or 'Controller is not ready'
            return response
        try:
            self.arm.preflight()
            self.arm.begin('ready')
        except Exception as exc:
            self._stop('ERROR', f'Arm preparation failed: {exc}')
            response.success, response.message = False, self.detail
            return response
        self.home, self.home_frame = self.pose, self.odom_frame
        self.mission_started = time.monotonic()
        self.approach_started = None
        self.target_color = self.target_center = None
        self.target_count = self.grasp_count = self.lost_count = 0
        self.grasp_depth_stamp = None
        self.pick_verified = False
        self.observation = None
        self.processed_id = self.observation_id
        self.settle_started = None
        self._transition('PREPARE', 'Saved starting pose; preparing the arm')
        response.success, response.message = True, 'Mission started'
        return response

    def stop_callback(self, request, response):
        self._stop('STOPPED', 'Operator stop; remaining arm steps cancelled')
        response.success, response.message = True, self.detail
        return response

    def confirm_pick_callback(self, request, response):
        if self.state != 'VERIFY_PICK':
            response.success, response.message = False, 'Not waiting for pickup confirmation'
        else:
            self.pick_verified = True
            self._transition('RETURN_HOME', 'Operator confirmed pickup; returning home')
            response.success, response.message = True, 'Pickup confirmed'
        return response

    def control_tick(self):
        if self.shutting_down:
            return
        try:
            self._control_tick(time.monotonic())
        except Exception as exc:
            self._stop('ERROR', f'Control failed: {exc}')

    def _control_tick(self, now):
        if now - self.last_status_time >= 0.5:
            self._publish_status()
        if self.state == 'INIT':
            self.stop_base()
            if now - self.state_started > self.config['startup_timeout_sec']:
                self._stop('ERROR', 'Startup timed out waiting for controller and sensors')
                return
            if self.ready_future is None and self.ready_client.service_is_ready():
                self.ready_future = self.ready_client.call_async(Trigger.Request())
            if self.ready_future is not None and self.ready_future.done():
                result = self.ready_future.result()
                if result is None or not result.success:
                    self._stop('ERROR', 'Controller initialization check failed')
                    return
                self.controller_ready = True
            if self.controller_ready and not self._sensor_problem(now, ('rgb', 'depth', 'odom')):
                self._transition('IDLE', 'Ready; call /retrieve_object/start to begin')
            return
        if self.state not in ACTIVE_STATES:
            self.stop_base()
            if self.state == 'IDLE' and self.config['auto_start'] and not self.auto_start_used:
                self.auto_start_used = True
                self.start_callback(None, Trigger.Response())
            return
        if now - self.mission_started > self.config['mission_timeout_sec']:
            self._stop('ERROR', 'Mission timed out')
            return
        problem = self._sensor_problem(now, ('rgb', 'depth', 'odom'))
        if problem:
            self._stop('ERROR', problem)
            return
        if not self.ready_client.service_is_ready():
            self._stop('ERROR', 'Controller readiness service disappeared')
            return
        if math.hypot(self.pose.x - self.home.x, self.pose.y - self.home.y) > self.config['max_home_distance_m']:
            self._stop('ERROR', 'Robot exceeded the configured distance from home')
            return
        if self.state in ('PREPARE', 'PICK', 'PLACE'):
            self.stop_base()
            if now - self.state_started > self.config['arm_timeout_sec']:
                self._stop('ERROR', 'Arm sequence timed out')
                return
            self.arm.tick(now)
            if not self.arm.active:
                if self.state == 'PREPARE':
                    self._transition('SEARCH', 'Searching for a colored object')
                elif self.state == 'PICK':
                    if self.config['allow_unverified_pick']:
                        self._transition('RETURN_HOME', 'Pickup commanded but unverified; returning home')
                    else:
                        self._transition('VERIFY_PICK', 'Confirm the object is held using /retrieve_object/confirm_pick')
                else:
                    self._transition('COMPLETE', 'At home; placement sequence finished (release unverified)')
            return
        if self.state == 'VERIFY_PICK':
            self.stop_base()
            if now - self.state_started > self.config['verify_timeout_sec']:
                self._stop('ERROR', 'Pickup confirmation timed out; object may still be held')
            return
        if self.state == 'RETURN_HOME':
            self._return_home(now)
            return
        if self.state == 'SEARCH' and now - self.state_started > self.config['search_timeout_sec']:
            self._stop('ERROR', 'Search timed out')
            return
        if self.approach_started is not None and now - self.approach_started > self.config['approach_timeout_sec']:
            self._stop('ERROR', 'Target acquisition/approach timed out')
            return
        self._track_and_drive(now)

    def _track_and_drive(self, now):
        # Confirmation counters advance only on distinct camera frames.
        new_frame = self.observation_id != self.processed_id
        self.processed_id = self.observation_id
        twist = Twist()
        if self.observation is None:
            self.stop_base()
            if new_frame:
                self.lost_count += 1
                self.grasp_count = self.target_count = 0
                if self.lost_count >= self.config['lost_frames_limit']:
                    self.target_color = self.target_center = None
                    if self.state != 'SEARCH':
                        self._transition('SEARCH', 'Target lost; searching again')
            if self.state == 'SEARCH':
                twist.angular.z = self.config['search_yaw_speed']
                self.cmd_pub.publish(twist)
            return
        cx, cy, width, color, depth, depth_stamp = self.observation
        if new_frame:
            self.lost_count = 0
            self.target_color, self.target_center = color, (cx, cy)
            self.target_count += 1
        if self.target_count < self.config['detection_confirm_frames']:
            self.stop_base()
            return
        if self.approach_started is None:
            self.approach_started = now
        if depth is None:
            self.grasp_count = 0
            self.stop_base()
            return
        if depth < self.config['grasp_distance_min']:
            # Do not reverse into an area the forward camera cannot see.
            self._stop('ERROR', 'Object is too close; reposition the robot manually')
            return
        error = cx - width / 2.0
        if abs(error) >= self.config['center_tolerance_px']:
            if self.state != 'ALIGN':
                self._transition('ALIGN', 'Centering the selected object')
            self.grasp_count = 0
            twist.angular.z = clamp(-0.004 * error, -self.config['max_yaw_speed'], self.config['max_yaw_speed'])
        elif depth <= self.config['grasp_distance_max']:
            self.stop_base()
            if new_frame and depth_stamp != self.grasp_depth_stamp:
                self.grasp_count += 1
                self.grasp_depth_stamp = depth_stamp
            if self.grasp_count >= self.config['confirm_frames']:
                self.arm.begin('pick')
                self._transition('PICK', 'Target stable in pickup range; picking')
            return
        else:
            if self.state != 'APPROACH':
                self._transition('APPROACH', 'Approaching the selected object')
            self.grasp_count = 0
            # Aim inside the pickup band so speed does not approach zero
            # before crossing its outer boundary.
            pickup_distance = (self.config['grasp_distance_min'] +
                               self.config['grasp_distance_max']) / 2.0
            twist.linear.x = min(self.config['max_linear_speed'],
                                 0.4 * (depth - pickup_distance))
            twist.angular.z = clamp(-0.004 * error, -self.config['max_yaw_speed'], self.config['max_yaw_speed'])
        self.cmd_pub.publish(twist)

    def _return_home(self, now):
        if now - self.state_started > self.config['return_timeout_sec']:
            self._stop('ERROR', 'Return home timed out')
            return
        linear, yaw, arrived = return_command(
            self.pose, self.home, self.config['home_position_tolerance_m'],
            self.config['home_heading_tolerance_rad'], self.config['return_linear_speed'],
            self.config['max_yaw_speed'], self.config['return_turn_tolerance_rad'])
        if arrived:
            self.stop_base()
            if self.settle_started is None:
                self.settle_started = now
            if now - self.settle_started >= self.config['home_settle_sec']:
                self.arm.begin('place')
                self._transition('PLACE', 'At starting pose; placing in front of the robot')
            return
        self.settle_started = None
        twist = Twist()
        twist.linear.x, twist.angular.z = linear, yaw
        self.cmd_pub.publish(twist)

    def shutdown(self):
        self.shutting_down = True
        self.arm.cancel()
        if rclpy.ok():
            self.stop_base()
        cv2.destroyAllWindows()


def main(args=None):
    # Keep the ROS context alive through KeyboardInterrupt so the final zero
    # velocity can be published before shutting it down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = RetrieveObjectNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
