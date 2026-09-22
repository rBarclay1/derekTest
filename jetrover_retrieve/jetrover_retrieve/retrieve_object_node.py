#!/usr/bin/env python3
"""Search, approach, grasp, and place a colored object on JetRover.

State machine:
  SEARCH  -> rotate until a matching color blob is found
  ALIGN   -> turn so the blob is centered
  APPROACH -> drive forward until depth is in the gripper workspace
  GRASP   -> stop the chassis and close the gripper
  PLACE   -> set the object down (center / left / right by color)
  RESET   -> return the arm to the look-down pose and search again
"""

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from jetrover_retrieve.color_detect import detect_largest_object, depth_to_meters
from jetrover_retrieve.arm_control import ArmController


def clamp(value, low, high):
    return max(low, min(high, value))


class RetrieveObjectNode(Node):
    def __init__(self):
        super().__init__('retrieve_object')
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('show_debug_image', True)
        self.declare_parameter('min_object_area', 500.0)
        self.declare_parameter('center_tolerance_px', 40.0)
        self.declare_parameter('grasp_distance_min', 0.16)
        self.declare_parameter('grasp_distance_max', 0.28)
        self.declare_parameter('search_yaw_speed', 0.35)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_yaw_speed', 0.6)
        self.declare_parameter('confirm_frames', 12)
        self.declare_parameter('lost_frames_limit', 20)
        self.declare_parameter('color_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')
        self.declare_parameter('servo_topic', '/servo_controller')
        self.declare_parameter('action_group_path',
                               '/home/ubuntu/software/arm_pc/ActionGroups')

        self.bridge = CvBridge()
        self.latest_depth = None
        self.state = 'INIT'
        self.busy = False
        self.confirm_count = 0
        self.lost_count = 0
        self.last_color = None
        self.running = True

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 1)
        self.arm = ArmController(
            self,
            self.get_parameter('servo_topic').value,
            self.get_parameter('action_group_path').value,
        )

        self.create_subscription(
            Image, self.get_parameter('color_topic').value, self.color_callback, 1)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.depth_callback, 1)

        threading.Thread(target=self._wait_then_start, daemon=True).start()
        self.get_logger().info(
            f'Retrieve node started. Target color: '
            f'{self.get_parameter("target_color").value}')

    def _wait_then_start(self):
        client = self.create_client(Trigger, '/controller_manager/init_finish')
        if client.wait_for_service(timeout_sec=8.0):
            self.get_logger().info('Controller is ready.')
        else:
            self.get_logger().warn(
                'controller_manager/init_finish not found. Continuing anyway.')
        self.arm.go_ready()
        time.sleep(1.0)
        self.state = 'SEARCH'
        self.get_logger().info('Searching for objects.')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    def color_callback(self, msg):
        if not self.running or self.busy or self.state in ('INIT', 'GRASP', 'PLACE', 'RESET'):
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        target = self.get_parameter('target_color').value
        min_area = self.get_parameter('min_object_area').value
        result = detect_largest_object(frame, target, min_area)

        if result is None:
            self.lost_count += 1
            self.confirm_count = 0
            if self.state != 'SEARCH' and self.lost_count > self.get_parameter('lost_frames_limit').value:
                self.get_logger().info('Lost object. Searching again.')
                self.state = 'SEARCH'
            if self.state == 'SEARCH':
                twist = Twist()
                twist.angular.z = self.get_parameter('search_yaw_speed').value
                self.cmd_pub.publish(twist)
            vis = frame
        else:
            self.lost_count = 0
            cx, cy, area, color_name, vis = result
            depth_m = depth_to_meters(self.latest_depth, cx, cy)
            self._drive_toward(frame, cx, cy, depth_m, color_name)
            if depth_m is not None:
                cv2.putText(vis, f'{depth_m:.2f} m  {self.state}', (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                cv2.putText(vis, f'depth unknown  {self.state}', (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        h, w = frame.shape[:2]
        cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        if self.get_parameter('show_debug_image').value:
            cv2.imshow('retrieve', vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                self.shutdown()

    def _drive_toward(self, frame, cx, cy, depth_m, color_name):
        h, w = frame.shape[:2]
        error_x = cx - (w / 2.0)
        center_tol = self.get_parameter('center_tolerance_px').value
        max_lin = self.get_parameter('max_linear_speed').value
        max_yaw = self.get_parameter('max_yaw_speed').value
        d_min = self.get_parameter('grasp_distance_min').value
        d_max = self.get_parameter('grasp_distance_max').value

        twist = Twist()
        aligned = abs(error_x) < center_tol
        twist.angular.z = clamp(-0.004 * error_x, -max_yaw, max_yaw)

        in_grasp_range = depth_m is not None and d_min <= depth_m <= d_max
        too_close = depth_m is not None and 0 < depth_m < d_min

        if not aligned:
            self.state = 'ALIGN'
            self.confirm_count = 0
            self.cmd_pub.publish(twist)
            return

        if too_close:
            self.state = 'APPROACH'
            twist.linear.x = -0.04
            self.confirm_count = 0
            self.cmd_pub.publish(twist)
            return

        if in_grasp_range:
            self.state = 'APPROACH'
            self.stop_base()
            self.confirm_count += 1
            if self.confirm_count >= int(self.get_parameter('confirm_frames').value):
                self.last_color = color_name
                self.busy = True
                threading.Thread(target=self._grasp_and_place, daemon=True).start()
            return

        self.state = 'APPROACH'
        self.confirm_count = 0
        if depth_m is None:
            # No depth yet: creep forward while staying aligned.
            twist.linear.x = 0.05
        else:
            # Slow down as the object gets closer.
            twist.linear.x = clamp(0.4 * (depth_m - d_max), 0.04, max_lin)
        self.cmd_pub.publish(twist)

    def _grasp_and_place(self):
        try:
            self.state = 'GRASP'
            self.stop_base()
            self.get_logger().info(f'Grasping {self.last_color} object.')
            self.arm.pick()
            self.state = 'PLACE'
            self.arm.place(self.last_color)
            self.state = 'RESET'
            self.arm.go_ready()
            time.sleep(0.5)
        except Exception as exc:
            self.get_logger().error(f'Grasp sequence failed: {exc}')
            self.stop_base()
        finally:
            self.confirm_count = 0
            self.busy = False
            self.state = 'SEARCH'
            self.get_logger().info('Ready to search for the next object.')

    def shutdown(self):
        self.running = False
        self.stop_base()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = RetrieveObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
