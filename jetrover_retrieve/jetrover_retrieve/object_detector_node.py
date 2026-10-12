#!/usr/bin/env python3
"""Inspect color detections. Published x/y are pixels, z is depth in meters."""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from jetrover_retrieve.color_detect import detect_largest_object, depth_to_meters


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        for name, value in {
            'target_color': 'red', 'color_topic': '/depth_cam/rgb/image_raw',
            'depth_topic': '/depth_cam/depth/image_raw', 'min_object_area': 500.0,
            'show_debug_image': False, 'sensor_timeout_sec': 0.5,
            'max_rgb_depth_skew_sec': 0.15, 'min_valid_depth_m': 0.05,
            'max_valid_depth_m': 5.0,
        }.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.latest_depth = None
        self.depth_stamp = None
        self.depth_received = None
        self.depth_encoding = None
        self.create_subscription(Image, self.get_parameter('color_topic').value,
                                 self.color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self.depth_callback, qos_profile_sensor_data)
        self.point_pub = self.create_publisher(PointStamped, '/detected_object_point', 10)
        self.get_logger().info('Detector ready; points contain pixel x/y and meter depth, not XYZ')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def fresh_stamp(self, msg):
        age = self.get_clock().now().nanoseconds / 1e9 - self.stamp(msg)
        return self.stamp(msg) > 0 and -0.1 <= age <= self.get_parameter('sensor_timeout_sec').value

    def depth_callback(self, msg):
        if not self.fresh_stamp(msg):
            return
        try:
            if msg.encoding.upper() not in ('16UC1', '32FC1'):
                raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if image.ndim != 2:
                raise ValueError('Depth image must have one channel')
            self.latest_depth = image
            self.depth_stamp, self.depth_received = self.stamp(msg), time.monotonic()
            self.depth_encoding = msg.encoding
        except Exception as exc:
            self.get_logger().error(f'Depth conversion failed: {exc}')

    def color_callback(self, msg):
        if not self.fresh_stamp(msg):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = detect_largest_object(frame, self.get_parameter('target_color').value,
                                           self.get_parameter('min_object_area').value)
            vis = frame
            if result is not None:
                cx, cy, area, color, vis = result
                depth = None
                if (self.latest_depth is not None and self.latest_depth.shape == frame.shape[:2]
                        and time.monotonic() - self.depth_received <= self.get_parameter('sensor_timeout_sec').value
                        and abs(self.stamp(msg) - self.depth_stamp) <= self.get_parameter('max_rgb_depth_skew_sec').value):
                    depth = depth_to_meters(
                        self.latest_depth, cx, cy, self.depth_encoding,
                        self.get_parameter('min_valid_depth_m').value,
                        self.get_parameter('max_valid_depth_m').value)
                # Unknown depth is not published as a valid zero-distance point.
                if depth is not None:
                    point = PointStamped()
                    point.header = msg.header
                    point.point.x, point.point.y, point.point.z = float(cx), float(cy), depth
                    self.point_pub.publish(point)
            if self.get_parameter('show_debug_image').value:
                cv2.imshow('detector', vis)
                cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f'Detection failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
