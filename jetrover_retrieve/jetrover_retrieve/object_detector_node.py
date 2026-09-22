#!/usr/bin/env python3
"""Detect a solid-colored object and publish its pixel location and depth."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

from jetrover_retrieve.color_detect import detect_largest_object, depth_to_meters


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('color_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('min_object_area', 500.0)
        self.declare_parameter('show_debug_image', True)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.target_color = self.get_parameter('target_color').value
        self.min_area = self.get_parameter('min_object_area').value
        self.show_debug = self.get_parameter('show_debug_image').value

        self.create_subscription(
            Image, self.get_parameter('color_topic').value, self.color_callback, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.depth_callback, 10)
        self.point_pub = self.create_publisher(PointStamped, '/detected_object_point', 10)
        self.get_logger().info(
            f'Detector started. Looking for {self.target_color} on '
            f'{self.get_parameter("color_topic").value}')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = detect_largest_object(frame, self.target_color, self.min_area)
        if result is None:
            if self.show_debug:
                cv2.imshow('detector', frame)
                cv2.waitKey(1)
            return

        cx, cy, area, color_name, vis = result
        depth_m = depth_to_meters(self.latest_depth, cx, cy)
        self.get_logger().info(
            f'{color_name} at pixel ({cx}, {cy}) area={area:.0f} depth={depth_m}')

        point_msg = PointStamped()
        point_msg.header = msg.header
        point_msg.point.x = float(cx)
        point_msg.point.y = float(cy)
        point_msg.point.z = depth_m if depth_m is not None else 0.0
        self.point_pub.publish(point_msg)

        if self.show_debug:
            cv2.imshow('detector', vis)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
