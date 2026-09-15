#!/usr/bin/env python3
"""Small, deliberately conservative ROS 2 movement test for JetRover.

Run this ON the robot after sourcing /home/ubuntu/ros2_ws/.robotrc.
It uses the JetRover ROS 2 chassis topic: /controller/cmd_vel.

Examples:
  python3 move_demo.py                         # dry run: does not move
  python3 move_demo.py --run forward
  python3 move_demo.py --run strafe-left --duration 0.7
  python3 move_demo.py --run turn-left --duration 0.5

Keep the robot off the ground for its first test, or leave clear space around
it. Ctrl-C always sends a stop command before the program exits.
"""

import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MOTIONS = {
    "forward": ("linear.x", 1.0),
    "backward": ("linear.x", -1.0),
    "strafe-left": ("linear.y", 1.0),
    "strafe-right": ("linear.y", -1.0),
    "turn-left": ("angular.z", 1.0),
    "turn-right": ("angular.z", -1.0),
}


class MoveDemo(Node):
    def __init__(self):
        super().__init__("jetrover_move_demo")
        self.publisher = self.create_publisher(Twist, "/controller/cmd_vel", 10)

    def stop(self):
        """Publish zero velocity several times so the controller receives it."""
        message = Twist()
        for _ in range(5):
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)

    def move(self, motion, linear_speed, angular_speed, duration):
        field, direction = MOTIONS[motion]
        message = Twist()
        speed = linear_speed if field.startswith("linear") else angular_speed
        parent, child = field.split(".")
        setattr(getattr(message, parent), child, direction * speed)

        self.get_logger().info(
            f"Moving {motion} for {duration:.1f} s "
            f"(linear={linear_speed:.2f} m/s, angular={angular_speed:.2f} rad/s)."
        )
        end_time = self.get_clock().now().nanoseconds + int(duration * 1_000_000_000)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)


def parse_args():
    parser = argparse.ArgumentParser(description="Safe one-motion JetRover ROS 2 test")
    parser.add_argument("motion", choices=MOTIONS, nargs="?", default="forward")
    parser.add_argument("--run", action="store_true", help="actually send the movement command")
    parser.add_argument("--duration", type=float, default=1.0, help="motion duration in seconds (default: 1.0)")
    parser.add_argument("--linear-speed", type=float, default=0.05, help="m/s; default is intentionally slow")
    parser.add_argument("--angular-speed", type=float, default=0.20, help="rad/s; default is intentionally slow")
    args = parser.parse_args()
    if args.duration <= 0 or args.duration > 5:
        parser.error("--duration must be greater than 0 and no more than 5 seconds")
    if not 0 < args.linear_speed <= 0.2:
        parser.error("--linear-speed must be in the range (0, 0.2] m/s")
    if not 0 < args.angular_speed <= 0.5:
        parser.error("--angular-speed must be in the range (0, 0.5] rad/s")
    return args


def main():
    args = parse_args()
    if not args.run:
        print(f"Dry run: would move {args.motion} for {args.duration:.1f} s. Add --run to move.")
        return

    rclpy.init()
    node = MoveDemo()
    try:
        node.stop()  # clear any stale velocity before starting
        node.move(args.motion, args.linear_speed, args.angular_speed, args.duration)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted; stopping the robot.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
