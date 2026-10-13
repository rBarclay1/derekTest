"""Use odometry to drive back to the starting position and heading."""

import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Pose2D:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


def pose_from_odometry(message):
    pose = message.pose.pose
    q = pose.orientation
    values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Odometry contains non-finite values')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('Odometry has an invalid orientation')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return Pose2D(pose.position.x, pose.position.y, yaw)


def return_command(current, home, position_tolerance, heading_tolerance,
                   max_linear, max_yaw, turn_tolerance):
    """Return (forward speed, yaw speed, arrived). No obstacle planning."""
    dx, dy = home.x - current.x, home.y - current.y
    distance = math.hypot(dx, dy)
    if distance <= position_tolerance:
        error = wrap_angle(home.yaw - current.yaw)
        if abs(error) <= heading_tolerance:
            return 0.0, 0.0, True
        return 0.0, clamp(1.5 * error, -max_yaw, max_yaw), False
    error = wrap_angle(math.atan2(dy, dx) - current.yaw)
    yaw = clamp(1.5 * error, -max_yaw, max_yaw)
    linear = 0.0
    if abs(error) <= turn_tolerance:
        linear = min(max_linear, 0.5 * distance)
    return linear, yaw, False
