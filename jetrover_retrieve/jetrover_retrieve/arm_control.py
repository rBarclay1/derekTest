"""Arm / gripper helpers for JetRover retrieve.

Prefers Hiwonder action groups (pick, place_*, pick_init) when those .d6a
files exist. Falls back to a conservative servo sequence so the node still
runs if the action files are missing.

Gripper servo 10 must stay in [200, 700]. 200 is open, ~600 is closed.
"""

import os
import time

from servo_controller_msgs.msg import ServoPosition, ServosPosition

try:
    from servo_controller.action_group_controller import ActionGroupController
except ImportError:
    ActionGroupController = None

GRIPPER_OPEN = 200
GRIPPER_CLOSED = 600
GRIPPER_MIN = 200
GRIPPER_MAX = 700

# Look-down pose used by Hiwonder object tracking.
READY_POSE = ((1, 500), (2, 750), (3, 40), (4, 210), (5, 500), (10, GRIPPER_OPEN))
# Lowered pose in front of the chassis. Tune on the real robot if grasp misses.
GRASP_POSE = ((1, 500), (2, 620), (3, 160), (4, 280), (5, 500), (10, GRIPPER_OPEN))
LIFT_POSE = ((1, 500), (2, 750), (3, 40), (4, 210), (5, 500), (10, GRIPPER_CLOSED))

PLACE_BASE = {
    'red': 500,
    'green': 350,
    'blue': 650,
}


class ArmController:
    def __init__(self, node, servo_topic, action_path):
        self.node = node
        self.pub = node.create_publisher(ServosPosition, servo_topic, 1)
        self.actions = None
        if ActionGroupController is not None and os.path.isdir(action_path):
            self.actions = ActionGroupController(self.pub, action_path)
            node.get_logger().info(f'Using action groups in {action_path}')
        else:
            node.get_logger().warn(
                'Action groups not found. Using built-in servo poses. '
                'Tune GRASP_POSE if the gripper misses.')

    def _set_servos(self, duration, positions):
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = 'pulse'
        data = []
        for servo_id, pulse in positions:
            servo = ServoPosition()
            servo.id = int(servo_id)
            pulse = float(pulse)
            if servo_id == 10:
                pulse = max(GRIPPER_MIN, min(GRIPPER_MAX, pulse))
            servo.position = pulse
            data.append(servo)
        msg.position = data
        self.pub.publish(msg)
        time.sleep(duration + 0.05)

    def _run_action(self, name):
        if self.actions is None:
            return False
        path = os.path.join(self.actions.action_path, name + '.d6a')
        if not os.path.exists(path):
            self.node.get_logger().warn(f'Action file missing: {path}')
            return False
        self.actions.run_action(name)
        return True

    def go_ready(self):
        if self._run_action('pick_init'):
            return
        self._set_servos(1.0, READY_POSE)

    def pick(self):
        if self._run_action('pick'):
            return
        self._set_servos(1.2, GRASP_POSE)
        self._set_servos(0.6, ((10, GRIPPER_CLOSED),))
        self._set_servos(1.2, LIFT_POSE)

    def place(self, color_name):
        action_by_color = {
            'red': 'place_center',
            'green': 'place_left',
            'blue': 'place_right',
        }
        if self._run_action(action_by_color.get(color_name, 'place_center')):
            return
        base = PLACE_BASE.get(color_name, 500)
        self._set_servos(1.0, ((1, base), (2, 620), (3, 160), (4, 280), (5, 500), (10, GRIPPER_CLOSED)))
        self._set_servos(0.5, ((10, GRIPPER_OPEN),))
        self._set_servos(1.0, READY_POSE)
