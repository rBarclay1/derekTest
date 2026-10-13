"""Timer-driven arm sequences; no sleeps or background motion threads."""

import math
import os
import sqlite3
from collections import deque

from rcl_interfaces.msg import ParameterDescriptor
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class ArmController:
    def __init__(self, node, servo_topic, action_path):
        self.node = node
        self.pub = node.create_publisher(ServosPosition, servo_topic, 1)
        self.action_path = action_path
        self.steps = deque()
        self.deadline = None
        self.cancelled = False
        defaults = {
            'use_action_groups': False,
            'ready_action': 'pick_init',
            'pick_action': 'pick',
            'place_action': 'place_center',
            'gripper_min': 200,
            'gripper_max': 700,
            'gripper_open': 200,
            'gripper_closed': 600,
            # Flattened servo ID / pulse pairs, excluding the gripper.
            'ready_pose': [1, 500, 2, 750, 3, 40, 4, 210, 5, 500],
            'grasp_pose': [1, 500, 2, 620, 3, 160, 4, 280, 5, 500],
            'carry_pose': [1, 500, 2, 750, 3, 40, 4, 210, 5, 500],
            'place_pose': [1, 500, 2, 620, 3, 160, 4, 280, 5, 500],
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.config = {name: node.get_parameter(name).value for name in defaults}
        low, high = self.config['gripper_min'], self.config['gripper_max']
        if not 0 <= low < high <= 1000:
            raise ValueError('Gripper limits must satisfy 0 <= min < max <= 1000')
        for name in ('gripper_open', 'gripper_closed'):
            if not low <= self.config[name] <= high:
                raise ValueError(f'{name} must be within the gripper limits')
        self.poses = {}
        for name in ('ready_pose', 'grasp_pose', 'carry_pose', 'place_pose'):
            values = self.config[name]
            if not values or len(values) % 2:
                raise ValueError(f'{name} must contain servo ID / pulse pairs')
            positions = list(zip(values[::2], values[1::2]))
            ids = [servo_id for servo_id, _ in positions]
            if len(set(ids)) != len(ids) or any(i not in range(1, 6) for i in ids):
                raise ValueError(f'{name} must use unique arm servo IDs 1–5')
            self.poses[name] = self._validate_positions(positions)

    def _validate_positions(self, positions):
        validated = []
        for servo_id, pulse in positions:
            if servo_id not in (1, 2, 3, 4, 5, 10):
                raise ValueError(f'Unsupported arm servo ID: {servo_id}')
            pulse = float(pulse)
            if not math.isfinite(pulse) or not 0 <= pulse <= 1000:
                raise ValueError(f'Invalid servo pulse: {pulse}')
            if servo_id == 10:
                bounded = max(self.config['gripper_min'], min(self.config['gripper_max'], pulse))
                if bounded != pulse:
                    self.node.get_logger().warn(f'Clamped gripper pulse {pulse} to {bounded}')
                pulse = bounded
            validated.append((int(servo_id), pulse))
        return validated

    def _pose(self, name, closed=False):
        gripper = self.config['gripper_closed' if closed else 'gripper_open']
        return self.poses[name] + [(10, gripper)]

    def _action(self, name):
        if not self.config['use_action_groups']:
            return None
        if not name or os.path.basename(name) != name or name in ('.', '..'):
            raise ValueError('Action names must be simple filenames without .d6a')
        path = os.path.join(self.action_path, name + '.d6a')
        if not os.path.isfile(path):
            raise ValueError(f'Configured action file is missing: {path}')
        # Read all rows before publishing. Vendor format: ID, ms, five arm
        # pulses, and optionally a sixth pulse for gripper servo 10.
        with sqlite3.connect(f'file:{path}?mode=ro', uri=True) as connection:
            rows = connection.execute('SELECT * FROM ActionGroup ORDER BY 1').fetchall()
        if not rows:
            raise ValueError(f'Empty action group: {path}')
        steps = []
        for row in rows:
            if not 3 <= len(row) <= 8:
                raise ValueError(f'Unexpected action group row in {path}')
            duration = float(row[1]) / 1000.0
            if not math.isfinite(duration) or not 0 < duration <= 30:
                raise ValueError(f'Invalid action duration in {path}')
            positions = [(10 if i == 5 else i + 1, pulse)
                         for i, pulse in enumerate(row[2:])]
            steps.append((duration, self._validate_positions(positions)))
        return steps

    def preflight(self):
        """Validate configured saved actions before starting a mission."""
        for key in ('ready_action', 'pick_action', 'place_action'):
            self._action(self.config[key])

    def begin(self, operation):
        if self.active:
            raise RuntimeError('An arm sequence is already active')
        opened = [(10, self.config['gripper_open'])]
        closed = [(10, self.config['gripper_closed'])]
        if operation in self.poses:
            # Single calibration pose, preserving the current gripper state.
            steps = [(1.5, self.poses[operation])]
        elif operation == 'open':
            steps = [(0.6, opened)]
        elif operation == 'close':
            steps = [(0.6, closed)]
        elif operation == 'ready':
            steps = self._action(self.config['ready_action']) or [(1.0, self._pose('ready_pose'))]
            steps = steps + [(0.5, opened)]
        elif operation == 'pick':
            steps = self._action(self.config['pick_action']) or [
                (1.2, self._pose('grasp_pose')), (0.6, closed)]
            steps = steps + [(1.2, self._pose('carry_pose', closed=True))]
        elif operation == 'place':
            steps = self._action(self.config['place_action']) or [
                (1.2, self._pose('place_pose', closed=True)), (0.6, opened)]
            steps = steps + [(1.0, self._pose('ready_pose'))]
        else:
            raise ValueError(f'Unknown arm operation: {operation}')
        self.cancelled = False
        self.steps = deque(steps)
        self.deadline = None

    @property
    def active(self):
        return bool(self.steps) or self.deadline is not None

    def tick(self, now):
        if self.cancelled or (self.deadline is not None and now < self.deadline):
            return
        self.deadline = None
        if not self.steps:
            return
        duration, positions = self.steps.popleft()
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = 'pulse'
        for servo_id, pulse in self._validate_positions(positions):
            servo = ServoPosition()
            servo.id, servo.position = servo_id, float(pulse)
            msg.position.append(servo)
        self.pub.publish(msg)
        self.deadline = now + duration + 0.05

    def cancel(self):
        """Prevent subsequent steps; an already sent servo move may finish."""
        self.cancelled = True
        self.steps.clear()
        self.deadline = None
