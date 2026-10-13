"""Move the arm through the poses in retrieve.yaml."""

import math

from rcl_interfaces.msg import ParameterDescriptor
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class ArmController:
    def __init__(self, node, servo_topic):
        self.node = node
        self.pub = node.create_publisher(ServosPosition, servo_topic, 1)
        self.steps = []
        self.deadline = None
        defaults = {
            'gripper_min': 200,
            'gripper_max': 700,
            'gripper_open': 200,
            'gripper_closed': 600,
            # Each pair is a servo ID followed by its pulse value.
            'ready_pose': [1, 500, 2, 750, 3, 40, 4, 210, 5, 500],
            'grasp_pose': [1, 500, 2, 620, 3, 160, 4, 280, 5, 500],
            'carry_pose': [1, 500, 2, 750, 3, 40, 4, 210, 5, 500],
            'place_pose': [1, 500, 2, 620, 3, 160, 4, 280, 5, 500],
        }
        self.config = {}
        for name, value in defaults.items():
            node.declare_parameter(name, value, ParameterDescriptor(read_only=True))
            self.config[name] = node.get_parameter(name).value
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
            positions = []
            ids = []
            for i in range(0, len(values), 2):
                servo_id = values[i]
                pulse = values[i + 1]
                if servo_id not in range(1, 6) or servo_id in ids:
                    raise ValueError(f'{name} must use unique arm servo IDs 1–5')
                ids.append(servo_id)
                positions.append((servo_id, pulse))
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
        if closed:
            gripper = self.config['gripper_closed']
        else:
            gripper = self.config['gripper_open']
        return self.poses[name] + [(10, gripper)]

    def begin(self, operation):
        if self.active:
            raise RuntimeError('An arm sequence is already active')
        opened = [(10, self.config['gripper_open'])]
        closed = [(10, self.config['gripper_closed'])]
        if operation in self.poses:
            # A pose check leaves the gripper alone.
            steps = [(1.5, self.poses[operation])]
        elif operation == 'open':
            steps = [(0.6, opened)]
        elif operation == 'close':
            steps = [(0.6, closed)]
        elif operation == 'ready':
            steps = [(1.0, self._pose('ready_pose')), (0.5, opened)]
        elif operation == 'pick':
            steps = [
                (1.2, self._pose('grasp_pose')),
                (0.6, closed),
                (1.2, self._pose('carry_pose', closed=True)),
            ]
        elif operation == 'place':
            steps = [
                (1.2, self._pose('place_pose', closed=True)),
                (0.6, opened),
                (1.0, self._pose('ready_pose')),
            ]
        else:
            raise ValueError(f'Unknown arm operation: {operation}')
        self.steps = steps
        self.deadline = None

    @property
    def active(self):
        return bool(self.steps) or self.deadline is not None

    def tick(self, now):
        if self.deadline is not None and now < self.deadline:
            return
        self.deadline = None
        if not self.steps:
            return
        duration, positions = self.steps.pop(0)
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = 'pulse'
        for servo_id, pulse in self._validate_positions(positions):
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = float(pulse)
            msg.position.append(servo)
        self.pub.publish(msg)
        self.deadline = now + duration + 0.05

    def cancel(self):
        """Prevent subsequent steps; an already sent servo move may finish."""
        self.steps.clear()
        self.deadline = None
