"""Test the mission and arm logic without ROS or a robot."""

import importlib
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jetrover_retrieve.navigation import Pose2D, return_command, wrap_angle


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class ServosPosition:
    def __init__(self):
        self.position = []


def load_control_modules():
    """Import real control code with only external ROS/image interfaces replaced."""
    definitions = {
        'cv2': {}, 'rclpy': {}, 'cv_bridge': {'CvBridge': object},
        'geometry_msgs.msg': {'Twist': Twist}, 'nav_msgs.msg': {'Odometry': object},
        'rcl_interfaces.msg': {'ParameterDescriptor': Mock()},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'DurabilityPolicy': Mock(), 'QoSProfile': Mock(),
                      'qos_profile_sensor_data': object()},
        'rclpy.signals': {'SignalHandlerOptions': Mock()},
        'sensor_msgs.msg': {'Image': object}, 'std_msgs.msg': {'String': types.SimpleNamespace},
        'std_srvs.srv': {'Trigger': types.SimpleNamespace(Response=types.SimpleNamespace)},
        'servo_controller_msgs.msg': {'ServoPosition': types.SimpleNamespace,
                                      'ServosPosition': ServosPosition},
        'jetrover_retrieve.color_detect': {'detect_largest_object': Mock(), 'depth_to_meters': Mock()},
    }
    stubs = {}
    for name, values in definitions.items():
        module = types.ModuleType(name)
        vars(module).update(values)
        stubs[name] = module
    with patch.dict(sys.modules, stubs):
        arm = importlib.import_module('jetrover_retrieve.arm_control')
        mission = importlib.import_module('jetrover_retrieve.retrieve_object_node')
    return arm, mission


arm_module, mission_module = load_control_modules()


def mission(mode='return_test'):
    node = mission_module.RetrieveObjectNode.__new__(mission_module.RetrieveObjectNode)
    node.config = dict(
        mission_mode=mode, return_test_distance_m=0.3, return_test_timeout_sec=20.,
        return_linear_speed=0.03, max_yaw_speed=0.2, max_linear_speed=0.04,
        home_position_tolerance_m=0.08, home_heading_tolerance_rad=0.12,
        return_turn_tolerance_rad=0.25, home_settle_sec=0.5,
        return_timeout_sec=90., sensor_timeout_sec=0.5, mission_timeout_sec=300.,
        max_home_distance_m=1., auto_start=False, detection_confirm_frames=1,
        confirm_frames=1, grasp_distance_min=0.16, grasp_distance_max=0.28,
        center_tolerance_px=40., lost_frames_limit=20, search_yaw_speed=0.12,
        arm_timeout_sec=30., allow_unverified_pick=False,
        verify_timeout_sec=120., search_timeout_sec=60., approach_timeout_sec=60.)
    node.state = 'IDLE'
    node.state_started = node.mission_started = node.last_status_time = 100.
    node.received = dict(rgb=100., depth=100., odom=100.)
    node.pose = node.home = Pose2D(0., 0., 0.)
    node.odom_frame = node.home_frame = ('odom', 'base_footprint')
    node.controller_ready = True
    node.ready_client = Mock()
    node.ready_client.service_is_ready.return_value = True
    node.arm = Mock()
    node.arm.active = False
    node.cmd_pub = Mock()
    node.get_logger = Mock()
    node._publish_status = Mock()
    node.observation_id = 1
    node.processed_id = 0
    node.observation = (320, 240, 640, 'red', 0.22, 1.)
    node.target_count = node.grasp_count = node.lost_count = 0
    node.grasp_depth_stamp = node.approach_started = node.settle_started = None
    return node


class NavigationTests(unittest.TestCase):
    def test_return_converges_from_varied_poses(self):
        home = Pose2D(0.2, -0.1, -3.1)
        for initial in (Pose2D(.8, .8, 0.), Pose2D(-.5, .4, 3.1), Pose2D(.2, -.1, 1.)):
            with self.subTest(initial=initial):
                pose = initial
                for _ in range(4000):
                    speed, yaw, arrived = return_command(pose, home, .08, .12, .03, .2, .25)
                    self.assertLessEqual(abs(speed), .03)
                    self.assertLessEqual(abs(yaw), .2)
                    if arrived:
                        break
                    pose = Pose2D(pose.x + speed * math.cos(pose.yaw) * .05,
                                  pose.y + speed * math.sin(pose.yaw) * .05,
                                  wrap_angle(pose.yaw + yaw * .05))
                self.assertTrue(arrived)
                self.assertLessEqual(math.hypot(home.x-pose.x, home.y-pose.y), .08)
                self.assertLessEqual(abs(wrap_angle(home.yaw-pose.yaw)), .12)

    def test_turns_before_driving_toward_home(self):
        speed, yaw, arrived = return_command(Pose2D(.3, 0., 0.), Pose2D(0., 0., 0.),
                                             .08, .12, .03, .2, .25)
        self.assertEqual(speed, 0.)
        self.assertNotEqual(yaw, 0.)
        self.assertFalse(arrived)


class MissionTests(unittest.TestCase):
    def setUp(self):
        clock = patch.object(mission_module.time, 'monotonic', return_value=100.)
        clock.start()
        self.addCleanup(clock.stop)

    def test_test_modes_start_without_arm_steps(self):
        for mode, state in (('approach_only', 'SEARCH'), ('return_test', 'TEST_OUTBOUND')):
            node = mission(mode)
            response = node.start_callback(None, types.SimpleNamespace())
            self.assertTrue(response.success)
            self.assertEqual(node.state, state)
            node.arm.begin.assert_not_called()

    def test_full_mode_still_prepares_arm(self):
        node = mission('full')
        node.start_callback(None, types.SimpleNamespace())
        self.assertEqual(node.state, 'PREPARE')
        node.arm.begin.assert_called_once_with('ready')

    def test_outbound_transitions_and_completes_without_arm(self):
        node = mission()
        node.state = 'TEST_OUTBOUND'
        node._control_tick(100.1)
        self.assertEqual(node.cmd_pub.publish.call_args.args[0].linear.x, .03)
        node.pose = Pose2D(.31, 0., 0.)
        node._control_tick(100.2)
        self.assertEqual(node.state, 'RETURN_HOME')
        node.pose = node.home
        node.state_started = 100.2
        node._return_home(100.3)
        node._return_home(100.9)
        self.assertEqual(node.state, 'COMPLETE')
        node.arm.begin.assert_not_called()

    def test_approach_only_completes_without_pick(self):
        node = mission('approach_only')
        node.state = 'APPROACH'
        node._track_and_drive(100.1)
        self.assertEqual(node.state, 'COMPLETE')
        node.arm.begin.assert_not_called()
        self.assertEqual(node.cmd_pub.publish.call_args.args[0].linear.x, 0.)

    def test_invalid_depth_does_not_move(self):
        node = mission('approach_only')
        node.state = 'APPROACH'
        node.observation = (400, 240, 640, 'red', None, 1.)
        node._track_and_drive(100.1)
        cmd = node.cmd_pub.publish.call_args.args[0]
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0., 0.))
        node.arm.begin.assert_not_called()

    def test_too_close_errors_without_arm(self):
        node = mission('approach_only')
        node.state = 'APPROACH'
        node.observation = (320, 240, 640, 'red', .1, 1.)
        node._track_and_drive(100.1)
        self.assertEqual(node.state, 'ERROR')
        node.arm.begin.assert_not_called()

    def test_stale_sensors_stop_every_active_state(self):
        for state in mission_module.ACTIVE_STATES:
            for stream in ('rgb', 'depth', 'odom'):
                with self.subTest(state=state, stream=stream):
                    node = mission()
                    node.state = state
                    node.received[stream] = 99.
                    node._control_tick(100.1)
                    self.assertEqual(node.state, 'ERROR')
                    node.arm.cancel.assert_called_once()
                    cmd = node.cmd_pub.publish.call_args.args[0]
                    self.assertEqual((cmd.linear.x, cmd.angular.z), (0., 0.))

    def test_outbound_timeout_stops_frozen_odometry(self):
        node = mission()
        node.state = 'TEST_OUTBOUND'
        node.state_started = 70.
        node._control_tick(100.1)
        self.assertEqual(node.state, 'ERROR')
        self.assertIn('outward leg timed out', node.detail)

    def test_operator_stop_cancels_and_cannot_restart(self):
        node = mission()
        node.state = 'TEST_OUTBOUND'
        node.stop_callback(None, types.SimpleNamespace())
        node.arm.cancel.assert_called_once()
        self.assertEqual(node.state, 'STOPPED')
        self.assertFalse(node.start_callback(None, types.SimpleNamespace()).success)

    def test_full_pick_requires_confirmation(self):
        node = mission('full')
        node.state = 'PICK'
        node._control_tick(100.1)
        self.assertEqual(node.state, 'VERIFY_PICK')
        self.assertTrue(node.confirm_pick_callback(None, types.SimpleNamespace()).success)
        self.assertEqual(node.state, 'RETURN_HOME')
        self.assertTrue(node.pick_verified)


class ArmTests(unittest.TestCase):
    def arm(self, overrides=None):
        node = Mock()
        params = dict(overrides or {})

        def declare_parameter(name, value, *args):
            params.setdefault(name, value)

        def get_parameter(name):
            return types.SimpleNamespace(value=params[name])

        node.declare_parameter.side_effect = declare_parameter
        node.get_parameter.side_effect = get_parameter
        return arm_module.ArmController(node, '/servo_controller')

    def test_pick_waits_for_grasp_then_closes_and_lifts(self):
        arm = self.arm()
        arm.begin('pick')
        arm.tick(100.0)
        first = arm.pub.publish.call_args.args[0]
        self.assertEqual(first.duration, 1.2)
        self.assertEqual([(p.id, p.position) for p in first.position],
                         [(1, 500), (2, 620), (3, 160), (4, 280), (5, 500), (10, 200)])
        arm.tick(101.0)
        self.assertEqual(arm.pub.publish.call_count, 1)
        arm.tick(101.3)
        close = arm.pub.publish.call_args.args[0]
        self.assertEqual([(p.id, p.position) for p in close.position], [(10, 600)])
        arm.tick(102.0)
        carry = arm.pub.publish.call_args.args[0]
        self.assertEqual([(p.id, p.position) for p in carry.position],
                         [(1, 500), (2, 750), (3, 40), (4, 210), (5, 500), (10, 600)])
        self.assertTrue(arm.active)
        arm.tick(103.3)
        self.assertFalse(arm.active)
        self.assertEqual(arm.pub.publish.call_count, 3)

    def test_place_lowers_before_releasing_then_returns_to_ready(self):
        arm = self.arm()
        arm.begin('place')
        arm.tick(100.0)
        arm.tick(101.3)
        arm.tick(102.0)
        messages = [call.args[0] for call in arm.pub.publish.call_args_list]
        self.assertEqual([msg.duration for msg in messages], [1.2, 0.6, 1.0])
        self.assertEqual([[(p.id, p.position) for p in msg.position] for msg in messages], [
            [(1, 500), (2, 620), (3, 160), (4, 280), (5, 500), (10, 600)],
            [(10, 200)],
            [(1, 500), (2, 750), (3, 40), (4, 210), (5, 500), (10, 200)],
        ])
        arm.tick(103.1)
        self.assertFalse(arm.active)

    def test_invalid_pose_or_gripper_settings_are_rejected(self):
        for overrides in [
            {'ready_pose': [1, 500, 1, 600]},
            {'ready_pose': [10, 500]},
            {'ready_pose': [1, 1001]},
            {'ready_pose': [1]},
            {'gripper_open': 100},
            {'gripper_min': 800},
        ]:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    self.arm(overrides)

    def test_can_start_a_new_operation_after_cancel(self):
        arm = self.arm()
        arm.begin('pick')
        arm.tick(100.0)
        arm.cancel()
        arm.begin('open')
        arm.tick(100.1)
        self.assertEqual(arm.pub.publish.call_count, 2)
        msg = arm.pub.publish.call_args.args[0]
        self.assertEqual([(p.id, p.position) for p in msg.position], [(10, 200)])

    def test_single_pose_leaves_gripper_alone_and_cancel_prevents_next_step(self):
        arm = self.arm()
        arm.begin('grasp_pose')
        arm.tick(100.)
        msg = arm.pub.publish.call_args.args[0]
        self.assertEqual([p.id for p in msg.position], [1, 2, 3, 4, 5])
        arm.cancel()
        arm.tick(200.)
        self.assertEqual(arm.pub.publish.call_count, 1)
        self.assertFalse(arm.active)

    def test_cancel_during_pick_prevents_close_and_lift(self):
        arm = self.arm()
        arm.begin('pick')
        arm.tick(100.)
        arm.cancel()
        arm.tick(200.)
        arm.pub.publish.assert_called_once()
        self.assertFalse(arm.active)

    def test_gripper_commands_only_address_gripper(self):
        for operation, expected in (('open', 200.), ('close', 600.)):
            arm = self.arm()
            arm.begin(operation)
            arm.tick(100.)
            positions = arm.pub.publish.call_args.args[0].position
            self.assertEqual([(p.id, p.position) for p in positions], [(10, expected)])


if __name__ == '__main__':
    unittest.main()
