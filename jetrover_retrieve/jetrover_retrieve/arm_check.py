"""Execute one explicitly selected arm check with the mission node stopped."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from jetrover_retrieve.arm_control import ArmController


class ArmCheck(Node):
    def __init__(self, operation):
        super().__init__('arm_check')
        self.declare_parameter('servo_topic', '/servo_controller')
        self.declare_parameter('action_group_path', '/home/ubuntu/software/arm_pc/ActionGroups')
        self.declare_parameter('controller_ready_service', '/controller_manager/init_finish')
        self.arm = ArmController(self, self.get_parameter('servo_topic').value,
                                 self.get_parameter('action_group_path').value)
        self.operation = operation
        self.started = time.monotonic()
        self.executing = False
        self.done = False
        self.failed = False
        self.future = None
        self.client = self.create_client(Trigger, self.get_parameter('controller_ready_service').value)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Waiting for controller before arm operation: {operation}')

    def finish(self, detail, failed=False):
        self.arm.cancel()
        self.failed, self.done = failed, True
        self.get_logger().info(detail)

    def stop(self, request, response):
        self.finish('Stopped; an already sent servo move may finish', failed=True)
        response.success, response.message = True, 'Remaining arm steps cancelled'
        return response

    def tick(self):
        if self.done:
            return
        try:
            now = time.monotonic()
            if not self.executing:
                if now - self.started > 10.0:
                    raise RuntimeError('Controller/service/subscriber unavailable after 10 seconds')
                if self.future is None and self.client.service_is_ready():
                    self.future = self.client.call_async(Trigger.Request())
                if self.future is None or not self.future.done():
                    return
                result = self.future.result()
                if result is None or not result.success:
                    raise RuntimeError('Controller readiness check failed')
                if self.arm.pub.get_subscription_count() == 0:
                    return
                self.arm.begin(self.operation)
                self.executing, self.started = True, now
                self.get_logger().info(f'Executing {self.operation}')
            if not self.client.service_is_ready():
                raise RuntimeError('Controller readiness service disappeared')
            if now - self.started > 30.0:
                raise RuntimeError('Arm operation exceeded 30 seconds')
            self.arm.tick(now)
            if not self.arm.active:
                self.finish('Command sequence complete; inspect the physical result')
        except Exception as exc:
            self.finish(str(exc), failed=True)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operation', required=True, choices=[
        'ready_pose', 'grasp_pose', 'carry_pose', 'place_pose',
        'open', 'close', 'ready', 'pick', 'place'])
    parser.add_argument('--execute', action='store_true',
                        help='Actually command the selected arm motion')
    options, ros_args = parser.parse_known_args(args)
    if not options.execute:
        parser.error('Motion requires --execute. Stop the mission and calibrate poses first.')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    failed = False
    try:
        node = ArmCheck(options.operation)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        failed = node.failed
    except KeyboardInterrupt:
        failed = True
    finally:
        if node is not None:
            node.arm.cancel()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
