"""Hold all four devices at measured positions after fresh feedback is ready."""

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

from dual_sharpa_wave.joint_names import joint_names


ARM_NAMES = {side: tuple(f'{side}_J{i}' for i in range(1, 7)) for side in ('left', 'right')}
HAND_NAMES = {side: joint_names(side) for side in ('left', 'right')}


def ordered_positions(message, expected, *, allow_extra=False):
    names = tuple(message.name)
    if len(names) != len(message.position) or len(set(names)) != len(names):
        raise ValueError('joint names and positions must have equal lengths and unique names')
    if (not allow_extra and set(names) != set(expected)) or not set(expected).issubset(names):
        raise ValueError('joint names do not match the expected device')
    values = dict(zip(names, message.position))
    positions = tuple(values[name] for name in expected)
    if not all(math.isfinite(value) for value in positions):
        raise ValueError('joint positions must be finite')
    return positions


class HoldAndTest(Node):
    def __init__(self):
        super().__init__('hold_and_test')
        self.feedback_timeout = float(self.declare_parameter('feedback_timeout_sec', 0.5).value)
        self.publish_rate = float(self.declare_parameter('publish_rate_hz', 100.0).value)
        if not math.isfinite(self.feedback_timeout) or self.feedback_timeout <= 0:
            raise ValueError('feedback_timeout_sec must be finite and positive')
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0:
            raise ValueError('publish_rate_hz must be finite and positive')

        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE)
        self.arm_command = self.create_publisher(
            JointState, '/crx5ia/joint_targets', command_qos)
        self.hand_commands = {
            side: self.create_publisher(
                JointState, f'/sharpa/{side}_hand/joint_command', command_qos)
            for side in ('left', 'right')
        }
        self.feedback = {}
        self.received_at = {}
        self.baseline = None
        self.fault = False
        for side in ('left', 'right'):
            self.create_subscription(
                JointState, f'/crx5ia/{side}/joint_states',
                lambda msg, s=side: self.on_feedback(f'arm_{s}', msg, ARM_NAMES[s], True),
                feedback_qos)
            self.create_subscription(
                JointState, f'/sharpa/{side}_hand/joint_states',
                lambda msg, s=side: self.on_feedback(f'hand_{s}', msg, HAND_NAMES[s], False),
                command_qos)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Waiting for fresh left/right arm and hand feedback; hold only')

    def on_feedback(self, key, message, names, allow_extra):
        try:
            positions = ordered_positions(message, names, allow_extra=allow_extra)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f'{key} feedback rejected: {error}', throttle_duration_sec=2.0)
            return
        self.feedback[key] = positions
        self.received_at[key] = time.monotonic()

    def tick(self):
        if self.fault:
            return
        now = time.monotonic()
        devices = ('arm_left', 'arm_right', 'hand_left', 'hand_right')
        stale = [key for key in devices
                 if key not in self.received_at or now - self.received_at[key] > self.feedback_timeout]
        if stale:
            if self.baseline is not None:
                self.fault = True
                self.get_logger().error(
                    f'Feedback lost for {", ".join(stale)}; stopped publishing. '
                    'Restart this node after resolving the issue. This is not an emergency stop.')
            return
        publishers = (self.arm_command, *self.hand_commands.values())
        if any(pub.get_subscription_count() == 0 for pub in publishers):
            if self.baseline is not None:
                self.fault = True
                self.get_logger().error('Command subscriber disappeared; stopped publishing.')
            return
        if self.baseline is None:
            self.baseline = {key: self.feedback[key] for key in devices}
            self.get_logger().info('All four feedback streams ready; holding measured positions')

        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        for side in ('left', 'right'):
            arm_msg.name.extend(ARM_NAMES[side])
            arm_msg.position.extend(self.baseline[f'arm_{side}'])
        self.arm_command.publish(arm_msg)
        for side in ('left', 'right'):
            msg = JointState()
            msg.header.stamp = arm_msg.header.stamp
            msg.name = list(HAND_NAMES[side])
            msg.position = list(self.baseline[f'hand_{side}'])
            self.hand_commands[side].publish(msg)


def main():
    rclpy.init()
    node = HoldAndTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
