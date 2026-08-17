import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from robot_perception.safety_status import safety_status_fault_reason


class SafetyArbiterLogic:
    def __init__(
        self,
        desired_timeout,
        safety_timeout,
        startup_confirmation_duration,
    ):
        self.desired_timeout = desired_timeout
        self.safety_timeout = safety_timeout
        self.startup_confirmation_duration = startup_confirmation_duration
        self.armed = False
        self.startup_zero_seen = False
        self.startup_ready_since = None
        self.fault_reason = None

    def latch_fault(self, reason):
        if self.fault_reason is None:
            self.fault_reason = reason

    def evaluate(
        self,
        now,
        enabled,
        desired_velocity,
        desired_received_at,
        desired_valid,
        safety,
        safety_received_at,
    ):
        if not enabled:
            return 'DISABLED', 'motion_disabled', 0.0, 0.0
        if self.fault_reason is not None:
            return 'LATCHED', self.fault_reason, 0.0, 0.0
        if desired_received_at is None:
            self.startup_zero_seen = False
            self.startup_ready_since = None
            return 'WAITING', 'waiting_for_desired_velocity', 0.0, 0.0
        if safety_received_at is None or safety is None:
            self.startup_ready_since = None
            return 'WAITING', 'waiting_for_safety', 0.0, 0.0

        desired_stale = now - desired_received_at > self.desired_timeout
        safety_stale = now - safety_received_at > self.safety_timeout
        if desired_stale:
            if self.armed:
                self.latch_fault('desired_velocity_timeout')
                return 'LATCHED', self.fault_reason, 0.0, 0.0
            self.startup_zero_seen = False
            self.startup_ready_since = None
            return 'WAITING', 'desired_velocity_timeout', 0.0, 0.0
        if safety_stale:
            if self.armed:
                self.latch_fault('safety_timeout')
                return 'LATCHED', self.fault_reason, 0.0, 0.0
            self.startup_ready_since = None
            return 'WAITING', 'safety_timeout', 0.0, 0.0
        safety_fault = safety_status_fault_reason(safety)
        if safety_fault is not None:
            if self.armed:
                self.latch_fault(safety_fault)
                return 'LATCHED', self.fault_reason, 0.0, 0.0
            self.startup_ready_since = None
            return 'BLOCKED', safety_fault, 0.0, 0.0
        if not desired_valid or desired_velocity is None:
            if self.armed:
                self.latch_fault('desired_velocity_invalid')
                return 'LATCHED', self.fault_reason, 0.0, 0.0
            self.startup_zero_seen = False
            self.startup_ready_since = None
            return 'BLOCKED', 'desired_velocity_invalid', 0.0, 0.0

        if not self.armed:
            command_is_zero = (
                abs(desired_velocity[0]) <= 1e-9
                and abs(desired_velocity[1]) <= 1e-9
            )
            if command_is_zero:
                self.startup_zero_seen = True
            if not self.startup_zero_seen:
                self.startup_ready_since = None
                return 'BLOCKED', 'waiting_for_zero_command', 0.0, 0.0
            if self.startup_ready_since is None:
                self.startup_ready_since = now
            if (
                now - self.startup_ready_since
                < self.startup_confirmation_duration
            ):
                return 'WAITING', 'confirming_safe_startup', 0.0, 0.0

        self.armed = True
        return (
            'ACTIVE',
            'command_allowed',
            desired_velocity[0],
            desired_velocity[1],
        )


class SafetyArbiterNode(Node):
    def __init__(self):
        super().__init__('safety_arbiter_node')

        self.declare_parameter(
            'desired_cmd_vel_topic',
            '/navigation/desired_cmd_vel',
        )
        self.declare_parameter('safety_topic', '/safety/status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'status_topic',
            '/navigation/safety_arbiter_status',
        )
        self.declare_parameter('enabled', False)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('desired_timeout', 0.25)
        self.declare_parameter('safety_timeout', 0.6)
        self.declare_parameter('startup_confirmation_duration', 0.5)
        self.declare_parameter('maximum_linear_speed', 0.06)
        self.declare_parameter('maximum_angular_speed', 0.30)

        desired_topic = self.get_parameter('desired_cmd_vel_topic').value
        safety_topic = self.get_parameter('safety_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.enabled = self.get_parameter('enabled').value
        control_rate_hz = self.get_parameter('control_rate_hz').value
        desired_timeout = self.get_parameter('desired_timeout').value
        safety_timeout = self.get_parameter('safety_timeout').value
        startup_confirmation_duration = self.get_parameter(
            'startup_confirmation_duration'
        ).value
        self.maximum_linear_speed = self.get_parameter(
            'maximum_linear_speed'
        ).value
        self.maximum_angular_speed = self.get_parameter(
            'maximum_angular_speed'
        ).value

        topics = (
            desired_topic,
            safety_topic,
            self.cmd_vel_topic,
            status_topic,
        )
        positive_values = (
            control_rate_hz,
            desired_timeout,
            safety_timeout,
            startup_confirmation_duration,
            self.maximum_linear_speed,
            self.maximum_angular_speed,
        )
        if not all(topics):
            raise ValueError('topic parameters must not be empty')
        if any(value <= 0.0 for value in positive_values):
            raise ValueError('rates, timeouts, and speed limits must be positive')

        self.logic = SafetyArbiterLogic(
            desired_timeout,
            safety_timeout,
            startup_confirmation_duration,
        )
        self.desired_velocity = None
        self.desired_valid = False
        self.desired_received_at = None
        self.safety = None
        self.safety_received_at = None
        self.safety_parse_error_reported = False
        self.last_status = None

        self.velocity_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            10,
        )
        self.create_subscription(
            Twist,
            desired_topic,
            self.desired_velocity_callback,
            10,
        )
        self.create_subscription(
            String,
            safety_topic,
            self.safety_callback,
            10,
        )
        self.create_timer(
            1.0 / control_rate_hz,
            self.control_timer_callback,
        )

        if self.enabled:
            self.get_logger().warning(
                'Safety arbiter ENABLED; valid desired commands may be forwarded'
            )
        else:
            self.get_logger().warning(
                'Safety arbiter disabled; publishing zero velocity only'
            )

    def desired_velocity_callback(self, message):
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        components_valid = (
            all(math.isfinite(value) for value in values)
            and -1e-9 <= message.linear.x <= self.maximum_linear_speed
            and abs(message.linear.y) <= 1e-9
            and abs(message.linear.z) <= 1e-9
            and abs(message.angular.x) <= 1e-9
            and abs(message.angular.y) <= 1e-9
            and abs(message.angular.z) <= self.maximum_angular_speed
        )
        self.desired_velocity = (
            (message.linear.x, message.angular.z)
            if components_valid
            else None
        )
        self.desired_valid = components_valid
        self.desired_received_at = time.monotonic()

    def safety_callback(self, message):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError('JSON root must be an object')
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            if not self.safety_parse_error_reported:
                self.get_logger().error(f'Invalid safety data: {error}')
                self.safety_parse_error_reported = True
            self.safety = {
                'valid': False,
                'emergency_stop': True,
            }
        else:
            self.safety_parse_error_reported = False
            self.safety = result
        self.safety_received_at = time.monotonic()

    def control_timer_callback(self):
        if self.enabled and self.count_publishers(self.cmd_vel_topic) > 1:
            self.logic.latch_fault('multiple_cmd_vel_publishers')

        state, reason, linear_x, angular_z = self.logic.evaluate(
            time.monotonic(),
            self.enabled,
            self.desired_velocity,
            self.desired_received_at,
            self.desired_valid,
            self.safety,
            self.safety_received_at,
        )
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.velocity_publisher.publish(command)
        self.publish_status(state, reason, command)

    def publish_status(self, state, reason, command):
        result = {
            'enabled': self.enabled,
            'armed': self.logic.armed,
            'latched': self.logic.fault_reason is not None,
            'state': state,
            'reason': reason,
            'linear_x': round(command.linear.x, 4),
            'angular_z': round(command.angular.z, 4),
            'cmd_vel_publisher_count': self.count_publishers(
                self.cmd_vel_topic
            ),
        }
        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.status_publisher.publish(message)

        status = (state, reason)
        if status != self.last_status:
            log_message = (
                f'[ARBITER] state={state} reason={reason} '
                f'armed={str(self.logic.armed).lower()} '
                f'linear={command.linear.x:.3f} '
                f'angular={command.angular.z:.3f}'
            )
            if state in {'BLOCKED', 'LATCHED'}:
                self.get_logger().warning(log_message)
            else:
                self.get_logger().info(log_message)
            self.last_status = status

    def publish_stop(self):
        if rclpy.ok():
            self.velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SafetyArbiterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.publish_stop()
                node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass


if __name__ == '__main__':
    main()
