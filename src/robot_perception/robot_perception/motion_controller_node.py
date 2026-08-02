import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class MotionControllerNode(Node):
    VALID_ACTIONS = {'stop', 'forward', 'turn_left', 'turn_right'}

    def __init__(self):
        super().__init__('motion_controller_node')

        self.declare_parameter('decision_topic', '/robot/decision')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enabled', False)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('decision_timeout', 1.0)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('turn_speed', 0.35)

        decision_topic = self.get_parameter('decision_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enabled = self.get_parameter('enabled').value
        control_rate_hz = self.get_parameter('control_rate_hz').value
        self.decision_timeout = self.get_parameter(
            'decision_timeout'
        ).value
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).value
        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        if not decision_topic or not cmd_vel_topic:
            raise ValueError('topic parameters must not be empty')
        if control_rate_hz <= 0.0 or self.decision_timeout <= 0.0:
            raise ValueError('rate and timeout parameters must be positive')
        if self.max_linear_speed <= 0.0 or self.max_angular_speed <= 0.0:
            raise ValueError('maximum speeds must be greater than zero')
        if not 0.0 <= self.forward_speed <= self.max_linear_speed:
            raise ValueError('forward_speed exceeds the configured limit')
        if not 0.0 <= self.turn_speed <= self.max_angular_speed:
            raise ValueError('turn_speed exceeds the configured limit')

        self.latest_action = 'stop'
        self.decision_received_at = None
        self.invalid_decision_reported = False
        self.last_control_status = None

        self.velocity_publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self.create_subscription(
            String,
            decision_topic,
            self.decision_callback,
            10,
        )
        self.create_timer(1.0 / control_rate_hz, self.control_timer_callback)

        if self.enabled:
            self.get_logger().warning(
                f'Motion control enabled; publishing to {cmd_vel_topic}'
            )
        else:
            self.get_logger().warning(
                'Motion control disabled; only zero velocity will be published'
            )

    def decision_callback(self, message):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError('JSON root must be an object')
            action = str(result.get('action', '')).strip().lower()
            if action not in self.VALID_ACTIONS:
                raise ValueError(f'unsupported action: {action or "empty"}')
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            if not self.invalid_decision_reported:
                self.get_logger().error(f'Invalid decision: {error}')
                self.invalid_decision_reported = True
            self.latest_action = 'stop'
            self.decision_received_at = time.monotonic()
            return

        self.invalid_decision_reported = False
        self.latest_action = action
        self.decision_received_at = time.monotonic()

    def control_timer_callback(self):
        action, reason = self.get_effective_action(time.monotonic())
        velocity = self.make_velocity(action)
        self.velocity_publisher.publish(velocity)

        control_status = (action, reason)
        if control_status != self.last_control_status:
            self.get_logger().info(
                f'[CONTROL] action={action} '
                f'linear={velocity.linear.x:.2f} '
                f'angular={velocity.angular.z:.2f} reason={reason}'
            )
            self.last_control_status = control_status

    def get_effective_action(self, now):
        if not self.enabled:
            return 'stop', 'controller_disabled'
        if self.decision_received_at is None:
            return 'stop', 'waiting_for_decision'
        if now - self.decision_received_at > self.decision_timeout:
            return 'stop', 'decision_timeout'
        return self.latest_action, 'decision_received'

    def make_velocity(self, action):
        velocity = Twist()
        if action == 'forward':
            velocity.linear.x = min(
                self.forward_speed,
                self.max_linear_speed,
            )
        elif action == 'turn_left':
            velocity.angular.z = min(
                self.turn_speed,
                self.max_angular_speed,
            )
        elif action == 'turn_right':
            velocity.angular.z = -min(
                self.turn_speed,
                self.max_angular_speed,
            )
        return velocity

    def publish_stop(self):
        if rclpy.ok():
            self.velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MotionControllerNode()
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
