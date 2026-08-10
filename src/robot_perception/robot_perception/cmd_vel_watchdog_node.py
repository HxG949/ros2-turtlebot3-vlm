import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class HeartbeatWatchdogLogic:
    def __init__(self, heartbeat_timeout, startup_timeout):
        self.heartbeat_timeout = heartbeat_timeout
        self.startup_timeout = startup_timeout
        self.started_at = None
        self.last_heartbeat_at = None
        self.fault_reason = None

    def record_heartbeat(self, received_at):
        if self.fault_reason is None:
            self.last_heartbeat_at = received_at

    def record_invalid_status(self):
        if self.last_heartbeat_at is not None and self.fault_reason is None:
            self.fault_reason = 'arbiter_status_invalid'

    def evaluate(self, now):
        if self.started_at is None:
            self.started_at = now
        if self.fault_reason is not None:
            return True, self.fault_reason
        if self.last_heartbeat_at is None:
            if now - self.started_at > self.startup_timeout:
                self.fault_reason = 'arbiter_startup_timeout'
                return True, self.fault_reason
            return False, 'waiting_for_arbiter_heartbeat'
        if now - self.last_heartbeat_at > self.heartbeat_timeout:
            self.fault_reason = 'arbiter_heartbeat_timeout'
            return True, self.fault_reason
        return False, 'arbiter_heartbeat_fresh'


class CmdVelWatchdogNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog_node')

        self.declare_parameter(
            'arbiter_status_topic',
            '/navigation/safety_arbiter_status',
        )
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'status_topic',
            '/navigation/cmd_vel_watchdog_status',
        )
        self.declare_parameter('check_rate_hz', 20.0)
        self.declare_parameter('heartbeat_timeout', 0.25)
        self.declare_parameter('startup_timeout', 2.0)

        arbiter_status_topic = self.get_parameter(
            'arbiter_status_topic'
        ).value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value
        check_rate_hz = self.get_parameter('check_rate_hz').value
        heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        startup_timeout = self.get_parameter('startup_timeout').value

        topics = (
            arbiter_status_topic,
            self.cmd_vel_topic,
            status_topic,
        )
        if not all(topics):
            raise ValueError('topic parameters must not be empty')
        if (
            check_rate_hz <= 0.0
            or heartbeat_timeout <= 0.0
            or startup_timeout <= 0.0
        ):
            raise ValueError('rate and watchdog timeouts must be positive')

        self.logic = HeartbeatWatchdogLogic(
            heartbeat_timeout,
            startup_timeout,
        )
        self.emergency_publisher = None
        self.last_status = None

        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            10,
        )
        self.create_subscription(
            String,
            arbiter_status_topic,
            self.arbiter_status_callback,
            10,
        )
        self.create_timer(
            1.0 / check_rate_hz,
            self.check_timer_callback,
        )

        self.get_logger().info(
            'Monitoring the safety arbiter heartbeat; emergency publisher idle'
        )

    def arbiter_status_callback(self, message):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError('JSON root must be an object')
            if not isinstance(result.get('state'), str):
                raise ValueError('state must be a string')
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            self.get_logger().error(f'Invalid arbiter status: {error}')
            self.logic.record_invalid_status()
            return

        self.logic.record_heartbeat(time.monotonic())

    def check_timer_callback(self):
        emergency_active, reason = self.logic.evaluate(time.monotonic())
        if emergency_active:
            self.publish_emergency_stop()
        self.publish_status(emergency_active, reason)

    def publish_emergency_stop(self):
        if self.emergency_publisher is None:
            self.emergency_publisher = self.create_publisher(
                Twist,
                self.cmd_vel_topic,
                10,
            )
            self.get_logger().error(
                'Safety arbiter heartbeat lost; taking over /cmd_vel with zero'
            )
        self.emergency_publisher.publish(Twist())

    def publish_status(self, emergency_active, reason):
        result = {
            'emergency_active': emergency_active,
            'reason': reason,
            'publishing_cmd_vel': self.emergency_publisher is not None,
        }
        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.status_publisher.publish(message)

        status = (emergency_active, reason)
        if status != self.last_status:
            log_message = (
                f'[CMD_WATCHDOG] emergency='
                f'{str(emergency_active).lower()} reason={reason}'
            )
            if emergency_active:
                self.get_logger().error(log_message)
            else:
                self.get_logger().info(log_message)
            self.last_status = status

    def publish_stop(self):
        if rclpy.ok() and self.emergency_publisher is not None:
            self.emergency_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CmdVelWatchdogNode()
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
