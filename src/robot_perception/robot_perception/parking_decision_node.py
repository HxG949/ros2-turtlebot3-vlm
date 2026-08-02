import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class ParkingDecisionNode(Node):
    def __init__(self):
        super().__init__('parking_decision_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('decision_topic', '/robot/decision')
        self.declare_parameter('goal_x', 0.8015)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', math.pi)
        self.declare_parameter('decision_rate_hz', 10.0)
        self.declare_parameter('mission_timeout', 40.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('heading_tolerance', 0.08)
        self.declare_parameter('final_yaw_tolerance', 0.05)

        odom_topic = self.get_parameter('odom_topic').value
        decision_topic = self.get_parameter('decision_topic').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        decision_rate_hz = self.get_parameter('decision_rate_hz').value
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.position_tolerance = self.get_parameter(
            'position_tolerance'
        ).value
        self.heading_tolerance = self.get_parameter(
            'heading_tolerance'
        ).value
        self.final_yaw_tolerance = self.get_parameter(
            'final_yaw_tolerance'
        ).value

        if not odom_topic or not decision_topic:
            raise ValueError('topic parameters must not be empty')
        positive_parameters = (
            decision_rate_hz,
            self.mission_timeout,
            self.odom_timeout,
            self.position_tolerance,
            self.heading_tolerance,
            self.final_yaw_tolerance,
        )
        if any(value <= 0.0 for value in positive_parameters):
            raise ValueError('rate, timeout, and tolerance values must be positive')

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.odom_received_at = None
        self.mission_started_at = None
        self.current_state = 'IDLE'
        self.last_reason = None

        self.decision_publisher = self.create_publisher(
            String,
            decision_topic,
            10,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(
            1.0 / decision_rate_hz,
            self.decision_timer_callback,
        )

        self.get_logger().info(
            f'Parking goal=({self.goal_x:.4f}, {self.goal_y:.4f}), '
            f'final_yaw={math.degrees(self.goal_yaw):.1f}deg'
        )

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

        if not all(math.isfinite(value) for value in (position.x, position.y, yaw)):
            self.get_logger().error('Odometry contains non-finite pose values')
            return

        now = time.monotonic()
        self.robot_x = position.x
        self.robot_y = position.y
        self.robot_yaw = yaw
        self.odom_received_at = now
        if self.mission_started_at is None:
            self.mission_started_at = now

    def decision_timer_callback(self):
        state, action, reason, distance_error, heading_error, elapsed = (
            self.evaluate_decision(time.monotonic())
        )
        result = {
            'state': state,
            'action': action,
            'reason': reason,
            'distance_error': self.round_or_none(distance_error),
            'heading_error': self.round_or_none(heading_error),
            'elapsed_sec': round(elapsed, 2),
        }

        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.decision_publisher.publish(message)

        if state != self.current_state or reason != self.last_reason:
            log_message = (
                f'[PARKING] {self.current_state} -> {state} '
                f'action={action} reason={reason} '
                f'distance={self.format_error(distance_error)} '
                f'yaw_error={self.format_angle(heading_error)} '
                f'elapsed={elapsed:.1f}s'
            )
            if state in {'STOPPED', 'TIMEOUT'}:
                self.get_logger().warning(log_message)
            else:
                self.get_logger().info(log_message)
            self.current_state = state
            self.last_reason = reason

    def evaluate_decision(self, now):
        if self.odom_received_at is None:
            return 'IDLE', 'stop', 'waiting_for_odom', None, None, 0.0

        elapsed = now - self.mission_started_at
        if now - self.odom_received_at > self.odom_timeout:
            return 'STOPPED', 'stop', 'odom_timeout', None, None, elapsed
        if elapsed > self.mission_timeout:
            return 'TIMEOUT', 'stop', 'mission_timeout', None, None, elapsed

        delta_x = self.goal_x - self.robot_x
        delta_y = self.goal_y - self.robot_y
        distance_error = math.hypot(delta_x, delta_y)

        if distance_error > self.position_tolerance:
            desired_yaw = math.atan2(delta_y, delta_x)
            heading_error = self.normalize_angle(
                desired_yaw - self.robot_yaw
            )
            if abs(heading_error) > self.heading_tolerance:
                action = 'turn_left' if heading_error > 0.0 else 'turn_right'
                return (
                    'ALIGN_TO_GOAL',
                    action,
                    'aligning_to_goal',
                    distance_error,
                    heading_error,
                    elapsed,
                )
            return (
                'DRIVE_TO_GOAL',
                'forward',
                'driving_to_goal',
                distance_error,
                heading_error,
                elapsed,
            )

        heading_error = self.normalize_angle(
            self.goal_yaw - self.robot_yaw
        )
        if abs(heading_error) > self.final_yaw_tolerance:
            action = 'turn_left' if heading_error > 0.0 else 'turn_right'
            return (
                'ALIGN_FINAL',
                action,
                'aligning_final_yaw',
                distance_error,
                heading_error,
                elapsed,
            )

        return (
            'GOAL_REACHED',
            'stop',
            'parking_complete',
            distance_error,
            heading_error,
            elapsed,
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def round_or_none(value):
        return round(value, 4) if value is not None else None

    @staticmethod
    def format_error(value):
        return f'{value:.3f}m' if value is not None else 'unknown'

    @staticmethod
    def format_angle(value):
        return f'{math.degrees(value):.1f}deg' if value is not None else 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ParkingDecisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
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
