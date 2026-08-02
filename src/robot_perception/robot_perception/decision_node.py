import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('vlm_topic', '/vlm/perception_result')
        self.declare_parameter('safety_topic', '/safety/status')
        self.declare_parameter('decision_topic', '/robot/decision')
        self.declare_parameter('decision_interval', 0.5)
        self.declare_parameter('vlm_timeout', 5.0)
        self.declare_parameter('safety_timeout', 1.0)
        self.declare_parameter('forward_clear_distance', 0.6)

        vlm_topic = self.get_parameter('vlm_topic').value
        safety_topic = self.get_parameter('safety_topic').value
        decision_topic = self.get_parameter('decision_topic').value
        decision_interval = self.get_parameter('decision_interval').value
        self.vlm_timeout = self.get_parameter('vlm_timeout').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.forward_clear_distance = self.get_parameter(
            'forward_clear_distance'
        ).value

        if not vlm_topic or not safety_topic or not decision_topic:
            raise ValueError('topic parameters must not be empty')
        if decision_interval <= 0.0:
            raise ValueError('decision_interval must be greater than zero')
        if self.vlm_timeout <= 0.0 or self.safety_timeout <= 0.0:
            raise ValueError('timeout parameters must be greater than zero')
        if self.forward_clear_distance <= 0.0:
            raise ValueError(
                'forward_clear_distance must be greater than zero'
            )

        self.latest_vlm = None
        self.latest_safety = None
        self.vlm_received_at = None
        self.safety_received_at = None
        self.parse_error_reported = {'vlm': False, 'safety': False}
        self.current_state = 'IDLE'
        self.last_reason = None

        self.decision_publisher = self.create_publisher(
            String,
            decision_topic,
            10,
        )
        self.create_subscription(String, vlm_topic, self.vlm_callback, 10)
        self.create_subscription(
            String,
            safety_topic,
            self.safety_callback,
            10,
        )
        self.create_timer(decision_interval, self.decision_timer_callback)

        self.get_logger().info(
            f'Waiting for VLM on {vlm_topic} and safety on {safety_topic}'
        )

    def vlm_callback(self, message):
        result = self.parse_json_message(message, 'vlm')
        self.latest_vlm = result or {'valid': False}
        self.vlm_received_at = time.monotonic()

    def safety_callback(self, message):
        result = self.parse_json_message(message, 'safety')
        self.latest_safety = result or {
            'valid': False,
            'emergency_stop': True,
        }
        self.safety_received_at = time.monotonic()

    def parse_json_message(self, message, source):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError('JSON root must be an object')
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            if not self.parse_error_reported[source]:
                self.get_logger().error(
                    f'Invalid {source} JSON: {error}'
                )
                self.parse_error_reported[source] = True
            return None

        self.parse_error_reported[source] = False
        return result

    def decision_timer_callback(self):
        state, action, reason = self.evaluate_decision(time.monotonic())
        result = {
            'state': state,
            'action': action,
            'reason': reason,
        }

        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.decision_publisher.publish(message)

        if state != self.current_state or reason != self.last_reason:
            log_message = (
                f'[FSM] {self.current_state} -> {state} '
                f'action={action} reason={reason}'
            )
            if reason in {'emergency_stop', 'safety_invalid', 'safety_timeout'}:
                self.get_logger().warning(log_message)
            else:
                self.get_logger().info(log_message)
            self.current_state = state
            self.last_reason = reason

    def evaluate_decision(self, now):
        if self.latest_safety is None:
            return 'IDLE', 'stop', 'waiting_for_safety'
        if now - self.safety_received_at > self.safety_timeout:
            return 'STOPPED', 'stop', 'safety_timeout'
        if self.latest_safety.get('valid') is not True:
            return 'STOPPED', 'stop', 'safety_invalid'
        if self.latest_safety.get('emergency_stop') is True:
            return 'STOPPED', 'stop', 'emergency_stop'

        if self.latest_vlm is None:
            return 'PERCEIVING', 'stop', 'waiting_for_vlm'
        if now - self.vlm_received_at > self.vlm_timeout:
            return 'PERCEIVING', 'stop', 'vlm_timeout'
        if self.latest_vlm.get('valid') is not True:
            return 'STOPPED', 'stop', 'vlm_result_invalid'

        suggested_action = str(
            self.latest_vlm.get('suggested_action', '')
        ).strip().lower()

        if suggested_action == 'turn_right':
            if self.latest_safety.get('right_safe') is True:
                return 'AVOIDING_RIGHT', 'turn_right', 'vlm_turn_right'
            return 'STOPPED', 'stop', 'right_side_unsafe'

        if suggested_action == 'turn_left':
            if self.latest_safety.get('left_safe') is True:
                return 'AVOIDING_LEFT', 'turn_left', 'vlm_turn_left'
            return 'STOPPED', 'stop', 'left_side_unsafe'

        if suggested_action in {'forward', 'go_straight'}:
            front_distance = self.latest_safety.get('front_distance')
            if (
                self.is_valid_distance(front_distance)
                and front_distance >= self.forward_clear_distance
            ):
                return 'MOVING_FORWARD', 'forward', 'vlm_forward'
            return 'STOPPED', 'stop', 'front_not_clear'

        if suggested_action == 'stop':
            return 'STOPPED', 'stop', 'vlm_requested_stop'

        return 'STOPPED', 'stop', 'unsupported_vlm_action'

    @staticmethod
    def is_valid_distance(value):
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            and value >= 0.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DecisionNode()
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
