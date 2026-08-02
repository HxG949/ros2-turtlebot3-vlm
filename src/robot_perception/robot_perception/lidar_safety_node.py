import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__('lidar_safety_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('status_topic', '/safety/status')
        self.declare_parameter('emergency_distance', 0.35)
        self.declare_parameter('side_safe_distance', 0.35)
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('side_min_angle_deg', 20.0)
        self.declare_parameter('side_max_angle_deg', 100.0)

        scan_topic = self.get_parameter('scan_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.emergency_distance = self.get_parameter(
            'emergency_distance'
        ).value
        self.side_safe_distance = self.get_parameter(
            'side_safe_distance'
        ).value
        front_half_angle_deg = self.get_parameter(
            'front_half_angle_deg'
        ).value
        side_min_angle_deg = self.get_parameter(
            'side_min_angle_deg'
        ).value
        side_max_angle_deg = self.get_parameter(
            'side_max_angle_deg'
        ).value

        if not scan_topic:
            raise ValueError('scan_topic must not be empty')
        if not status_topic:
            raise ValueError('status_topic must not be empty')
        if self.emergency_distance <= 0.0:
            raise ValueError('emergency_distance must be greater than zero')
        if self.side_safe_distance <= 0.0:
            raise ValueError('side_safe_distance must be greater than zero')
        if not 0.0 < front_half_angle_deg <= side_min_angle_deg:
            raise ValueError(
                'front_half_angle_deg must be in (0, side_min_angle_deg]'
            )
        if not side_min_angle_deg < side_max_angle_deg <= 180.0:
            raise ValueError(
                'side angles must satisfy min < max <= 180 degrees'
            )

        self.front_half_angle = math.radians(front_half_angle_deg)
        self.side_min_angle = math.radians(side_min_angle_deg)
        self.side_max_angle = math.radians(side_max_angle_deg)
        self.last_emergency_stop = None

        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            10,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Monitoring {scan_topic}; emergency distance='
            f'{self.emergency_distance:.2f}m'
        )

    def scan_callback(self, message):
        front_distances = []
        left_distances = []
        right_distances = []

        for index, raw_distance in enumerate(message.ranges):
            distance = self.normalize_distance(raw_distance, message)
            if distance is None:
                continue

            angle = message.angle_min + index * message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) <= self.front_half_angle:
                front_distances.append(distance)
            elif self.side_min_angle < angle <= self.side_max_angle:
                left_distances.append(distance)
            elif -self.side_max_angle <= angle < -self.side_min_angle:
                right_distances.append(distance)

        front_distance = self.minimum_or_none(front_distances)
        left_distance = self.minimum_or_none(left_distances)
        right_distance = self.minimum_or_none(right_distances)
        valid = all(
            distance is not None
            for distance in (front_distance, left_distance, right_distance)
        )

        emergency_stop = (
            not valid or front_distance < self.emergency_distance
        )
        result = {
            'front_distance': self.round_distance(front_distance),
            'left_distance': self.round_distance(left_distance),
            'right_distance': self.round_distance(right_distance),
            'left_safe': (
                valid and left_distance >= self.side_safe_distance
            ),
            'right_safe': (
                valid and right_distance >= self.side_safe_distance
            ),
            'emergency_stop': emergency_stop,
            'valid': valid,
        }

        output = String()
        output.data = json.dumps(result, ensure_ascii=True)
        self.status_publisher.publish(output)

        if emergency_stop != self.last_emergency_stop:
            log_message = (
                f'[LIDAR] front={self.format_distance(front_distance)} '
                f'left={self.format_distance(left_distance)} '
                f'right={self.format_distance(right_distance)} '
                f'emergency={str(emergency_stop).lower()}'
            )
            if emergency_stop:
                self.get_logger().warning(log_message)
            else:
                self.get_logger().info(log_message)
            self.last_emergency_stop = emergency_stop

    @staticmethod
    def normalize_distance(distance, message):
        if math.isfinite(distance):
            if message.range_min <= distance <= message.range_max:
                return distance
            return None
        if math.isinf(distance) and distance > 0.0:
            return float(message.range_max)
        return None

    @staticmethod
    def minimum_or_none(distances):
        return min(distances) if distances else None

    @staticmethod
    def round_distance(distance):
        return round(distance, 3) if distance is not None else None

    @staticmethod
    def format_distance(distance):
        return f'{distance:.2f}m' if distance is not None else 'invalid'


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LidarSafetyNode()
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
