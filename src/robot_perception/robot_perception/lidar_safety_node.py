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
        self.declare_parameter('emergency_distance', 0.175)
        self.declare_parameter('side_safe_distance', 0.35)
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('side_min_angle_deg', 20.0)
        self.declare_parameter('side_max_angle_deg', 100.0)
        self.declare_parameter('minimum_valid_fraction', 0.8)
        self.declare_parameter('rotation_safe_distance', 0.18)
        self.declare_parameter('rotation_blocked_beam_count', 3)
        self.declare_parameter('rotation_unsafe_confirmation_scans', 5)
        self.declare_parameter('laser_offset_x', -0.032)
        self.declare_parameter('laser_offset_y', 0.0)

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
        self.minimum_valid_fraction = self.get_parameter(
            'minimum_valid_fraction'
        ).value
        self.rotation_safe_distance = self.get_parameter(
            'rotation_safe_distance'
        ).value
        self.rotation_blocked_beam_count = self.get_parameter(
            'rotation_blocked_beam_count'
        ).value
        self.rotation_unsafe_confirmation_scans = self.get_parameter(
            'rotation_unsafe_confirmation_scans'
        ).value
        self.laser_offset_x = self.get_parameter('laser_offset_x').value
        self.laser_offset_y = self.get_parameter('laser_offset_y').value

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
        if not 0.0 < self.minimum_valid_fraction <= 1.0:
            raise ValueError('minimum_valid_fraction must be in (0, 1]')
        if self.rotation_safe_distance <= 0.0:
            raise ValueError('rotation_safe_distance must be positive')
        if self.rotation_blocked_beam_count <= 0:
            raise ValueError('rotation_blocked_beam_count must be positive')
        if self.rotation_unsafe_confirmation_scans <= 0:
            raise ValueError(
                'rotation_unsafe_confirmation_scans must be positive'
            )
        if not all(math.isfinite(value) for value in (
            self.laser_offset_x,
            self.laser_offset_y,
        )):
            raise ValueError('laser offsets must be finite')

        self.front_half_angle = math.radians(front_half_angle_deg)
        self.side_min_angle = math.radians(side_min_angle_deg)
        self.side_max_angle = math.radians(side_max_angle_deg)
        self.last_emergency_stop = None
        self.rotation_unsafe_streak = 0

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
        if not self.scan_metadata_is_valid(message):
            result = {
                'front_distance': None,
                'left_distance': None,
                'right_distance': None,
                'minimum_distance': None,
                'rotation_minimum_distance': None,
                'front_valid_fraction': 0.0,
                'left_valid_fraction': 0.0,
                'right_valid_fraction': 0.0,
                'full_valid_fraction': 0.0,
                'left_safe': False,
                'right_safe': False,
                'rotation_safe': False,
                'rotation_safe_raw': False,
                'rotation_caution': False,
                'rotation_unsafe_streak': 0,
                'emergency_stop': True,
                'valid': False,
            }
            self.publish_safety_result(result, None, None, None)
            return

        front_distances = []
        left_distances = []
        right_distances = []
        all_distances = []
        ordered_rotation_distances = []
        total_counts = {'front': 0, 'left': 0, 'right': 0}
        valid_counts = {'front': 0, 'left': 0, 'right': 0}

        for index, raw_distance in enumerate(message.ranges):
            distance = self.normalize_distance(raw_distance, message)
            angle = message.angle_min + index * message.angle_increment
            rotation_distance = (
                self.distance_from_robot_center(
                    distance,
                    angle,
                    self.laser_offset_x,
                    self.laser_offset_y,
                )
                if distance is not None
                else None
            )
            ordered_rotation_distances.append(rotation_distance)
            if distance is not None:
                all_distances.append(distance)

            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) <= self.front_half_angle:
                sector = 'front'
            elif self.side_min_angle < angle <= self.side_max_angle:
                sector = 'left'
            elif -self.side_max_angle <= angle < -self.side_min_angle:
                sector = 'right'
            else:
                continue

            total_counts[sector] += 1
            if distance is None:
                continue
            valid_counts[sector] += 1
            if sector == 'front':
                front_distances.append(distance)
            elif sector == 'left':
                left_distances.append(distance)
            else:
                right_distances.append(distance)

        front_distance = self.minimum_or_none(front_distances)
        left_distance = self.minimum_or_none(left_distances)
        right_distance = self.minimum_or_none(right_distances)
        minimum_distance = self.minimum_or_none(all_distances)
        rotation_minimum_distance = self.minimum_or_none([
            distance
            for distance in ordered_rotation_distances
            if distance is not None
        ])
        sector_validity = {
            sector: self.coverage_is_valid(
                valid_counts[sector],
                total_counts[sector],
                self.minimum_valid_fraction,
            )
            for sector in total_counts
        }
        full_scan_valid = self.coverage_is_valid(
            len(all_distances),
            len(message.ranges),
            self.minimum_valid_fraction,
        )
        valid = (
            all(sector_validity.values())
            and full_scan_valid
            and all(
                distance is not None
                for distance in (
                    front_distance,
                    left_distance,
                    right_distance,
                )
            )
        )

        emergency_stop = self.emergency_stop_required(
            front_distance,
            valid,
            self.emergency_distance,
        )
        rotation_safe_raw = self.rotation_is_safe(
            ordered_rotation_distances,
            valid,
            self.rotation_safe_distance,
            self.rotation_blocked_beam_count,
        )
        (
            self.rotation_unsafe_streak,
            rotation_caution,
            rotation_unsafe_confirmed,
        ) = self.update_rotation_confirmation(
            rotation_safe_raw,
            self.rotation_unsafe_streak,
            self.rotation_unsafe_confirmation_scans,
        )
        result = {
            'front_distance': self.round_distance(front_distance),
            'left_distance': self.round_distance(left_distance),
            'right_distance': self.round_distance(right_distance),
            'minimum_distance': self.round_distance(minimum_distance),
            'rotation_minimum_distance': self.round_distance(
                rotation_minimum_distance
            ),
            'front_valid_fraction': self.valid_fraction(
                valid_counts['front'], total_counts['front']
            ),
            'left_valid_fraction': self.valid_fraction(
                valid_counts['left'], total_counts['left']
            ),
            'right_valid_fraction': self.valid_fraction(
                valid_counts['right'], total_counts['right']
            ),
            'full_valid_fraction': self.valid_fraction(
                len(all_distances), len(message.ranges)
            ),
            'left_safe': (
                valid and left_distance >= self.side_safe_distance
            ),
            'right_safe': (
                valid and right_distance >= self.side_safe_distance
            ),
            'rotation_safe': (
                rotation_safe_raw and not rotation_unsafe_confirmed
            ),
            'rotation_safe_raw': rotation_safe_raw,
            'rotation_caution': rotation_caution,
            'rotation_unsafe_streak': self.rotation_unsafe_streak,
            'emergency_stop': emergency_stop,
            'valid': valid,
        }

        self.publish_safety_result(
            result,
            front_distance,
            left_distance,
            right_distance,
        )

    def publish_safety_result(
        self,
        result,
        front_distance,
        left_distance,
        right_distance,
    ):
        output = String()
        output.data = json.dumps(result, ensure_ascii=True)
        self.status_publisher.publish(output)

        emergency_stop = result['emergency_stop']
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
    def scan_metadata_is_valid(message):
        metadata = (
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
        )
        return (
            bool(message.ranges)
            and all(math.isfinite(value) for value in metadata)
            and message.angle_increment > 0.0
            and message.range_min >= 0.0
            and message.range_max > message.range_min
        )

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
    def coverage_is_valid(valid_count, total_count, minimum_fraction):
        return (
            total_count > 0
            and valid_count / total_count >= minimum_fraction
        )

    @staticmethod
    def emergency_stop_required(
        front_distance,
        scan_valid,
        emergency_distance,
    ):
        return (
            not scan_valid
            or front_distance is None
            or front_distance < emergency_distance
        )

    @staticmethod
    def valid_fraction(valid_count, total_count):
        if total_count == 0:
            return 0.0
        return round(valid_count / total_count, 3)

    @staticmethod
    def rotation_is_safe(
        ordered_distances,
        scan_valid,
        safe_distance,
        blocked_beam_count,
    ):
        if (
            not scan_valid
            or not ordered_distances
            or blocked_beam_count <= 0
        ):
            return False

        blocked = [
            distance is None or distance < safe_distance
            for distance in ordered_distances
        ]
        circular_blocked = blocked + blocked[:blocked_beam_count - 1]
        consecutive_count = 0
        for beam_blocked in circular_blocked:
            if beam_blocked:
                consecutive_count += 1
                if consecutive_count >= blocked_beam_count:
                    return False
            else:
                consecutive_count = 0
        return True

    @staticmethod
    def distance_from_robot_center(
        scan_distance,
        scan_angle,
        laser_offset_x,
        laser_offset_y,
    ):
        point_x = laser_offset_x + scan_distance * math.cos(scan_angle)
        point_y = laser_offset_y + scan_distance * math.sin(scan_angle)
        return math.hypot(point_x, point_y)

    @staticmethod
    def update_rotation_confirmation(
        rotation_safe_raw,
        current_streak,
        confirmation_scans,
    ):
        if rotation_safe_raw:
            return 0, False, False
        next_streak = current_streak + 1
        confirmed = next_streak >= confirmation_scans
        caution = not confirmed
        return next_streak, caution, confirmed

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
