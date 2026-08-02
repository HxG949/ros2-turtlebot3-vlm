import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FrontDistanceNode(Node):
    def __init__(self):
        super().__init__('front_distance_node')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        front_distances = []

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            normalized_angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(normalized_angle) <= math.radians(15):
                if (math.isfinite(distance)
                        and msg.range_min <= distance <= msg.range_max):
                    front_distances.append(distance)

        if front_distances:
            front_distance = min(front_distances)
            self.get_logger().info(f'行进方向最近障碍距离: {front_distance:.2f} m')
        else:
            self.get_logger().info('行进方向未检测到有效障碍物')


def main():
    rclpy.init()
    node = FrontDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
