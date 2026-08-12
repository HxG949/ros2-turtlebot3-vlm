import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REQUIRED_SPACE_FIELDS = {
    'id',
    'center_x',
    'center_y',
    'entry_yaw',
    'final_yaw',
    'length',
    'width',
    'approach_distance',
}


def parse_parking_spaces(spaces_json):
    try:
        spaces = json.loads(spaces_json)
    except (json.JSONDecodeError, TypeError) as error:
        raise ValueError(f'parking_spaces_json is invalid: {error}') from error
    if not isinstance(spaces, list) or not spaces:
        raise ValueError('parking_spaces_json must contain a non-empty list')

    normalized = []
    seen_ids = set()
    for space in spaces:
        if not isinstance(space, dict):
            raise ValueError('each parking space must be an object')
        if not REQUIRED_SPACE_FIELDS.issubset(space):
            raise ValueError('parking space is missing required fields')
        space_id = str(space['id']).strip()
        numeric_fields = REQUIRED_SPACE_FIELDS - {'id'}
        values = {field: space[field] for field in numeric_fields}
        if (
            not space_id
            or space_id in seen_ids
            or any(isinstance(value, bool) for value in values.values())
            or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in values.values()
            )
            or values['length'] <= 0.0
            or values['width'] <= 0.0
            or values['approach_distance'] <= 0.0
        ):
            raise ValueError('parking space fields must be unique and valid')
        seen_ids.add(space_id)
        normalized.append({
            'id': space_id,
            **{field: float(value) for field, value in values.items()},
        })
    return normalized


def select_parking_space(spaces, selected_space_id):
    selected_space_id = str(selected_space_id).strip()
    for space in spaces:
        if space['id'] == selected_space_id:
            return space
    raise ValueError(f'unknown selected_space_id: {selected_space_id}')


class ParkingTargetNode(Node):
    def __init__(self):
        super().__init__('parking_target_node')

        self.declare_parameter('candidates_topic', '/parking/candidates')
        self.declare_parameter(
            'selected_target_topic',
            '/parking/selected_target',
        )
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('selected_space_id', 'space_2')
        self.declare_parameter('parking_spaces_json', '[]')

        candidates_topic = self.get_parameter('candidates_topic').value
        selected_target_topic = self.get_parameter(
            'selected_target_topic'
        ).value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        selected_space_id = self.get_parameter('selected_space_id').value
        spaces_json = self.get_parameter('parking_spaces_json').value

        if (
            not candidates_topic
            or not selected_target_topic
            or not self.frame_id
        ):
            raise ValueError('topic and frame parameters must not be empty')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self.spaces = parse_parking_spaces(spaces_json)
        self.selected_space = select_parking_space(
            self.spaces,
            selected_space_id,
        )
        self.candidates_publisher = self.create_publisher(
            String,
            candidates_topic,
            10,
        )
        self.selected_target_publisher = self.create_publisher(
            String,
            selected_target_topic,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self.publish_targets)

        self.get_logger().info(
            f'Selected parking space {self.selected_space["id"]} at '
            f'({self.selected_space["center_x"]:.4f}, '
            f'{self.selected_space["center_y"]:.4f})'
        )

    def publish_targets(self):
        candidates = String()
        candidates.data = json.dumps({
            'valid': True,
            'frame_id': self.frame_id,
            'spaces': self.spaces,
        }, ensure_ascii=True)
        self.candidates_publisher.publish(candidates)

        selected = String()
        selected.data = json.dumps({
            'valid': True,
            'frame_id': self.frame_id,
            'target': self.selected_space,
        }, ensure_ascii=True)
        self.selected_target_publisher.publish(selected)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingTargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
