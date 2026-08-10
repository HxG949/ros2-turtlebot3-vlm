import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


@dataclass(frozen=True)
class PlanningGeometry:
    start_x: float
    start_y: float
    cp1_x: float
    cp1_y: float
    cp2_x: float
    field_y_min: float
    field_y_max: float
    robot_radius: float
    safety_margin: float
    candidate_spacing: float


def point_to_horizontal_segment_distance(
    point_x,
    point_y,
    segment_x_min,
    segment_x_max,
    segment_y,
):
    nearest_x = min(max(point_x, segment_x_min), segment_x_max)
    return math.hypot(point_x - nearest_x, point_y - segment_y)


def is_planning_obstacle_range(distance, range_min, min_range_padding):
    return distance > range_min + min_range_padding


def is_robot_self_point(local_x, local_y, robot_radius, padding):
    return math.hypot(local_x, local_y) <= robot_radius + padding


def calculate_lane_clearance(lane_y, obstacle_points, geometry):
    wall_distance = min(
        lane_y - geometry.field_y_min,
        geometry.field_y_max - lane_y,
    )
    minimum_center_distance = wall_distance
    segment_x_min = min(geometry.cp1_x, geometry.cp2_x)
    segment_x_max = max(geometry.cp1_x, geometry.cp2_x)
    for point_x, point_y in obstacle_points:
        distance = point_to_horizontal_segment_distance(
            point_x,
            point_y,
            segment_x_min,
            segment_x_max,
            lane_y,
        )
        minimum_center_distance = min(minimum_center_distance, distance)

    return minimum_center_distance - geometry.robot_radius


def generate_lane_candidates(geometry):
    center_y_min = (
        geometry.field_y_min
        + geometry.robot_radius
        + geometry.safety_margin
    )
    center_y_max = (
        geometry.field_y_max
        - geometry.robot_radius
        - geometry.safety_margin
    )
    candidates = []
    lane_y = center_y_min
    while lane_y <= center_y_max + 1e-9:
        candidates.append(lane_y)
        lane_y += geometry.candidate_spacing

    if not candidates or candidates[-1] < center_y_max - 1e-9:
        candidates.append(center_y_max)
    if center_y_min <= geometry.cp1_y <= center_y_max:
        candidates.append(geometry.cp1_y)

    return sorted(set(round(candidate, 9) for candidate in candidates))


def plan_axis_aligned_path(
    obstacle_points,
    geometry,
    committed_y=None,
    lane_selection_buffer=0.0,
):
    candidates = generate_lane_candidates(geometry)
    lane_results = [
        (
            lane_y,
            calculate_lane_clearance(lane_y, obstacle_points, geometry),
        )
        for lane_y in candidates
    ]
    safe_lanes = [
        (lane_y, clearance)
        for lane_y, clearance in lane_results
        if clearance + 1e-9 >= geometry.safety_margin
    ]
    selection_lanes = [
        (lane_y, clearance)
        for lane_y, clearance in lane_results
        if clearance + 1e-9
        >= geometry.safety_margin + lane_selection_buffer
    ]

    current_lane = next(
        (
            (lane_y, clearance)
            for lane_y, clearance in safe_lanes
            if math.isclose(lane_y, geometry.cp1_y, abs_tol=1e-9)
        ),
        None,
    )
    left_lanes = [
        (lane_y, clearance)
        for lane_y, clearance in selection_lanes
        if lane_y + 1e-9 >= geometry.cp1_y
    ]
    nearest_left_lane = min(
        left_lanes,
        default=None,
        key=lambda item: item[0],
    )
    committed_lane = next(
        (
            (lane_y, clearance)
            for lane_y, clearance in safe_lanes
            if committed_y is not None
            and math.isclose(lane_y, committed_y, abs_tol=1e-6)
        ),
        None,
    )
    if committed_lane is not None:
        selected_y, clearance = committed_lane
        reason = 'committed_lane_held'
    elif committed_y is None and current_lane is not None:
        selected_y, clearance = current_lane
        reason = 'current_lane_safe'
    elif nearest_left_lane is None:
        return {
            'valid': False,
            'reason': 'no_safe_lane',
            'selected_y': None,
            'minimum_clearance': None,
            'waypoints': [],
        }
    elif committed_y is not None:
        selected_y, clearance = nearest_left_lane
        reason = 'unsafe_committed_lane_replaced'
    else:
        selected_y, clearance = nearest_left_lane
        reason = 'nearest_left_lane_selected'

    waypoints = [
        {
            'role': 'start',
            'x': round(geometry.start_x, 4),
            'y': round(geometry.start_y, 4),
        },
        {
            'role': 'cp1',
            'x': round(geometry.cp1_x, 4),
            'y': round(geometry.cp1_y, 4),
        },
    ]
    if not math.isclose(selected_y, geometry.cp1_y, abs_tol=1e-6):
        waypoints.append({
            'role': 'lane_entry',
            'x': round(geometry.cp1_x, 4),
            'y': round(selected_y, 4),
        })
    waypoints.append({
        'role': 'cp2',
        'x': round(geometry.cp2_x, 4),
        'y': round(selected_y, 4),
    })

    return {
        'valid': True,
        'reason': reason,
        'selected_y': round(selected_y, 4),
        'minimum_clearance': round(clearance, 4),
        'waypoints': waypoints,
    }


class AxisAlignedPlannerNode(Node):
    def __init__(self):
        super().__init__('axis_aligned_planner_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('plan_topic', '/navigation/plan')
        self.declare_parameter('path_topic', '/navigation/path')
        self.declare_parameter('marker_topic', '/navigation/markers')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('planning_rate_hz', 5.0)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('minimum_scan_points', 10)
        self.declare_parameter('minimum_scan_valid_fraction', 0.8)
        self.declare_parameter('obstacle_min_range_padding', 0.015)
        self.declare_parameter('self_filter_padding', 0.02)
        self.declare_parameter('laser_offset_x', -0.032)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('start_x', -0.9015)
        self.declare_parameter('start_y', -0.845)
        self.declare_parameter('cp1_x', -0.39)
        self.declare_parameter('cp1_y', -0.845)
        self.declare_parameter('obstacle_x_min', -0.20)
        self.declare_parameter('obstacle_x_max', 0.0)
        self.declare_parameter('obstacle_filter_padding', 0.03)
        self.declare_parameter('cp2_x', 0.20)
        self.declare_parameter('field_y_min', -1.05)
        self.declare_parameter('field_y_max', 1.05)
        self.declare_parameter('robot_radius', 0.105)
        self.declare_parameter('safety_margin', 0.070)
        self.declare_parameter('candidate_spacing', 0.02)
        self.declare_parameter('lane_selection_buffer', 0.05)

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        path_topic = self.get_parameter('path_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        planning_rate_hz = self.get_parameter('planning_rate_hz').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.minimum_scan_points = self.get_parameter(
            'minimum_scan_points'
        ).value
        self.minimum_scan_valid_fraction = self.get_parameter(
            'minimum_scan_valid_fraction'
        ).value
        self.obstacle_min_range_padding = self.get_parameter(
            'obstacle_min_range_padding'
        ).value
        self.self_filter_padding = self.get_parameter(
            'self_filter_padding'
        ).value
        self.laser_offset_x = self.get_parameter('laser_offset_x').value
        self.laser_offset_y = self.get_parameter('laser_offset_y').value
        self.obstacle_x_min = self.get_parameter('obstacle_x_min').value
        self.obstacle_x_max = self.get_parameter('obstacle_x_max').value
        self.obstacle_filter_padding = self.get_parameter(
            'obstacle_filter_padding'
        ).value
        self.geometry = PlanningGeometry(
            start_x=self.get_parameter('start_x').value,
            start_y=self.get_parameter('start_y').value,
            cp1_x=self.get_parameter('cp1_x').value,
            cp1_y=self.get_parameter('cp1_y').value,
            cp2_x=self.get_parameter('cp2_x').value,
            field_y_min=self.get_parameter('field_y_min').value,
            field_y_max=self.get_parameter('field_y_max').value,
            robot_radius=self.get_parameter('robot_radius').value,
            safety_margin=self.get_parameter('safety_margin').value,
            candidate_spacing=self.get_parameter('candidate_spacing').value,
        )
        self.lane_selection_buffer = self.get_parameter(
            'lane_selection_buffer'
        ).value

        self.validate_parameters(
            scan_topic,
            odom_topic,
            plan_topic,
            path_topic,
            marker_topic,
            planning_rate_hz,
        )

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.odom_received_at = None
        self.scan_received_at = None
        self.scan_valid = False
        self.scan_point_count = 0
        self.scan_valid_ray_count = 0
        self.scan_total_ray_count = 0
        self.obstacle_points = []
        self.committed_lane_y = None
        self.last_plan_status = None

        self.plan_publisher = self.create_publisher(String, plan_topic, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            10,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(
            1.0 / planning_rate_hz,
            self.planning_timer_callback,
        )

        self.get_logger().info(
            f'Planning axis-aligned paths from CP1 '
            f'({self.geometry.cp1_x:.3f}, {self.geometry.cp1_y:.3f})'
        )

    def validate_parameters(
        self,
        scan_topic,
        odom_topic,
        plan_topic,
        path_topic,
        marker_topic,
        planning_rate_hz,
    ):
        topics = (
            scan_topic,
            odom_topic,
            plan_topic,
            path_topic,
            marker_topic,
        )
        if not all(topics) or not self.frame_id:
            raise ValueError('topic parameters must not be empty')
        positive_values = (
            planning_rate_hz,
            self.scan_timeout,
            self.odom_timeout,
            self.minimum_scan_points,
            self.obstacle_min_range_padding,
            self.self_filter_padding,
            self.obstacle_filter_padding,
            self.geometry.robot_radius,
            self.geometry.safety_margin,
            self.geometry.candidate_spacing,
            self.lane_selection_buffer,
        )
        if any(value <= 0 for value in positive_values):
            raise ValueError('rate, timeout, count, and distances must be positive')
        if not 0.0 < self.minimum_scan_valid_fraction <= 1.0:
            raise ValueError(
                'minimum_scan_valid_fraction must be in (0, 1]'
            )
        if not (
            self.geometry.start_x
            < self.geometry.cp1_x
            < self.obstacle_x_min
            < self.obstacle_x_max
            < self.geometry.cp2_x
        ):
            raise ValueError('x coordinates must progress from start to exit')
        if not math.isclose(
            self.geometry.start_y,
            self.geometry.cp1_y,
            abs_tol=1e-9,
        ):
            raise ValueError('start and CP1 must form an x-axis segment')
        if self.geometry.field_y_min >= self.geometry.field_y_max:
            raise ValueError('field_y_min must be less than field_y_max')
        required_half_width = (
            self.geometry.robot_radius + self.geometry.safety_margin
        )
        if (
            self.geometry.field_y_max - self.geometry.field_y_min
            <= 2.0 * required_half_width
        ):
            raise ValueError('field is too narrow for the configured clearance')

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
        pose = (position.x, position.y, yaw)
        if not all(math.isfinite(value) for value in pose):
            self.get_logger().error('Odometry contains non-finite pose values')
            return

        self.robot_x, self.robot_y, self.robot_yaw = pose
        self.odom_received_at = time.monotonic()

    def scan_callback(self, message):
        self.scan_received_at = time.monotonic()
        self.scan_valid = False
        self.scan_point_count = 0
        self.scan_valid_ray_count = 0
        self.scan_total_ray_count = len(message.ranges)
        self.obstacle_points = []

        if self.robot_x is None:
            return
        scan_values = (
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
        )
        if (
            not message.ranges
            or not all(math.isfinite(value) for value in scan_values)
            or message.angle_increment <= 0.0
            or message.range_min < 0.0
            or message.range_max <= message.range_min
        ):
            return

        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        x_min = self.obstacle_x_min - self.obstacle_filter_padding
        x_max = self.obstacle_x_max + self.obstacle_filter_padding

        for index, distance in enumerate(message.ranges):
            if math.isinf(distance) and distance > 0.0:
                self.scan_valid_ray_count += 1
                continue
            if (
                not math.isfinite(distance)
                or distance < message.range_min
                or distance > message.range_max
            ):
                continue

            self.scan_valid_ray_count += 1
            self.scan_point_count += 1
            if not is_planning_obstacle_range(
                distance,
                message.range_min,
                self.obstacle_min_range_padding,
            ):
                continue
            angle = message.angle_min + index * message.angle_increment
            local_x = self.laser_offset_x + distance * math.cos(angle)
            local_y = self.laser_offset_y + distance * math.sin(angle)
            if is_robot_self_point(
                local_x,
                local_y,
                self.geometry.robot_radius,
                self.self_filter_padding,
            ):
                continue
            world_x = self.robot_x + cos_yaw * local_x - sin_yaw * local_y
            world_y = self.robot_y + sin_yaw * local_x + cos_yaw * local_y

            if (
                x_min <= world_x <= x_max
                and self.geometry.field_y_min
                <= world_y
                <= self.geometry.field_y_max
            ):
                self.obstacle_points.append((world_x, world_y))

        valid_fraction = (
            self.scan_valid_ray_count / self.scan_total_ray_count
            if self.scan_total_ray_count > 0
            else 0.0
        )
        self.scan_valid = (
            self.scan_point_count >= self.minimum_scan_points
            and valid_fraction >= self.minimum_scan_valid_fraction
        )

    def planning_timer_callback(self):
        now = time.monotonic()
        result = self.evaluate_plan(now)
        result['obstacle_point_count'] = len(self.obstacle_points)
        result['scan_point_count'] = self.scan_point_count
        result['scan_valid_fraction'] = self.scan_valid_fraction()
        result['robot_pose'] = self.robot_pose_result()

        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.plan_publisher.publish(message)
        self.publish_visualization(result)

        plan_status = (result['valid'], result['reason'], result['selected_y'])
        if plan_status != self.last_plan_status:
            log_message = (
                f'[PLANNER] valid={str(result["valid"]).lower()} '
                f'reason={result["reason"]} '
                f'selected_y={result["selected_y"]} '
                f'obstacle_points={len(self.obstacle_points)}'
            )
            if result['valid']:
                self.get_logger().info(log_message)
            else:
                self.get_logger().warning(log_message)
            self.last_plan_status = plan_status

    def evaluate_plan(self, now):
        if self.odom_received_at is None:
            return self.invalid_plan('waiting_for_odom')
        if now - self.odom_received_at > self.odom_timeout:
            return self.invalid_plan('odom_timeout')
        if self.scan_received_at is None:
            return self.invalid_plan('waiting_for_scan')
        if now - self.scan_received_at > self.scan_timeout:
            return self.invalid_plan('scan_timeout')
        if not self.scan_valid:
            return self.invalid_plan('scan_invalid')

        result = plan_axis_aligned_path(
            self.obstacle_points,
            self.geometry,
            committed_y=self.committed_lane_y,
            lane_selection_buffer=self.lane_selection_buffer,
        )
        if result['valid']:
            self.committed_lane_y = result['selected_y']
        return result

    def publish_visualization(self, result):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp

        if result['valid']:
            for waypoint in result['waypoints']:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(waypoint['x'])
                pose.pose.position.y = float(waypoint['y'])
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_publisher.publish(path)

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        markers.markers.extend(self.make_static_markers(stamp))
        if result['valid']:
            markers.markers.extend(self.make_plan_markers(result, stamp))
        self.marker_publisher.publish(markers)

    def make_static_markers(self, stamp):
        field_border = self.new_marker(
            stamp,
            'field',
            1,
            Marker.LINE_STRIP,
        )
        field_border.scale.x = 0.015
        self.set_color(field_border, 0.85, 0.85, 0.85, 1.0)
        field_border.points = [
            self.make_point(-1.05, -1.05, 0.02),
            self.make_point(1.05, -1.05, 0.02),
            self.make_point(1.05, 1.05, 0.02),
            self.make_point(-1.05, 1.05, 0.02),
            self.make_point(-1.05, -1.05, 0.02),
        ]

        obstacle_region = self.new_marker(
            stamp,
            'field',
            2,
            Marker.CUBE,
        )
        obstacle_region.pose.position.x = (
            self.obstacle_x_min + self.obstacle_x_max
        ) / 2.0
        obstacle_region.pose.position.z = 0.005
        obstacle_region.pose.orientation.w = 1.0
        obstacle_region.scale.x = (
            self.obstacle_x_max - self.obstacle_x_min
        )
        obstacle_region.scale.y = (
            self.geometry.field_y_max - self.geometry.field_y_min
        )
        obstacle_region.scale.z = 0.01
        self.set_color(obstacle_region, 1.0, 0.55, 0.0, 0.22)

        cp1 = self.make_waypoint_marker(
            stamp,
            10,
            self.geometry.cp1_x,
            self.geometry.cp1_y,
            1.0,
            0.85,
            0.0,
        )
        cp1_text = self.make_text_marker(
            stamp,
            11,
            self.geometry.cp1_x,
            self.geometry.cp1_y,
            'CP1',
        )
        start = self.make_waypoint_marker(
            stamp,
            12,
            self.geometry.start_x,
            self.geometry.start_y,
            0.75,
            0.35,
            1.0,
        )
        start_text = self.make_text_marker(
            stamp,
            13,
            self.geometry.start_x,
            self.geometry.start_y,
            'start',
        )
        start_ring = self.make_ring_marker(
            stamp,
            14,
            self.geometry.start_x,
            self.geometry.start_y,
            0.75,
            0.35,
            1.0,
        )
        cp1_ring = self.make_ring_marker(
            stamp,
            15,
            self.geometry.cp1_x,
            self.geometry.cp1_y,
            1.0,
            0.85,
            0.0,
        )
        markers = [
            field_border,
            obstacle_region,
            cp1,
            cp1_text,
            start,
            start_text,
            start_ring,
            cp1_ring,
        ]
        if self.robot_x is not None:
            robot_center = self.make_waypoint_marker(
                stamp,
                16,
                self.robot_x,
                self.robot_y,
                1.0,
                1.0,
                1.0,
            )
            robot_center.ns = 'robot_center'
            robot_center.scale.x = 0.045
            robot_center.scale.y = 0.045
            robot_center.scale.z = 0.045
            markers.append(robot_center)
        return markers

    def make_plan_markers(self, result, stamp):
        selected_y = float(result['selected_y'])
        corridor = self.new_marker(
            stamp,
            'plan',
            20,
            Marker.CUBE,
        )
        corridor.pose.position.x = (
            self.geometry.cp1_x + self.geometry.cp2_x
        ) / 2.0
        corridor.pose.position.y = selected_y
        corridor.pose.position.z = 0.015
        corridor.pose.orientation.w = 1.0
        corridor.scale.x = abs(self.geometry.cp2_x - self.geometry.cp1_x)
        corridor.scale.y = 2.0 * (
            self.geometry.robot_radius + self.geometry.safety_margin
        )
        corridor.scale.z = 0.02
        self.set_color(corridor, 0.1, 0.85, 0.25, 0.22)

        markers = [corridor]
        if result['reason'] == 'nearest_left_lane_selected':
            blocked_lane = self.new_marker(
                stamp,
                'plan',
                21,
                Marker.LINE_STRIP,
            )
            blocked_lane.scale.x = 0.025
            self.set_color(blocked_lane, 0.95, 0.1, 0.1, 1.0)
            blocked_lane.points = [
                self.make_point(
                    self.geometry.cp1_x,
                    self.geometry.cp1_y,
                    0.04,
                ),
                self.make_point(
                    self.geometry.cp2_x,
                    self.geometry.cp1_y,
                    0.04,
                ),
            ]
            markers.append(blocked_lane)

        waypoint_colors = {
            'lane_entry': (0.1, 0.65, 1.0),
            'cp2': (0.1, 0.9, 0.25),
        }
        for index, waypoint in enumerate(result['waypoints'], start=30):
            role = waypoint['role']
            if role in {'start', 'cp1'}:
                continue
            red, green, blue = waypoint_colors[role]
            markers.append(self.make_waypoint_marker(
                stamp,
                index,
                waypoint['x'],
                waypoint['y'],
                red,
                green,
                blue,
            ))
            markers.append(self.make_text_marker(
                stamp,
                index + 100,
                waypoint['x'],
                waypoint['y'],
                role,
            ))
        return markers

    def new_marker(self, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    def make_waypoint_marker(
        self,
        stamp,
        marker_id,
        x_position,
        y_position,
        red,
        green,
        blue,
    ):
        marker = self.new_marker(
            stamp,
            'waypoints',
            marker_id,
            Marker.SPHERE,
        )
        marker.pose.position.x = float(x_position)
        marker.pose.position.y = float(y_position)
        marker.pose.position.z = 0.07
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        self.set_color(marker, red, green, blue, 1.0)
        return marker

    def make_text_marker(
        self,
        stamp,
        marker_id,
        x_position,
        y_position,
        text,
    ):
        marker = self.new_marker(
            stamp,
            'labels',
            marker_id,
            Marker.TEXT_VIEW_FACING,
        )
        marker.pose.position.x = float(x_position)
        marker.pose.position.y = float(y_position)
        marker.pose.position.z = 0.18
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.10
        marker.text = text
        self.set_color(marker, 1.0, 1.0, 1.0, 1.0)
        return marker

    def make_ring_marker(
        self,
        stamp,
        marker_id,
        x_position,
        y_position,
        red,
        green,
        blue,
    ):
        marker = self.new_marker(
            stamp,
            'checkpoint_rings',
            marker_id,
            Marker.LINE_STRIP,
        )
        marker.scale.x = 0.035
        self.set_color(marker, red, green, blue, 1.0)
        radius = 0.12
        point_count = 48
        marker.points = [
            self.make_point(
                float(x_position) + radius * math.cos(2.0 * math.pi * index / point_count),
                float(y_position) + radius * math.sin(2.0 * math.pi * index / point_count),
                0.055,
            )
            for index in range(point_count + 1)
        ]
        return marker

    @staticmethod
    def make_point(x_position, y_position, z_position):
        point = Point()
        point.x = float(x_position)
        point.y = float(y_position)
        point.z = float(z_position)
        return point

    @staticmethod
    def set_color(marker, red, green, blue, alpha):
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)

    def robot_pose_result(self):
        if self.robot_x is None:
            return None
        return {
            'x': round(self.robot_x, 4),
            'y': round(self.robot_y, 4),
            'yaw': round(self.robot_yaw, 4),
        }

    def scan_valid_fraction(self):
        if self.scan_total_ray_count == 0:
            return 0.0
        return round(
            self.scan_valid_ray_count / self.scan_total_ray_count,
            3,
        )

    @staticmethod
    def invalid_plan(reason):
        return {
            'valid': False,
            'reason': reason,
            'selected_y': None,
            'minimum_clearance': None,
            'waypoints': [],
        }


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = AxisAlignedPlannerNode()
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
