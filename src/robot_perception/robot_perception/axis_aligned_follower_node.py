import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_axis_aligned_waypoints(waypoints, tolerance=1e-6):
    if not isinstance(waypoints, list) or len(waypoints) < 5:
        raise ValueError('a complete obstacle and parking path is required')

    normalized = []
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            raise ValueError('each waypoint must be an object')
        role = str(waypoint.get('role', '')).strip()
        x_position = waypoint.get('x')
        y_position = waypoint.get('y')
        stop_required = waypoint.get('stop_required')
        values = (x_position, y_position)
        if (
            not role
            or not isinstance(stop_required, bool)
            or any(isinstance(value, bool) for value in values)
            or not all(isinstance(value, (int, float)) for value in values)
            or not all(math.isfinite(value) for value in values)
        ):
            raise ValueError('waypoint role and coordinates must be valid')
        normalized.append({
            'role': role,
            'x': float(x_position),
            'y': float(y_position),
            'stop_required': stop_required,
        })

    roles = [waypoint['role'] for waypoint in normalized]
    if roles[0] != 'start' or roles[1] != 'cp1':
        raise ValueError('path must begin with start followed by CP1')
    if roles[-1] != 'parking_goal':
        raise ValueError('path must end at the parking goal')
    allowed_role_sequences = {
        (
            'start', 'cp1', 'cp2', 'parking_approach', 'parking_goal',
        ),
        (
            'start', 'cp1', 'cp2', 'parking_transition',
            'parking_approach', 'parking_goal',
        ),
        (
            'start', 'cp1', 'lane_entry', 'cp2',
            'parking_approach', 'parking_goal',
        ),
        (
            'start', 'cp1', 'lane_entry', 'cp2',
            'parking_transition', 'parking_approach', 'parking_goal',
        ),
    }
    if tuple(roles) not in allowed_role_sequences:
        raise ValueError('path waypoint roles are not in the required order')
    cp2_index = roles.index('cp2')
    if normalized[cp2_index]['stop_required']:
        raise ValueError('CP2 must be a pass-through waypoint')
    if any(
        not waypoint['stop_required']
        for index, waypoint in enumerate(normalized)
        if index != cp2_index
    ):
        raise ValueError('only CP2 may be a pass-through waypoint')

    goal_source = waypoints[-1]
    parking_fields = (
        'final_yaw',
        'parking_length',
        'parking_width',
    )
    parking_values = {
        field: goal_source.get(field)
        for field in parking_fields
    }
    parking_space_id = str(goal_source.get('parking_space_id', '')).strip()
    if (
        not parking_space_id
        or any(
            isinstance(value, bool)
            for value in parking_values.values()
        )
        or not all(
            isinstance(value, (int, float))
            for value in parking_values.values()
        )
        or not all(
            math.isfinite(value)
            for value in parking_values.values()
        )
        or parking_values['parking_length'] <= 0.0
        or parking_values['parking_width'] <= 0.0
    ):
        raise ValueError('parking goal fields must be valid')
    normalized[-1].update({
        'final_yaw': float(parking_values['final_yaw']),
        'parking_space_id': parking_space_id,
        'parking_length': float(parking_values['parking_length']),
        'parking_width': float(parking_values['parking_width']),
    })

    for previous, target in zip(normalized, normalized[1:]):
        delta_x = target['x'] - previous['x']
        delta_y = target['y'] - previous['y']
        x_changes = abs(delta_x) > tolerance
        y_changes = abs(delta_y) > tolerance
        if x_changes == y_changes:
            raise ValueError('path segments must be non-zero and axis-aligned')

    cp2_previous = normalized[cp2_index - 1]
    cp2 = normalized[cp2_index]
    cp2_next = normalized[cp2_index + 1]
    if not math.isclose(
        segment_heading(cp2_previous, cp2),
        segment_heading(cp2, cp2_next),
        abs_tol=tolerance,
    ):
        raise ValueError('CP2 pass-through segments must be collinear')

    return normalized


def segment_heading(previous, target):
    delta_x = target['x'] - previous['x']
    delta_y = target['y'] - previous['y']
    if abs(delta_x) > abs(delta_y):
        return 0.0 if delta_x > 0.0 else math.pi
    return math.pi / 2.0 if delta_y > 0.0 else -math.pi / 2.0


def execution_plan_signature(waypoints, stop_at_cp1):
    active_waypoints = waypoints[:2] if stop_at_cp1 else waypoints
    return tuple(
        (
            waypoint['role'],
            waypoint['x'],
            waypoint['y'],
            waypoint['stop_required'],
            waypoint.get('final_yaw'),
            waypoint.get('parking_space_id'),
            waypoint.get('parking_length'),
            waypoint.get('parking_width'),
        )
        for waypoint in active_waypoints
    )


def parking_envelope_is_inside(
    robot_x,
    robot_y,
    robot_yaw,
    goal,
    robot_length,
    robot_width,
    tolerance=1e-9,
):
    delta_x = robot_x - goal['x']
    delta_y = robot_y - goal['y']
    cos_yaw = math.cos(goal['final_yaw'])
    sin_yaw = math.sin(goal['final_yaw'])
    local_x = cos_yaw * delta_x + sin_yaw * delta_y
    local_y = -sin_yaw * delta_x + cos_yaw * delta_y
    relative_yaw = normalize_angle(robot_yaw - goal['final_yaw'])
    projected_half_length = (
        abs(math.cos(relative_yaw)) * robot_length / 2.0
        + abs(math.sin(relative_yaw)) * robot_width / 2.0
    )
    projected_half_width = (
        abs(math.sin(relative_yaw)) * robot_length / 2.0
        + abs(math.cos(relative_yaw)) * robot_width / 2.0
    )
    return (
        abs(local_x) + projected_half_length
        <= goal['parking_length'] / 2.0 + tolerance
        and abs(local_y) + projected_half_width
        <= goal['parking_width'] / 2.0 + tolerance
    )


def pass_through_is_safe(
    robot_yaw,
    robot_angular_speed,
    desired_yaw,
    maximum_heading_error,
    maximum_angular_speed,
):
    return (
        abs(normalize_angle(desired_yaw - robot_yaw))
        <= maximum_heading_error
        and abs(robot_angular_speed) <= maximum_angular_speed
    )


def calculate_linear_speed(
    remaining_distance,
    position_tolerance,
    deceleration_distance,
    minimum_speed,
    maximum_speed,
):
    if remaining_distance <= position_tolerance:
        return 0.0
    scaled_speed = maximum_speed * (
        remaining_distance / deceleration_distance
    )
    return min(maximum_speed, max(minimum_speed, scaled_speed))


def calculate_turn_speed(
    heading_error,
    gain,
    minimum_speed,
    maximum_speed,
):
    magnitude = min(
        maximum_speed,
        max(minimum_speed, gain * abs(heading_error)),
    )
    return math.copysign(magnitude, heading_error)


def heading_is_settled(
    heading_error,
    angular_speed,
    heading_tolerance,
    stopped_angular_speed,
):
    return (
        abs(heading_error) <= heading_tolerance
        and abs(angular_speed) <= stopped_angular_speed
    )


def plan_is_stable(now, stable_since, stable_duration):
    return (
        stable_since is not None
        and now - stable_since + 1e-9 >= stable_duration
    )


def arbiter_status_is_active(value):
    return (
        isinstance(value, dict)
        and value.get('enabled') is True
        and value.get('armed') is True
        and value.get('latched') is False
        and value.get('state') == 'ACTIVE'
    )


def evidence_subscribers_are_ready(
    desired_count,
    status_count,
    required_desired_count,
    required_status_count,
):
    return (
        desired_count >= required_desired_count
        and status_count >= required_status_count
    )


class AxisAlignedFollowerNode(Node):
    def __init__(self):
        super().__init__('axis_aligned_follower_node')

        self.declare_parameter('plan_topic', '/navigation/plan')
        self.declare_parameter('safety_topic', '/safety/status')
        self.declare_parameter(
            'arbiter_status_topic',
            '/navigation/safety_arbiter_status',
        )
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter(
            'desired_cmd_vel_topic',
            '/navigation/desired_cmd_vel',
        )
        self.declare_parameter(
            'status_topic',
            '/navigation/control_status',
        )
        self.declare_parameter('enabled', False)
        self.declare_parameter('stop_at_cp1', False)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('plan_timeout', 0.6)
        self.declare_parameter('safety_timeout', 0.6)
        self.declare_parameter('arbiter_status_timeout', 0.6)
        self.declare_parameter('required_desired_subscriber_count', 1)
        self.declare_parameter('required_status_subscriber_count', 0)
        self.declare_parameter(
            'subscriber_confirmation_duration',
            0.0,
        )
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('mission_timeout', 60.0)
        self.declare_parameter('plan_stable_duration', 0.6)
        self.declare_parameter('maximum_linear_speed', 0.06)
        self.declare_parameter('minimum_linear_speed', 0.02)
        self.declare_parameter('deceleration_distance', 0.20)
        self.declare_parameter('maximum_angular_speed', 0.30)
        self.declare_parameter('minimum_angular_speed', 0.08)
        self.declare_parameter('turn_gain', 1.2)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('cross_track_gain', 2.0)
        self.declare_parameter('maximum_heading_correction', 0.12)
        self.declare_parameter('position_tolerance', 0.015)
        self.declare_parameter('heading_tolerance', 0.04)
        self.declare_parameter('cross_track_tolerance', 0.03)
        self.declare_parameter('maximum_cross_track_error', 0.06)
        self.declare_parameter('start_tolerance', 0.03)
        self.declare_parameter('stable_duration', 0.5)
        self.declare_parameter('heading_stable_duration', 0.3)
        self.declare_parameter('stopped_linear_speed', 0.005)
        self.declare_parameter('stopped_angular_speed', 0.02)
        self.declare_parameter('pass_through_max_angular_speed', 0.02)
        self.declare_parameter('robot_footprint_length', 0.210)
        self.declare_parameter('robot_footprint_width', 0.178)
        self.declare_parameter('final_yaw_tolerance', 0.05)

        plan_topic = self.get_parameter('plan_topic').value
        safety_topic = self.get_parameter('safety_topic').value
        arbiter_status_topic = self.get_parameter(
            'arbiter_status_topic'
        ).value
        odom_topic = self.get_parameter('odom_topic').value
        desired_cmd_vel_topic = self.get_parameter(
            'desired_cmd_vel_topic'
        ).value
        status_topic = self.get_parameter('status_topic').value
        self.enabled = self.get_parameter('enabled').value
        self.stop_at_cp1 = self.get_parameter('stop_at_cp1').value
        control_rate_hz = self.get_parameter('control_rate_hz').value
        self.plan_timeout = self.get_parameter('plan_timeout').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.arbiter_status_timeout = self.get_parameter(
            'arbiter_status_timeout'
        ).value
        self.required_desired_subscriber_count = self.get_parameter(
            'required_desired_subscriber_count'
        ).value
        self.required_status_subscriber_count = self.get_parameter(
            'required_status_subscriber_count'
        ).value
        self.subscriber_confirmation_duration = self.get_parameter(
            'subscriber_confirmation_duration'
        ).value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.plan_stable_duration = self.get_parameter(
            'plan_stable_duration'
        ).value
        self.maximum_linear_speed = self.get_parameter(
            'maximum_linear_speed'
        ).value
        self.minimum_linear_speed = self.get_parameter(
            'minimum_linear_speed'
        ).value
        self.deceleration_distance = self.get_parameter(
            'deceleration_distance'
        ).value
        self.maximum_angular_speed = self.get_parameter(
            'maximum_angular_speed'
        ).value
        self.minimum_angular_speed = self.get_parameter(
            'minimum_angular_speed'
        ).value
        self.turn_gain = self.get_parameter('turn_gain').value
        self.heading_gain = self.get_parameter('heading_gain').value
        self.cross_track_gain = self.get_parameter(
            'cross_track_gain'
        ).value
        self.maximum_heading_correction = self.get_parameter(
            'maximum_heading_correction'
        ).value
        self.position_tolerance = self.get_parameter(
            'position_tolerance'
        ).value
        self.heading_tolerance = self.get_parameter(
            'heading_tolerance'
        ).value
        self.cross_track_tolerance = self.get_parameter(
            'cross_track_tolerance'
        ).value
        self.maximum_cross_track_error = self.get_parameter(
            'maximum_cross_track_error'
        ).value
        self.start_tolerance = self.get_parameter('start_tolerance').value
        self.stable_duration = self.get_parameter('stable_duration').value
        self.heading_stable_duration = self.get_parameter(
            'heading_stable_duration'
        ).value
        self.stopped_linear_speed = self.get_parameter(
            'stopped_linear_speed'
        ).value
        self.stopped_angular_speed = self.get_parameter(
            'stopped_angular_speed'
        ).value
        self.pass_through_max_angular_speed = self.get_parameter(
            'pass_through_max_angular_speed'
        ).value
        self.robot_footprint_length = self.get_parameter(
            'robot_footprint_length'
        ).value
        self.robot_footprint_width = self.get_parameter(
            'robot_footprint_width'
        ).value
        self.final_yaw_tolerance = self.get_parameter(
            'final_yaw_tolerance'
        ).value

        topics = (
            plan_topic,
            safety_topic,
            arbiter_status_topic,
            odom_topic,
            desired_cmd_vel_topic,
            status_topic,
        )
        self.validate_parameters(topics, control_rate_hz)

        self.waypoints = None
        self.plan_signature = None
        self.plan_received_at = None
        self.plan_stable_since = None
        self.safety = None
        self.safety_received_at = None
        self.arbiter_status = None
        self.arbiter_status_received_at = None
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.robot_linear_speed = None
        self.robot_angular_speed = None
        self.odom_received_at = None
        self.target_index = 1
        self.arrival_started_at = None
        self.heading_stable_started_at = None
        self.aligned_target_index = None
        self.mission_started_at = None
        self.mission_complete = False
        self.completion_reason = None
        self.final_alignment_started = False
        self.fault_reason = None
        self.evidence_subscribers_ready_since = None
        self.last_control_status = None
        self.parse_error_reported = {
            'plan': False,
            'safety': False,
            'arbiter': False,
        }

        self.velocity_publisher = self.create_publisher(
            Twist,
            desired_cmd_vel_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            10,
        )
        self.create_subscription(String, plan_topic, self.plan_callback, 10)
        self.create_subscription(
            String,
            safety_topic,
            self.safety_callback,
            10,
        )
        self.create_subscription(
            String,
            arbiter_status_topic,
            self.arbiter_status_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(
            1.0 / control_rate_hz,
            self.control_timer_callback,
        )

        if self.enabled:
            self.get_logger().warning(
                'Axis-aligned motion is ENABLED; controller may move the robot'
            )
        else:
            self.get_logger().warning(
                'Axis-aligned motion is disabled; publishing zero velocity only'
            )

    def validate_parameters(self, topics, control_rate_hz):
        if not all(topics):
            raise ValueError('topic parameters must not be empty')
        positive_values = (
            control_rate_hz,
            self.plan_timeout,
            self.safety_timeout,
            self.arbiter_status_timeout,
            self.odom_timeout,
            self.mission_timeout,
            self.plan_stable_duration,
            self.maximum_linear_speed,
            self.minimum_linear_speed,
            self.deceleration_distance,
            self.maximum_angular_speed,
            self.minimum_angular_speed,
            self.turn_gain,
            self.heading_gain,
            self.cross_track_gain,
            self.maximum_heading_correction,
            self.position_tolerance,
            self.heading_tolerance,
            self.cross_track_tolerance,
            self.maximum_cross_track_error,
            self.start_tolerance,
            self.stable_duration,
            self.heading_stable_duration,
            self.stopped_linear_speed,
            self.stopped_angular_speed,
            self.pass_through_max_angular_speed,
            self.robot_footprint_length,
            self.robot_footprint_width,
            self.final_yaw_tolerance,
        )
        if any(value <= 0.0 for value in positive_values):
            raise ValueError(
                'controller rates, limits, and tolerances must be positive'
            )
        if (
            self.required_desired_subscriber_count < 1
            or self.required_status_subscriber_count < 0
            or self.subscriber_confirmation_duration < 0.0
        ):
            raise ValueError('subscriber requirements must be non-negative')
        if self.minimum_linear_speed > self.maximum_linear_speed:
            raise ValueError('minimum linear speed exceeds maximum')
        if self.minimum_angular_speed > self.maximum_angular_speed:
            raise ValueError('minimum angular speed exceeds maximum')
        if self.cross_track_tolerance > self.maximum_cross_track_error:
            raise ValueError('cross-track tolerance exceeds maximum error')

    def plan_callback(self, message):
        if self.mission_complete:
            return
        result = self.parse_json(message, 'plan')
        self.plan_received_at = time.monotonic()
        if result is None or result.get('valid') is not True:
            self.waypoints = None
            if self.enabled and self.mission_started_at is not None:
                self.fault_reason = 'plan_invalid_during_motion'
            else:
                self.plan_stable_since = None
            return

        try:
            waypoints = validate_axis_aligned_waypoints(
                result.get('waypoints'),
            )
        except ValueError as error:
            self.report_parse_error('plan', error)
            self.waypoints = None
            if self.enabled and self.mission_started_at is not None:
                self.fault_reason = 'plan_invalid_during_motion'
            return

        signature = execution_plan_signature(
            waypoints,
            self.stop_at_cp1,
        )
        if self.plan_signature is None or signature != self.plan_signature:
            if self.enabled and self.mission_started_at is not None:
                self.fault_reason = 'plan_changed_during_motion'
                return
            self.waypoints = waypoints
            self.plan_signature = signature
            self.reset_progress()
            self.plan_stable_since = self.plan_received_at
        else:
            self.waypoints = waypoints
            if self.plan_stable_since is None:
                self.plan_stable_since = self.plan_received_at

    def safety_callback(self, message):
        result = self.parse_json(message, 'safety')
        self.safety = result or {
            'valid': False,
            'emergency_stop': True,
        }
        self.safety_received_at = time.monotonic()

    def arbiter_status_callback(self, message):
        self.arbiter_status = self.parse_json(message, 'arbiter')
        self.arbiter_status_received_at = time.monotonic()

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
        values = (
            position.x,
            position.y,
            yaw,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Odometry contains non-finite values')
            return

        self.robot_x = position.x
        self.robot_y = position.y
        self.robot_yaw = yaw
        self.robot_linear_speed = message.twist.twist.linear.x
        self.robot_angular_speed = message.twist.twist.angular.z
        self.odom_received_at = time.monotonic()

    def parse_json(self, message, source):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError('JSON root must be an object')
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            self.report_parse_error(source, error)
            return None

        self.parse_error_reported[source] = False
        return result

    def report_parse_error(self, source, error):
        if not self.parse_error_reported[source]:
            self.get_logger().error(f'Invalid {source} data: {error}')
            self.parse_error_reported[source] = True

    def reset_progress(self):
        self.target_index = 1
        self.arrival_started_at = None
        self.heading_stable_started_at = None
        self.aligned_target_index = None
        self.mission_started_at = None
        self.mission_complete = False
        self.completion_reason = None
        self.final_alignment_started = False
        self.fault_reason = None

    def control_timer_callback(self):
        now = time.monotonic()
        state, reason, command, target_role, remaining = (
            self.evaluate_control(now)
        )
        self.velocity_publisher.publish(command)
        self.publish_status(
            state,
            reason,
            command,
            target_role,
            remaining,
        )

    def evaluate_control(self, now):
        stop = Twist()
        if not self.enabled:
            return 'DISABLED', 'motion_disabled', stop, None, None
        if self.mission_complete:
            return 'COMPLETE', self.completion_reason, stop, None, 0.0
        if self.fault_reason is not None:
            return 'FAULT', self.fault_reason, stop, None, None
        subscribers_ready = evidence_subscribers_are_ready(
            self.velocity_publisher.get_subscription_count(),
            self.status_publisher.get_subscription_count(),
            self.required_desired_subscriber_count,
            self.required_status_subscriber_count,
        )
        if not subscribers_ready:
            self.evidence_subscribers_ready_since = None
            if self.mission_started_at is not None:
                self.fault_reason = 'evidence_subscriber_lost'
                return 'FAULT', self.fault_reason, stop, None, None
            return (
                'WAITING',
                'waiting_for_evidence_subscribers',
                stop,
                None,
                None,
            )
        if self.evidence_subscribers_ready_since is None:
            self.evidence_subscribers_ready_since = now
        if (
            now - self.evidence_subscribers_ready_since
            < self.subscriber_confirmation_duration
        ):
            return (
                'WAITING',
                'confirming_evidence_subscribers',
                stop,
                None,
                None,
            )
        if self.waypoints is None or self.plan_received_at is None:
            return 'WAITING', 'waiting_for_valid_plan', stop, None, None
        if now - self.plan_received_at > self.plan_timeout:
            return self.stop_or_latch('plan_timeout', stop)
        if self.safety is None or self.safety_received_at is None:
            return 'WAITING', 'waiting_for_safety', stop, None, None
        if now - self.safety_received_at > self.safety_timeout:
            return self.stop_or_latch('safety_timeout', stop)
        if self.safety.get('valid') is not True:
            return self.stop_or_latch('safety_invalid', stop)
        if self.safety.get('emergency_stop') is True:
            return self.stop_or_latch('emergency_stop', stop)
        if self.odom_received_at is None:
            return 'WAITING', 'waiting_for_odom', stop, None, None
        if now - self.odom_received_at > self.odom_timeout:
            return self.stop_or_latch('odom_timeout', stop)
        if (
            self.arbiter_status is None
            or self.arbiter_status_received_at is None
        ):
            return 'WAITING', 'waiting_for_arbiter_status', stop, None, None
        if (
            now - self.arbiter_status_received_at
            > self.arbiter_status_timeout
        ):
            return self.stop_or_latch('arbiter_status_timeout', stop)
        if not arbiter_status_is_active(self.arbiter_status):
            if self.arbiter_status.get('latched') is True:
                return self.stop_or_latch('safety_arbiter_latched', stop)
            if self.mission_started_at is not None:
                return self.stop_or_latch('safety_arbiter_inactive', stop)
            return 'WAITING', 'waiting_for_arbiter_active', stop, None, None
        if self.mission_started_at is None:
            if not plan_is_stable(
                now,
                self.plan_stable_since,
                self.plan_stable_duration,
            ):
                return 'WAITING', 'waiting_for_stable_plan', stop, None, None
            start = self.waypoints[0]
            start_error = math.hypot(
                self.robot_x - start['x'],
                self.robot_y - start['y'],
            )
            if start_error > self.start_tolerance:
                self.fault_reason = 'start_pose_mismatch'
                return 'FAULT', self.fault_reason, stop, 'start', start_error
            self.mission_started_at = now
        elif now - self.mission_started_at > self.mission_timeout:
            self.fault_reason = 'mission_timeout'
            return 'FAULT', self.fault_reason, stop, None, None

        return self.follow_current_segment(now)

    def stop_or_latch(self, reason, stop):
        if self.mission_started_at is not None:
            self.fault_reason = reason
            return 'FAULT', reason, stop, None, None
        return 'STOPPED', reason, stop, None, None

    def follow_current_segment(self, now):
        stop = Twist()
        if self.target_index >= len(self.waypoints):
            self.fault_reason = 'parking_goal_missing'
            return 'FAULT', self.fault_reason, stop, None, None

        previous = self.waypoints[self.target_index - 1]
        target = self.waypoints[self.target_index]
        desired_yaw = segment_heading(previous, target)
        direction_x = math.cos(desired_yaw)
        direction_y = math.sin(desired_yaw)
        target_delta_x = target['x'] - self.robot_x
        target_delta_y = target['y'] - self.robot_y
        remaining = (
            target_delta_x * direction_x
            + target_delta_y * direction_y
        )
        relative_x = self.robot_x - previous['x']
        relative_y = self.robot_y - previous['y']
        signed_cross_track = (
            -direction_y * relative_x
            + direction_x * relative_y
        )
        cross_track_error = abs(signed_cross_track)

        if (
            not target['stop_required']
            and -self.position_tolerance <= remaining <= self.position_tolerance
            and cross_track_error <= self.cross_track_tolerance
        ):
            if not pass_through_is_safe(
                self.robot_yaw,
                self.robot_angular_speed,
                desired_yaw,
                self.heading_tolerance,
                self.pass_through_max_angular_speed,
            ):
                self.fault_reason = 'cp2_heading_unstable'
                return (
                    'FAULT',
                    self.fault_reason,
                    stop,
                    target['role'],
                    remaining,
                )
            self.target_index += 1
            self.aligned_target_index = self.target_index
            return self.follow_current_segment(now)

        if remaining < -self.position_tolerance:
            self.fault_reason = f'{target["role"]}_overshoot'
            return (
                'FAULT',
                self.fault_reason,
                stop,
                target['role'],
                remaining,
            )
        if cross_track_error > self.maximum_cross_track_error:
            self.fault_reason = f'{target["role"]}_cross_track_error'
            return (
                'FAULT',
                self.fault_reason,
                stop,
                target['role'],
                remaining,
            )

        if remaining <= self.position_tolerance:
            return self.hold_at_waypoint(
                now,
                target,
                remaining,
                cross_track_error,
            )

        self.arrival_started_at = None
        base_heading_error = normalize_angle(desired_yaw - self.robot_yaw)
        if self.aligned_target_index != self.target_index:
            if abs(base_heading_error) > self.heading_tolerance:
                self.heading_stable_started_at = None
                if target['role'] == 'cp1':
                    self.fault_reason = 'cp1_heading_mismatch'
                    return (
                        'FAULT',
                        self.fault_reason,
                        stop,
                        target['role'],
                        remaining,
                    )
                if self.safety.get('rotation_caution') is True:
                    return (
                        'ROTATION_CLEARANCE_WAIT',
                        'confirming_rotation_clearance',
                        stop,
                        target['role'],
                        remaining,
                    )
                if self.safety.get('rotation_safe') is not True:
                    self.fault_reason = 'rotation_space_unsafe'
                    return (
                        'FAULT',
                        self.fault_reason,
                        stop,
                        target['role'],
                        remaining,
                    )
                command = Twist()
                command.angular.z = calculate_turn_speed(
                    base_heading_error,
                    self.turn_gain,
                    self.minimum_angular_speed,
                    self.maximum_angular_speed,
                )
                return (
                    'ALIGNING',
                    f'aligning_to_{target["role"]}',
                    command,
                    target['role'],
                    remaining,
                )

            if not heading_is_settled(
                base_heading_error,
                self.robot_angular_speed,
                self.heading_tolerance,
                self.stopped_angular_speed,
            ):
                self.heading_stable_started_at = None
                return (
                    'ALIGNMENT_SETTLING',
                    f'stopping_rotation_for_{target["role"]}',
                    stop,
                    target['role'],
                    remaining,
                )

            if self.heading_stable_started_at is None:
                self.heading_stable_started_at = now
            if (
                now - self.heading_stable_started_at
                < self.heading_stable_duration
            ):
                return (
                    'ALIGNMENT_SETTLING',
                    f'settling_heading_for_{target["role"]}',
                    stop,
                    target['role'],
                    remaining,
                )
            self.aligned_target_index = self.target_index
            self.heading_stable_started_at = None
            return (
                'WAYPOINT_ALIGNED',
                f'aligned_to_{target["role"]}',
                stop,
                target['role'],
                remaining,
            )

        correction = 0.0
        if target['role'] != 'cp1':
            correction = min(
                self.maximum_heading_correction,
                max(
                    -self.maximum_heading_correction,
                    -self.cross_track_gain * signed_cross_track,
                ),
            )
        corrected_yaw = desired_yaw + correction
        heading_error = normalize_angle(corrected_yaw - self.robot_yaw)
        command = Twist()
        speed_remaining = remaining
        if not target['stop_required']:
            next_target = self.waypoints[self.target_index + 1]
            speed_remaining += math.hypot(
                next_target['x'] - target['x'],
                next_target['y'] - target['y'],
            )
        command.linear.x = calculate_linear_speed(
            speed_remaining,
            self.position_tolerance,
            self.deceleration_distance,
            self.minimum_linear_speed,
            self.maximum_linear_speed,
        )
        if target['role'] != 'cp1':
            command.angular.z = min(
                self.maximum_heading_correction,
                max(
                    -self.maximum_heading_correction,
                    self.heading_gain * heading_error,
                ),
            )
        return (
            'DRIVING',
            f'driving_to_{target["role"]}',
            command,
            target['role'],
            remaining,
        )

    def hold_at_waypoint(
        self,
        now,
        target,
        remaining,
        cross_track_error,
    ):
        stop = Twist()
        if cross_track_error > self.cross_track_tolerance:
            self.fault_reason = f'{target["role"]}_cross_track_error'
            return (
                'FAULT',
                self.fault_reason,
                stop,
                target['role'],
                remaining,
            )

        if target['role'] == 'parking_goal':
            return self.finish_parking(now, target, remaining)

        stopped = (
            abs(self.robot_linear_speed) <= self.stopped_linear_speed
            and abs(self.robot_angular_speed) <= self.stopped_angular_speed
        )
        if not self.final_alignment_started and not stopped:
            self.arrival_started_at = None
            return (
                'STOPPING',
                f'stopping_at_{target["role"]}',
                stop,
                target['role'],
                remaining,
            )

        if self.arrival_started_at is None:
            self.arrival_started_at = now
        if now - self.arrival_started_at < self.stable_duration:
            return (
                'SETTLING',
                f'settling_at_{target["role"]}',
                stop,
                target['role'],
                remaining,
            )

        if target['role'] == 'cp1' and self.stop_at_cp1:
            self.mission_complete = True
            self.completion_reason = 'cp1_reached'
            return 'COMPLETE', 'cp1_reached', stop, 'cp1', remaining

        self.target_index += 1
        self.arrival_started_at = None
        self.heading_stable_started_at = None
        self.aligned_target_index = None
        if self.target_index >= len(self.waypoints):
            self.fault_reason = 'parking_goal_missing'
            return 'FAULT', self.fault_reason, stop, None, None
        return (
            'WAYPOINT_REACHED',
            f'{target["role"]}_reached',
            stop,
            target['role'],
            remaining,
        )

    def finish_parking(self, now, target, remaining):
        stop = Twist()
        stopped = (
            abs(self.robot_linear_speed) <= self.stopped_linear_speed
            and abs(self.robot_angular_speed) <= self.stopped_angular_speed
        )
        if not self.final_alignment_started and not stopped:
            self.arrival_started_at = None
            return (
                'STOPPING',
                'stopping_at_parking_goal',
                stop,
                target['role'],
                remaining,
            )
        self.final_alignment_started = True

        heading_error = normalize_angle(target['final_yaw'] - self.robot_yaw)
        if abs(heading_error) > self.final_yaw_tolerance:
            self.arrival_started_at = None
            if self.safety.get('rotation_caution') is True:
                return (
                    'ROTATION_CLEARANCE_WAIT',
                    'confirming_final_rotation_clearance',
                    stop,
                    target['role'],
                    remaining,
                )
            if self.safety.get('rotation_safe') is not True:
                self.fault_reason = 'final_rotation_space_unsafe'
                return (
                    'FAULT',
                    self.fault_reason,
                    stop,
                    target['role'],
                    remaining,
                )
            command = Twist()
            command.angular.z = calculate_turn_speed(
                heading_error,
                self.turn_gain,
                self.minimum_angular_speed,
                self.maximum_angular_speed,
            )
            return (
                'FINAL_ALIGNMENT',
                'aligning_final_yaw',
                command,
                target['role'],
                remaining,
            )

        if not parking_envelope_is_inside(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            target,
            self.robot_footprint_length,
            self.robot_footprint_width,
        ):
            self.fault_reason = 'parking_envelope_outside_space'
            return (
                'FAULT',
                self.fault_reason,
                stop,
                target['role'],
                remaining,
            )

        if not heading_is_settled(
            heading_error,
            self.robot_angular_speed,
            self.final_yaw_tolerance,
            self.stopped_angular_speed,
        ):
            self.arrival_started_at = None
            return (
                'FINAL_ALIGNMENT_SETTLING',
                'stopping_final_rotation',
                stop,
                target['role'],
                remaining,
            )
        if self.arrival_started_at is None:
            self.arrival_started_at = now
        if now - self.arrival_started_at < self.stable_duration:
            return (
                'PARKING_SETTLING',
                'settling_in_parking_space',
                stop,
                target['role'],
                remaining,
            )
        self.mission_complete = True
        self.completion_reason = 'parking_complete'
        return (
            'COMPLETE',
            'parking_complete',
            stop,
            target['role'],
            remaining,
        )

    def publish_status(
        self,
        state,
        reason,
        command,
        target_role,
        remaining,
    ):
        result = {
            'enabled': self.enabled,
            'stop_at_cp1': self.stop_at_cp1,
            'state': state,
            'reason': reason,
            'target_role': target_role,
            'remaining_distance': (
                round(remaining, 4) if remaining is not None else None
            ),
            'linear_x': round(command.linear.x, 4),
            'angular_z': round(command.angular.z, 4),
        }
        message = String()
        message.data = json.dumps(result, ensure_ascii=True)
        self.status_publisher.publish(message)

        control_status = (state, reason, target_role)
        if control_status != self.last_control_status:
            log_message = (
                f'[FOLLOWER] state={state} reason={reason} '
                f'target={target_role} remaining={result["remaining_distance"]} '
                f'linear={command.linear.x:.3f} '
                f'angular={command.angular.z:.3f}'
            )
            if state in {'FAULT', 'STOPPED'}:
                self.get_logger().warning(log_message)
            else:
                self.get_logger().info(log_message)
            self.last_control_status = control_status

    def publish_stop(self):
        if rclpy.ok():
            self.velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = AxisAlignedFollowerNode()
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
