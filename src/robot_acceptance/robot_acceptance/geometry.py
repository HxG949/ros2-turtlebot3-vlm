"""Parking-envelope and directed CP2 geometry from the P0 specification."""

from dataclasses import dataclass
import math


CP2_LONGITUDINAL_TOLERANCE_M = 0.015
CP2_CROSS_TRACK_TOLERANCE_M = 0.030
CP2_HEADING_TOLERANCE_RAD = 0.020
CP2_ANGULAR_SPEED_TOLERANCE_RADPS = 0.020
ENVELOPE_EPSILON_M = 1e-9
NUMERIC_COMPARISON_EPSILON = 1e-12


@dataclass(frozen=True)
class Pose2D:
    """Represent a planar pose."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ParkingSpace:
    """Represent an oriented inner parking rectangle."""

    center: Pose2D
    length: float
    width: float


@dataclass(frozen=True)
class Margins:
    """Hold the robot clearance to all four parking-space edges."""

    front: float
    rear: float
    left: float
    right: float
    minimum: float

    def contained(self, epsilon=ENVELOPE_EPSILON_M):
        """Return whether all corners are inside within floating-point epsilon."""
        return self.minimum >= -epsilon


@dataclass(frozen=True)
class PoseErrors:
    """Hold center-position and shortest-yaw errors."""

    position: float
    yaw: float


@dataclass(frozen=True)
class CP2Result:
    """Hold measurements and the result of a directed CP2 crossing check."""

    passed: bool
    longitudinal_error: float
    cross_track_error: float
    heading_error: float
    angular_speed: float
    forward_progress: float


def normalize_angle(angle):
    """Normalize an angle to the half-open interval [-pi, pi)."""
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    normalized = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return 0.0 if normalized == 0.0 else normalized


def parking_margins(robot, robot_length, robot_width, parking_space):
    """Calculate four-edge margins from the robot's actual rotated corners."""
    values = (
        robot.x, robot.y, robot.yaw, robot_length, robot_width,
        parking_space.center.x, parking_space.center.y,
        parking_space.center.yaw, parking_space.length, parking_space.width,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('parking geometry values must be finite')
    if min(robot_length, robot_width, parking_space.length,
           parking_space.width) <= 0.0:
        raise ValueError('parking and robot dimensions must be positive')

    dx = robot.x - parking_space.center.x
    dy = robot.y - parking_space.center.y
    cos_parking = math.cos(parking_space.center.yaw)
    sin_parking = math.sin(parking_space.center.yaw)
    center_x = cos_parking * dx + sin_parking * dy
    center_y = -sin_parking * dx + cos_parking * dy
    delta = normalize_angle(robot.yaw - parking_space.center.yaw)
    cos_delta = math.cos(delta)
    sin_delta = math.sin(delta)

    corners = []
    for local_x in (-robot_length / 2.0, robot_length / 2.0):
        for local_y in (-robot_width / 2.0, robot_width / 2.0):
            corners.append((
                center_x + cos_delta * local_x - sin_delta * local_y,
                center_y + sin_delta * local_x + cos_delta * local_y,
            ))
    x_values = [corner[0] for corner in corners]
    y_values = [corner[1] for corner in corners]
    rear = min(x_values) + parking_space.length / 2.0
    front = parking_space.length / 2.0 - max(x_values)
    right = min(y_values) + parking_space.width / 2.0
    left = parking_space.width / 2.0 - max(y_values)
    return Margins(
        front=front,
        rear=rear,
        left=left,
        right=right,
        minimum=min(front, rear, left, right),
    )


def pose_errors(actual, target):
    """Calculate center distance and absolute shortest-yaw error."""
    values = (actual.x, actual.y, actual.yaw, target.x, target.y, target.yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pose values must be finite')
    return PoseErrors(
        position=math.hypot(actual.x - target.x, actual.y - target.y),
        yaw=abs(normalize_angle(actual.yaw - target.yaw)),
    )


def check_cp2_crossing(previous_pose, sample_pose, before, cp2, after,
                       angular_speed):
    """Check one near-CP2 sample against the directed collinear plan segment."""
    values = (
        previous_pose.x, previous_pose.y, previous_pose.yaw,
        sample_pose.x, sample_pose.y, sample_pose.yaw,
        before.x, before.y, cp2.x, cp2.y, after.x, after.y, angular_speed,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('CP2 geometry values must be finite')
    segment_x = after.x - before.x
    segment_y = after.y - before.y
    segment_length = math.hypot(segment_x, segment_y)
    if segment_length <= 0.0:
        raise ValueError('CP2 surrounding segment must have positive length')
    direction_x = segment_x / segment_length
    direction_y = segment_y / segment_length
    cp2_cross_track = abs(
        -direction_y * (cp2.x - before.x)
        + direction_x * (cp2.y - before.y)
    )
    before_projection = (
        (cp2.x - before.x) * direction_x
        + (cp2.y - before.y) * direction_y
    )
    after_projection = (
        (after.x - cp2.x) * direction_x
        + (after.y - cp2.y) * direction_y
    )
    if (
        cp2_cross_track > ENVELOPE_EPSILON_M
        or before_projection <= 0.0
        or after_projection <= 0.0
    ):
        raise ValueError('CP2 must lie between collinear before/after points')

    relative_x = sample_pose.x - before.x
    relative_y = sample_pose.y - before.y
    longitudinal_error = (
        (cp2.x - sample_pose.x) * direction_x
        + (cp2.y - sample_pose.y) * direction_y
    )
    cross_track_error = abs(
        -direction_y * relative_x + direction_x * relative_y
    )
    desired_yaw = math.atan2(direction_y, direction_x)
    heading_error = abs(normalize_angle(sample_pose.yaw - desired_yaw))
    forward_progress = (
        (sample_pose.x - previous_pose.x) * direction_x
        + (sample_pose.y - previous_pose.y) * direction_y
    )
    passed = (
        abs(longitudinal_error)
        <= CP2_LONGITUDINAL_TOLERANCE_M + NUMERIC_COMPARISON_EPSILON
        and cross_track_error
        <= CP2_CROSS_TRACK_TOLERANCE_M + NUMERIC_COMPARISON_EPSILON
        and heading_error
        <= CP2_HEADING_TOLERANCE_RAD + NUMERIC_COMPARISON_EPSILON
        and abs(angular_speed)
        <= CP2_ANGULAR_SPEED_TOLERANCE_RADPS + NUMERIC_COMPARISON_EPSILON
        and forward_progress > 0.0
    )
    return CP2Result(
        passed=passed,
        longitudinal_error=longitudinal_error,
        cross_track_error=cross_track_error,
        heading_error=heading_error,
        angular_speed=angular_speed,
        forward_progress=forward_progress,
    )
