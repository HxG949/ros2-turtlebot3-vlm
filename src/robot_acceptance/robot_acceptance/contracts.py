"""Strict parsers for the production String/JSON telemetry contracts."""

from dataclasses import dataclass
import json
import math


class ContractError(ValueError):
    """Report a malformed or semantically invalid telemetry payload."""


@dataclass(frozen=True)
class ParkingTarget:
    """Describe one selected parking target."""

    space_id: str
    center_x: float
    center_y: float
    entry_yaw: float
    final_yaw: float
    length: float
    width: float
    approach_distance: float


@dataclass(frozen=True)
class SelectedTarget:
    """Represent the selected-target message."""

    valid: bool
    frame_id: str
    target: ParkingTarget


@dataclass(frozen=True)
class Waypoint:
    """Represent one planned waypoint."""

    role: str
    x: float
    y: float
    stop_required: bool
    final_yaw: float | None = None
    parking_space_id: str | None = None
    parking_length: float | None = None
    parking_width: float | None = None


@dataclass(frozen=True)
class Plan:
    """Represent the normalized fields of a production plan message."""

    valid: bool
    reason: str
    selected_y: float | None
    minimum_clearance: float | None
    waypoints: tuple[Waypoint, ...]
    parking_space_id: str | None
    obstacle_point_count: int
    scan_point_count: int
    scan_valid_fraction: float
    robot_pose: tuple[float, float, float] | None


@dataclass(frozen=True)
class ControlStatus:
    """Represent follower state and command telemetry."""

    enabled: bool
    stop_at_cp1: bool
    state: str
    reason: str
    target_role: str | None
    remaining_distance: float | None
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class SafetyStatus:
    """Represent lidar safety telemetry."""

    front_distance: float | None
    left_distance: float | None
    right_distance: float | None
    minimum_distance: float | None
    rotation_minimum_distance: float | None
    front_valid_fraction: float
    left_valid_fraction: float
    right_valid_fraction: float
    full_valid_fraction: float
    left_safe: bool
    right_safe: bool
    rotation_safe: bool
    rotation_safe_raw: bool
    rotation_caution: bool
    rotation_unsafe_streak: int
    emergency_stop: bool
    valid: bool


@dataclass(frozen=True)
class ArbiterStatus:
    """Represent safety arbiter state and output telemetry."""

    enabled: bool
    armed: bool
    latched: bool
    state: str
    reason: str
    linear_x: float
    angular_z: float
    cmd_vel_publisher_count: int


@dataclass(frozen=True)
class WatchdogStatus:
    """Represent command watchdog state."""

    emergency_active: bool
    reason: str
    publishing_cmd_vel: bool


WAYPOINT_ROLES = {
    'start',
    'cp1',
    'lane_entry',
    'cp2',
    'parking_transition',
    'parking_approach',
    'parking_goal',
}

CONTROL_STATES = {
    'DISABLED',
    'COMPLETE',
    'FAULT',
    'WAITING',
    'STOPPED',
    'ROTATION_CLEARANCE_WAIT',
    'ALIGNING',
    'ALIGNMENT_SETTLING',
    'WAYPOINT_ALIGNED',
    'DRIVING',
    'STOPPING',
    'SETTLING',
    'WAYPOINT_REACHED',
    'FINAL_ALIGNMENT',
    'FINAL_ALIGNMENT_SETTLING',
    'PARKING_SETTLING',
}

ARBITER_STATES = {'DISABLED', 'LATCHED', 'WAITING', 'BLOCKED', 'ACTIVE'}


def _reject_constant(value):
    raise ContractError(f'non-finite JSON number is not allowed: {value}')


def _object_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ContractError(f'duplicate field: {key}')
        result[key] = value
    return result


def _load(payload, contract):
    if not isinstance(payload, str):
        raise ContractError(f'{contract}: payload must be a JSON string')
    try:
        value = json.loads(
            payload,
            parse_constant=_reject_constant,
            object_pairs_hook=_object_pairs,
        )
    except ContractError:
        raise
    except (json.JSONDecodeError, TypeError) as error:
        raise ContractError(f'{contract}: invalid JSON: {error}') from error
    if not isinstance(value, dict):
        raise ContractError(f'{contract}: JSON root must be an object')
    _ensure_json_finite(value, contract)
    return value


def _ensure_json_finite(value, path):
    if isinstance(value, dict):
        for key, item in value.items():
            _ensure_json_finite(item, f'{path}.{key}')
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _ensure_json_finite(item, f'{path}[{index}]')
    elif not isinstance(value, bool) and isinstance(value, (int, float)):
        try:
            finite = math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            raise ContractError(f'{path} must be finite')


def _object(value, path):
    if not isinstance(value, dict):
        raise ContractError(f'{path} must be an object')
    return value


def _required(value, fields, path):
    missing = sorted(set(fields) - set(value))
    if missing:
        raise ContractError(f'{path} missing required fields: {", ".join(missing)}')


def _no_extra(value, fields, path):
    unexpected = sorted(set(value) - set(fields))
    if unexpected:
        raise ContractError(
            f'{path} has unexpected fields: {", ".join(unexpected)}'
        )


def _boolean(value, path):
    if type(value) is not bool:
        raise ContractError(f'{path} must be a boolean')
    return value


def _string(value, path, nullable=False):
    if value is None and nullable:
        return None
    if not isinstance(value, str) or not value.strip():
        suffix = ' or null' if nullable else ''
        raise ContractError(f'{path} must be a non-empty string{suffix}')
    return value


def _number(value, path, nullable=False):
    if value is None and nullable:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        suffix = ' or null' if nullable else ''
        raise ContractError(f'{path} must be a number{suffix}')
    if not math.isfinite(value):
        raise ContractError(f'{path} must be finite')
    return float(value)


def _integer(value, path):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError(f'{path} must be an integer')
    return value


def _state(value, allowed, path):
    state = _string(value, path)
    if state not in allowed:
        raise ContractError(f'{path} has unknown state: {state}')
    return state


def parse_selected_target(payload):
    """Parse a `/parking/selected_target` String payload."""
    root = _load(payload, 'selected_target')
    root_fields = {'valid', 'frame_id', 'target'}
    _required(root, root_fields, 'selected_target')
    _no_extra(root, root_fields, 'selected_target')
    target = _object(root['target'], 'selected_target.target')
    fields = {
        'id', 'center_x', 'center_y', 'entry_yaw', 'final_yaw',
        'length', 'width', 'approach_distance',
    }
    _required(target, fields, 'selected_target.target')
    _no_extra(target, fields, 'selected_target.target')
    result = ParkingTarget(
        space_id=_string(target['id'], 'selected_target.target.id'),
        center_x=_number(target['center_x'], 'selected_target.target.center_x'),
        center_y=_number(target['center_y'], 'selected_target.target.center_y'),
        entry_yaw=_number(target['entry_yaw'], 'selected_target.target.entry_yaw'),
        final_yaw=_number(target['final_yaw'], 'selected_target.target.final_yaw'),
        length=_number(target['length'], 'selected_target.target.length'),
        width=_number(target['width'], 'selected_target.target.width'),
        approach_distance=_number(
            target['approach_distance'],
            'selected_target.target.approach_distance',
        ),
    )
    if result.length <= 0.0 or result.width <= 0.0:
        raise ContractError('selected_target target dimensions must be positive')
    if result.approach_distance <= 0.0:
        raise ContractError('selected_target approach_distance must be positive')
    return SelectedTarget(
        valid=_boolean(root['valid'], 'selected_target.valid'),
        frame_id=_string(root['frame_id'], 'selected_target.frame_id'),
        target=result,
    )


def _parse_waypoint(value, index):
    path = f'plan.waypoints[{index}]'
    waypoint = _object(value, path)
    _required(waypoint, {'role', 'x', 'y', 'stop_required'}, path)
    role = _string(waypoint['role'], f'{path}.role')
    if role not in WAYPOINT_ROLES:
        raise ContractError(f'{path}.role is unknown: {role}')
    parking_fields = (
        'final_yaw', 'parking_space_id', 'parking_length', 'parking_width',
    )
    _no_extra(
        waypoint,
        {'role', 'x', 'y', 'stop_required', *parking_fields},
        path,
    )
    if role == 'parking_goal':
        _required(waypoint, set(parking_fields), path)
    elif any(field in waypoint for field in parking_fields):
        raise ContractError(f'{path} has parking fields on a non-goal waypoint')
    return Waypoint(
        role=role,
        x=_number(waypoint['x'], f'{path}.x'),
        y=_number(waypoint['y'], f'{path}.y'),
        stop_required=_boolean(waypoint['stop_required'], f'{path}.stop_required'),
        final_yaw=(
            _number(waypoint['final_yaw'], f'{path}.final_yaw')
            if role == 'parking_goal' else None
        ),
        parking_space_id=(
            _string(waypoint['parking_space_id'], f'{path}.parking_space_id')
            if role == 'parking_goal' else None
        ),
        parking_length=(
            _number(waypoint['parking_length'], f'{path}.parking_length')
            if role == 'parking_goal' else None
        ),
        parking_width=(
            _number(waypoint['parking_width'], f'{path}.parking_width')
            if role == 'parking_goal' else None
        ),
    )


def parse_plan(payload):
    """Parse a `/navigation/plan` String payload."""
    root = _load(payload, 'plan')
    fields = {
        'valid', 'reason', 'selected_y', 'minimum_clearance', 'waypoints',
        'obstacle_point_count', 'scan_point_count', 'scan_valid_fraction',
        'robot_pose',
    }
    _required(root, fields, 'plan')
    _no_extra(root, fields | {'parking_space_id'}, 'plan')
    if not isinstance(root['waypoints'], list):
        raise ContractError('plan.waypoints must be an array')
    valid = _boolean(root['valid'], 'plan.valid')
    waypoints = tuple(
        _parse_waypoint(value, index)
        for index, value in enumerate(root['waypoints'])
    )
    selected_y = _number(root['selected_y'], 'plan.selected_y', nullable=True)
    clearance = _number(
        root['minimum_clearance'],
        'plan.minimum_clearance',
        nullable=True,
    )
    parking_space_id = (
        _string(root.get('parking_space_id'), 'plan.parking_space_id')
        if 'parking_space_id' in root else None
    )
    obstacle_count = _integer(
        root['obstacle_point_count'], 'plan.obstacle_point_count'
    )
    scan_count = _integer(root['scan_point_count'], 'plan.scan_point_count')
    scan_fraction = _number(
        root['scan_valid_fraction'], 'plan.scan_valid_fraction'
    )
    if obstacle_count < 0 or scan_count < 0:
        raise ContractError('plan point counts must be non-negative')
    if not 0.0 <= scan_fraction <= 1.0:
        raise ContractError('plan.scan_valid_fraction must be within [0, 1]')
    robot_pose = None
    if root['robot_pose'] is not None:
        pose = _object(root['robot_pose'], 'plan.robot_pose')
        _required(pose, {'x', 'y', 'yaw'}, 'plan.robot_pose')
        _no_extra(pose, {'x', 'y', 'yaw'}, 'plan.robot_pose')
        robot_pose = (
            _number(pose['x'], 'plan.robot_pose.x'),
            _number(pose['y'], 'plan.robot_pose.y'),
            _number(pose['yaw'], 'plan.robot_pose.yaw'),
        )
    if valid:
        if selected_y is None or clearance is None or parking_space_id is None:
            raise ContractError('valid plan requires lane, clearance, and parking ID')
        roles = [waypoint.role for waypoint in waypoints]
        if roles.count('cp2') != 1 or roles.count('parking_goal') != 1:
            raise ContractError('valid plan requires exactly one cp2 and parking_goal')
        if not roles or roles[-1] != 'parking_goal':
            raise ContractError('parking_goal must be the final waypoint')
        if any(
            waypoint.stop_required == (waypoint.role == 'cp2')
            for waypoint in waypoints
        ):
            raise ContractError('cp2 must pass through; all other waypoints must stop')
        goal = waypoints[-1]
        if goal.parking_space_id != parking_space_id:
            raise ContractError('plan parking IDs do not match')
        if goal.parking_length <= 0.0 or goal.parking_width <= 0.0:
            raise ContractError('parking_goal dimensions must be positive')
    elif (
        waypoints or selected_y is not None or clearance is not None
        or parking_space_id is not None
    ):
        raise ContractError('invalid plan must not contain route data')
    return Plan(
        valid=valid,
        reason=_string(root['reason'], 'plan.reason'),
        selected_y=selected_y,
        minimum_clearance=clearance,
        waypoints=waypoints,
        parking_space_id=parking_space_id,
        obstacle_point_count=obstacle_count,
        scan_point_count=scan_count,
        scan_valid_fraction=scan_fraction,
        robot_pose=robot_pose,
    )


def parse_control_status(payload):
    """Parse a `/navigation/control_status` String payload."""
    root = _load(payload, 'control_status')
    fields = {
        'enabled', 'stop_at_cp1', 'state', 'reason', 'target_role',
        'remaining_distance', 'linear_x', 'angular_z',
    }
    _required(root, fields, 'control_status')
    _no_extra(root, fields, 'control_status')
    role = _string(root['target_role'], 'control_status.target_role', nullable=True)
    if role is not None and role not in WAYPOINT_ROLES:
        raise ContractError(f'control_status.target_role is unknown: {role}')
    return ControlStatus(
        enabled=_boolean(root['enabled'], 'control_status.enabled'),
        stop_at_cp1=_boolean(root['stop_at_cp1'], 'control_status.stop_at_cp1'),
        state=_state(root['state'], CONTROL_STATES, 'control_status.state'),
        reason=_string(root['reason'], 'control_status.reason'),
        target_role=role,
        remaining_distance=_number(
            root['remaining_distance'],
            'control_status.remaining_distance',
            nullable=True,
        ),
        linear_x=_number(root['linear_x'], 'control_status.linear_x'),
        angular_z=_number(root['angular_z'], 'control_status.angular_z'),
    )


def parse_safety_status(payload):
    """Parse a `/safety/status` String payload."""
    root = _load(payload, 'safety_status')
    distance_fields = (
        'front_distance', 'left_distance', 'right_distance',
        'minimum_distance', 'rotation_minimum_distance',
    )
    fraction_fields = (
        'front_valid_fraction', 'left_valid_fraction',
        'right_valid_fraction', 'full_valid_fraction',
    )
    boolean_fields = (
        'left_safe', 'right_safe', 'rotation_safe', 'rotation_safe_raw',
        'rotation_caution', 'emergency_stop', 'valid',
    )
    fields = set(distance_fields + fraction_fields + boolean_fields)
    fields.add('rotation_unsafe_streak')
    _required(root, fields, 'safety_status')
    _no_extra(root, fields, 'safety_status')
    distances = {
        field: _number(root[field], f'safety_status.{field}', nullable=True)
        for field in distance_fields
    }
    if any(value is not None and value < 0.0 for value in distances.values()):
        raise ContractError('safety_status distances must be non-negative')
    fractions = {
        field: _number(root[field], f'safety_status.{field}')
        for field in fraction_fields
    }
    if any(not 0.0 <= value <= 1.0 for value in fractions.values()):
        raise ContractError('safety_status valid fractions must be within [0, 1]')
    booleans = {
        field: _boolean(root[field], f'safety_status.{field}')
        for field in boolean_fields
    }
    streak = _integer(root['rotation_unsafe_streak'],
                      'safety_status.rotation_unsafe_streak')
    if streak < 0:
        raise ContractError('safety_status.rotation_unsafe_streak must be non-negative')
    return SafetyStatus(
        **distances,
        **fractions,
        **booleans,
        rotation_unsafe_streak=streak,
    )


def parse_arbiter_status(payload):
    """Parse a `/navigation/safety_arbiter_status` String payload."""
    root = _load(payload, 'arbiter_status')
    fields = {
        'enabled', 'armed', 'latched', 'state', 'reason', 'linear_x',
        'angular_z', 'cmd_vel_publisher_count',
    }
    _required(root, fields, 'arbiter_status')
    _no_extra(root, fields, 'arbiter_status')
    count = _integer(
        root['cmd_vel_publisher_count'],
        'arbiter_status.cmd_vel_publisher_count',
    )
    if count < 0:
        raise ContractError('arbiter_status publisher count must be non-negative')
    return ArbiterStatus(
        enabled=_boolean(root['enabled'], 'arbiter_status.enabled'),
        armed=_boolean(root['armed'], 'arbiter_status.armed'),
        latched=_boolean(root['latched'], 'arbiter_status.latched'),
        state=_state(root['state'], ARBITER_STATES, 'arbiter_status.state'),
        reason=_string(root['reason'], 'arbiter_status.reason'),
        linear_x=_number(root['linear_x'], 'arbiter_status.linear_x'),
        angular_z=_number(root['angular_z'], 'arbiter_status.angular_z'),
        cmd_vel_publisher_count=count,
    )


def parse_watchdog_status(payload):
    """Parse a `/navigation/cmd_vel_watchdog_status` String payload."""
    root = _load(payload, 'watchdog_status')
    fields = {'emergency_active', 'reason', 'publishing_cmd_vel'}
    _required(root, fields, 'watchdog_status')
    _no_extra(root, fields, 'watchdog_status')
    return WatchdogStatus(
        emergency_active=_boolean(
            root['emergency_active'],
            'watchdog_status.emergency_active',
        ),
        reason=_string(root['reason'], 'watchdog_status.reason'),
        publishing_cmd_vel=_boolean(
            root['publishing_cmd_vel'],
            'watchdog_status.publishing_cmd_vel',
        ),
    )
