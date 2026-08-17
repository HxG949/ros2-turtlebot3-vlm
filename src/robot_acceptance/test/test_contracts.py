"""Tests for strict production String/JSON contracts."""

import json
import math

import pytest

from robot_acceptance.contracts import ContractError
from robot_acceptance.contracts import parse_arbiter_status
from robot_acceptance.contracts import parse_control_status
from robot_acceptance.contracts import parse_plan
from robot_acceptance.contracts import parse_safety_status
from robot_acceptance.contracts import parse_selected_target
from robot_acceptance.contracts import parse_watchdog_status


def target_data():
    """Return one valid production selected-target object."""
    return {
        'valid': True,
        'frame_id': 'odom',
        'target': {
            'id': 'space_2',
            'center_x': 0.8015,
            'center_y': 0.0,
            'entry_yaw': 0.0,
            'final_yaw': math.pi,
            'length': 0.297,
            'width': 0.210,
            'approach_distance': 0.4,
        },
    }


def plan_data():
    """Return one valid production plan object."""
    return {
        'valid': True,
        'reason': 'current_lane_safe',
        'selected_y': 0.0,
        'minimum_clearance': 0.12,
        'parking_space_id': 'space_2',
        'obstacle_point_count': 15,
        'scan_point_count': 20,
        'scan_valid_fraction': 1.0,
        'robot_pose': {'x': -0.8, 'y': 0.0, 'yaw': 0.0},
        'waypoints': [
            {'role': 'start', 'x': -0.8, 'y': 0.0, 'stop_required': True},
            {'role': 'cp1', 'x': -0.4, 'y': 0.0, 'stop_required': True},
            {'role': 'cp2', 'x': 0.4, 'y': 0.0, 'stop_required': False},
            {
                'role': 'parking_goal',
                'x': 0.8015,
                'y': 0.0,
                'stop_required': True,
                'final_yaw': math.pi,
                'parking_space_id': 'space_2',
                'parking_length': 0.297,
                'parking_width': 0.210,
            },
        ],
    }


def control_data():
    """Return one valid follower status object."""
    return {
        'enabled': True,
        'stop_at_cp1': False,
        'state': 'COMPLETE',
        'reason': 'parking_complete',
        'target_role': 'parking_goal',
        'remaining_distance': 0.0,
        'linear_x': 0.0,
        'angular_z': 0.0,
    }


def safety_data():
    """Return one valid lidar safety status object."""
    return {
        'front_distance': 0.8,
        'left_distance': 0.5,
        'right_distance': 0.5,
        'minimum_distance': 0.5,
        'rotation_minimum_distance': 0.4,
        'front_valid_fraction': 1.0,
        'left_valid_fraction': 1.0,
        'right_valid_fraction': 1.0,
        'full_valid_fraction': 1.0,
        'left_safe': True,
        'right_safe': True,
        'rotation_safe': True,
        'rotation_safe_raw': True,
        'rotation_caution': False,
        'rotation_unsafe_streak': 0,
        'emergency_stop': False,
        'valid': True,
    }


def arbiter_data():
    """Return one valid safety arbiter status object."""
    return {
        'enabled': True,
        'armed': True,
        'latched': False,
        'state': 'ACTIVE',
        'reason': 'command_allowed',
        'linear_x': 0.0,
        'angular_z': 0.0,
        'cmd_vel_publisher_count': 1,
    }


def watchdog_data():
    """Return one valid command watchdog status object."""
    return {
        'emergency_active': False,
        'reason': 'arbiter_heartbeat_fresh',
        'publishing_cmd_vel': False,
    }


def test_all_six_production_contracts_parse_to_immutable_values():
    """Parse all required String contracts and expose normalized values."""
    target = parse_selected_target(json.dumps(target_data()))
    plan = parse_plan(json.dumps(plan_data()))
    control = parse_control_status(json.dumps(control_data()))
    safety = parse_safety_status(json.dumps(safety_data()))
    arbiter = parse_arbiter_status(json.dumps(arbiter_data()))
    watchdog = parse_watchdog_status(json.dumps(watchdog_data()))

    assert target.target.space_id == 'space_2'
    assert plan.waypoints[2].role == 'cp2'
    assert control.reason == 'parking_complete'
    assert safety.valid is True
    assert arbiter.cmd_vel_publisher_count == 1
    assert watchdog.emergency_active is False
    with pytest.raises(AttributeError):
        target.valid = False


@pytest.mark.parametrize('payload', ['[]', 'null', '1', 'true', '"text"'])
def test_non_object_roots_are_rejected(payload):
    """Reject every JSON root type other than object."""
    with pytest.raises(ContractError, match='root must be an object'):
        parse_watchdog_status(payload)


@pytest.mark.parametrize('constant', ['NaN', 'Infinity', '-Infinity'])
def test_non_finite_json_constants_are_rejected(constant):
    """Reject Python JSON decoder's non-standard number constants."""
    payload = json.dumps(target_data()).replace('0.8015', constant)
    with pytest.raises(ContractError, match='non-finite'):
        parse_selected_target(payload)


def test_overflowed_json_number_is_rejected_even_in_unknown_nested_data():
    """Reject every non-finite decoded number, not only consumed fields."""
    data = watchdog_data()
    data['diagnostic'] = {'value': float('inf')}
    payload = json.dumps(data).replace('Infinity', '1e400')
    with pytest.raises(ContractError, match='must be finite'):
        parse_watchdog_status(payload)


@pytest.mark.parametrize(
    'parser,factory,field',
    [
        (parse_selected_target, target_data, 'frame_id'),
        (parse_plan, plan_data, 'waypoints'),
        (parse_control_status, control_data, 'reason'),
        (parse_safety_status, safety_data, 'valid'),
        (parse_arbiter_status, arbiter_data, 'armed'),
        (parse_watchdog_status, watchdog_data, 'reason'),
    ],
)
def test_missing_field_is_rejected_for_each_contract(parser, factory, field):
    """Require every defined field rather than applying defaults."""
    data = factory()
    del data[field]
    with pytest.raises(ContractError, match='missing required fields'):
        parser(json.dumps(data))


@pytest.mark.parametrize(
    'parser,data,field',
    [
        (parse_selected_target, target_data(), ('target', 'length')),
        (parse_plan, plan_data(), ('waypoints', 0, 'x')),
        (parse_control_status, control_data(), ('linear_x',)),
        (parse_safety_status, safety_data(), ('front_valid_fraction',)),
        (parse_arbiter_status, arbiter_data(), ('angular_z',)),
    ],
)
def test_boolean_is_never_accepted_as_a_number(parser, data, field):
    """Defend against bool being a subclass of int in Python."""
    target = data
    if len(field) == 1:
        target[field[0]] = True
    elif field[0] == 'target':
        target['target'][field[1]] = True
    else:
        target['waypoints'][field[1]][field[2]] = True
    with pytest.raises(ContractError, match='must be a number'):
        parser(json.dumps(target))


def test_wrong_types_unknown_states_and_duplicate_fields_are_rejected():
    """Reject type confusion, unknown states, and ambiguous duplicate keys."""
    control = control_data()
    control['state'] = 'MAGIC'
    with pytest.raises(ContractError, match='unknown state'):
        parse_control_status(json.dumps(control))
    with pytest.raises(ContractError, match='must be a boolean'):
        parse_watchdog_status(json.dumps({**watchdog_data(),
                                          'emergency_active': 0}))
    with pytest.raises(ContractError, match='duplicate field'):
        parse_watchdog_status(
            '{"reason":"a","reason":"b",'
            '"emergency_active":false,"publishing_cmd_vel":false}'
        )


@pytest.mark.parametrize(
    'parser,factory',
    [
        (parse_selected_target, target_data),
        (parse_plan, plan_data),
        (parse_control_status, control_data),
        (parse_safety_status, safety_data),
        (parse_arbiter_status, arbiter_data),
        (parse_watchdog_status, watchdog_data),
    ],
)
def test_unknown_root_fields_are_rejected(parser, factory):
    """Freeze each production JSON contract against silent field drift."""
    data = factory()
    data['unexpected'] = 'field'
    with pytest.raises(ContractError, match='unexpected fields'):
        parser(json.dumps(data))


def test_valid_plan_enforces_cp2_and_terminal_parking_semantics():
    """Reject routes that violate the frozen pass-through shape."""
    plan = plan_data()
    plan['waypoints'][2]['stop_required'] = True
    with pytest.raises(ContractError, match='cp2 must pass through'):
        parse_plan(json.dumps(plan))

    plan = plan_data()
    plan['waypoints'].append(plan['waypoints'].pop(2))
    with pytest.raises(ContractError, match='parking_goal must be the final'):
        parse_plan(json.dumps(plan))


def test_plan_strictly_checks_production_diagnostic_fields():
    """Do not silently ignore malformed planner scan or pose diagnostics."""
    plan = plan_data()
    plan['scan_point_count'] = True
    with pytest.raises(ContractError, match='must be an integer'):
        parse_plan(json.dumps(plan))

    plan = plan_data()
    plan['robot_pose']['yaw'] = 'zero'
    with pytest.raises(ContractError, match='must be a number'):
        parse_plan(json.dumps(plan))
