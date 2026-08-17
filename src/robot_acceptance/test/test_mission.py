"""Pure online mission-state tests without ROS, Gazebo, or movement."""

import json
import math

import pytest

from robot_acceptance.contracts import parse_arbiter_status
from robot_acceptance.contracts import parse_control_status
from robot_acceptance.contracts import parse_plan
from robot_acceptance.contracts import parse_safety_status
from robot_acceptance.contracts import parse_selected_target
from robot_acceptance.contracts import parse_watchdog_status
from robot_acceptance.geometry import Pose2D
from robot_acceptance.mission import MissionState
from robot_acceptance.mission import MotionSample
from robot_acceptance.mission import TOPIC_CMD
from robot_acceptance.mission import TOPIC_COLLISION_STATUS
from robot_acceptance.mission import TOPIC_CONTROL_STATUS
from robot_acceptance.mission import TOPIC_DESIRED_CMD
from robot_acceptance.mission import TOPIC_SAFETY_STATUS
from robot_acceptance.mission import TOPIC_SCAN
from robot_acceptance.runner import load_contract
from robot_acceptance.verdict import Code
from test_synthetic_monitor import arbiter_status
from test_synthetic_monitor import control_status
from test_synthetic_monitor import safety_status
from test_synthetic_monitor import selected_target
from test_synthetic_monitor import valid_plan
from test_synthetic_monitor import watchdog_status


def decoded(parser, value):
    """Parse one test dictionary through its production contract."""
    return parser(json.dumps(value, allow_nan=False))


def sample(x, y, yaw, linear=0.0, angular=0.0):
    """Build one finite planar motion sample."""
    return MotionSample(Pose2D(x, y, yaw), linear, angular)


def collision_status(count=0):
    """Build valid fresh normalized collision evidence."""
    return {
        'valid': True,
        'all_sensors_fresh': True,
        'collision_count': count,
    }


def waiting_control():
    """Build an enabled pre-terminal follower status."""
    value = control_status('WAITING', 'waiting_for_plan')
    value['target_role'] = None
    value['remaining_distance'] = None
    return decoded(parse_control_status, value)


def make_ready():
    """Create a fully ready state with aligned stationary evidence."""
    state = MissionState(
        load_contract(),
        True,
        'turtlebot3_burger_low_lidar',
        started_monotonic=0.0,
        freshness_timeout_s=2.0,
    )
    now = 0.01
    state.observe_target(
        decoded(parse_selected_target, selected_target()), now
    )
    state.observe_plan(decoded(parse_plan, valid_plan()), now)
    state.observe_control(waiting_control(), now)
    state.observe_safety(decoded(parse_safety_status, safety_status()), now)
    state.observe_arbiter(
        decoded(parse_arbiter_status, arbiter_status()), now
    )
    state.observe_watchdog(
        decoded(parse_watchdog_status, watchdog_status()), now
    )
    state.observe_presence(TOPIC_SCAN, now)
    state.observe_command(TOPIC_DESIRED_CMD, 0.0, 0.0, now)
    state.observe_command(TOPIC_CMD, 0.0, 0.0, now)
    state.observe_collision_status(collision_status(), now)
    initial = sample(-0.9015, -0.845, 0.0)
    state.observe_odom(initial, now)
    state.observe_world(initial, now + 0.001)
    state.tick(0.02)
    assert state.ready_time == pytest.approx(0.02)
    state.observe_command(TOPIC_CMD, 0.02, 0.0, 0.03)
    assert state.motion_start_time == pytest.approx(0.03)
    return state


def cross_cp2(state, now=0.10):
    """Supply a directed odometry sample at frozen-plan CP2."""
    state.observe_odom(sample(0.19, -0.315, 0.0), now - 0.01)
    state.observe_odom(sample(0.20, -0.315, 0.0), now)
    assert state.cp2_result is not None


def complete_and_hold(state, final_pose=None):
    """Complete the mission and provide every required hold observation."""
    pose = final_pose or sample(0.8015, 0.0, math.pi)
    state.observe_odom(pose, 0.20)
    state.observe_world(pose, 0.201)
    complete = decoded(parse_control_status, control_status())
    state.observe_control(complete, 0.30)
    state.observe_control(complete, 0.40)
    state.observe_safety(
        decoded(parse_safety_status, safety_status()), 0.40
    )
    state.observe_arbiter(
        decoded(parse_arbiter_status, arbiter_status()), 0.40
    )
    state.observe_watchdog(
        decoded(parse_watchdog_status, watchdog_status()), 0.40
    )
    state.observe_odom(pose, 0.40)
    state.observe_world(pose, 0.401)
    state.observe_command(TOPIC_DESIRED_CMD, 0.0, 0.0, 0.40)
    state.observe_command(TOPIC_CMD, 0.0, 0.0, 0.40)
    state.observe_collision_status(collision_status(), 0.40)
    state.tick(1.31)


def test_complete_success_sequence_produces_measured_pass():
    """Require readiness, CP2, terminal, hold, and dual-pose geometry."""
    state = make_ready()
    cross_cp2(state)
    complete_and_hold(state)

    assert state.result['code'] == Code.PASS.value
    assert state.result['mission']['plan_stable'] is True
    assert state.result['mission']['cp2_validated'] is True
    assert state.result['metrics']['frame_alignment']['validated'] is True
    assert state.result['metrics']['stationary']['validated'] is True
    assert state.result['metrics']['collision_count'] == 0
    transitions = [
        event['to'] for event in state.events
        if event['event'] == 'state_transition'
    ]
    assert transitions == [
        'PREPARING', 'READY', 'RUNNING', 'POST_COMPLETE_HOLD',
        'FINALIZING', 'PASS',
    ]


@pytest.mark.parametrize(
    'fault,expected',
    [
        ('controller', Code.CONTROLLER_FAULT),
        ('arbiter', Code.SAFETY_CHAIN_FAULT),
        ('watchdog', Code.SAFETY_CHAIN_FAULT),
        ('collision', Code.COLLISION_DETECTED),
    ],
)
def test_explicit_runtime_faults_terminate_immediately(fault, expected):
    """Classify each independent controller and safety-chain fault."""
    state = make_ready()
    if fault == 'controller':
        state.observe_control(decoded(
            parse_control_status,
            control_status('FAULT', 'controller_failed'),
        ), 0.10)
    elif fault == 'arbiter':
        state.observe_arbiter(decoded(
            parse_arbiter_status,
            arbiter_status('LATCHED', 'desired_timeout', True),
        ), 0.10)
    elif fault == 'watchdog':
        state.observe_watchdog(decoded(
            parse_watchdog_status, watchdog_status(True)
        ), 0.10)
    else:
        state.observe_collision_status(collision_status(1), 0.10)
    assert state.result['code'] == expected.value


def test_plan_change_after_motion_is_rejected_by_canonical_hash():
    """Reject normalized plan content changes after output motion starts."""
    state = make_ready()
    changed = valid_plan()
    changed['selected_y'] = -0.314
    state.observe_plan(decoded(parse_plan, changed), 0.10)
    assert state.result['code'] == Code.PLAN_CHANGED.value


def test_complete_without_cp2_is_a_specific_failure():
    """Never infer CP2 from follower target-role or terminal state."""
    state = make_ready()
    state.observe_control(
        decoded(parse_control_status, control_status()), 0.20
    )
    assert state.result['code'] == Code.CP2_VALIDATION_FAILED.value


def test_parking_edge_crossing_fails_even_with_position_in_tolerance():
    """Use rotated footprint margins independently of center tolerance."""
    state = make_ready()
    cross_cp2(state)
    pose = sample(0.8015, 0.012, math.pi + 0.049)
    complete_and_hold(state, pose)
    assert state.result['code'] == Code.PARKING_ENVELOPE_FAILED.value


def test_nonzero_command_during_hold_is_post_complete_motion():
    """Fail immediately when output resumes after parking_complete."""
    state = make_ready()
    cross_cp2(state)
    pose = sample(0.8015, 0.0, math.pi)
    state.observe_odom(pose, 0.20)
    state.observe_world(pose, 0.201)
    state.observe_control(
        decoded(parse_control_status, control_status()), 0.30
    )
    state.observe_command(TOPIC_CMD, 0.001, 0.0, 0.31)
    assert state.result['code'] == Code.POST_COMPLETE_MOTION.value


def test_missing_hold_observation_cannot_pass():
    """Downgrade a completed task when hold evidence has a missing source."""
    state = make_ready()
    cross_cp2(state)
    pose = sample(0.8015, 0.0, math.pi)
    state.observe_odom(pose, 0.20)
    state.observe_world(pose, 0.201)
    complete = decoded(parse_control_status, control_status())
    state.observe_control(complete, 0.30)
    for topic, operation in (
        (TOPIC_CONTROL_STATUS, lambda: state.observe_control(complete, 0.4)),
        (TOPIC_SAFETY_STATUS, lambda: state.observe_safety(
            decoded(parse_safety_status, safety_status()), 0.4
        )),
        (TOPIC_CMD, lambda: state.observe_command(TOPIC_CMD, 0.0, 0.0, 0.4)),
        (TOPIC_COLLISION_STATUS, lambda: state.observe_collision_status(
            collision_status(), 0.4
        )),
    ):
        assert topic
        operation()
    state.observe_arbiter(
        decoded(parse_arbiter_status, arbiter_status()), 0.4
    )
    state.observe_watchdog(
        decoded(parse_watchdog_status, watchdog_status()), 0.4
    )
    state.observe_odom(pose, 0.4)
    state.observe_world(pose, 0.401)
    state.tick(1.31)
    assert state.result['code'] == Code.EVIDENCE_INCOMPLETE.value
    details = state.result['failures'][0]['details']
    assert TOPIC_DESIRED_CMD in details['missing_hold_topics']
