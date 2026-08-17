"""Tests for monitor cache behavior and read-only safety invariants."""

import ast
from pathlib import Path

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import pytest

from robot_acceptance.contracts import ContractError
from robot_acceptance.monitor_node import MonitorCache
from robot_acceptance.monitor_node import parse_collision_object
from robot_acceptance.monitor_node import REQUIRED_TOPICS
from robot_acceptance.monitor_node import validate_model_states
from robot_acceptance.monitor_node import validate_twist


def monitor_source():
    """Read the installed package source used by this test suite."""
    import robot_acceptance.monitor_node as monitor_node
    return Path(monitor_node.__file__).read_text(encoding='ascii')


def test_cache_uses_receive_time_for_missing_and_stale_topics():
    """Freshness depends only on supplied monotonic receive timestamps."""
    cache = MonitorCache()
    cache.record('/a', object(), monotonic_time=10.0, ros_time_ns=25)

    assert cache.stale_topics(10.5, 1.0, ('/a', '/b')) == ('/b',)
    assert cache.stale_topics(11.01, 1.0, ('/a', '/b')) == ('/a', '/b')
    assert cache.samples['/a'].ros_time_ns == 25


def test_invalid_contract_does_not_replace_last_valid_sample():
    """Preserve prior evidence but separately expose telemetry invalidity."""
    cache = MonitorCache()
    cache.record('/status', 'valid', 1.0, 2)
    cache.record_error('/status', 'bad payload', 1.5, 3)
    assert cache.samples['/status'].value == 'valid'
    issue = cache.contract_errors['/status']
    assert issue.error == 'bad payload'
    assert issue.monotonic_time == 1.5
    assert issue.ros_time_ns == 3


def test_non_finite_command_and_model_state_are_rejected():
    """Never cache invalid numeric evidence as a usable ROS sample."""
    command = Twist()
    command.linear.x = float('nan')
    with pytest.raises(ContractError, match='non-finite'):
        validate_twist(command, '/cmd_vel')

    states = ModelStates()
    states.name = ['robot', 'extra']
    states.pose = [Pose()]
    states.twist = [Twist()]
    with pytest.raises(ContractError, match='lengths differ'):
        validate_model_states(states, '/gazebo/model_states')


def test_collision_json_rejects_non_finite_and_bad_count():
    """Keep collision evidence strict without inventing its future schema."""
    with pytest.raises(ContractError, match='finite'):
        parse_collision_object('{"point": [1e400]}', '/collision')
    with pytest.raises(ContractError, match='non-negative integer'):
        parse_collision_object('{"collision_count": true}', '/collision')


def test_collision_events_are_not_required_for_zero_collision_freshness():
    """Honor the specification exception for an empty collision-event topic."""
    assert '/acceptance/collision_status' in REQUIRED_TOPICS
    assert '/acceptance/collision_events' not in REQUIRED_TOPICS


def test_monitor_source_has_no_publishers_clients_or_parameter_mutation():
    """Statically guarantee the monitor cannot actuate or reconfigure peers."""
    source = monitor_source()
    tree = ast.parse(source)
    forbidden_methods = {
        'create_publisher',
        'create_client',
        'set_parameters',
        'set_parameters_atomically',
    }
    calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert calls.isdisjoint(forbidden_methods)
    assert source.count('create_subscription(') == 13


def test_monitor_never_creates_control_or_desired_publishers():
    """Explicitly lock both forbidden output topics to subscription-only use."""
    source = monitor_source()
    tree = ast.parse(source)
    publisher_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    assert publisher_calls == []
    import robot_acceptance.monitor_node as monitor_node
    assert monitor_node.TOPIC_CMD == '/cmd_vel'
    assert monitor_node.TOPIC_DESIRED_CMD == '/navigation/desired_cmd_vel'
