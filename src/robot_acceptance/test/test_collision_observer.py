"""Tests for collision filtering, deduplication, and safety isolation."""

from pathlib import Path

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ContactState
import pytest

from robot_acceptance.collision import ContactTracker
from robot_acceptance.collision import is_approved_ground_contact


ROBOT_LEFT = (
    'turtlebot3_burger_low_lidar::wheel_left_link::wheel_left_collision'
)
FLOOR = 'competition_field::field_link::floor_collision'
BOTTLE = 'competition_bottle_1::bottle_link::bottle_collision'


def contacts(first, second):
    """Build one Gazebo ContactsState message with a single pair."""
    message = ContactsState()
    state = ContactState()
    state.collision1_name = first
    state.collision2_name = second
    message.states = [state]
    return message


def test_only_explicit_wheel_or_caster_floor_pairs_are_allowed():
    """Keep wheel-floor normal while preserving wheel-obstacle collisions."""
    assert is_approved_ground_contact('wheel_left', ROBOT_LEFT, FLOOR)
    assert is_approved_ground_contact('wheel_left', FLOOR, ROBOT_LEFT)
    assert not is_approved_ground_contact('wheel_left', ROBOT_LEFT, BOTTLE)
    assert not is_approved_ground_contact('base', ROBOT_LEFT, FLOOR)


def test_unexpected_contact_is_counted_once_until_release_grace_expires():
    """Deduplicate continuous 100 Hz samples but count a later new onset."""
    tracker = ContactTracker()
    message = contacts(ROBOT_LEFT, BOTTLE)
    assert len(tracker.observe('wheel_left', message, 1.0)) == 1
    assert tracker.observe('wheel_left', message, 1.01) == ()
    assert tracker.collision_count == 1
    tracker.observe('wheel_left', ContactsState(), 1.10)
    assert len(tracker.observe('wheel_left', message, 1.11)) == 1
    assert tracker.collision_count == 2


def test_sensor_heartbeat_exposes_missing_and_fresh_inputs():
    """Represent never-seen sensors with None rather than fake zero ages."""
    tracker = ContactTracker()
    tracker.observe('base', ContactsState(), 5.0)
    ages = tracker.sensor_ages(5.1)
    assert ages['base'] == pytest.approx(0.1)
    assert ages['lidar'] is None


def test_cpp_observer_publishes_only_acceptance_topics():
    """Prevent collision evidence code from acquiring control capability."""
    source_path = (
        Path(__file__).resolve().parents[2]
        / 'robot_collision_observer'
        / 'src'
        / 'collision_observer_node.cpp'
    )
    source = source_path.read_text(encoding='ascii')
    forbidden = {'/cmd_vel', '/navigation/desired_cmd_vel', '/safety/status'}
    assert all(topic not in source for topic in forbidden)
    assert '"/acceptance/collision_status"' in source
    assert '"/acceptance/collision_events"' in source
