"""Tests for explicit rosbag topics, QoS, and metadata completeness."""

from pathlib import Path

import yaml

from robot_acceptance.bag_session import build_record_command
from robot_acceptance.bag_session import OPTIONAL_TOPICS
from robot_acceptance.bag_session import RECORD_TOPICS
from robot_acceptance.bag_session import REQUIRED_TOPICS
from robot_acceptance.bag_session import validate_metadata


def test_record_command_is_explicit_loss_averse_and_uses_nonexistent_path(
        tmp_path):
    """Build the required sqlite3, zero-cache, simulated-time invocation."""
    qos = tmp_path / 'qos.yaml'
    qos.write_text('{}\n')
    output = tmp_path / 'bag'
    command = build_record_command(output, qos)

    assert command[:3] == ['ros2', 'bag', 'record']
    assert command[command.index('--storage') + 1] == 'sqlite3'
    assert command[command.index('--max-cache-size') + 1] == '0'
    assert '--use-sim-time' in command
    assert '--include-unpublished-topics' in command
    assert command[command.index('-o') + 1] == str(output)
    assert command[-len(RECORD_TOPICS):] == list(RECORD_TOPICS)
    assert len(RECORD_TOPICS) == len(set(RECORD_TOPICS))


def test_topics_include_contacts_collision_and_tf():
    """Record all five raw contacts plus normalized and transform evidence."""
    contacts = {
        '/gazebo/contacts/base',
        '/gazebo/contacts/lidar',
        '/gazebo/contacts/wheel_left',
        '/gazebo/contacts/wheel_right',
        '/gazebo/contacts/caster',
    }
    assert contacts <= set(REQUIRED_TOPICS)
    assert '/acceptance/collision_status' in REQUIRED_TOPICS
    assert OPTIONAL_TOPICS == ('/acceptance/collision_events',)
    assert {'/tf', '/tf_static'} <= set(REQUIRED_TOPICS)


def _write_metadata(path, counts):
    entries = [{
        'topic_metadata': {'name': topic, 'type': 'test/Message'},
        'message_count': count,
    } for topic, count in counts.items()]
    value = {'rosbag2_bagfile_information': {
        'topics_with_message_count': entries,
    }}
    path.write_text(yaml.safe_dump(value))


def test_metadata_allows_missing_or_empty_collision_events(tmp_path):
    """Do not require an event message to prove a zero-collision run."""
    metadata = tmp_path / 'metadata.yaml'
    _write_metadata(metadata, {topic: 1 for topic in REQUIRED_TOPICS})
    validation = validate_metadata(metadata)
    assert validation.complete
    assert validation.missing_topics == ()

    counts = {topic: 2 for topic in REQUIRED_TOPICS}
    counts[OPTIONAL_TOPICS[0]] = 0
    _write_metadata(metadata, counts)
    validation = validate_metadata(metadata)
    assert validation.complete
    assert validation.topic_message_counts[OPTIONAL_TOPICS[0]] == 0


def test_metadata_requires_positive_counts_for_every_required_topic(tmp_path):
    """Classify absent and zero-count required evidence as incomplete."""
    metadata = tmp_path / 'metadata.yaml'
    counts = {topic: 1 for topic in REQUIRED_TOPICS}
    counts['/scan'] = 0
    counts.pop('/odom')
    _write_metadata(metadata, counts)
    validation = validate_metadata(metadata)
    assert not validation.complete
    assert validation.missing_topics == ('/scan', '/odom')


def test_qos_overrides_match_sensor_status_and_static_tf_contract():
    """Keep sensors best-effort, statuses reliable, and static TF durable."""
    path = Path(__file__).parents[1] / 'config' / 'rosbag_qos_overrides.yaml'
    qos = yaml.safe_load(path.read_text())
    for topic in ('/scan', '/odom', '/gazebo/model_states',
                  '/gazebo/contacts/base'):
        assert qos[topic]['reliability'] == 'best_effort'
        assert qos[topic]['durability'] == 'volatile'
    assert qos['/navigation/control_status']['reliability'] == 'reliable'
    assert qos['/acceptance/collision_status']['reliability'] == 'reliable'
    assert qos['/tf_static']['durability'] == 'transient_local'
