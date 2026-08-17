"""rosbag2 command construction, lifecycle, and metadata validation."""

from dataclasses import dataclass
from pathlib import Path

import yaml


OPTIONAL_TOPICS = ('/acceptance/collision_events',)
REQUIRED_TOPICS = (
    '/clock',
    '/scan',
    '/odom',
    '/parking/candidates',
    '/parking/selected_target',
    '/navigation/plan',
    '/navigation/control_status',
    '/navigation/desired_cmd_vel',
    '/navigation/safety_arbiter_status',
    '/navigation/cmd_vel_watchdog_status',
    '/safety/status',
    '/cmd_vel',
    '/gazebo/model_states',
    '/gazebo/contacts/base',
    '/gazebo/contacts/lidar',
    '/gazebo/contacts/wheel_left',
    '/gazebo/contacts/wheel_right',
    '/gazebo/contacts/caster',
    '/acceptance/collision_status',
    '/tf',
    '/tf_static',
)
RECORD_TOPICS = REQUIRED_TOPICS + OPTIONAL_TOPICS


@dataclass(frozen=True)
class BagValidation:
    """Describe topic counts and completeness from rosbag metadata."""

    complete: bool
    topic_message_counts: dict
    missing_topics: tuple
    errors: tuple


def build_record_command(output_path, qos_overrides_path):
    """Build the explicit, loss-averse P0 rosbag2 argv."""
    output = Path(output_path)
    qos = Path(qos_overrides_path)
    if output.exists():
        raise FileExistsError(f'bag output path already exists: {output}')
    if not qos.is_file():
        raise FileNotFoundError(f'QoS override file does not exist: {qos}')
    return [
        'ros2', 'bag', 'record',
        '--storage', 'sqlite3',
        '--max-cache-size', '0',
        '--use-sim-time',
        '--include-unpublished-topics',
        '--qos-profile-overrides-path', str(qos),
        '-o', str(output),
        *RECORD_TOPICS,
    ]


def validate_metadata(metadata_path):
    """Parse rosbag2 metadata and require a message on each required topic."""
    path = Path(metadata_path)
    if not path.is_file():
        return BagValidation(False, {}, REQUIRED_TOPICS,
                             ('metadata.yaml is missing',))
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
        entries = value['rosbag2_bagfile_information'][
            'topics_with_message_count'
        ]
        counts = {}
        for entry in entries:
            topic = entry['topic_metadata']['name']
            count = entry['message_count']
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ValueError(f'invalid message count for {topic}')
            counts[topic] = counts.get(topic, 0) + count
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        return BagValidation(False, {}, REQUIRED_TOPICS,
                             (f'invalid metadata.yaml: {error}',))
    missing = tuple(topic for topic in REQUIRED_TOPICS if counts.get(topic, 0) <= 0)
    return BagValidation(not missing, counts, missing, ())


class BagSession:
    """Manage one rosbag recorder through a process supervisor."""

    def __init__(self, supervisor, output_path, qos_overrides_path):
        self.supervisor = supervisor
        self.output_path = Path(output_path)
        self.qos_overrides_path = Path(qos_overrides_path)
        self.process = None

    def start(self):
        """Start recording to a path that rosbag2 can create."""
        command = build_record_command(
            self.output_path,
            self.qos_overrides_path,
        )
        self.process = self.supervisor.start('rosbag', command)
        return self.process

    def stop(self):
        """Request a normal rosbag close; supervisor sends SIGINT first."""
        return self.supervisor.stop('rosbag')

    def validate(self):
        """Validate metadata produced after the recorder has stopped."""
        return validate_metadata(self.output_path / 'metadata.yaml')
