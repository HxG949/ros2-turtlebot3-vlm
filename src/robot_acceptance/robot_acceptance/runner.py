"""Safe orchestration and evidence gating for the P0 acceptance chain."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

import yaml

from robot_acceptance.artifacts import ArtifactRun
from robot_acceptance.artifacts import utc_timestamp
from robot_acceptance.bag_session import BagSession
from robot_acceptance.bag_session import REQUIRED_TOPICS as BAG_REQUIRED_TOPICS
from robot_acceptance.process_supervisor import ProcessSupervisor
from robot_acceptance.reporting import build_result
from robot_acceptance.reporting import write_reports
from robot_acceptance.verdict import Code
from robot_acceptance.verdict import EXIT_CODES


BASIC_TOPICS = ('/clock', '/scan', '/odom', '/gazebo/model_states')
PLANNING_TOPICS = (
    '/parking/candidates',
    '/parking/selected_target',
    '/navigation/plan',
    '/safety/status',
)
MOTION_GATE_TOPICS = (
    '/gazebo/contacts/base',
    '/gazebo/contacts/lidar',
    '/gazebo/contacts/wheel_left',
    '/gazebo/contacts/wheel_right',
    '/gazebo/contacts/caster',
    '/acceptance/collision_status',
)
READINESS_TOPICS = BAG_REQUIRED_TOPICS
PACKAGE_NAMES = (
    'robot_simulation',
    'robot_perception',
    'robot_acceptance',
    'robot_collision_observer',
)


class RunnerFailure(RuntimeError):
    """Carry a stable result code out of orchestration."""

    def __init__(self, code, reason, details=None):
        super().__init__(reason)
        self.code = Code(code)
        self.reason = reason
        self.details = details or {}


@dataclass(frozen=True)
class RunOptions:
    """Contain immutable command-line choices for one run."""

    enable_motion: bool = False
    seed: int = 42
    headless: bool = True
    artifacts_root: Path = Path('artifacts')


def package_config_path(filename):
    """Locate installed config, with a source-tree fallback for tests."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('robot_acceptance'))
        candidate = share / 'config' / filename
        if candidate.is_file():
            return candidate
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parents[1] / 'config' / filename


def load_contract(path=None):
    """Load and minimally enforce the frozen P0 configuration."""
    config_path = Path(path) if path else package_config_path('p0_contract.yaml')
    try:
        value = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise RunnerFailure(Code.CONFIG_INVALID, str(error)) from error
    if (
        not isinstance(value, dict)
        or value.get('layout_seed') != 42
        or value.get('parking_space_id') != 'space_2'
        or value.get('stop_at_cp1') is not False
    ):
        raise RunnerFailure(
            Code.CONFIG_INVALID,
            'P0 contract must fix seed=42, space_2, and stop_at_cp1=false',
        )
    return value


def simulation_command(options):
    """Build argv for the existing simulation launch file."""
    return [
        'ros2', 'launch', 'robot_simulation', 'bottle_world.launch.py',
        f'layout_seed:={options.seed}',
        f'use_gui:={str(not options.headless).lower()}',
    ]


def planning_command():
    """Build argv for perception and planning without control publishers."""
    return [
        'ros2', 'launch', 'robot_perception',
        'obstacle_planning.launch.py',
        'parking_space_id:=space_2',
        'use_rviz:=false',
    ]


def navigation_command(options):
    """Build control argv with motion controlled only by the explicit option."""
    return [
        'ros2', 'launch', 'robot_perception',
        'obstacle_control.launch.py',
        f'enable_motion:={str(options.enable_motion).lower()}',
        'stop_at_cp1:=false',
        'required_desired_subscribers:=3',
        'required_status_subscribers:=2',
        'subscriber_confirmation_duration:=0.5',
    ]


def monitor_command(output_path, enable_motion,
                    robot_entity_name='turtlebot3_burger_low_lidar'):
    """Build argv for the read-only monitor executable."""
    return [
        'ros2', 'run', 'robot_acceptance', 'robot_acceptance_monitor',
        '--ros-args',
        '-p', f'output_path:={Path(output_path).resolve()}',
        '-p', f'enable_motion:={str(bool(enable_motion)).lower()}',
        '-p', f'robot_entity_name:={robot_entity_name}',
    ]


def collision_observer_command():
    """Build argv for the acceptance-only collision observer."""
    return [
        'ros2', 'run', 'robot_collision_observer',
        'robot_collision_observer_node',
    ]


def _run_probe(argv):
    return subprocess.run(
        argv,
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
    )


def preflight(probe=_run_probe):
    """Verify ROS commands/packages and reject an occupied /cmd_vel graph."""
    if shutil.which('ros2') is None:
        raise RunnerFailure(Code.PREFLIGHT_FAILED, 'ros2 executable not found')
    for package in PACKAGE_NAMES:
        result = probe(['ros2', 'pkg', 'prefix', package])
        if result.returncode != 0:
            raise RunnerFailure(
                Code.PREFLIGHT_FAILED,
                f'ROS package is unavailable: {package}',
            )
    bag_help = probe(['ros2', 'bag', 'record', '--help'])
    if bag_help.returncode != 0:
        raise RunnerFailure(Code.PREFLIGHT_FAILED,
                            'ros2 bag record is unavailable')
    topics = probe(['ros2', 'topic', 'list', '--no-daemon'])
    if topics.returncode != 0:
        raise RunnerFailure(Code.PREFLIGHT_FAILED,
                            'unable to inspect the current ROS graph')
    if '/cmd_vel' not in topics.stdout.splitlines():
        return
    info = probe([
        'ros2', 'topic', 'info', '/cmd_vel', '--verbose', '--no-daemon',
    ])
    if info.returncode != 0 or 'Publisher count: 0' not in info.stdout:
        raise RunnerFailure(
            Code.PREFLIGHT_FAILED,
            'existing /cmd_vel publishers make the run unsafe',
            {'cmd_vel_graph': info.stdout.strip()},
        )


def wait_for_topics(required, timeout, supervisor, probe=_run_probe):
    """Wait for topic names while detecting early managed-process exits."""
    started = time.monotonic()
    deadline = started + timeout
    missing = tuple(required)
    while time.monotonic() < deadline:
        for name in tuple(supervisor.processes):
            returncode = supervisor.poll(name)
            if returncode is not None:
                raise RunnerFailure(
                    Code.PROCESS_EXITED,
                    f'{name} exited before readiness with code {returncode}',
                )
        result = probe(['ros2', 'topic', 'list', '--no-daemon'])
        if result.returncode == 0:
            available = set(result.stdout.splitlines())
            missing = tuple(topic for topic in required if topic not in available)
            if not missing:
                return time.monotonic() - started
        time.sleep(0.2)
    raise RunnerFailure(
        Code.READINESS_TIMEOUT,
        'required acceptance interfaces did not become ready',
        {'missing_topics': list(missing)},
    )


def wait_for_node(node_name, timeout, supervisor, probe=_run_probe):
    """Wait until a required ROS node is discoverable and its process is alive."""
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        for name in tuple(supervisor.processes):
            returncode = supervisor.poll(name)
            if returncode is not None:
                raise RunnerFailure(
                    Code.PROCESS_EXITED,
                    f'{name} exited before node readiness with code {returncode}',
                )
        result = probe(['ros2', 'node', 'list', '--no-daemon'])
        if result.returncode == 0 and node_name in result.stdout.splitlines():
            return
        time.sleep(0.1)
    raise RunnerFailure(
        Code.READINESS_TIMEOUT,
        f'ROS node did not become ready: {node_name}',
    )


def wait_for_collision_ready(timeout, supervisor, probe=_run_probe):
    """Require fresh, valid, zero-collision sensor evidence before navigation."""
    wait_for_topics(MOTION_GATE_TOPICS, timeout, supervisor, probe)
    started = time.monotonic()
    deadline = started + timeout
    last_status = None
    while time.monotonic() < deadline:
        result = probe([
            'ros2', 'topic', 'echo', '/acceptance/collision_status',
            'std_msgs/msg/String', '--once', '--field', 'data',
            '--qos-reliability', 'reliable', '--no-daemon',
        ])
        if result.returncode != 0:
            continue
        try:
            documents = [
                value for value in yaml.safe_load_all(result.stdout)
                if value is not None and value != '---'
            ]
            payload = documents[0] if documents else result.stdout.strip()
            if isinstance(payload, dict) and set(payload) == {'data'}:
                payload = payload['data']
            status = json.loads(payload) if isinstance(payload, str) else payload
            if not isinstance(status, dict):
                raise ValueError('collision status root is not an object')
            valid = status['valid']
            fresh = status['all_sensors_fresh']
            count = status['collision_count']
            if type(valid) is not bool or type(fresh) is not bool:
                raise ValueError('collision status flags are not booleans')
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ValueError('collision count is invalid')
        except (IndexError, KeyError, TypeError, ValueError,
                json.JSONDecodeError, yaml.YAMLError) as error:
            raise RunnerFailure(
                Code.TELEMETRY_INVALID,
                f'collision status contract invalid: {error}',
            ) from error
        last_status = status
        if count > 0:
            raise RunnerFailure(
                Code.COLLISION_DETECTED,
                'unexpected collision detected before navigation startup',
                {'collision_count': count},
            )
        if valid and fresh:
            return status
        time.sleep(0.1)
    raise RunnerFailure(
        Code.COLLISION_EVIDENCE_MISSING,
        'contact sensors did not become fresh before navigation startup',
        {'last_collision_status': last_status},
    )


def read_monitor_result(path):
    """Strictly load the monitor's atomic terminal result."""
    result_path = Path(path)

    def reject_constant(value):
        raise ValueError(f'non-finite JSON number: {value}')

    try:
        value = json.loads(
            result_path.read_text(encoding='ascii'),
            parse_constant=reject_constant,
        )
        if not isinstance(value, dict):
            raise ValueError('monitor result root is not an object')
        required = {'code', 'reason', 'mission', 'metrics', 'failures'}
        missing = sorted(required - set(value))
        if missing:
            raise ValueError(
                'monitor result missing fields: ' + ', '.join(missing)
            )
        Code(value['code'])
        if not isinstance(value['reason'], str) or not value['reason']:
            raise ValueError('monitor result reason is empty')
        if not isinstance(value['mission'], dict):
            raise ValueError('monitor mission is not an object')
        if not isinstance(value['metrics'], dict):
            raise ValueError('monitor metrics is not an object')
        if not isinstance(value['failures'], list):
            raise ValueError('monitor failures is not an array')
        _require_finite_json(value)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise RunnerFailure(
            Code.EVIDENCE_INCOMPLETE,
            f'monitor terminal evidence is invalid: {error}',
        ) from error
    return value


def _require_finite_json(value, path='monitor_result'):
    if isinstance(value, dict):
        for key, item in value.items():
            _require_finite_json(item, f'{path}.{key}')
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _require_finite_json(item, f'{path}[{index}]')
    elif not isinstance(value, bool) and isinstance(value, (int, float)):
        if not math.isfinite(value):
            raise ValueError(f'{path} must be finite')


def wait_for_monitor_terminal(supervisor, output_path, timeout):
    """Wait for monitor output while supervising every evidence process."""
    deadline = time.monotonic() + float(timeout)
    protected = (
        'simulation', 'collision_observer', 'planning', 'rosbag',
        'navigation',
    )
    while time.monotonic() < deadline:
        if Path(output_path).is_file():
            return read_monitor_result(output_path)
        monitor_returncode = supervisor.poll('monitor')
        if monitor_returncode is not None:
            return read_monitor_result(output_path)
        for name in protected:
            if name not in supervisor.processes:
                continue
            returncode = supervisor.poll(name)
            if returncode is not None:
                raise RunnerFailure(
                    Code.PROCESS_EXITED,
                    f'{name} exited during mission with code {returncode}',
                    {'process': name, 'returncode': returncode},
                )
        time.sleep(0.05)
    raise RunnerFailure(
        Code.MISSION_TIMEOUT,
        'monitor did not reach a terminal result within 125 seconds',
    )


def _completed_hold(monitor_data, required_hold):
    if not isinstance(monitor_data, dict):
        return False
    mission = monitor_data.get('mission', {})
    follower = mission.get('follower', {})
    metrics = monitor_data.get('metrics', {})
    hold = metrics.get('post_complete_hold_s')
    return (
        follower.get('state') == 'COMPLETE'
        and follower.get('reason') == 'parking_complete'
        and isinstance(hold, (int, float))
        and not isinstance(hold, bool)
        and hold >= required_hold
    )


class AcceptanceRunner:
    """Run P0 orchestration and always emit a conservative result."""

    def __init__(self, options, argv=None, repository=None,
                 supervisor_factory=ProcessSupervisor):
        self.options = options
        self.argv = list(argv or [])
        self.repository = Path(repository or os.getcwd()).resolve()
        self.supervisor_factory = supervisor_factory

    def run(self):
        """Execute the process chain, clean it safely, and publish artifacts."""
        started_at = utc_timestamp()
        artifacts = ArtifactRun(self.options.artifacts_root)
        supervisor = self.supervisor_factory(artifacts.logs_path)
        bag = None
        contract = None
        code = Code.EVIDENCE_INCOMPLETE
        reason = 'acceptance evidence is incomplete'
        details = {}
        readiness_duration = 0.0
        normal_completion = False
        collision_status = None
        monitor_data = None
        monitor_output = artifacts.path('monitor_result.json')
        validation = None
        bag_stopped = False
        readiness_reached = False
        try:
            contract = load_contract()
            if self.options.seed != contract['layout_seed']:
                raise RunnerFailure(
                    Code.CONFIG_INVALID,
                    'P0 layout seed is fixed at 42',
                )
            preflight()
            supervisor.start('simulation', simulation_command(self.options))
            wait_for_topics(
                BASIC_TOPICS,
                contract['thresholds']['readiness_timeout_s'],
                supervisor,
            )
            supervisor.start(
                'collision_observer',
                collision_observer_command(),
            )
            collision_status = wait_for_collision_ready(
                contract['thresholds']['readiness_timeout_s'],
                supervisor,
            )
            supervisor.start('planning', planning_command())
            wait_for_topics(
                PLANNING_TOPICS,
                contract['thresholds']['readiness_timeout_s'],
                supervisor,
            )
            bag = BagSession(
                supervisor,
                artifacts.path('rosbag'),
                package_config_path('rosbag_qos_overrides.yaml'),
            )
            bag.start()
            supervisor.start(
                'monitor',
                monitor_command(
                    monitor_output,
                    self.options.enable_motion,
                ),
            )
            wait_for_node(
                '/robot_acceptance_monitor',
                contract['thresholds']['readiness_timeout_s'],
                supervisor,
            )
            mission_wait_started = time.monotonic()
            supervisor.start('navigation', navigation_command(self.options))
            readiness_duration = wait_for_topics(
                READINESS_TOPICS,
                contract['thresholds']['readiness_timeout_s'],
                supervisor,
            )
            readiness_reached = True
            if self.options.enable_motion:
                total_monitor_time = (
                    contract['thresholds']['mission_timeout_s']
                    + contract['thresholds']['monitor_grace_s']
                )
                monitor_data = wait_for_monitor_terminal(
                    supervisor,
                    monitor_output,
                    max(
                        0.0,
                        total_monitor_time
                        - (time.monotonic() - mission_wait_started),
                    ),
                )
                code = Code(monitor_data['code'])
                reason = monitor_data['reason']
                details = {
                    'monitor_failures': monitor_data.get('failures', []),
                }
            else:
                time.sleep(float(contract['dry_run_evidence_duration_s']))
                code = Code.EVIDENCE_INCOMPLETE
                reason = (
                    'dry run collected readiness evidence with motion disabled; '
                    'no mission PASS is claimed'
                )
            normal_completion = (
                not self.options.enable_motion
                or _completed_hold(
                    monitor_data,
                    contract['thresholds']['post_complete_hold_s'],
                )
            )
            if normal_completion and bag is not None:
                bag.stop()
                bag_stopped = True
                validation = bag.validate()
                if not validation.complete:
                    code = Code.EVIDENCE_INCOMPLETE
                    reason = 'rosbag evidence is incomplete'
                    details = {
                        'missing_topics': list(validation.missing_topics),
                        'metadata_errors': list(validation.errors),
                    }
        except RunnerFailure as error:
            code = error.code
            reason = error.reason
            details = error.details
        except (OSError, subprocess.SubprocessError, ValueError) as error:
            code = Code.LAUNCH_FAILED
            reason = str(error)
        except KeyboardInterrupt:
            code = Code.UNEXPECTED_SHUTDOWN
            reason = 'runner interrupted before evidence was complete'
        except Exception as error:
            code = Code.UNEXPECTED_SHUTDOWN
            reason = f'unexpected runner error: {error}'
        finally:
            if normal_completion:
                if bag is not None and not bag_stopped:
                    bag.stop()
                    bag_stopped = True
                    validation = bag.validate()
                supervisor.stop_many((
                    'navigation',
                    'monitor',
                    'planning',
                    'collision_observer',
                    'simulation',
                ))
            else:
                supervisor.stop('navigation')
                if bag is not None and not bag_stopped:
                    bag.stop()
                    bag_stopped = True
                    validation = bag.validate()
                supervisor.stop_many((
                    'monitor',
                    'planning',
                    'collision_observer',
                    'simulation',
                ))

        if bag is not None and validation is None:
            validation = bag.validate()
        if (
            readiness_reached
            and normal_completion
            and (validation is None or not validation.complete)
        ):
            code = Code.EVIDENCE_INCOMPLETE
            reason = 'rosbag evidence is incomplete'
            details = {
                'missing_topics': (
                    list(validation.missing_topics) if validation else []
                ),
            }
        ended_at = utc_timestamp()
        manifest = artifacts.manifest(
            self.argv,
            self.repository,
            started_at,
            ended_at,
        )
        final_directory = artifacts.final_path
        evidence = self._evidence(final_directory, validation)
        runner_data = {
            'layout_seed': self.options.seed,
            'enable_motion': self.options.enable_motion,
            'headless': self.options.headless,
            'readiness_duration_s': readiness_duration,
            'collision_count': (
                monitor_data.get('metrics', {}).get('collision_count')
                if monitor_data is not None
                else collision_status['collision_count']
                if collision_status is not None
                else None
            ),
            'failure_details': details,
        }
        if contract is None:
            contract = load_contract()
        result = build_result(
            manifest,
            contract,
            code,
            reason,
            evidence,
            runner_data=runner_data,
            monitor_data=monitor_data,
        )
        code = Code(result['outcome']['primary_code'])
        artifacts.write_json('manifest.json', manifest)
        artifacts.write_json('runner.json', {
            **runner_data,
            'processes': supervisor.records(),
            'code': code.value,
            'reason': reason,
        })
        event = {
            'event': 'run_finished',
            'timestamp_utc': ended_at,
            'code': code.value,
        }
        events_path = artifacts.path('events.jsonl')
        if events_path.exists():
            with events_path.open('a', encoding='ascii') as stream:
                stream.write(json.dumps(
                    event, allow_nan=False, sort_keys=True
                ) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
        else:
            artifacts.write_jsonl('events.jsonl', [event])
        for name, header in (
            ('pose.csv', 'monotonic_s,ros_time_ns,source,x,y,yaw\n'),
            ('cmd_vel.csv', 'monotonic_s,ros_time_ns,topic,linear_x,angular_z\n'),
            ('safety.csv', 'monotonic_s,ros_time_ns,state,reason\n'),
            ('parking_margins.csv',
             'monotonic_s,source,front,rear,left,right,minimum\n'),
        ):
            if not artifacts.path(name).exists():
                artifacts.write_text(name, header)
        write_reports(artifacts, result)
        artifacts.finalize()
        return EXIT_CODES[code], final_directory

    @staticmethod
    def _evidence(final_directory, validation):
        counts = validation.topic_message_counts if validation else {}
        missing = validation.missing_topics if validation else BAG_REQUIRED_TOPICS
        bag_path = final_directory / 'rosbag'
        return {
            'artifact_directory': str(final_directory),
            'bag_path': str(bag_path) if validation else None,
            'bag_complete': bool(validation and validation.complete),
            'required_topics': list(BAG_REQUIRED_TOPICS),
            'topic_message_counts': counts,
            'missing_topics': list(missing),
            'events_path': str(final_directory / 'events.jsonl'),
            'report_path': str(final_directory / 'report.md'),
            'junit_path': str(final_directory / 'junit.xml'),
            'logs_path': str(final_directory / 'logs'),
            'video_path': None,
        }
