"""Tests for runner safety defaults, launch argv, and conservative reports."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from robot_acceptance import runner
from robot_acceptance.bag_session import BagValidation
from robot_acceptance.bag_session import REQUIRED_TOPICS
from robot_acceptance.cli import build_parser
from robot_acceptance.reporting import build_result
from robot_acceptance.runner import AcceptanceRunner
from robot_acceptance.runner import navigation_command
from robot_acceptance.runner import monitor_command
from robot_acceptance.runner import planning_command
from robot_acceptance.runner import preflight
from robot_acceptance.runner import RunnerFailure
from robot_acceptance.runner import RunOptions
from robot_acceptance.runner import simulation_command
from robot_acceptance.runner import wait_for_collision_ready
from robot_acceptance.runner import wait_for_monitor_terminal
from robot_acceptance.verdict import Code
from test_mission import complete_and_hold
from test_mission import cross_cp2
from test_mission import make_ready


def test_cli_defaults_to_headless_motion_disabled_and_seed_42():
    """Never enable movement through defaults or an ambiguous option."""
    arguments = build_parser().parse_args([])
    assert arguments.enable_motion is False
    assert arguments.seed == 42
    assert arguments.gui is False


def test_motion_requires_explicit_flag_and_reaches_only_navigation_argv():
    """Map the explicit flag to the production launch argument exactly once."""
    disabled = RunOptions()
    enabled = RunOptions(enable_motion=True)
    assert 'enable_motion:=false' in navigation_command(disabled)
    assert 'enable_motion:=true' in navigation_command(enabled)
    assert all('enable_motion' not in item for item in simulation_command(enabled))
    assert build_parser().parse_args(['--enable-motion']).enable_motion is True


def test_monitor_command_receives_output_motion_and_entity_parameters(tmp_path):
    """Wire the monitor artifact and explicit motion gate through ROS argv."""
    command = monitor_command(tmp_path / 'monitor_result.json', True)
    assert any(item.startswith('output_path:=') for item in command)
    assert 'enable_motion:=true' in command
    assert (
        'robot_entity_name:=turtlebot3_burger_low_lidar' in command
    )


def test_launch_commands_fix_space_stop_rviz_seed_and_headless():
    """Pin all frozen P0 launch arguments rather than inheriting defaults."""
    options = RunOptions(seed=42, headless=True)
    assert simulation_command(options)[-2:] == [
        'layout_seed:=42',
        'use_gui:=false',
    ]
    navigation = navigation_command(options)
    assert 'stop_at_cp1:=false' in navigation
    assert 'required_desired_subscribers:=3' in navigation
    assert 'required_status_subscribers:=2' in navigation
    assert 'subscriber_confirmation_duration:=0.5' in navigation
    planning = planning_command()
    assert 'parking_space_id:=space_2' in planning
    assert 'use_rviz:=false' in planning


def test_preflight_rejects_existing_cmd_vel_publishers(monkeypatch):
    """Fail closed when another graph already has command publishers."""
    monkeypatch.setattr(runner.shutil, 'which', lambda name: '/usr/bin/ros2')

    def probe(argv):
        if argv[1:3] == ['topic', 'list']:
            return SimpleNamespace(returncode=0, stdout='/cmd_vel\n')
        if argv[1:4] == ['topic', 'info', '/cmd_vel']:
            return SimpleNamespace(returncode=0, stdout='Publisher count: 1\n')
        return SimpleNamespace(returncode=0, stdout='available\n')

    with pytest.raises(RunnerFailure, match='unsafe'):
        preflight(probe)


def test_preflight_accepts_an_unoccupied_graph(monkeypatch):
    """Allow startup only after graph inspection proves no command topic."""
    monkeypatch.setattr(runner.shutil, 'which', lambda name: '/usr/bin/ros2')

    def probe(argv):
        output = '/scan\n' if argv[1:3] == ['topic', 'list'] else 'available\n'
        return SimpleNamespace(returncode=0, stdout=output)

    preflight(probe)


def test_reporting_downgrades_unproven_pass():
    """Require monitor and bag evidence before PASS can be represented."""
    contract = runner.load_contract()
    result = build_result(
        {
            'run_id': 'test',
            'started_at_utc': '2026-08-17T00:00:00Z',
            'ended_at_utc': '2026-08-17T00:00:01Z',
        },
        contract,
        Code.PASS,
        'not real',
        {'bag_complete': False, 'missing_topics': ['/odom']},
    )
    assert result['outcome']['primary_code'] == 'EVIDENCE_INCOMPLETE'
    assert result['outcome']['verdict'] == 'ERROR'


class FakeSupervisor:
    """Record lifecycle calls without starting ROS, Gazebo, or movement."""

    instances = []

    def __init__(self, logs_path):
        self.logs_path = logs_path
        self.processes = {}
        self.calls = []
        self.instances.append(self)

    def start(self, name, argv):
        """Record a process launch request."""
        self.processes[name] = tuple(argv)
        self.calls.append(('start', name))
        return object()

    def poll(self, name):
        """Keep every fake process alive."""
        return None

    def stop(self, name):
        """Record one requested process stop."""
        if name in self.processes:
            self.calls.append(('stop', name))
        return 0

    def stop_many(self, names):
        """Record stops in supplied order."""
        for name in names:
            self.stop(name)

    def records(self):
        """Return minimal JSON-compatible fake records."""
        return [{
            'name': name,
            'argv': list(argv),
            'pid': 1,
            'returncode': 0,
        } for name, argv in self.processes.items()]


class FakeBagSession:
    """Provide configurable complete metadata without running rosbag2."""

    complete = True

    def __init__(self, supervisor, output_path, qos_path):
        self.supervisor = supervisor
        self.output_path = output_path

    def start(self):
        """Record a fake rosbag launch."""
        return self.supervisor.start('rosbag', ['fake-rosbag'])

    def stop(self):
        """Record the required bag stop point."""
        return self.supervisor.stop('rosbag')

    def validate(self):
        """Return complete or deliberately incomplete metadata."""
        missing = () if self.complete else ('/odom',)
        counts = {
            topic: 1 if topic not in missing else 0
            for topic in REQUIRED_TOPICS
        }
        return BagValidation(self.complete, counts, missing, ())


def passing_monitor_data():
    """Produce a real pure-state PASS document for runner tests."""
    state = make_ready()
    cross_cp2(state)
    complete_and_hold(state)
    assert state.result['code'] == 'PASS'
    return state.result


def test_reporting_rejects_nested_missing_pass_metric():
    """Treat a missing final-pose field as incomplete PASS evidence."""
    monitor = passing_monitor_data()
    del monitor['metrics']['final_odom']['x_m']
    contract = runner.load_contract()
    result = build_result(
        {
            'run_id': 'test',
            'started_at_utc': '2026-08-17T00:00:00Z',
            'ended_at_utc': '2026-08-17T00:00:01Z',
        },
        contract,
        Code.PASS,
        'claimed pass',
        {'bag_complete': True, 'missing_topics': []},
        runner_data={'enable_motion': True},
        monitor_data=monitor,
    )
    assert result['outcome']['primary_code'] == 'EVIDENCE_INCOMPLETE'
    assert result['metrics']['final_odom'] is None


def test_readiness_failure_generates_error_artifacts_without_ros(
        monkeypatch, tmp_path):
    """Preserve evidence and emit ERROR when phase-5 topics are unavailable."""
    FakeSupervisor.instances.clear()
    monkeypatch.setattr(runner, 'preflight', lambda: None)
    monkeypatch.setattr(
        runner,
        'wait_for_collision_ready',
        lambda *args: {'collision_count': 0},
    )
    calls = {'count': 0}

    def fake_wait(required, timeout, supervisor):
        calls['count'] += 1
        if calls['count'] < 3:
            return 0.1
        raise RunnerFailure(
            Code.READINESS_TIMEOUT,
            'collision interfaces are not ready',
            {'missing_topics': ['/acceptance/collision_status']},
        )

    monkeypatch.setattr(runner, 'wait_for_topics', fake_wait)
    monkeypatch.setattr(runner, 'wait_for_node', lambda *args: None)
    options = RunOptions(artifacts_root=tmp_path)
    exit_code, artifact_directory = AcceptanceRunner(
        options,
        argv=['robot_acceptance_run'],
        repository=Path(__file__).parents[3],
        supervisor_factory=FakeSupervisor,
    ).run()

    result = json.loads((artifact_directory / 'result.json').read_text())
    assert exit_code == 14
    assert result['outcome']['verdict'] == 'ERROR'
    assert result['outcome']['primary_code'] == 'READINESS_TIMEOUT'
    assert result['run']['enable_motion'] is False
    assert (artifact_directory / 'manifest.json').is_file()
    assert (artifact_directory / 'report.md').is_file()
    assert (artifact_directory / 'junit.xml').is_file()

    lifecycle = FakeSupervisor.instances[-1].calls
    assert lifecycle[:4] == [
        ('start', 'simulation'),
        ('start', 'collision_observer'),
        ('start', 'planning'),
        ('start', 'rosbag'),
    ]
    assert lifecycle[4:6] == [
        ('start', 'monitor'),
        ('start', 'navigation'),
    ]
    assert lifecycle[6:] == [
        ('stop', 'navigation'),
        ('stop', 'rosbag'),
        ('stop', 'monitor'),
        ('stop', 'planning'),
        ('stop', 'collision_observer'),
        ('stop', 'simulation'),
    ]


def test_normal_dry_run_closes_bag_before_other_processes(monkeypatch,
                                                          tmp_path):
    """Flush bag evidence first on a non-exceptional dry-run shutdown."""
    FakeSupervisor.instances.clear()
    monkeypatch.setattr(runner, 'preflight', lambda: None)
    monkeypatch.setattr(
        runner,
        'wait_for_collision_ready',
        lambda *args: {'collision_count': 0},
    )
    monkeypatch.setattr(
        runner,
        'wait_for_topics',
        lambda required, timeout, supervisor: 0.1,
    )
    monkeypatch.setattr(runner, 'wait_for_node', lambda *args: None)
    exit_code, artifact_directory = AcceptanceRunner(
        RunOptions(artifacts_root=tmp_path),
        supervisor_factory=FakeSupervisor,
    ).run()

    result = json.loads((artifact_directory / 'result.json').read_text())
    assert exit_code == 16
    assert result['outcome']['primary_code'] == 'EVIDENCE_INCOMPLETE'
    lifecycle = FakeSupervisor.instances[-1].calls
    assert lifecycle[6:] == [
        ('stop', 'rosbag'),
        ('stop', 'navigation'),
        ('stop', 'monitor'),
        ('stop', 'planning'),
        ('stop', 'collision_observer'),
        ('stop', 'simulation'),
    ]


def test_explicit_motion_is_gated_before_navigation(monkeypatch, tmp_path):
    """Never launch enabled navigation without collision topic readiness."""
    FakeSupervisor.instances.clear()
    monkeypatch.setattr(runner, 'preflight', lambda: None)
    monkeypatch.setattr(
        runner,
        'wait_for_topics',
        lambda required, timeout, supervisor: 0.1,
    )
    monkeypatch.setattr(runner, 'wait_for_node', lambda *args: None)

    def fake_collision_wait(timeout, supervisor):
        raise RunnerFailure(
            Code.READINESS_TIMEOUT,
            'collision evidence gate is unavailable',
        )

    monkeypatch.setattr(runner, 'wait_for_collision_ready', fake_collision_wait)
    exit_code, artifact_directory = AcceptanceRunner(
        RunOptions(enable_motion=True, artifacts_root=tmp_path),
        supervisor_factory=FakeSupervisor,
    ).run()

    result = json.loads((artifact_directory / 'result.json').read_text())
    assert exit_code == 14
    assert result['outcome']['verdict'] == 'ERROR'
    lifecycle = FakeSupervisor.instances[-1].calls
    assert ('start', 'navigation') not in lifecycle
    assert lifecycle == [
        ('start', 'simulation'),
        ('start', 'collision_observer'),
        ('stop', 'collision_observer'),
        ('stop', 'simulation'),
    ]


def test_collision_gate_requires_fresh_zero_collision_status():
    """Reject stale or colliding contact evidence before navigation starts."""
    class Supervisor:
        processes = {}

    calls = {'count': 0}

    def probe(argv):
        if argv[1:3] == ['topic', 'list']:
            return SimpleNamespace(
                returncode=0,
                stdout='\n'.join(runner.MOTION_GATE_TOPICS) + '\n',
            )
        calls['count'] += 1
        status = {
            'valid': calls['count'] > 1,
            'all_sensors_fresh': calls['count'] > 1,
            'collision_count': 0,
        }
        return SimpleNamespace(
            returncode=0,
            stdout=yaml.safe_dump({'data': json.dumps(status)}),
        )

    status = wait_for_collision_ready(1.0, Supervisor(), probe)
    assert status['collision_count'] == 0
    assert calls['count'] == 2


def test_monitor_result_file_is_terminal_while_monitor_process_is_alive(
        tmp_path):
    """Use the atomic result file instead of waiting for rclpy process exit."""
    output = tmp_path / 'monitor_result.json'
    output.write_text(json.dumps(passing_monitor_data()), encoding='ascii')

    class Supervisor:
        processes = {}

        @staticmethod
        def poll(name):
            raise AssertionError(f'process polling was unnecessary: {name}')

    result = wait_for_monitor_terminal(Supervisor(), output, 1.0)
    assert result['code'] == 'PASS'


@pytest.mark.parametrize(
    'bag_complete,expected_code',
    [(True, 'PASS'), (False, 'EVIDENCE_INCOMPLETE')],
)
def test_fake_process_motion_pass_and_bag_completeness_override(
        monkeypatch, tmp_path, bag_complete, expected_code):
    """Allow proven PASS but let incomplete metadata override monitor PASS."""
    FakeSupervisor.instances.clear()
    monkeypatch.setattr(runner, 'preflight', lambda: None)
    monkeypatch.setattr(
        runner,
        'wait_for_collision_ready',
        lambda *args: {'collision_count': 0},
    )
    monkeypatch.setattr(
        runner,
        'wait_for_topics',
        lambda required, timeout, supervisor: 0.1,
    )
    monkeypatch.setattr(runner, 'wait_for_node', lambda *args: None)
    FakeBagSession.complete = bag_complete
    monkeypatch.setattr(runner, 'BagSession', FakeBagSession)
    monitor = passing_monitor_data()
    monkeypatch.setattr(
        runner,
        'wait_for_monitor_terminal',
        lambda *args: monitor,
    )
    exit_code, artifact_directory = AcceptanceRunner(
        RunOptions(enable_motion=True, artifacts_root=tmp_path),
        supervisor_factory=FakeSupervisor,
    ).run()

    result = json.loads((artifact_directory / 'result.json').read_text())
    assert result['outcome']['primary_code'] == expected_code
    assert exit_code == (0 if bag_complete else 16)
    lifecycle = FakeSupervisor.instances[-1].calls
    assert lifecycle[6:12] == [
        ('stop', 'rosbag'),
        ('stop', 'navigation'),
        ('stop', 'monitor'),
        ('stop', 'planning'),
        ('stop', 'collision_observer'),
        ('stop', 'simulation'),
    ]
    if bag_complete:
        assert result['outcome']['verdict'] == 'PASS'
        assert result['failures'] == []
    else:
        assert result['outcome']['verdict'] == 'ERROR'
