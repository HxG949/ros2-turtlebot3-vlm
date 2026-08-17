"""Minimal structured JSON, Markdown, and JUnit acceptance reporting."""

import argparse
import json
import math
import os
from pathlib import Path
from xml.etree import ElementTree

from robot_acceptance.verdict import Classification
from robot_acceptance.verdict import Code
from robot_acceptance.verdict import ERROR_CODES
from robot_acceptance.verdict import EXIT_CODES


def _classification(code):
    return Classification.ERROR if code in ERROR_CODES else Classification.FAIL


def build_result(manifest, contract, code, reason, evidence,
                 runner_data=None, monitor_data=None):
    """Combine monitor and bag evidence without allowing an unproven PASS."""
    code = Code(code)
    runner = runner_data or {}
    monitor = monitor_data or {}
    readiness_duration = max(0.0, float(runner.get('readiness_duration_s', 0.0)))
    mission = _merge_shape(
        _empty_mission(), monitor.get('mission', {})
    )
    metrics = _merge_shape(_empty_metrics(
        readiness_duration,
        runner.get('collision_count'),
    ), monitor.get('metrics', {}))
    for key in ('final_odom', 'final_ground_truth'):
        metrics[key] = _normalize_final_pose(metrics[key])
    if code == Code.PASS:
        missing = _pass_gate_failures(
            contract, evidence, runner, monitor, mission, metrics
        )
        if missing:
            code = Code.EVIDENCE_INCOMPLETE
            reason = 'PASS rejected because required evidence is incomplete'
            runner = {
                **runner,
                'failure_source': 'reporting',
                'failure_details': {'pass_gate_failures': missing},
            }
    failures = _normalize_failures(monitor.get('failures', []))
    if code != Code.PASS and not any(
        failure.get('code') == code.value for failure in failures
    ):
        failures.append({
            'code': code.value,
            'source': runner.get('failure_source', 'runner'),
            'reason': reason,
            'observed_at_s': readiness_duration,
            'details': runner.get('failure_details', {}),
        })
    return {
        'schema_version': contract['schema_version'],
        'run': {
            'run_id': manifest['run_id'],
            'started_at_utc': manifest['started_at_utc'],
            'ended_at_utc': manifest['ended_at_utc'],
            'git_commit': manifest.get('git_commit') or '0' * 40,
            'git_dirty': bool(manifest.get('git_dirty')),
            'layout_seed': runner.get('layout_seed', contract['layout_seed']),
            'parking_space_id': contract['parking_space_id'],
            'enable_motion': bool(runner.get('enable_motion', False)),
            'stop_at_cp1': False,
            'headless': bool(runner.get('headless', True)),
        },
        'outcome': {
            'verdict': (
                Classification.PASS.value if code == Code.PASS
                else _classification(code).value
            ),
            'primary_code': code.value,
            'exit_code': EXIT_CODES[code],
            'summary': reason,
        },
        'thresholds': contract['thresholds'],
        'target': contract['target'],
        'mission': mission,
        'metrics': metrics,
        'evidence': evidence,
        'failures': failures,
    }


def _empty_mission():
    return {
        'ready': False,
        'motion_started': False,
        'first_nonzero_cmd_time_s': None,
        'plan_stable': False,
        'plan_sha256': None,
        'cp2_validated': False,
        'follower': {
            'state': None,
            'reason': None,
            'target_role': None,
            'fault_seen': False,
        },
        'arbiter': {
            'state': None,
            'reason': None,
            'armed': False,
            'latched': False,
            'cmd_vel_publisher_count': None,
        },
        'watchdog': {
            'emergency_active': False,
            'reason': None,
            'publishing_cmd_vel': False,
        },
        'safety': {
            'valid': False,
            'emergency_stop': False,
            'fault_seen': False,
        },
        'collision': {
            'valid': False,
            'all_sensors_fresh': False,
            'event_seen': False,
        },
    }


def _merge_shape(template, observed):
    if not isinstance(template, dict):
        return observed
    source = observed if isinstance(observed, dict) else {}
    return {
        key: (
            _merge_shape(default, source[key])
            if key in source else default
        )
        for key, default in template.items()
    }


def _normalize_final_pose(value):
    if not isinstance(value, dict):
        return None
    margins = value.get('margins_m')
    fields = (
        'x_m', 'y_m', 'yaw_rad', 'position_error_m', 'yaw_error_rad',
    )
    margin_fields = ('front', 'rear', 'left', 'right', 'minimum')
    if (
        any(field not in value for field in fields)
        or not isinstance(margins, dict)
        or any(field not in margins for field in margin_fields)
    ):
        return None
    return {
        field: value[field] for field in fields
    } | {
        'margins_m': {
            field: margins[field] for field in margin_fields
        },
    }


def _normalize_failures(values):
    failures = []
    if not isinstance(values, list):
        return failures
    for value in values:
        if not isinstance(value, dict):
            continue
        try:
            code = Code(value.get('code'))
        except ValueError:
            continue
        reason = value.get('reason')
        source = value.get('source')
        observed = value.get('observed_at_s')
        details = value.get('details')
        if (
            not isinstance(reason, str) or not reason
            or not isinstance(source, str) or not source
            or isinstance(observed, bool)
            or not isinstance(observed, (int, float))
            or not math.isfinite(observed) or observed < 0.0
            or not isinstance(details, dict)
        ):
            continue
        failures.append({
            'code': code.value,
            'source': source,
            'reason': reason,
            'observed_at_s': observed,
            'details': details,
        })
    return failures


def _pass_gate_failures(contract, evidence, runner, monitor, mission, metrics):
    failures = []
    if monitor.get('code') != Code.PASS.value:
        failures.append('monitor code is not PASS')
    if not runner.get('enable_motion'):
        failures.append('motion was not explicitly enabled')
    if not evidence.get('bag_complete') or evidence.get('missing_topics'):
        failures.append('rosbag metadata is incomplete')
    required_mission = (
        mission.get('ready'),
        mission.get('motion_started'),
        mission.get('plan_stable'),
        mission.get('cp2_validated'),
    )
    if not all(value is True for value in required_mission):
        failures.append('mission readiness, plan, motion, or CP2 is invalid')
    if mission.get('first_nonzero_cmd_time_s') is None:
        failures.append('first non-zero output command is missing')
    follower = mission.get('follower', {})
    if (
        follower.get('state') != 'COMPLETE'
        or follower.get('reason') != 'parking_complete'
        or follower.get('fault_seen') is not False
    ):
        failures.append('follower terminal state is not valid')
    arbiter = mission.get('arbiter', {})
    if (
        arbiter.get('armed') is not True
        or arbiter.get('latched') is not False
        or arbiter.get('cmd_vel_publisher_count') != 1
    ):
        failures.append('arbiter state or publisher count is not valid')
    watchdog = mission.get('watchdog', {})
    if (
        watchdog.get('emergency_active') is not False
        or watchdog.get('publishing_cmd_vel') is not False
    ):
        failures.append('watchdog state is not normal')
    safety = mission.get('safety', {})
    if (
        safety.get('valid') is not True
        or safety.get('emergency_stop') is not False
        or safety.get('fault_seen') is not False
    ):
        failures.append('safety status is not normal')
    collision = mission.get('collision', {})
    if (
        collision.get('valid') is not True
        or collision.get('all_sensors_fresh') is not True
        or collision.get('event_seen') is not False
    ):
        failures.append('collision heartbeat or event state is invalid')
    if not _complete_finite_metrics(metrics):
        failures.append('one or more required metrics are null or non-finite')
    else:
        if (
            type(metrics['collision_count']) is not int
            or metrics['collision_count'] != 0
        ):
            failures.append('collision count is not zero')
        if metrics['frame_alignment']['validated'] is not True:
            failures.append('frame alignment was not validated')
        if metrics['cp2']['validated'] is not True:
            failures.append('CP2 was not validated')
        if metrics['stationary']['validated'] is not True:
            failures.append('completion hold was not stationary')
        if (
            metrics['post_complete_hold_s']
            < contract['thresholds']['post_complete_hold_s']
        ):
            failures.append('completion hold duration is too short')
    return failures


def _complete_finite_metrics(value):
    numeric_paths = (
        ('readiness_duration_s',),
        ('mission_duration_s',),
        ('post_complete_hold_s',),
        ('selected_y_m',),
        ('minimum_planned_clearance_m',),
        ('frame_alignment', 'odom_to_world_x_m'),
        ('frame_alignment', 'odom_to_world_y_m'),
        ('frame_alignment', 'odom_to_world_yaw_rad'),
        ('frame_alignment', 'position_residual_m'),
        ('frame_alignment', 'yaw_residual_rad'),
        ('frame_alignment', 'sample_time_delta_s'),
        ('odom_ground_truth_position_delta_m',),
        ('odom_ground_truth_yaw_delta_rad',),
        ('cp2', 'crossing_time_s'),
        ('cp2', 'longitudinal_error_m'),
        ('cp2', 'cross_track_error_m'),
        ('cp2', 'heading_error_rad'),
        ('cp2', 'angular_speed_radps'),
        ('stationary', 'maximum_linear_speed_mps'),
        ('stationary', 'maximum_angular_speed_radps'),
        ('stationary', 'maximum_desired_command'),
        ('stationary', 'maximum_output_command'),
        ('stationary', 'position_drift_m'),
        ('stationary', 'yaw_drift_rad'),
        ('collision_count',),
    )
    for pose_name in ('final_odom', 'final_ground_truth'):
        numeric_paths += tuple(
            (pose_name, field) for field in (
                'x_m', 'y_m', 'yaw_rad', 'position_error_m',
                'yaw_error_rad',
            )
        )
        numeric_paths += tuple(
            (pose_name, 'margins_m', field) for field in (
                'front', 'rear', 'left', 'right', 'minimum',
            )
        )
    for path in numeric_paths:
        item = _nested_value(value, path)
        if (
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(item)
        ):
            return False
    return True


def _nested_value(value, path):
    current = value
    for name in path:
        if not isinstance(current, dict) or name not in current:
            return None
        current = current[name]
    return current


def _empty_metrics(readiness_duration, collision_count=None):
    return {
        'readiness_duration_s': readiness_duration,
        'mission_duration_s': None,
        'post_complete_hold_s': 0.0,
        'selected_y_m': None,
        'minimum_planned_clearance_m': None,
        'frame_alignment': {
            'validated': False,
            'odom_to_world_x_m': None,
            'odom_to_world_y_m': None,
            'odom_to_world_yaw_rad': None,
            'position_residual_m': None,
            'yaw_residual_rad': None,
            'sample_time_delta_s': None,
        },
        'final_odom': None,
        'final_ground_truth': None,
        'odom_ground_truth_position_delta_m': None,
        'odom_ground_truth_yaw_delta_rad': None,
        'cp2': {
            'validated': False,
            'crossing_time_s': None,
            'longitudinal_error_m': None,
            'cross_track_error_m': None,
            'heading_error_rad': None,
            'angular_speed_radps': None,
        },
        'stationary': {
            'validated': False,
            'maximum_linear_speed_mps': None,
            'maximum_angular_speed_radps': None,
            'maximum_desired_command': None,
            'maximum_output_command': None,
            'position_drift_m': None,
            'yaw_drift_rad': None,
        },
        'collision_count': collision_count,
    }


def render_markdown(result):
    """Render a compact human-readable report without overstating success."""
    outcome = result['outcome']
    run = result['run']
    return (
        '# P0 Acceptance Report\n\n'
        f'- Run: `{run["run_id"]}`\n'
        f'- Verdict: **{outcome["verdict"]}**\n'
        f'- Code: `{outcome["primary_code"]}`\n'
        f'- Motion explicitly enabled: `{str(run["enable_motion"]).lower()}`\n'
        f'- Summary: {outcome["summary"]}\n\n'
        'No PASS is claimed unless complete phase-5 evidence is evaluated.\n'
    )


def render_junit(result):
    """Render one JUnit testcase, using error for infrastructure outcomes."""
    outcome = result['outcome']
    suite = ElementTree.Element(
        'testsuite',
        name='robot_acceptance.p0',
        tests='1',
        failures='1' if outcome['verdict'] == 'FAIL' else '0',
        errors='1' if outcome['verdict'] == 'ERROR' else '0',
    )
    case = ElementTree.SubElement(
        suite,
        'testcase',
        classname='robot_acceptance',
        name='p0_parking',
    )
    if outcome['verdict'] == 'PASS':
        return ElementTree.tostring(suite, encoding='unicode') + '\n'
    element_name = 'error' if outcome['verdict'] == 'ERROR' else 'failure'
    detail = ElementTree.SubElement(
        case,
        element_name,
        type=outcome['primary_code'],
        message=outcome['summary'],
    )
    detail.text = outcome['summary']
    return ElementTree.tostring(suite, encoding='unicode') + '\n'


def write_reports(artifacts, result):
    """Atomically create result.json, report.md, and junit.xml."""
    artifacts.write_json('result.json', result)
    artifacts.write_text('report.md', render_markdown(result))
    artifacts.write_text('junit.xml', render_junit(result))


def main(argv=None):
    """Create missing Markdown and JUnit views for an existing result.json."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifact_directory', type=Path)
    arguments = parser.parse_args(argv)
    result_path = arguments.artifact_directory / 'result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    outputs = {
        'report.md': render_markdown(result),
        'junit.xml': render_junit(result),
    }
    for name, value in outputs.items():
        path = arguments.artifact_directory / name
        temporary = path.with_name(f'.{name}.tmp.{os.getpid()}')
        try:
            with temporary.open('x', encoding='utf-8') as stream:
                stream.write(value)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
    return 0
