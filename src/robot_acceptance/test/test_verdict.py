"""Tests for final snapshot verdict classification and priority."""

from dataclasses import replace

import pytest

from robot_acceptance.geometry import Margins
from robot_acceptance.verdict import Classification
from robot_acceptance.verdict import Code
from robot_acceptance.verdict import evaluate_snapshot
from robot_acceptance.verdict import EXIT_CODES
from robot_acceptance.verdict import Snapshot


def passing_snapshot():
    """Return a snapshot with every PASS condition and evidence present."""
    margins = Margins(0.0435, 0.0435, 0.016, 0.016, 0.016)
    return Snapshot(
        follower_state='COMPLETE',
        follower_reason='parking_complete',
        collision_count=0,
        plan_stable=True,
        cp2_validated=True,
        odom_position_error=0.015,
        ground_truth_position_error=0.015,
        odom_yaw_error=0.050,
        ground_truth_yaw_error=0.050,
        odom_margins=margins,
        ground_truth_margins=margins,
        stationary_validated=True,
        post_complete_motion=False,
        telemetry_valid=True,
        frame_alignment_valid=True,
        collision_evidence_complete=True,
        evidence_complete=True,
    )


def assert_primary(snapshot, code, classification=Classification.FAIL):
    """Assert one expected primary result and its frozen exit code."""
    outcome = evaluate_snapshot(snapshot)
    assert outcome.primary_code == code
    assert outcome.verdict == classification
    assert outcome.exit_code == EXIT_CODES[code]


def test_pass_requires_all_conditions_at_inclusive_geometry_limits():
    """Accept exact limits only when every evidence field is complete."""
    outcome = evaluate_snapshot(passing_snapshot())
    assert outcome.verdict == Classification.PASS
    assert outcome.primary_code == Code.PASS
    assert outcome.exit_code == 0
    assert outcome.failures == ()


@pytest.mark.parametrize(
    'changes,code',
    [
        ({'follower_fault_seen': True}, Code.CONTROLLER_FAULT),
        ({'arbiter_latched_seen': True}, Code.SAFETY_CHAIN_FAULT),
        ({'watchdog_emergency_seen': True}, Code.SAFETY_CHAIN_FAULT),
        ({'collision_count': 1}, Code.COLLISION_DETECTED),
        ({'plan_stable': False}, Code.PLAN_CHANGED),
        ({'cp2_validated': False}, Code.CP2_VALIDATION_FAILED),
        ({'follower_reason': 'cp1_reached'}, Code.WRONG_TERMINAL_STATE),
        ({'odom_position_error': 0.015001}, Code.FINAL_POSITION_FAILED),
        ({'ground_truth_yaw_error': 0.050001}, Code.FINAL_YAW_FAILED),
        ({'stationary_validated': False}, Code.NOT_STATIONARY),
        ({'post_complete_motion': True}, Code.POST_COMPLETE_MOTION),
    ],
)
def test_observed_task_violations_are_fail(changes, code):
    """Classify observed DUT violations as FAIL with their exact codes."""
    assert_primary(replace(passing_snapshot(), **changes), code)


def test_crossing_any_parking_edge_fails_the_envelope():
    """Use the strict 1e-9 envelope epsilon in final verdicts."""
    outside = Margins(0.01, 0.01, -1.1e-9, 0.01, -1.1e-9)
    assert_primary(
        replace(passing_snapshot(), ground_truth_margins=outside),
        Code.PARKING_ENVELOPE_FAILED,
    )


@pytest.mark.parametrize(
    'changes,code',
    [
        ({'telemetry_valid': False}, Code.TELEMETRY_INVALID),
        ({'frame_alignment_valid': False}, Code.FRAME_ALIGNMENT_INVALID),
        ({'collision_evidence_complete': False},
         Code.COLLISION_EVIDENCE_MISSING),
        ({'evidence_complete': False}, Code.EVIDENCE_INCOMPLETE),
        ({'odom_position_error': None}, Code.EVIDENCE_INCOMPLETE),
    ],
)
def test_unreliable_evidence_is_error_not_fail(changes, code):
    """Classify environment, telemetry, and evidence gaps as ERROR."""
    assert_primary(
        replace(passing_snapshot(), **changes),
        code,
        Classification.ERROR,
    )


def test_all_failures_are_retained_with_stable_specification_priority():
    """Keep secondary failures while collision remains the primary cause."""
    outcome = evaluate_snapshot(replace(
        passing_snapshot(),
        collision_count=2,
        arbiter_latched_seen=True,
        plan_stable=False,
        follower_fault_seen=True,
        cp2_validated=False,
        odom_position_error=1.0,
        stationary_validated=False,
        telemetry_valid=False,
    ))
    codes = {failure.code for failure in outcome.failures}
    assert outcome.primary_code == Code.COLLISION_DETECTED
    assert codes >= {
        Code.COLLISION_DETECTED,
        Code.SAFETY_CHAIN_FAULT,
        Code.PLAN_CHANGED,
        Code.CONTROLLER_FAULT,
        Code.CP2_VALIDATION_FAILED,
        Code.FINAL_POSITION_FAILED,
        Code.NOT_STATIONARY,
        Code.TELEMETRY_INVALID,
    }


def test_secondary_safety_latch_does_not_replace_first_root_cause():
    """Retain a secondary latch but select the earlier controller root cause."""
    outcome = evaluate_snapshot(replace(
        passing_snapshot(),
        follower_fault_seen=True,
        arbiter_latched_seen=True,
        safety_fault_is_secondary=True,
    ))
    assert outcome.primary_code == Code.CONTROLLER_FAULT
    assert {failure.code for failure in outcome.failures} >= {
        Code.CONTROLLER_FAULT,
        Code.SAFETY_CHAIN_FAULT,
    }


def test_missing_terminal_is_evidence_error_not_wrong_terminal_fail():
    """Do not claim a DUT terminal violation when telemetry never observed it."""
    assert_primary(
        replace(passing_snapshot(), follower_state=None,
                follower_reason=None),
        Code.EVIDENCE_INCOMPLETE,
        Classification.ERROR,
    )


@pytest.mark.parametrize(
    'changes',
    [
        {'odom_position_error': float('nan')},
        {'ground_truth_yaw_error': float('inf')},
        {'collision_count': -1},
        {'plan_stable': 1},
    ],
)
def test_invalid_snapshot_values_cannot_bypass_thresholds(changes):
    """Classify malformed final measurements as telemetry ERROR."""
    assert_primary(
        replace(passing_snapshot(), **changes),
        Code.TELEMETRY_INVALID,
        Classification.ERROR,
    )
