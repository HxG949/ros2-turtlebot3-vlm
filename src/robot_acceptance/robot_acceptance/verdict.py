"""Pure final-snapshot verdict classification for P0 acceptance."""

from dataclasses import dataclass
from enum import Enum
import math

from robot_acceptance.geometry import Margins


POSITION_TOLERANCE_M = 0.015
YAW_TOLERANCE_RAD = 0.050


class Classification(str, Enum):
    """Define the three externally visible verdict classes."""

    PASS = 'PASS'
    FAIL = 'FAIL'
    ERROR = 'ERROR'


class Code(str, Enum):
    """Define all frozen P0 result codes."""

    PASS = 'PASS'
    CONFIG_INVALID = 'CONFIG_INVALID'
    PREFLIGHT_FAILED = 'PREFLIGHT_FAILED'
    LAUNCH_FAILED = 'LAUNCH_FAILED'
    PROCESS_EXITED = 'PROCESS_EXITED'
    READINESS_TIMEOUT = 'READINESS_TIMEOUT'
    TELEMETRY_INVALID = 'TELEMETRY_INVALID'
    EVIDENCE_INCOMPLETE = 'EVIDENCE_INCOMPLETE'
    REPORT_FAILED = 'REPORT_FAILED'
    UNEXPECTED_SHUTDOWN = 'UNEXPECTED_SHUTDOWN'
    FRAME_ALIGNMENT_INVALID = 'FRAME_ALIGNMENT_INVALID'
    MISSION_TIMEOUT = 'MISSION_TIMEOUT'
    CONTROLLER_FAULT = 'CONTROLLER_FAULT'
    SAFETY_CHAIN_FAULT = 'SAFETY_CHAIN_FAULT'
    COLLISION_DETECTED = 'COLLISION_DETECTED'
    WRONG_TERMINAL_STATE = 'WRONG_TERMINAL_STATE'
    FINAL_POSITION_FAILED = 'FINAL_POSITION_FAILED'
    FINAL_YAW_FAILED = 'FINAL_YAW_FAILED'
    PARKING_ENVELOPE_FAILED = 'PARKING_ENVELOPE_FAILED'
    NOT_STATIONARY = 'NOT_STATIONARY'
    POST_COMPLETE_MOTION = 'POST_COMPLETE_MOTION'
    CP2_VALIDATION_FAILED = 'CP2_VALIDATION_FAILED'
    PLAN_CHANGED = 'PLAN_CHANGED'
    COLLISION_EVIDENCE_MISSING = 'COLLISION_EVIDENCE_MISSING'


EXIT_CODES = {
    code: exit_code
    for exit_code, code in (
        (0, Code.PASS), (10, Code.CONFIG_INVALID),
        (11, Code.PREFLIGHT_FAILED), (12, Code.LAUNCH_FAILED),
        (13, Code.PROCESS_EXITED), (14, Code.READINESS_TIMEOUT),
        (15, Code.TELEMETRY_INVALID), (16, Code.EVIDENCE_INCOMPLETE),
        (17, Code.REPORT_FAILED), (18, Code.UNEXPECTED_SHUTDOWN),
        (19, Code.FRAME_ALIGNMENT_INVALID), (20, Code.MISSION_TIMEOUT),
        (21, Code.CONTROLLER_FAULT), (22, Code.SAFETY_CHAIN_FAULT),
        (23, Code.COLLISION_DETECTED), (24, Code.WRONG_TERMINAL_STATE),
        (25, Code.FINAL_POSITION_FAILED), (26, Code.FINAL_YAW_FAILED),
        (27, Code.PARKING_ENVELOPE_FAILED), (28, Code.NOT_STATIONARY),
        (29, Code.POST_COMPLETE_MOTION), (30, Code.CP2_VALIDATION_FAILED),
        (31, Code.PLAN_CHANGED), (32, Code.COLLISION_EVIDENCE_MISSING),
    )
}

ERROR_CODES = {
    Code.CONFIG_INVALID, Code.PREFLIGHT_FAILED, Code.LAUNCH_FAILED,
    Code.PROCESS_EXITED, Code.READINESS_TIMEOUT, Code.TELEMETRY_INVALID,
    Code.EVIDENCE_INCOMPLETE, Code.REPORT_FAILED,
    Code.UNEXPECTED_SHUTDOWN, Code.FRAME_ALIGNMENT_INVALID,
    Code.COLLISION_EVIDENCE_MISSING,
}

PRIMARY_PRIORITY = {
    Code.COLLISION_DETECTED: 0,
    Code.SAFETY_CHAIN_FAULT: 10,
    Code.PLAN_CHANGED: 20,
    Code.CONTROLLER_FAULT: 30,
    Code.WRONG_TERMINAL_STATE: 31,
    Code.CP2_VALIDATION_FAILED: 40,
    Code.MISSION_TIMEOUT: 50,
    Code.FINAL_POSITION_FAILED: 60,
    Code.FINAL_YAW_FAILED: 61,
    Code.PARKING_ENVELOPE_FAILED: 62,
    Code.NOT_STATIONARY: 70,
    Code.POST_COMPLETE_MOTION: 71,
    Code.FRAME_ALIGNMENT_INVALID: 80,
    Code.COLLISION_EVIDENCE_MISSING: 81,
    Code.TELEMETRY_INVALID: 82,
    Code.EVIDENCE_INCOMPLETE: 83,
}


@dataclass(frozen=True)
class Failure:
    """Describe one independently observed acceptance failure."""

    code: Code
    detail: str

    @property
    def classification(self):
        """Return the specification classification for this failure."""
        return Classification.ERROR if self.code in ERROR_CODES else Classification.FAIL


@dataclass(frozen=True)
class Snapshot:
    """Contain all final observations required for a reliable verdict."""

    follower_state: str | None = None
    follower_reason: str | None = None
    follower_fault_seen: bool = False
    arbiter_latched_seen: bool = False
    watchdog_emergency_seen: bool = False
    safety_fault_seen: bool = False
    safety_fault_is_secondary: bool = False
    collision_count: int | None = None
    plan_stable: bool | None = None
    cp2_validated: bool | None = None
    mission_timed_out: bool = False
    odom_position_error: float | None = None
    ground_truth_position_error: float | None = None
    odom_yaw_error: float | None = None
    ground_truth_yaw_error: float | None = None
    odom_margins: Margins | None = None
    ground_truth_margins: Margins | None = None
    stationary_validated: bool | None = None
    post_complete_motion: bool | None = None
    telemetry_valid: bool = True
    frame_alignment_valid: bool | None = None
    collision_evidence_complete: bool = False
    evidence_complete: bool = False


@dataclass(frozen=True)
class Outcome:
    """Contain the final verdict, stable primary code, and every failure."""

    verdict: Classification
    primary_code: Code
    exit_code: int
    failures: tuple[Failure, ...]


def _add(failures, code, detail):
    failures.append(Failure(code=code, detail=detail))


def _primary(failures, safety_fault_is_secondary=False):
    candidates = failures
    if safety_fault_is_secondary and len(failures) > 1:
        without_secondary = [
            failure
            for failure in failures
            if failure.code != Code.SAFETY_CHAIN_FAULT
        ]
        if without_secondary:
            candidates = without_secondary
    return min(
        candidates,
        key=lambda failure: (
            PRIMARY_PRIORITY.get(failure.code, 90),
            EXIT_CODES[failure.code],
            failure.detail,
        ),
    )


def _invalid_snapshot_fields(snapshot):
    invalid = []
    measurements = (
        'odom_position_error', 'ground_truth_position_error',
        'odom_yaw_error', 'ground_truth_yaw_error',
    )
    for name in measurements:
        value = getattr(snapshot, name)
        if value is not None and (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0.0
        ):
            invalid.append(name)
    count = snapshot.collision_count
    if count is not None and (
        isinstance(count, bool) or not isinstance(count, int) or count < 0
    ):
        invalid.append('collision_count')
    optional_booleans = (
        'plan_stable', 'cp2_validated', 'stationary_validated',
        'post_complete_motion', 'frame_alignment_valid',
    )
    for name in optional_booleans:
        value = getattr(snapshot, name)
        if value is not None and type(value) is not bool:
            invalid.append(name)
    booleans = (
        'follower_fault_seen', 'arbiter_latched_seen',
        'watchdog_emergency_seen', 'safety_fault_seen',
        'safety_fault_is_secondary', 'mission_timed_out',
        'telemetry_valid', 'collision_evidence_complete',
        'evidence_complete',
    )
    for name in booleans:
        if type(getattr(snapshot, name)) is not bool:
            invalid.append(name)
    for name in ('follower_state', 'follower_reason'):
        value = getattr(snapshot, name)
        if value is not None and (
            not isinstance(value, str) or not value.strip()
        ):
            invalid.append(name)
    for name in ('odom_margins', 'ground_truth_margins'):
        margin = getattr(snapshot, name)
        if margin is None:
            continue
        if not isinstance(margin, Margins) or not all(
            math.isfinite(value) for value in (
                margin.front, margin.rear, margin.left,
                margin.right, margin.minimum,
            )
        ):
            invalid.append(name)
    return tuple(invalid)


def evaluate_snapshot(snapshot):
    """Evaluate a final snapshot without performing ROS or filesystem I/O."""
    failures = []
    invalid_fields = set(_invalid_snapshot_fields(snapshot))
    if (
        'collision_count' not in invalid_fields
        and snapshot.collision_count is not None
        and snapshot.collision_count > 0
    ):
        _add(failures, Code.COLLISION_DETECTED,
             f'{snapshot.collision_count} unexpected collision(s) observed')
    if (
        snapshot.arbiter_latched_seen is True
        or snapshot.watchdog_emergency_seen is True
        or snapshot.safety_fault_seen is True
    ):
        _add(failures, Code.SAFETY_CHAIN_FAULT,
             'safety fault, arbiter latch, or watchdog takeover observed')
    if snapshot.plan_stable is False:
        _add(failures, Code.PLAN_CHANGED, 'plan changed after motion started')
    if (
        snapshot.follower_fault_seen is True
        or snapshot.follower_state == 'FAULT'
    ):
        _add(failures, Code.CONTROLLER_FAULT, 'follower fault observed')
    if (
        'follower_state' not in invalid_fields
        and 'follower_reason' not in invalid_fields
        and snapshot.follower_state is not None
        and snapshot.follower_reason is not None
        and (
            snapshot.follower_state != 'COMPLETE'
            or snapshot.follower_reason != 'parking_complete'
        )
    ):
        _add(failures, Code.WRONG_TERMINAL_STATE,
             'terminal state is not COMPLETE / parking_complete')
    if snapshot.cp2_validated is False:
        _add(failures, Code.CP2_VALIDATION_FAILED,
             'directed CP2 crossing was not validated')
    if snapshot.mission_timed_out is True:
        _add(failures, Code.MISSION_TIMEOUT, 'mission deadline exceeded')

    position_errors = (
        ('odom_position_error', snapshot.odom_position_error),
        ('ground_truth_position_error',
         snapshot.ground_truth_position_error),
    )
    if any(
        name not in invalid_fields
        and error is not None
        and error > POSITION_TOLERANCE_M
        for name, error in position_errors
    ):
        _add(failures, Code.FINAL_POSITION_FAILED,
             'odom or ground-truth center error exceeds 0.015 m')
    yaw_errors = (
        ('odom_yaw_error', snapshot.odom_yaw_error),
        ('ground_truth_yaw_error', snapshot.ground_truth_yaw_error),
    )
    if any(
        name not in invalid_fields
        and error is not None
        and error > YAW_TOLERANCE_RAD
        for name, error in yaw_errors
    ):
        _add(failures, Code.FINAL_YAW_FAILED,
             'odom or ground-truth yaw error exceeds 0.050 rad')
    margins = (
        ('odom_margins', snapshot.odom_margins),
        ('ground_truth_margins', snapshot.ground_truth_margins),
    )
    if any(
        name not in invalid_fields
        and margin is not None
        and not margin.contained()
        for name, margin in margins
    ):
        _add(failures, Code.PARKING_ENVELOPE_FAILED,
             'odom or ground-truth footprint crosses a parking edge')
    if snapshot.stationary_validated is False:
        _add(failures, Code.NOT_STATIONARY,
             'robot was not stationary during the completion hold')
    if snapshot.post_complete_motion is True:
        _add(failures, Code.POST_COMPLETE_MOTION,
             'command or ground-truth motion occurred after completion')

    if snapshot.frame_alignment_valid is False:
        _add(failures, Code.FRAME_ALIGNMENT_INVALID,
             'odom-to-world alignment was not validated')
    if not snapshot.collision_evidence_complete:
        _add(failures, Code.COLLISION_EVIDENCE_MISSING,
             'collision sensor heartbeat evidence is incomplete')
    if not snapshot.telemetry_valid or invalid_fields:
        detail = 'one or more telemetry contracts were invalid'
        if invalid_fields:
            detail += ': ' + ', '.join(sorted(invalid_fields))
        _add(failures, Code.TELEMETRY_INVALID,
             detail)

    required_values = (
        snapshot.follower_state,
        snapshot.follower_reason,
        snapshot.collision_count,
        snapshot.plan_stable,
        snapshot.cp2_validated,
        snapshot.odom_position_error,
        snapshot.ground_truth_position_error,
        snapshot.odom_yaw_error,
        snapshot.ground_truth_yaw_error,
        snapshot.odom_margins,
        snapshot.ground_truth_margins,
        snapshot.stationary_validated,
        snapshot.post_complete_motion,
        snapshot.frame_alignment_valid,
    )
    if not snapshot.evidence_complete or any(
        value is None for value in required_values
    ):
        _add(failures, Code.EVIDENCE_INCOMPLETE,
             'required final observations or artifacts are missing')

    if not failures:
        return Outcome(
            verdict=Classification.PASS,
            primary_code=Code.PASS,
            exit_code=EXIT_CODES[Code.PASS],
            failures=(),
        )
    primary = _primary(failures, snapshot.safety_fault_is_secondary)
    return Outcome(
        verdict=primary.classification,
        primary_code=primary.code,
        exit_code=EXIT_CODES[primary.code],
        failures=tuple(failures),
    )
