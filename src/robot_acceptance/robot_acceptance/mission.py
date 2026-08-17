"""Pure online mission state and evidence evaluation for P0 acceptance."""

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math

from robot_acceptance.geometry import check_cp2_crossing
from robot_acceptance.geometry import normalize_angle
from robot_acceptance.geometry import parking_margins
from robot_acceptance.geometry import ParkingSpace
from robot_acceptance.geometry import Pose2D
from robot_acceptance.geometry import pose_errors
from robot_acceptance.verdict import Classification
from robot_acceptance.verdict import Code
from robot_acceptance.verdict import ERROR_CODES
from robot_acceptance.verdict import evaluate_snapshot
from robot_acceptance.verdict import Snapshot


TOPIC_SELECTED_TARGET = '/parking/selected_target'
TOPIC_PLAN = '/navigation/plan'
TOPIC_CONTROL_STATUS = '/navigation/control_status'
TOPIC_SAFETY_STATUS = '/safety/status'
TOPIC_ARBITER_STATUS = '/navigation/safety_arbiter_status'
TOPIC_WATCHDOG_STATUS = '/navigation/cmd_vel_watchdog_status'
TOPIC_SCAN = '/scan'
TOPIC_ODOM = '/odom'
TOPIC_DESIRED_CMD = '/navigation/desired_cmd_vel'
TOPIC_CMD = '/cmd_vel'
TOPIC_MODEL_STATES = '/gazebo/model_states'
TOPIC_COLLISION_STATUS = '/acceptance/collision_status'
TOPIC_COLLISION_EVENTS = '/acceptance/collision_events'

REQUIRED_TOPICS = (
    TOPIC_SELECTED_TARGET,
    TOPIC_PLAN,
    TOPIC_CONTROL_STATUS,
    TOPIC_SAFETY_STATUS,
    TOPIC_ARBITER_STATUS,
    TOPIC_WATCHDOG_STATUS,
    TOPIC_SCAN,
    TOPIC_ODOM,
    TOPIC_DESIRED_CMD,
    TOPIC_CMD,
    TOPIC_MODEL_STATES,
    TOPIC_COLLISION_STATUS,
)


@dataclass(frozen=True)
class MotionSample:
    """Represent a planar pose and measured planar velocity."""

    pose: Pose2D
    linear_speed: float
    angular_speed: float


def canonical_plan_sha256(plan):
    """Hash the executable route while excluding live planner diagnostics."""
    route = {
        'valid': plan.valid,
        'selected_y': plan.selected_y,
        'parking_space_id': plan.parking_space_id,
        'waypoints': [asdict(waypoint) for waypoint in plan.waypoints],
    }
    payload = json.dumps(
        route,
        allow_nan=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('ascii')
    return hashlib.sha256(payload).hexdigest()


def _utc_timestamp():
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _command_magnitude(linear_x, angular_z):
    return max(abs(float(linear_x)), abs(float(angular_z)))


def _margins_dict(value):
    return {
        'front': value.front,
        'rear': value.rear,
        'left': value.left,
        'right': value.right,
        'minimum': value.minimum,
    }


class MissionState:
    """Consume telemetry in receive order and produce one terminal result."""

    def __init__(self, config, enable_motion, robot_entity_name,
                 started_monotonic=0.0, freshness_timeout_s=1.0):
        self.config = config
        self.thresholds = config['thresholds']
        self.enable_motion = bool(enable_motion)
        self.robot_entity_name = str(robot_entity_name)
        self.started = float(started_monotonic)
        self.freshness_timeout = float(freshness_timeout_s)
        self.phase = 'PREPARING'
        self.samples = {}
        self.contract_errors = []
        self.events = []
        self.result = None
        self.target = None
        self.plan = None
        self.plan_sha256 = None
        self.frozen_plan = None
        self.ready_time = None
        self.motion_start_time = None
        self.first_nonzero_cmd_time = None
        self.previous_odom = None
        self.latest_odom = None
        self.latest_world = None
        self.frame_alignment = self._empty_frame_alignment()
        self.frame_problem = None
        self.cp2_result = None
        self.cp2_time = None
        self.closest_cp2_result = None
        self.closest_cp2_distance = math.inf
        self.control = None
        self.safety = None
        self.arbiter = None
        self.watchdog = None
        self.collision_status = None
        self.collision_count = None
        self.collision_event_seen = False
        self.follower_fault_seen = False
        self.arbiter_latched_seen = False
        self.watchdog_emergency_seen = False
        self.safety_fault_seen = False
        self.plan_stable = True
        self.hold_started = None
        self.hold_seen = set()
        self.hold_initial_world = None
        self.hold_max_linear = None
        self.hold_max_angular = None
        self.hold_max_desired = None
        self.hold_max_output = None
        self.hold_position_drift = None
        self.hold_yaw_drift = None
        self.hold_odom_margins = None
        self.hold_world_margins = None
        self.post_complete_motion = False
        self._transition('PREPARING', self.started, 'monitor_started')

    @property
    def finished(self):
        """Return whether a terminal structured result is available."""
        return self.result is not None

    def _transition(self, phase, now, reason):
        if self.phase == phase and self.events:
            return
        previous = self.phase
        self.phase = phase
        self.events.append({
            'event': 'state_transition',
            'monotonic_s': max(0.0, float(now) - self.started),
            'from': previous,
            'to': phase,
            'reason': reason,
        })

    def _mark_sample(self, topic, now):
        self.samples[topic] = float(now)
        if self.hold_started is not None and now >= self.hold_started:
            self.hold_seen.add(topic)

    def observe_contract_error(self, topic, error, now):
        """Make any malformed telemetry permanently disqualifying."""
        detail = {'topic': topic, 'error': str(error)}
        self.contract_errors.append(detail)
        self.events.append({
            'event': 'contract_error',
            'monotonic_s': max(0.0, now - self.started),
            **detail,
        })
        self._finish(
            Code.TELEMETRY_INVALID,
            f'{topic} telemetry contract is invalid',
            now,
            detail,
        )

    def observe_presence(self, topic, now):
        """Record a validated topic whose payload needs no mission fields."""
        self._mark_sample(topic, now)

    def observe_target(self, value, now):
        """Track and validate the selected parking target."""
        self._mark_sample(TOPIC_SELECTED_TARGET, now)
        self.target = value
        if not self._target_matches(value) and self.enable_motion:
            self._finish(
                Code.TELEMETRY_INVALID,
                'selected parking target does not match the P0 contract',
                now,
                {'parking_space_id': value.target.space_id},
            )

    def observe_plan(self, value, now):
        """Track a candidate plan and enforce the frozen motion-time hash."""
        self._mark_sample(TOPIC_PLAN, now)
        self.plan = value
        digest = canonical_plan_sha256(value)
        if self.motion_start_time is not None:
            if not value.valid or digest != self.plan_sha256:
                self.plan_stable = False
                self._finish(
                    Code.PLAN_CHANGED,
                    'plan became invalid or changed after motion started',
                    now,
                    {
                        'frozen_sha256': self.plan_sha256,
                        'observed_sha256': digest,
                        'valid': value.valid,
                    },
                )

    def observe_control(self, value, now):
        """Track follower state, faults, and the unique valid terminal."""
        self._mark_sample(TOPIC_CONTROL_STATUS, now)
        self.control = value
        if self.ready_time is not None and (
            not value.enabled or value.stop_at_cp1
        ):
            self._finish(
                Code.CONTROLLER_FAULT,
                'follower enable or stop_at_cp1 invariant changed',
                now,
                {
                    'enabled': value.enabled,
                    'stop_at_cp1': value.stop_at_cp1,
                },
            )
            return
        if value.state == 'FAULT':
            self.follower_fault_seen = True
            self._finish(
                Code.CONTROLLER_FAULT,
                f'follower entered FAULT: {value.reason}',
                now,
                {'state': value.state, 'reason': value.reason},
            )
            return
        if self.hold_started is not None and (
            value.state != 'COMPLETE' or value.reason != 'parking_complete'
        ):
            self._finish(
                Code.WRONG_TERMINAL_STATE,
                'follower reversed the parking_complete terminal state',
                now,
                {'state': value.state, 'reason': value.reason},
            )
            return
        if value.state != 'COMPLETE':
            return
        if value.reason != 'parking_complete':
            self._finish(
                Code.WRONG_TERMINAL_STATE,
                'follower COMPLETE reason is not parking_complete',
                now,
                {'state': value.state, 'reason': value.reason},
            )
            return
        if not self.enable_motion or self.motion_start_time is None:
            if self.enable_motion:
                self._finish(
                    Code.WRONG_TERMINAL_STATE,
                    'follower completed before output motion was observed',
                    now,
                )
            return
        if self.cp2_result is None:
            details = {}
            if self.closest_cp2_result is not None:
                details = self._cp2_dict(self.closest_cp2_result)
            self._finish(
                Code.CP2_VALIDATION_FAILED,
                'no valid directed CP2 crossing was observed',
                now,
                details,
            )
            return
        if self.hold_started is None:
            self.hold_started = float(now)
            self.hold_initial_world = None
            self._transition(
                'POST_COMPLETE_HOLD', now, 'parking_complete_observed'
            )

    def observe_safety(self, value, now):
        """Track lidar safety validity and emergency state."""
        self._mark_sample(TOPIC_SAFETY_STATUS, now)
        self.safety = value
        fault = not value.valid or value.emergency_stop
        self.safety_fault_seen = self.safety_fault_seen or fault
        if fault and self.ready_time is not None:
            self._finish(
                Code.SAFETY_CHAIN_FAULT,
                'lidar safety status became invalid or emergency',
                now,
                {
                    'valid': value.valid,
                    'emergency_stop': value.emergency_stop,
                },
            )

    def observe_arbiter(self, value, now):
        """Track arbiter latching, arming, and output publisher count."""
        self._mark_sample(TOPIC_ARBITER_STATUS, now)
        self.arbiter = value
        self.arbiter_latched_seen = self.arbiter_latched_seen or value.latched
        if value.latched:
            self._finish(
                Code.SAFETY_CHAIN_FAULT,
                f'safety arbiter latched: {value.reason}',
                now,
                {'state': value.state, 'reason': value.reason},
            )
        elif self.ready_time is not None and (
            not value.enabled or not value.armed
            or value.cmd_vel_publisher_count != 1
        ):
            self._finish(
                Code.SAFETY_CHAIN_FAULT,
                'arbiter readiness or cmd_vel publisher invariant failed',
                now,
                {
                    'enabled': value.enabled,
                    'armed': value.armed,
                    'cmd_vel_publisher_count': value.cmd_vel_publisher_count,
                },
            )

    def observe_watchdog(self, value, now):
        """Track any watchdog emergency takeover."""
        self._mark_sample(TOPIC_WATCHDOG_STATUS, now)
        self.watchdog = value
        self.watchdog_emergency_seen = (
            self.watchdog_emergency_seen or value.emergency_active
        )
        if value.emergency_active:
            self._finish(
                Code.SAFETY_CHAIN_FAULT,
                f'command watchdog emergency: {value.reason}',
                now,
                {'reason': value.reason},
            )
        elif self.ready_time is not None and value.publishing_cmd_vel:
            self._finish(
                Code.SAFETY_CHAIN_FAULT,
                'watchdog published cmd_vel without a declared emergency',
                now,
                {'reason': value.reason},
            )

    def observe_collision_status(self, value, now):
        """Require valid fresh contact heartbeats and a monotonic zero count."""
        self._mark_sample(TOPIC_COLLISION_STATUS, now)
        required = ('valid', 'all_sensors_fresh', 'collision_count')
        if any(field not in value for field in required):
            self.observe_contract_error(
                TOPIC_COLLISION_STATUS,
                'missing valid, all_sensors_fresh, or collision_count',
                now,
            )
            return
        valid = value['valid']
        fresh = value['all_sensors_fresh']
        count = value['collision_count']
        if type(valid) is not bool or type(fresh) is not bool:
            self.observe_contract_error(
                TOPIC_COLLISION_STATUS,
                'valid and all_sensors_fresh must be booleans',
                now,
            )
            return
        previous_count = self.collision_count
        self.collision_status = value
        self.collision_count = count
        if previous_count is not None and count < previous_count:
            self._finish(
                Code.TELEMETRY_INVALID,
                'collision count decreased',
                now,
                {'previous': previous_count, 'observed': count},
            )
        elif count > 0:
            self._finish(
                Code.COLLISION_DETECTED,
                f'{count} unexpected collision(s) observed',
                now,
                {'collision_count': count},
            )
        elif self.ready_time is not None and (not valid or not fresh):
            self._finish(
                Code.COLLISION_EVIDENCE_MISSING,
                'collision sensor heartbeat became invalid or stale',
                now,
                {'valid': valid, 'all_sensors_fresh': fresh},
            )

    def observe_collision_event(self, value, now):
        """Treat every normalized event as collision evidence."""
        self.collision_event_seen = True
        self._mark_sample(TOPIC_COLLISION_EVENTS, now)
        self._finish(
            Code.COLLISION_DETECTED,
            'unexpected collision event observed',
            now,
            {'event': value},
        )

    def observe_odom(self, value, now):
        """Track odometry, CP2 crossing, hold speed, and final margins."""
        self._mark_sample(TOPIC_ODOM, now)
        self.previous_odom = self.latest_odom
        self.latest_odom = value
        self._update_frame_alignment(now)
        if (
            self.motion_start_time is not None
            and self.previous_odom is not None
            and self.frozen_plan is not None
            and self.cp2_result is None
        ):
            self._check_cp2(self.previous_odom, value, now)
        if self.hold_started is not None:
            self.hold_max_linear = self._maximum(
                self.hold_max_linear, value.linear_speed
            )
            self.hold_max_angular = self._maximum(
                self.hold_max_angular, abs(value.angular_speed)
            )
            self.hold_odom_margins = self._minimum_margins(
                self.hold_odom_margins,
                self._parking_margins(value.pose, world=False),
            )

    def observe_world(self, value, now, entity_found=True):
        """Track the configured Gazebo entity pose and completion drift."""
        self._mark_sample(TOPIC_MODEL_STATES, now)
        if not entity_found:
            self.frame_problem = (
                f'Gazebo entity not found: {self.robot_entity_name}'
            )
            if self.enable_motion:
                self._finish(
                    Code.FRAME_ALIGNMENT_INVALID,
                    self.frame_problem,
                    now,
                    {'robot_entity_name': self.robot_entity_name},
                )
            return
        self.latest_world = value
        self._update_frame_alignment(now)
        if self.hold_started is not None:
            if self.hold_initial_world is None:
                self.hold_initial_world = value.pose
            drift = pose_errors(value.pose, self.hold_initial_world)
            self.hold_position_drift = self._maximum(
                self.hold_position_drift, drift.position
            )
            self.hold_yaw_drift = self._maximum(
                self.hold_yaw_drift, drift.yaw
            )
            self.hold_world_margins = self._minimum_margins(
                self.hold_world_margins,
                self._parking_margins(value.pose, world=True),
            )

    def observe_command(self, topic, linear_x, angular_z, now):
        """Track desired/output commands and detect first output motion."""
        self._mark_sample(topic, now)
        magnitude = _command_magnitude(linear_x, angular_z)
        if self.hold_started is not None:
            if topic == TOPIC_DESIRED_CMD:
                self.hold_max_desired = self._maximum(
                    self.hold_max_desired, magnitude
                )
            elif topic == TOPIC_CMD:
                self.hold_max_output = self._maximum(
                    self.hold_max_output, magnitude
                )
            if magnitude > self.thresholds['command_zero_tolerance']:
                self.post_complete_motion = True
                self._finish(
                    Code.POST_COMPLETE_MOTION,
                    f'non-zero {topic} observed after parking_complete',
                    now,
                    {'topic': topic, 'magnitude': magnitude},
                )
            return
        if (
            topic == TOPIC_CMD
            and magnitude > self.thresholds['command_zero_tolerance']
            and self.motion_start_time is None
        ):
            if not self.enable_motion:
                self._finish(
                    Code.SAFETY_CHAIN_FAULT,
                    'output motion occurred while enable_motion is false',
                    now,
                    {'magnitude': magnitude},
                )
                return
            if self.ready_time is None:
                self._finish(
                    Code.READINESS_TIMEOUT,
                    'output motion occurred before monitor readiness',
                    now,
                    {'magnitude': magnitude},
                )
                return
            stale = self._stale_topics(now)
            if stale:
                self._finish(
                    Code.TELEMETRY_INVALID,
                    'output motion started with stale mission telemetry',
                    now,
                    {'stale_topics': list(stale)},
                )
                return
            if (
                self.plan is None or not self.plan.valid
                or canonical_plan_sha256(self.plan) != self.plan_sha256
            ):
                self.plan_stable = False
                self._finish(
                    Code.PLAN_CHANGED,
                    'ready plan changed before first output motion',
                    now,
                )
                return
            self.motion_start_time = float(now)
            self.first_nonzero_cmd_time = float(now)
            self._transition('RUNNING', now, 'first_nonzero_output_command')

    def tick(self, now):
        """Advance readiness, deadlines, and completion hold using monotonic time."""
        if self.finished:
            return
        now = float(now)
        if self.ready_time is None:
            self._try_ready(now)
        if self.finished:
            return
        total_limit = (
            self.thresholds['mission_timeout_s']
            + self.thresholds['monitor_grace_s']
        )
        if self.enable_motion and now - self.started >= total_limit:
            self._finish(
                Code.MISSION_TIMEOUT,
                'monitor mission deadline and grace period exceeded',
                now,
            )
            return
        if self.ready_time is None:
            if now - self.started >= self.thresholds['readiness_timeout_s']:
                if self.frame_problem is not None:
                    code = Code.FRAME_ALIGNMENT_INVALID
                    reason = self.frame_problem
                else:
                    code = Code.READINESS_TIMEOUT
                    reason = 'acceptance readiness conditions were not met'
                self._finish(code, reason, now, self._readiness_details(now))
            return
        if (
            self.enable_motion
            and self.motion_start_time is None
            and now - self.ready_time
            >= self.thresholds['motion_start_timeout_s']
        ):
            self._finish(
                Code.MISSION_TIMEOUT,
                'no non-zero output command was observed after readiness',
                now,
            )
            return
        if self.motion_start_time is not None:
            stale = self._stale_topics(now)
            if stale:
                self._finish(
                    Code.TELEMETRY_INVALID,
                    'required mission telemetry became stale',
                    now,
                    {'stale_topics': list(stale)},
                )
                return
        if self.hold_started is not None:
            duration = now - self.hold_started
            if duration >= self.thresholds['post_complete_hold_s']:
                self._finalize_hold(now, duration)

    def interrupt(self, now, reason='monitor stopped before a terminal result'):
        """Produce conservative evidence output when externally stopped."""
        if not self.finished:
            self._finish(Code.EVIDENCE_INCOMPLETE, reason, float(now))

    def _try_ready(self, now):
        if not self.enable_motion or self.contract_errors:
            return
        if self._stale_topics(now):
            return
        if not self._target_matches(self.target):
            return
        if self.plan is None or not self.plan.valid:
            return
        if not self._plan_matches_target(self.plan, self.target):
            return
        if not self.frame_alignment['validated']:
            return
        if self.control is None or not self.control.enabled:
            return
        if self.control.stop_at_cp1:
            return
        if self.safety is None or not self.safety.valid:
            return
        if self.safety.emergency_stop:
            return
        if self.arbiter is None or not self.arbiter.enabled:
            return
        if not self.arbiter.armed or self.arbiter.latched:
            return
        if self.arbiter.cmd_vel_publisher_count != 1:
            return
        if self.watchdog is None or self.watchdog.emergency_active:
            return
        if self.watchdog.publishing_cmd_vel:
            return
        collision = self.collision_status
        if collision is None or self.collision_count != 0:
            return
        if not collision['valid'] or not collision['all_sensors_fresh']:
            return
        self.frozen_plan = self.plan
        self.plan_sha256 = canonical_plan_sha256(self.plan)
        self.ready_time = float(now)
        self._transition('READY', now, 'all_readiness_conditions_met')

    def _target_matches(self, value):
        if value is None or not value.valid:
            return False
        expected = self.config['target']
        target = value.target
        values = (
            (value.frame_id, expected['frame_id']),
            (target.space_id, expected['parking_space_id']),
        )
        if any(actual != wanted for actual, wanted in values):
            return False
        numeric = (
            (target.center_x, expected['center_x_m']),
            (target.center_y, expected['center_y_m']),
            (target.entry_yaw, expected['entry_yaw_rad']),
            (target.length, expected['length_m']),
            (target.width, expected['width_m']),
            (target.approach_distance, expected['approach_distance_m']),
        )
        if any(abs(actual - wanted) > 1e-9 for actual, wanted in numeric):
            return False
        return abs(normalize_angle(
            target.final_yaw - expected['final_yaw_rad']
        )) <= 1e-9

    @staticmethod
    def _plan_matches_target(plan, target):
        if target is None or not plan.valid or not plan.waypoints:
            return False
        goal = plan.waypoints[-1]
        expected = target.target
        return (
            plan.parking_space_id == expected.space_id
            and goal.role == 'parking_goal'
            and goal.parking_space_id == expected.space_id
            and abs(goal.x - expected.center_x) <= 1e-9
            and abs(goal.y - expected.center_y) <= 1e-9
            and abs(normalize_angle(goal.final_yaw - expected.final_yaw)) <= 1e-9
            and abs(goal.parking_length - expected.length) <= 1e-9
            and abs(goal.parking_width - expected.width) <= 1e-9
        )

    def _update_frame_alignment(self, now):
        if self.ready_time is not None:
            return
        if self.latest_odom is None or self.latest_world is None:
            return
        odom_time = self.samples.get(TOPIC_ODOM)
        world_time = self.samples.get(TOPIC_MODEL_STATES)
        sync_error = abs(odom_time - world_time)
        maximum_sync = self.thresholds['frame_alignment_sync_tolerance_s']
        if sync_error > maximum_sync:
            self.frame_problem = (
                'odom and Gazebo model samples are not synchronized'
            )
            self.frame_alignment = {
                **self._empty_frame_alignment(),
                'sample_time_delta_s': sync_error,
            }
            return
        odom = self.latest_odom.pose
        world = self.latest_world.pose
        yaw = normalize_angle(world.yaw - odom.yaw)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        x_value = world.x - (cosine * odom.x - sine * odom.y)
        y_value = world.y - (sine * odom.x + cosine * odom.y)
        position_residual = math.hypot(x_value, y_value)
        yaw_residual = abs(yaw)
        valid = (
            position_residual
            <= self.thresholds['frame_alignment_position_tolerance_m']
            and yaw_residual
            <= self.thresholds['frame_alignment_yaw_tolerance_rad']
        )
        self.frame_alignment = {
            'validated': valid,
            'odom_to_world_x_m': x_value,
            'odom_to_world_y_m': y_value,
            'odom_to_world_yaw_rad': yaw,
            'position_residual_m': position_residual,
            'yaw_residual_rad': yaw_residual,
            'sample_time_delta_s': sync_error,
        }
        self.frame_problem = None if valid else (
            'odom-to-world transform is not identity within tolerance'
        )
        if not valid and self.enable_motion:
            self._finish(
                Code.FRAME_ALIGNMENT_INVALID,
                self.frame_problem,
                now,
                {'frame_alignment': self.frame_alignment},
            )

    def _check_cp2(self, previous, current, now):
        waypoints = self.frozen_plan.waypoints
        index = next(
            item for item, waypoint in enumerate(waypoints)
            if waypoint.role == 'cp2'
        )
        if index == 0 or index == len(waypoints) - 1:
            self._finish(
                Code.TELEMETRY_INVALID,
                'CP2 lacks surrounding frozen-plan waypoints',
                now,
            )
            return
        before = waypoints[index - 1]
        cp2 = waypoints[index]
        after = waypoints[index + 1]
        try:
            result = check_cp2_crossing(
                previous.pose,
                current.pose,
                Pose2D(before.x, before.y, 0.0),
                Pose2D(cp2.x, cp2.y, 0.0),
                Pose2D(after.x, after.y, 0.0),
                current.angular_speed,
            )
        except ValueError as error:
            self._finish(
                Code.TELEMETRY_INVALID,
                f'frozen CP2 geometry is invalid: {error}',
                now,
            )
            return
        distance = abs(result.longitudinal_error)
        if distance < self.closest_cp2_distance:
            self.closest_cp2_distance = distance
            self.closest_cp2_result = result
        if result.passed:
            self.cp2_result = result
            self.cp2_time = float(now)
            self.events.append({
                'event': 'cp2_validated',
                'monotonic_s': now - self.started,
                **self._cp2_dict(result),
            })

    def _finalize_hold(self, now, duration):
        required = {
            TOPIC_CONTROL_STATUS,
            TOPIC_SAFETY_STATUS,
            TOPIC_ARBITER_STATUS,
            TOPIC_WATCHDOG_STATUS,
            TOPIC_ODOM,
            TOPIC_DESIRED_CMD,
            TOPIC_CMD,
            TOPIC_MODEL_STATES,
            TOPIC_COLLISION_STATUS,
        }
        missing = sorted(required - self.hold_seen)
        if missing:
            self._finish(
                Code.EVIDENCE_INCOMPLETE,
                'completion hold lacks required observations',
                now,
                {'missing_hold_topics': missing},
            )
            return
        stationary = (
            self.hold_max_linear
            <= self.thresholds['stopped_linear_speed_mps']
            and self.hold_max_angular
            <= self.thresholds['stopped_angular_speed_radps']
        )
        command_motion = (
            self.hold_max_desired
            > self.thresholds['command_zero_tolerance']
            or self.hold_max_output
            > self.thresholds['command_zero_tolerance']
        )
        drift_motion = (
            self.hold_position_drift
            > self.thresholds['post_complete_position_drift_m']
            or self.hold_yaw_drift
            > self.thresholds['post_complete_yaw_drift_rad']
        )
        self.post_complete_motion = command_motion or drift_motion
        metrics = self._metrics(now, duration, stationary)
        snapshot = Snapshot(
            follower_state=self.control.state,
            follower_reason=self.control.reason,
            follower_fault_seen=self.follower_fault_seen,
            arbiter_latched_seen=self.arbiter_latched_seen,
            watchdog_emergency_seen=self.watchdog_emergency_seen,
            safety_fault_seen=self.safety_fault_seen,
            collision_count=self.collision_count,
            plan_stable=self.plan_stable,
            cp2_validated=self.cp2_result is not None,
            odom_position_error=(
                metrics['final_odom']['position_error_m']
            ),
            ground_truth_position_error=(
                metrics['final_ground_truth']['position_error_m']
            ),
            odom_yaw_error=metrics['final_odom']['yaw_error_rad'],
            ground_truth_yaw_error=(
                metrics['final_ground_truth']['yaw_error_rad']
            ),
            odom_margins=self.hold_odom_margins,
            ground_truth_margins=self.hold_world_margins,
            stationary_validated=stationary,
            post_complete_motion=self.post_complete_motion,
            telemetry_valid=not self.contract_errors,
            frame_alignment_valid=self.frame_alignment['validated'],
            collision_evidence_complete=(
                self.collision_status['valid']
                and self.collision_status['all_sensors_fresh']
            ),
            evidence_complete=True,
        )
        outcome = evaluate_snapshot(snapshot)
        self._transition('FINALIZING', now, 'completion_hold_finished')
        failures = [{
            'code': failure.code.value,
            'source': 'monitor',
            'reason': failure.detail,
            'observed_at_s': max(0.0, now - self.started),
            'details': {},
        } for failure in outcome.failures]
        self.result = self._result_document(
            outcome.primary_code,
            (
                'mission and online evidence satisfy P0 acceptance'
                if outcome.primary_code == Code.PASS
                else failures[0]['reason']
            ),
            now,
            metrics,
            failures,
        )
        self._transition(outcome.verdict.value, now, outcome.primary_code.value)

    def _metrics(self, now, hold_duration, stationary):
        target = self._target_pose(world=False)
        world_target = self._target_pose(world=True)
        odom_pose = self.latest_odom.pose
        world_pose = self.latest_world.pose
        odom_error = pose_errors(odom_pose, target)
        world_error = pose_errors(world_pose, world_target)
        delta = pose_errors(odom_pose, world_pose)
        return {
            'readiness_duration_s': self.ready_time - self.started,
            'mission_duration_s': now - self.motion_start_time,
            'post_complete_hold_s': hold_duration,
            'selected_y_m': self.frozen_plan.selected_y,
            'minimum_planned_clearance_m': (
                self.frozen_plan.minimum_clearance
            ),
            'frame_alignment': dict(self.frame_alignment),
            'final_odom': self._final_pose(
                odom_pose, odom_error, self.hold_odom_margins
            ),
            'final_ground_truth': self._final_pose(
                world_pose, world_error, self.hold_world_margins
            ),
            'odom_ground_truth_position_delta_m': delta.position,
            'odom_ground_truth_yaw_delta_rad': delta.yaw,
            'cp2': {
                'validated': self.cp2_result is not None,
                'crossing_time_s': self.cp2_time - self.started,
                **self._cp2_dict(self.cp2_result),
            },
            'stationary': {
                'validated': stationary,
                'maximum_linear_speed_mps': self.hold_max_linear,
                'maximum_angular_speed_radps': self.hold_max_angular,
                'maximum_desired_command': self.hold_max_desired,
                'maximum_output_command': self.hold_max_output,
                'position_drift_m': self.hold_position_drift,
                'yaw_drift_rad': self.hold_yaw_drift,
            },
            'collision_count': self.collision_count,
        }

    def _result_document(self, code, reason, now, metrics, failures):
        classification = (
            Classification.PASS if code == Code.PASS
            else Classification.ERROR if code in ERROR_CODES
            else Classification.FAIL
        )
        return {
            'code': code.value,
            'reason': reason,
            'classification': classification.value,
            'timestamp_utc': _utc_timestamp(),
            'observed_at_s': max(0.0, now - self.started),
            'mission': self._mission_document(),
            'metrics': metrics,
            'failures': failures,
        }

    def _finish(self, code, reason, now, details=None):
        if self.finished:
            return
        code = Code(code)
        failure = {
            'code': code.value,
            'source': 'monitor',
            'reason': reason,
            'observed_at_s': max(0.0, now - self.started),
            'details': details or {},
        }
        self.result = self._result_document(
            code,
            reason,
            now,
            self._partial_metrics(now),
            [] if code == Code.PASS else [failure],
        )
        classification = self.result['classification']
        self._transition(classification, now, code.value)

    def _mission_document(self):
        control = self.control
        arbiter = self.arbiter
        watchdog = self.watchdog
        safety = self.safety
        collision = self.collision_status
        return {
            'ready': self.ready_time is not None,
            'motion_started': self.motion_start_time is not None,
            'first_nonzero_cmd_time_s': (
                None if self.first_nonzero_cmd_time is None
                else self.first_nonzero_cmd_time - self.started
            ),
            'plan_stable': self.plan_stable,
            'plan_sha256': self.plan_sha256,
            'cp2_validated': self.cp2_result is not None,
            'follower': {
                'state': None if control is None else control.state,
                'reason': None if control is None else control.reason,
                'target_role': None if control is None else control.target_role,
                'fault_seen': self.follower_fault_seen,
            },
            'arbiter': {
                'state': None if arbiter is None else arbiter.state,
                'reason': None if arbiter is None else arbiter.reason,
                'armed': False if arbiter is None else arbiter.armed,
                'latched': self.arbiter_latched_seen,
                'cmd_vel_publisher_count': (
                    None if arbiter is None
                    else arbiter.cmd_vel_publisher_count
                ),
            },
            'watchdog': {
                'emergency_active': self.watchdog_emergency_seen,
                'reason': None if watchdog is None else watchdog.reason,
                'publishing_cmd_vel': (
                    False if watchdog is None
                    else watchdog.publishing_cmd_vel
                ),
            },
            'safety': {
                'valid': False if safety is None else safety.valid,
                'emergency_stop': (
                    False if safety is None else safety.emergency_stop
                ),
                'fault_seen': self.safety_fault_seen,
            },
            'collision': {
                'valid': False if collision is None else collision['valid'],
                'all_sensors_fresh': (
                    False if collision is None
                    else collision['all_sensors_fresh']
                ),
                'event_seen': self.collision_event_seen,
            },
        }

    def _partial_metrics(self, now):
        cp2 = {
            'validated': self.cp2_result is not None,
            'crossing_time_s': (
                None if self.cp2_time is None else self.cp2_time - self.started
            ),
            'longitudinal_error_m': None,
            'cross_track_error_m': None,
            'heading_error_rad': None,
            'angular_speed_radps': None,
        }
        if self.cp2_result is not None:
            cp2.update(self._cp2_dict(self.cp2_result))
        return {
            'readiness_duration_s': (
                max(0.0, now - self.started)
                if self.ready_time is None
                else self.ready_time - self.started
            ),
            'mission_duration_s': (
                None if self.motion_start_time is None
                else max(0.0, now - self.motion_start_time)
            ),
            'post_complete_hold_s': (
                0.0 if self.hold_started is None
                else max(0.0, now - self.hold_started)
            ),
            'selected_y_m': (
                None if self.frozen_plan is None
                else self.frozen_plan.selected_y
            ),
            'minimum_planned_clearance_m': (
                None if self.frozen_plan is None
                else self.frozen_plan.minimum_clearance
            ),
            'frame_alignment': dict(self.frame_alignment),
            'final_odom': None,
            'final_ground_truth': None,
            'odom_ground_truth_position_delta_m': None,
            'odom_ground_truth_yaw_delta_rad': None,
            'cp2': cp2,
            'stationary': {
                'validated': False,
                'maximum_linear_speed_mps': self.hold_max_linear,
                'maximum_angular_speed_radps': self.hold_max_angular,
                'maximum_desired_command': self.hold_max_desired,
                'maximum_output_command': self.hold_max_output,
                'position_drift_m': self.hold_position_drift,
                'yaw_drift_rad': self.hold_yaw_drift,
            },
            'collision_count': self.collision_count,
        }

    def _readiness_details(self, now):
        return {
            'stale_topics': list(self._stale_topics(now)),
            'target_matches': self._target_matches(self.target),
            'plan_valid': bool(self.plan is not None and self.plan.valid),
            'frame_alignment': self.frame_alignment,
        }

    def _stale_topics(self, now):
        return tuple(
            topic for topic in REQUIRED_TOPICS
            if topic not in self.samples
            or now - self.samples[topic] > self.freshness_timeout
        )

    def _target_pose(self, world):
        expected = self.config['target']
        pose = Pose2D(
            expected['center_x_m'],
            expected['center_y_m'],
            expected['final_yaw_rad'],
        )
        if not world:
            return pose
        transform = self.frame_alignment
        yaw = transform['odom_to_world_yaw_rad']
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return Pose2D(
            transform['odom_to_world_x_m']
            + cosine * pose.x - sine * pose.y,
            transform['odom_to_world_y_m']
            + sine * pose.x + cosine * pose.y,
            normalize_angle(yaw + pose.yaw),
        )

    def _parking_margins(self, pose, world):
        target = self._target_pose(world)
        expected = self.config['target']
        space = ParkingSpace(
            target,
            expected['length_m'],
            expected['width_m'],
        )
        return parking_margins(
            pose,
            self.thresholds['robot_footprint_length_m'],
            self.thresholds['robot_footprint_width_m'],
            space,
        )

    @staticmethod
    def _minimum_margins(current, value):
        if current is None:
            return value
        values = {
            name: min(getattr(current, name), getattr(value, name))
            for name in ('front', 'rear', 'left', 'right')
        }
        from robot_acceptance.geometry import Margins
        return Margins(**values, minimum=min(values.values()))

    @staticmethod
    def _maximum(current, value):
        value = abs(float(value))
        return value if current is None else max(current, value)

    @staticmethod
    def _final_pose(pose, error, margins):
        return {
            'x_m': pose.x,
            'y_m': pose.y,
            'yaw_rad': pose.yaw,
            'position_error_m': error.position,
            'yaw_error_rad': error.yaw,
            'margins_m': _margins_dict(margins),
        }

    @staticmethod
    def _cp2_dict(result):
        return {
            'longitudinal_error_m': result.longitudinal_error,
            'cross_track_error_m': result.cross_track_error,
            'heading_error_rad': result.heading_error,
            'angular_speed_radps': abs(result.angular_speed),
        }

    @staticmethod
    def _empty_frame_alignment():
        return {
            'validated': False,
            'odom_to_world_x_m': None,
            'odom_to_world_y_m': None,
            'odom_to_world_yaw_rad': None,
            'position_residual_m': None,
            'yaw_residual_rad': None,
            'sample_time_delta_s': None,
        }
