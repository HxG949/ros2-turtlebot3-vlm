"""Read-only ROS monitor for P0 production and evidence topics."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from robot_acceptance.contracts import ContractError
from robot_acceptance.contracts import parse_arbiter_status
from robot_acceptance.contracts import parse_control_status
from robot_acceptance.contracts import parse_plan
from robot_acceptance.contracts import parse_safety_status
from robot_acceptance.contracts import parse_selected_target
from robot_acceptance.contracts import parse_watchdog_status
from robot_acceptance.geometry import normalize_angle
from robot_acceptance.geometry import Pose2D
from robot_acceptance.mission import MissionState
from robot_acceptance.mission import MotionSample
from robot_acceptance.mission import REQUIRED_TOPICS
from robot_acceptance.mission import TOPIC_ARBITER_STATUS
from robot_acceptance.mission import TOPIC_CMD
from robot_acceptance.mission import TOPIC_COLLISION_EVENTS
from robot_acceptance.mission import TOPIC_COLLISION_STATUS
from robot_acceptance.mission import TOPIC_CONTROL_STATUS
from robot_acceptance.mission import TOPIC_DESIRED_CMD
from robot_acceptance.mission import TOPIC_MODEL_STATES
from robot_acceptance.mission import TOPIC_ODOM
from robot_acceptance.mission import TOPIC_PLAN
from robot_acceptance.mission import TOPIC_SAFETY_STATUS
from robot_acceptance.mission import TOPIC_SCAN
from robot_acceptance.mission import TOPIC_SELECTED_TARGET
from robot_acceptance.mission import TOPIC_WATCHDOG_STATUS
from robot_acceptance.runner import load_contract
from robot_acceptance.verdict import EXIT_CODES
from robot_acceptance.verdict import Code


@dataclass(frozen=True)
class ReceivedSample:
    """Store parsed data with local monotonic and ROS receive times."""

    value: object
    monotonic_time: float
    ros_time_ns: int


@dataclass(frozen=True)
class ContractIssue:
    """Store a telemetry error at the time its message was received."""

    error: str
    monotonic_time: float
    ros_time_ns: int


class MonitorCache:
    """Maintain latest samples and deterministic freshness checks."""

    def __init__(self):
        self.samples = {}
        self.contract_errors = {}

    def record(self, topic, value, monotonic_time, ros_time_ns):
        """Record one successfully decoded sample."""
        if not math.isfinite(monotonic_time):
            raise ValueError('monotonic receive time must be finite')
        self.samples[topic] = ReceivedSample(
            value=value,
            monotonic_time=float(monotonic_time),
            ros_time_ns=int(ros_time_ns),
        )
        self.contract_errors.pop(topic, None)

    def record_error(self, topic, error, monotonic_time, ros_time_ns):
        """Retain a timed contract error without replacing valid data."""
        if not math.isfinite(monotonic_time):
            raise ValueError('monotonic receive time must be finite')
        self.contract_errors[topic] = ContractIssue(
            error=str(error),
            monotonic_time=float(monotonic_time),
            ros_time_ns=int(ros_time_ns),
        )

    def stale_topics(self, now, maximum_age, required_topics=REQUIRED_TOPICS):
        """Return missing or stale required topics in stable order."""
        if not math.isfinite(now):
            raise ValueError('current monotonic time must be finite')
        if maximum_age <= 0.0 or not math.isfinite(maximum_age):
            raise ValueError('maximum_age must be positive and finite')
        return tuple(
            topic
            for topic in required_topics
            if topic not in self.samples
            or now - self.samples[topic].monotonic_time > maximum_age
        )


def parse_collision_object(payload, topic):
    """Reject malformed collision JSON while leaving its evolving schema opaque."""
    if not isinstance(payload, str):
        raise ContractError(f'{topic}: payload must be a JSON string')

    def reject_constant(value):
        raise ContractError(f'{topic}: non-finite JSON number: {value}')

    def object_pairs(pairs):
        result = {}
        for key, item in pairs:
            if key in result:
                raise ContractError(f'{topic}: duplicate field: {key}')
            result[key] = item
        return result

    try:
        value = json.loads(
            payload,
            parse_constant=reject_constant,
            object_pairs_hook=object_pairs,
        )
    except ContractError:
        raise
    except (json.JSONDecodeError, TypeError) as error:
        raise ContractError(f'{topic}: invalid JSON: {error}') from error
    if not isinstance(value, dict):
        raise ContractError(f'{topic}: JSON root must be an object')
    _validate_json_numbers(value, topic)
    if 'collision_count' in value:
        count = value['collision_count']
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ContractError(
                f'{topic}.collision_count must be a non-negative integer'
            )
    return value


def _validate_json_numbers(value, path):
    """Recursively reject non-finite numbers in collision evidence."""
    if isinstance(value, dict):
        for key, item in value.items():
            _validate_json_numbers(item, f'{path}.{key}')
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _validate_json_numbers(item, f'{path}[{index}]')
    elif not isinstance(value, bool) and isinstance(value, (int, float)):
        try:
            finite = math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            raise ContractError(f'{path} must be finite')


def _twist_values(message):
    return (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z,
    )


def _pose_values(message):
    return (
        message.position.x, message.position.y, message.position.z,
        message.orientation.x, message.orientation.y,
        message.orientation.z, message.orientation.w,
    )


def _require_finite(values, topic):
    if not all(math.isfinite(value) for value in values):
        raise ContractError(f'{topic} contains a non-finite numeric value')


def validate_odometry(message, topic):
    """Validate numeric odometry evidence before caching it."""
    _require_finite(
        _pose_values(message.pose.pose) + _twist_values(message.twist.twist),
        topic,
    )


def validate_twist(message, topic):
    """Validate a command velocity before caching it."""
    _require_finite(_twist_values(message), topic)


def validate_model_states(message, topic):
    """Validate Gazebo model-state array shape and numeric values."""
    if len(message.name) != len(message.pose) or len(message.name) != len(
        message.twist
    ):
        raise ContractError(f'{topic} name, pose, and twist lengths differ')
    for pose, twist in zip(message.pose, message.twist):
        _require_finite(_pose_values(pose) + _twist_values(twist), topic)


def _yaw(orientation):
    """Return planar yaw from a validated quaternion."""
    siny = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cosy = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return normalize_angle(math.atan2(siny, cosy))


def _motion_sample(pose, twist):
    return MotionSample(
        pose=Pose2D(pose.position.x, pose.position.y,
                    _yaw(pose.orientation)),
        linear_speed=math.hypot(twist.linear.x, twist.linear.y),
        angular_speed=twist.angular.z,
    )


class EvidenceWriter:
    """Stream measured CSV/JSONL evidence and atomically publish the result."""

    def __init__(self, output_path):
        self.output_path = Path(output_path).expanduser().resolve()
        self.directory = self.output_path.parent
        if not self.directory.is_dir():
            raise FileNotFoundError(
                f'monitor output directory does not exist: {self.directory}'
            )
        self.streams = {
            'events': self._open(
                'events.jsonl', None
            ),
            'pose': self._open(
                'pose.csv',
                'monotonic_s,ros_time_ns,source,x,y,yaw\n',
            ),
            'cmd': self._open(
                'cmd_vel.csv',
                'monotonic_s,ros_time_ns,topic,linear_x,angular_z\n',
            ),
            'safety': self._open(
                'safety.csv',
                'monotonic_s,ros_time_ns,topic,state,reason,valid,'
                'emergency_stop,latched,publisher_count\n',
            ),
            'margins': self._open(
                'parking_margins.csv',
                'monotonic_s,source,front,rear,left,right,minimum\n',
            ),
        }
        self.event_index = 0
        self.written = False

    def _open(self, name, header):
        stream = (self.directory / name).open('x', encoding='ascii', newline='')
        if header is not None:
            stream.write(header)
        return stream

    def pose(self, elapsed, ros_time_ns, source, sample):
        """Append one actual odom or world pose sample."""
        pose = sample.pose
        self.streams['pose'].write(
            f'{elapsed:.9f},{ros_time_ns},{source},'
            f'{pose.x:.12g},{pose.y:.12g},{pose.yaw:.12g}\n'
        )

    def command(self, elapsed, ros_time_ns, topic, message):
        """Append one desired or output command sample."""
        self.streams['cmd'].write(
            f'{elapsed:.9f},{ros_time_ns},{topic},'
            f'{message.linear.x:.12g},{message.angular.z:.12g}\n'
        )

    def status(self, elapsed, ros_time_ns, topic, state='', reason='',
               valid='', emergency='', latched='', publisher_count=''):
        """Append one parsed safety-chain or follower state sample."""
        fields = (
            f'{elapsed:.9f}', str(ros_time_ns), topic, str(state), str(reason),
            str(valid), str(emergency), str(latched), str(publisher_count),
        )
        self.streams['safety'].write(','.join(fields) + '\n')

    def drain_events(self, events):
        """Append newly created mission transitions and safety events."""
        while self.event_index < len(events):
            value = events[self.event_index]
            payload = json.dumps(value, allow_nan=False, sort_keys=True)
            self.streams['events'].write(payload + '\n')
            self.event_index += 1

    def finish(self, result, events):
        """Flush streams and atomically link a strict terminal JSON file."""
        if self.written:
            return
        self.drain_events(events)
        metrics = result.get('metrics', {})
        elapsed = float(result.get('observed_at_s', 0.0))
        for source, key in (
            ('odom', 'final_odom'),
            ('ground_truth', 'final_ground_truth'),
        ):
            final_pose = metrics.get(key)
            if final_pose is None:
                continue
            margins = final_pose['margins_m']
            self.streams['margins'].write(
                f'{elapsed:.9f},{source},{margins["front"]:.12g},'
                f'{margins["rear"]:.12g},{margins["left"]:.12g},'
                f'{margins["right"]:.12g},{margins["minimum"]:.12g}\n'
            )
        for stream in self.streams.values():
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()
        document = {**result, 'events_count': len(events)}
        payload = json.dumps(
            document, allow_nan=False, indent=2, sort_keys=True
        ) + '\n'
        temporary = self.output_path.with_name(
            f'.{self.output_path.name}.tmp.{os.getpid()}'
        )
        try:
            with temporary.open('x', encoding='ascii') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, self.output_path)
        finally:
            temporary.unlink(missing_ok=True)
        self.written = True


class AcceptanceMonitor(Node):
    """Subscribe to acceptance evidence without any actuation capability."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'robot_acceptance_monitor',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('freshness_timeout_s', 1.0)
        self.declare_parameter('output_path', '')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter(
            'robot_entity_name', 'turtlebot3_burger_low_lidar'
        )
        self.freshness_timeout = float(
            self.get_parameter('freshness_timeout_s').value
        )
        if self.freshness_timeout <= 0.0:
            raise ValueError('freshness_timeout_s must be positive')
        self.cache = MonitorCache()
        self.last_stale_topics = None
        self.last_status = {}
        self.started_monotonic = time.monotonic()
        output_path = str(self.get_parameter('output_path').value)
        enable_motion = bool(self.get_parameter('enable_motion').value)
        if enable_motion and not output_path:
            raise ValueError('output_path is required when motion is enabled')
        robot_entity_name = str(
            self.get_parameter('robot_entity_name').value
        )
        self.mission = MissionState(
            load_contract(),
            enable_motion,
            robot_entity_name,
            started_monotonic=self.started_monotonic,
            freshness_timeout_s=self.freshness_timeout,
        )
        self.writer = EvidenceWriter(output_path) if output_path else None
        self._shutdown_requested = False
        self._acceptance_subscriptions = [
            self.create_subscription(
                String, TOPIC_SELECTED_TARGET,
                self._json_callback(TOPIC_SELECTED_TARGET,
                                    parse_selected_target), 10),
            self.create_subscription(
                String, TOPIC_PLAN,
                self._json_callback(TOPIC_PLAN, parse_plan), 10),
            self.create_subscription(
                String, TOPIC_CONTROL_STATUS,
                self._json_callback(TOPIC_CONTROL_STATUS,
                                    parse_control_status), 10),
            self.create_subscription(
                String, TOPIC_SAFETY_STATUS,
                self._json_callback(TOPIC_SAFETY_STATUS,
                                    parse_safety_status), 10),
            self.create_subscription(
                String, TOPIC_ARBITER_STATUS,
                self._json_callback(TOPIC_ARBITER_STATUS,
                                    parse_arbiter_status), 10),
            self.create_subscription(
                String, TOPIC_WATCHDOG_STATUS,
                self._json_callback(TOPIC_WATCHDOG_STATUS,
                                    parse_watchdog_status), 10),
            self.create_subscription(
                LaserScan, TOPIC_SCAN, self._message_callback(TOPIC_SCAN),
                qos_profile_sensor_data),
            self.create_subscription(
                Odometry, TOPIC_ODOM,
                self._message_callback(TOPIC_ODOM, validate_odometry),
                qos_profile_sensor_data),
            self.create_subscription(
                Twist, TOPIC_DESIRED_CMD,
                self._message_callback(TOPIC_DESIRED_CMD, validate_twist), 10),
            self.create_subscription(
                Twist, TOPIC_CMD,
                self._message_callback(TOPIC_CMD, validate_twist), 10),
            self.create_subscription(
                ModelStates, TOPIC_MODEL_STATES,
                self._message_callback(TOPIC_MODEL_STATES,
                                       validate_model_states),
                qos_profile_sensor_data),
            self.create_subscription(
                String, TOPIC_COLLISION_STATUS,
                self._collision_callback(TOPIC_COLLISION_STATUS), 10),
            self.create_subscription(
                String, TOPIC_COLLISION_EVENTS,
                self._collision_callback(TOPIC_COLLISION_EVENTS), 10),
        ]
        self.create_timer(0.5, self._freshness_timer)
        self.get_logger().info(
            'Read-only acceptance monitor subscribed to production evidence'
        )

    def _receive_times(self):
        return time.monotonic(), self.get_clock().now().nanoseconds

    def _record(self, topic, value, monotonic_time, ros_time_ns):
        self.cache.record(topic, value, monotonic_time, ros_time_ns)

    def _after_observation(self):
        if self.writer is not None:
            self.writer.drain_events(self.mission.events)
        if self.mission.finished:
            self._finalize_output()

    def _finalize_output(self):
        if self.writer is not None:
            self.writer.finish(self.mission.result, self.mission.events)
            if not self._shutdown_requested:
                self._shutdown_requested = True
                self.get_logger().info(
                    'Acceptance monitor reached terminal result: '
                    + self.mission.result['code']
                )

    def close(self, reason='monitor stopped before a terminal result'):
        """Conservatively finalize evidence during external shutdown."""
        self.mission.interrupt(time.monotonic(), reason)
        self._finalize_output()

    def _json_callback(self, topic, parser):
        def callback(message):
            monotonic_time, ros_time_ns = self._receive_times()
            try:
                value = parser(message.data)
            except ContractError as error:
                self.cache.record_error(
                    topic, error, monotonic_time, ros_time_ns
                )
                self.get_logger().error(f'{topic} contract invalid: {error}')
                self.mission.observe_contract_error(
                    topic, error, monotonic_time
                )
                self._after_observation()
                return
            self._record(topic, value, monotonic_time, ros_time_ns)
            observers = {
                TOPIC_SELECTED_TARGET: self.mission.observe_target,
                TOPIC_PLAN: self.mission.observe_plan,
                TOPIC_CONTROL_STATUS: self.mission.observe_control,
                TOPIC_SAFETY_STATUS: self.mission.observe_safety,
                TOPIC_ARBITER_STATUS: self.mission.observe_arbiter,
                TOPIC_WATCHDOG_STATUS: self.mission.observe_watchdog,
            }
            observers[topic](value, monotonic_time)
            if self.writer is not None:
                self._write_status(
                    topic, value, monotonic_time, ros_time_ns
                )
            self._log_status(topic, value)
            self._log_fault_state(topic, value)
            self._after_observation()
        return callback

    def _message_callback(self, topic, validator=None):
        def callback(message):
            monotonic_time, ros_time_ns = self._receive_times()
            try:
                if validator is not None:
                    validator(message, topic)
            except ContractError as error:
                self.cache.record_error(
                    topic, error, monotonic_time, ros_time_ns
                )
                self.get_logger().error(f'{topic} telemetry invalid: {error}')
                self.mission.observe_contract_error(
                    topic, error, monotonic_time
                )
                self._after_observation()
                return
            self._record(topic, message, monotonic_time, ros_time_ns)
            if topic == TOPIC_SCAN:
                self.mission.observe_presence(topic, monotonic_time)
            elif topic == TOPIC_ODOM:
                sample = _motion_sample(
                    message.pose.pose, message.twist.twist
                )
                self.mission.observe_odom(sample, monotonic_time)
                if self.writer is not None:
                    self.writer.pose(
                        monotonic_time - self.started_monotonic,
                        ros_time_ns,
                        'odom',
                        sample,
                    )
            elif topic in (TOPIC_DESIRED_CMD, TOPIC_CMD):
                self.mission.observe_command(
                    topic,
                    message.linear.x,
                    message.angular.z,
                    monotonic_time,
                )
                if self.writer is not None:
                    self.writer.command(
                        monotonic_time - self.started_monotonic,
                        ros_time_ns,
                        topic,
                        message,
                    )
            elif topic == TOPIC_MODEL_STATES:
                try:
                    index = message.name.index(
                        self.mission.robot_entity_name
                    )
                except ValueError:
                    self.mission.observe_world(
                        None, monotonic_time, entity_found=False
                    )
                else:
                    sample = _motion_sample(
                        message.pose[index], message.twist[index]
                    )
                    self.mission.observe_world(sample, monotonic_time)
                    if self.writer is not None:
                        self.writer.pose(
                            monotonic_time - self.started_monotonic,
                            ros_time_ns,
                            'ground_truth',
                            sample,
                        )
            self._after_observation()
        return callback

    def _collision_callback(self, topic):
        def callback(message):
            monotonic_time, ros_time_ns = self._receive_times()
            try:
                value = parse_collision_object(message.data, topic)
            except ContractError as error:
                self.cache.record_error(
                    topic, error, monotonic_time, ros_time_ns
                )
                self.get_logger().error(f'{topic} contract invalid: {error}')
                self.mission.observe_contract_error(
                    topic, error, monotonic_time
                )
                self._after_observation()
                return
            self._record(topic, value, monotonic_time, ros_time_ns)
            if topic == TOPIC_COLLISION_EVENTS:
                self.get_logger().error('Unexpected collision event received')
                self.mission.observe_collision_event(value, monotonic_time)
            elif value.get('collision_count', 0) != 0:
                self.get_logger().error('Collision status reports collisions')
                self.mission.observe_collision_status(value, monotonic_time)
            else:
                self.mission.observe_collision_status(value, monotonic_time)
            self._after_observation()
        return callback

    def _write_status(self, topic, value, monotonic_time, ros_time_ns):
        elapsed = monotonic_time - self.started_monotonic
        if topic == TOPIC_CONTROL_STATUS:
            self.writer.status(
                elapsed, ros_time_ns, topic, value.state, value.reason
            )
        elif topic == TOPIC_SAFETY_STATUS:
            self.writer.status(
                elapsed, ros_time_ns, topic, valid=value.valid,
                emergency=value.emergency_stop,
            )
        elif topic == TOPIC_ARBITER_STATUS:
            self.writer.status(
                elapsed, ros_time_ns, topic, value.state, value.reason,
                latched=value.latched,
                publisher_count=value.cmd_vel_publisher_count,
            )
        elif topic == TOPIC_WATCHDOG_STATUS:
            self.writer.status(
                elapsed, ros_time_ns, topic, reason=value.reason,
                emergency=value.emergency_active,
            )

    def _log_status(self, topic, value):
        status = None
        if topic == TOPIC_CONTROL_STATUS:
            status = (value.state, value.reason)
        elif topic == TOPIC_SAFETY_STATUS:
            status = (value.valid, value.emergency_stop)
        elif topic == TOPIC_ARBITER_STATUS:
            status = (value.state, value.reason, value.armed, value.latched)
        elif topic == TOPIC_WATCHDOG_STATUS:
            status = (value.emergency_active, value.reason)
        elif topic == TOPIC_PLAN:
            status = (value.valid, value.reason)
        if status is not None and self.last_status.get(topic) != status:
            self.last_status[topic] = status
            self.get_logger().info(f'{topic} status changed: {status}')

    def _log_fault_state(self, topic, value):
        if topic == TOPIC_CONTROL_STATUS and value.state == 'FAULT':
            self.get_logger().error(f'Follower fault: {value.reason}')
        elif topic == TOPIC_SAFETY_STATUS and (
            not value.valid or value.emergency_stop
        ):
            self.get_logger().warning('Safety status invalid or emergency')
        elif topic == TOPIC_ARBITER_STATUS and value.latched:
            self.get_logger().error(f'Arbiter latched: {value.reason}')
        elif topic == TOPIC_WATCHDOG_STATUS and value.emergency_active:
            self.get_logger().error(f'Watchdog emergency: {value.reason}')
        elif topic == TOPIC_PLAN and not value.valid:
            self.get_logger().warning(f'Plan invalid: {value.reason}')

    def _freshness_timer(self):
        now = time.monotonic()
        self.mission.tick(now)
        self._after_observation()
        stale = self.cache.stale_topics(
            now,
            self.freshness_timeout,
        )
        if stale == self.last_stale_topics:
            return
        self.last_stale_topics = stale
        if stale:
            self.get_logger().warning(
                'Missing/stale acceptance topics: ' + ', '.join(stale)
            )
        else:
            self.get_logger().info('All required acceptance topics are fresh')


def main(args=None):
    """Run the read-only acceptance monitor."""
    rclpy.init(args=args)
    node = None
    exit_code = EXIT_CODES[Code.EVIDENCE_INCOMPLETE]
    try:
        node = AcceptanceMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            exit_code = EXIT_CODES[Code(node.mission.result['code'])]
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
