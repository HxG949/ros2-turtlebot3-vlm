"""ROS integration tests using synthetic publishers without Gazebo."""

import json
import math
import time

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from robot_acceptance.monitor_node import AcceptanceMonitor
from robot_acceptance.monitor_node import REQUIRED_TOPICS
from robot_acceptance.monitor_node import TOPIC_ARBITER_STATUS
from robot_acceptance.monitor_node import TOPIC_CMD
from robot_acceptance.monitor_node import TOPIC_COLLISION_EVENTS
from robot_acceptance.monitor_node import TOPIC_COLLISION_STATUS
from robot_acceptance.monitor_node import TOPIC_CONTROL_STATUS
from robot_acceptance.monitor_node import TOPIC_DESIRED_CMD
from robot_acceptance.monitor_node import TOPIC_MODEL_STATES
from robot_acceptance.monitor_node import TOPIC_ODOM
from robot_acceptance.monitor_node import TOPIC_PLAN
from robot_acceptance.monitor_node import TOPIC_SAFETY_STATUS
from robot_acceptance.monitor_node import TOPIC_SCAN
from robot_acceptance.monitor_node import TOPIC_SELECTED_TARGET
from robot_acceptance.monitor_node import TOPIC_WATCHDOG_STATUS


def as_string(value):
    """Encode one strict JSON object in a ROS String message."""
    message = String()
    message.data = json.dumps(value, allow_nan=False)
    return message


def selected_target():
    """Return the frozen selected parking target payload."""
    return {
        'valid': True,
        'frame_id': 'odom',
        'target': {
            'id': 'space_2',
            'center_x': 0.8015,
            'center_y': 0.0,
            'entry_yaw': 0.0,
            'final_yaw': math.pi,
            'length': 0.297,
            'width': 0.210,
            'approach_distance': 0.4,
        },
    }


def valid_plan():
    """Return a compact valid P0 route with production diagnostics."""
    return {
        'valid': True,
        'reason': 'current_lane_safe',
        'selected_y': -0.315,
        'minimum_clearance': 0.12,
        'waypoints': [
            {
                'role': 'start',
                'x': -0.9015,
                'y': -0.845,
                'stop_required': True,
            },
            {
                'role': 'cp1',
                'x': -0.39,
                'y': -0.845,
                'stop_required': True,
            },
            {
                'role': 'lane_entry',
                'x': -0.39,
                'y': -0.315,
                'stop_required': True,
            },
            {
                'role': 'cp2',
                'x': 0.2,
                'y': -0.315,
                'stop_required': False,
            },
            {
                'role': 'parking_transition',
                'x': 0.4015,
                'y': -0.315,
                'stop_required': True,
            },
            {
                'role': 'parking_approach',
                'x': 0.4015,
                'y': 0.0,
                'stop_required': True,
            },
            {
                'role': 'parking_goal',
                'x': 0.8015,
                'y': 0.0,
                'stop_required': True,
                'final_yaw': math.pi,
                'parking_space_id': 'space_2',
                'parking_length': 0.297,
                'parking_width': 0.210,
            },
        ],
        'parking_space_id': 'space_2',
        'obstacle_point_count': 3,
        'scan_point_count': 360,
        'scan_valid_fraction': 1.0,
        'robot_pose': {'x': -0.9015, 'y': -0.845, 'yaw': 0.0},
    }


def control_status(state='COMPLETE', reason='parking_complete'):
    """Return follower telemetry for one requested state."""
    return {
        'enabled': True,
        'stop_at_cp1': False,
        'state': state,
        'reason': reason,
        'target_role': 'parking_goal',
        'remaining_distance': 0.0,
        'linear_x': 0.0,
        'angular_z': 0.0,
    }


def safety_status():
    """Return a valid and non-emergency lidar safety payload."""
    return {
        'front_distance': 0.8,
        'left_distance': 0.5,
        'right_distance': 0.5,
        'minimum_distance': 0.4,
        'rotation_minimum_distance': 0.4,
        'front_valid_fraction': 1.0,
        'left_valid_fraction': 1.0,
        'right_valid_fraction': 1.0,
        'full_valid_fraction': 1.0,
        'left_safe': True,
        'right_safe': True,
        'rotation_safe': True,
        'rotation_safe_raw': True,
        'rotation_caution': False,
        'rotation_unsafe_streak': 0,
        'emergency_stop': False,
        'valid': True,
    }


def arbiter_status(state='ACTIVE', reason='command_allowed', latched=False):
    """Return safety arbiter telemetry for one requested state."""
    return {
        'enabled': True,
        'armed': True,
        'latched': latched,
        'state': state,
        'reason': reason,
        'linear_x': 0.0,
        'angular_z': 0.0,
        'cmd_vel_publisher_count': 1,
    }


def watchdog_status(emergency=False):
    """Return watchdog telemetry for normal or takeover state."""
    return {
        'emergency_active': emergency,
        'reason': (
            'arbiter_heartbeat_timeout'
            if emergency else 'arbiter_heartbeat_fresh'
        ),
        'publishing_cmd_vel': emergency,
    }


class SyntheticPublishers(Node):
    """Publish isolated test inputs that must never run with the simulator."""

    def __init__(self):
        super().__init__('robot_acceptance_synthetic_publishers')
        self.publishers_by_topic = {
            TOPIC_SELECTED_TARGET: self.create_publisher(
                String, TOPIC_SELECTED_TARGET, 10
            ),
            TOPIC_PLAN: self.create_publisher(String, TOPIC_PLAN, 10),
            TOPIC_CONTROL_STATUS: self.create_publisher(
                String, TOPIC_CONTROL_STATUS, 10
            ),
            TOPIC_SAFETY_STATUS: self.create_publisher(
                String, TOPIC_SAFETY_STATUS, 10
            ),
            TOPIC_ARBITER_STATUS: self.create_publisher(
                String, TOPIC_ARBITER_STATUS, 10
            ),
            TOPIC_WATCHDOG_STATUS: self.create_publisher(
                String, TOPIC_WATCHDOG_STATUS, 10
            ),
            TOPIC_SCAN: self.create_publisher(
                LaserScan, TOPIC_SCAN, qos_profile_sensor_data
            ),
            TOPIC_ODOM: self.create_publisher(
                Odometry, TOPIC_ODOM, qos_profile_sensor_data
            ),
            TOPIC_DESIRED_CMD: self.create_publisher(
                Twist, TOPIC_DESIRED_CMD, 10
            ),
            TOPIC_CMD: self.create_publisher(Twist, TOPIC_CMD, 10),
            TOPIC_MODEL_STATES: self.create_publisher(
                ModelStates, TOPIC_MODEL_STATES, qos_profile_sensor_data
            ),
            TOPIC_COLLISION_STATUS: self.create_publisher(
                String, TOPIC_COLLISION_STATUS, 10
            ),
            TOPIC_COLLISION_EVENTS: self.create_publisher(
                String, TOPIC_COLLISION_EVENTS, 10
            ),
        }

    def publish_nominal(self):
        """Publish one complete nominal telemetry set."""
        json_messages = {
            TOPIC_SELECTED_TARGET: selected_target(),
            TOPIC_PLAN: valid_plan(),
            TOPIC_CONTROL_STATUS: control_status(),
            TOPIC_SAFETY_STATUS: safety_status(),
            TOPIC_ARBITER_STATUS: arbiter_status(),
            TOPIC_WATCHDOG_STATUS: watchdog_status(),
            TOPIC_COLLISION_STATUS: {
                'valid': True,
                'collision_count': 0,
                'all_sensors_fresh': True,
            },
        }
        for topic, value in json_messages.items():
            self.publishers_by_topic[topic].publish(as_string(value))
        self.publishers_by_topic[TOPIC_SCAN].publish(LaserScan())
        self.publishers_by_topic[TOPIC_ODOM].publish(Odometry())
        self.publishers_by_topic[TOPIC_DESIRED_CMD].publish(Twist())
        self.publishers_by_topic[TOPIC_CMD].publish(Twist())
        model_states = ModelStates()
        model_states.name = ['turtlebot3_burger_low_lidar']
        model_states.pose = [Pose()]
        model_states.twist = [Twist()]
        self.publishers_by_topic[TOPIC_MODEL_STATES].publish(model_states)


def spin_until(executor, predicate, timeout=3.0, publish=None):
    """Spin until a predicate succeeds while optionally republishing data."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if publish is not None:
            publish()
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('synthetic ROS messages were not received in time')


def odometry(x, y, yaw, linear=0.0, angular=0.0):
    """Build one synthetic planar odometry message."""
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.twist.twist.linear.x = linear
    message.twist.twist.angular.z = angular
    return message


def model_states(x, y, yaw):
    """Build one synthetic model-state message for the configured robot."""
    message = ModelStates()
    message.name = ['turtlebot3_burger_low_lidar']
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    message.pose = [pose]
    message.twist = [Twist()]
    return message


def test_synthetic_publishers_cover_nominal_fault_and_timeout_paths():
    """Drive the read-only monitor through all non-Gazebo input paths."""
    rclpy.init()
    monitor = AcceptanceMonitor()
    publishers = SyntheticPublishers()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(publishers)
    try:
        spin_until(
            executor,
            lambda: not monitor.cache.stale_topics(
                time.monotonic(), 1.0, REQUIRED_TOPICS
            ),
            publish=publishers.publish_nominal,
        )
        assert monitor.cache.contract_errors == {}
        assert set(REQUIRED_TOPICS) <= set(monitor.cache.samples)

        stale = monitor.cache.stale_topics(
            time.monotonic() + 2.0, 1.0, REQUIRED_TOPICS
        )
        assert stale == REQUIRED_TOPICS

        updates = {
            TOPIC_CONTROL_STATUS: control_status(
                'FAULT', 'plan_changed_during_motion'
            ),
            TOPIC_ARBITER_STATUS: arbiter_status(
                'LATCHED', 'desired_velocity_timeout', True
            ),
            TOPIC_WATCHDOG_STATUS: watchdog_status(True),
            TOPIC_COLLISION_EVENTS: {
                'event_type': 'collision',
                'collision1_name': 'robot::base_collision',
                'collision2_name': 'competition_bottle_1::bottle_collision',
            },
        }
        received = {
            TOPIC_CONTROL_STATUS: lambda: (
                monitor.cache.samples[TOPIC_CONTROL_STATUS].value.state
                == 'FAULT'
            ),
            TOPIC_ARBITER_STATUS: lambda: (
                monitor.cache.samples[TOPIC_ARBITER_STATUS].value.latched
            ),
            TOPIC_WATCHDOG_STATUS: lambda: (
                monitor.cache.samples[TOPIC_WATCHDOG_STATUS]
                .value.emergency_active
            ),
            TOPIC_COLLISION_EVENTS: lambda: (
                monitor.cache.samples[TOPIC_COLLISION_EVENTS].value
                == updates[TOPIC_COLLISION_EVENTS]
            ),
        }
        for topic, value in updates.items():
            publishers.publishers_by_topic[topic].publish(as_string(value))
            spin_until(
                executor,
                lambda topic=topic: (
                    topic in monitor.cache.samples and received[topic]()
                ),
            )

        assert monitor.cache.samples[TOPIC_CONTROL_STATUS].value.state == 'FAULT'
        assert monitor.cache.samples[TOPIC_ARBITER_STATUS].value.latched
        assert monitor.cache.samples[TOPIC_WATCHDOG_STATUS].value.emergency_active
        assert TOPIC_COLLISION_EVENTS in monitor.cache.samples
    finally:
        executor.remove_node(publishers)
        executor.remove_node(monitor)
        publishers.destroy_node()
        monitor.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_synthetic_full_success_writes_atomic_measured_result(tmp_path):
    """Drive a complete PASS through rclpy callbacks without Gazebo."""
    rclpy.init()
    output = tmp_path / 'monitor_result.json'
    monitor = AcceptanceMonitor(parameter_overrides=[
        Parameter('output_path', Parameter.Type.STRING, str(output)),
        Parameter('enable_motion', Parameter.Type.BOOL, True),
    ])
    publishers = SyntheticPublishers()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(publishers)

    def publish_json(topic, value):
        publishers.publishers_by_topic[topic].publish(as_string(value))

    def publish_readiness():
        waiting = control_status('WAITING', 'waiting_for_plan')
        waiting['target_role'] = None
        waiting['remaining_distance'] = None
        for topic, value in (
            (TOPIC_SELECTED_TARGET, selected_target()),
            (TOPIC_PLAN, valid_plan()),
            (TOPIC_CONTROL_STATUS, waiting),
            (TOPIC_SAFETY_STATUS, safety_status()),
            (TOPIC_ARBITER_STATUS, arbiter_status()),
            (TOPIC_WATCHDOG_STATUS, watchdog_status()),
            (TOPIC_COLLISION_STATUS, {
                'valid': True,
                'all_sensors_fresh': True,
                'collision_count': 0,
            }),
        ):
            publish_json(topic, value)
        publishers.publishers_by_topic[TOPIC_SCAN].publish(LaserScan())
        publishers.publishers_by_topic[TOPIC_ODOM].publish(
            odometry(-0.9015, -0.845, 0.0)
        )
        publishers.publishers_by_topic[TOPIC_MODEL_STATES].publish(
            model_states(-0.9015, -0.845, 0.0)
        )
        publishers.publishers_by_topic[TOPIC_DESIRED_CMD].publish(Twist())
        publishers.publishers_by_topic[TOPIC_CMD].publish(Twist())

    def publish_hold():
        publish_readiness_status = (
            (TOPIC_SELECTED_TARGET, selected_target()),
            (TOPIC_PLAN, valid_plan()),
            (TOPIC_CONTROL_STATUS, control_status()),
            (TOPIC_SAFETY_STATUS, safety_status()),
            (TOPIC_ARBITER_STATUS, arbiter_status()),
            (TOPIC_WATCHDOG_STATUS, watchdog_status()),
            (TOPIC_COLLISION_STATUS, {
                'valid': True,
                'all_sensors_fresh': True,
                'collision_count': 0,
            }),
        )
        for topic, value in publish_readiness_status:
            publish_json(topic, value)
        publishers.publishers_by_topic[TOPIC_SCAN].publish(LaserScan())
        publishers.publishers_by_topic[TOPIC_ODOM].publish(
            odometry(0.8015, 0.0, math.pi)
        )
        publishers.publishers_by_topic[TOPIC_MODEL_STATES].publish(
            model_states(0.8015, 0.0, math.pi)
        )
        publishers.publishers_by_topic[TOPIC_DESIRED_CMD].publish(Twist())
        publishers.publishers_by_topic[TOPIC_CMD].publish(Twist())

    try:
        spin_until(
            executor,
            lambda: monitor.mission.ready_time is not None,
            publish=publish_readiness,
        )
        moving = Twist()
        moving.linear.x = 0.02
        spin_until(
            executor,
            lambda: monitor.mission.motion_start_time is not None,
            publish=lambda: publishers.publishers_by_topic[
                TOPIC_CMD
            ].publish(moving),
        )
        publishers.publishers_by_topic[TOPIC_ODOM].publish(
            odometry(0.19, -0.315, 0.0)
        )
        spin_until(
            executor,
            lambda: (
                monitor.mission.latest_odom is not None
                and monitor.mission.latest_odom.pose.x == 0.19
            ),
        )
        publishers.publishers_by_topic[TOPIC_ODOM].publish(
            odometry(0.20, -0.315, 0.0)
        )
        spin_until(
            executor,
            lambda: monitor.mission.cp2_result is not None,
        )
        publishers.publishers_by_topic[TOPIC_ODOM].publish(
            odometry(0.8015, 0.0, math.pi)
        )
        publishers.publishers_by_topic[TOPIC_MODEL_STATES].publish(
            model_states(0.8015, 0.0, math.pi)
        )
        publish_json(TOPIC_CONTROL_STATUS, control_status())
        spin_until(
            executor,
            lambda: monitor.mission.hold_started is not None,
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not output.exists():
            publish_hold()
            try:
                executor.spin_once(timeout_sec=0.05)
            except Exception as error:
                if rclpy.ok():
                    raise
                assert error is not None
        assert output.is_file()
        result = json.loads(output.read_text(encoding='ascii'))
        assert result['code'] == 'PASS'
        assert result['metrics']['cp2']['validated'] is True
        assert len((tmp_path / 'pose.csv').read_text().splitlines()) > 2
        assert len((tmp_path / 'cmd_vel.csv').read_text().splitlines()) > 2
        assert 'state_transition' in (
            tmp_path / 'events.jsonl'
        ).read_text()
    finally:
        executor.remove_node(publishers)
        executor.remove_node(monitor)
        publishers.destroy_node()
        monitor.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
