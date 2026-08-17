"""Pure collision-name filtering used by tests and offline analysis."""

import math


CONTACT_TOPICS = {
    'base': '/gazebo/contacts/base',
    'lidar': '/gazebo/contacts/lidar',
    'wheel_left': '/gazebo/contacts/wheel_left',
    'wheel_right': '/gazebo/contacts/wheel_right',
    'caster': '/gazebo/contacts/caster',
}
CONTACT_RELEASE_GRACE_S = 0.05

ROBOT_COLLISIONS = {
    'base': 'turtlebot3_burger_low_lidar::base_link::base_collision',
    'lidar': (
        'turtlebot3_burger_low_lidar::base_scan::lidar_sensor_collision'
    ),
    'wheel_left': (
        'turtlebot3_burger_low_lidar::wheel_left_link::wheel_left_collision'
    ),
    'wheel_right': (
        'turtlebot3_burger_low_lidar::wheel_right_link::wheel_right_collision'
    ),
    'caster': (
        'turtlebot3_burger_low_lidar::caster_back_link::caster_collision'
    ),
}
APPROVED_FLOORS = {
    'competition_field::field_link::floor_collision',
    'ground_plane::link::collision',
}
GROUND_CONTACT_SENSORS = {'wheel_left', 'wheel_right', 'caster'}


def scoped_name_matches(actual, expected):
    """Match a Gazebo scoped name while allowing an optional world prefix."""
    return actual == expected or actual.endswith(f'::{expected}')


def is_approved_ground_contact(sensor_name, first, second):
    """Allow only wheel/caster contact with an explicitly known floor."""
    if sensor_name not in GROUND_CONTACT_SENSORS:
        return False
    robot_collision = ROBOT_COLLISIONS[sensor_name]
    pairs = ((first, second), (second, first))
    return any(
        scoped_name_matches(robot_name, robot_collision)
        and any(scoped_name_matches(floor_name, floor) for floor in APPROVED_FLOORS)
        for robot_name, floor_name in pairs
    )


def unordered_pair(first, second):
    """Return a stable unordered collision pair."""
    return tuple(sorted((first, second)))


class ContactTracker:
    """Track sensor heartbeat and deduplicated unexpected contact onsets."""

    def __init__(self):
        self.last_received = {}
        self.active_until = {}
        self.collision_count = 0

    def observe(self, sensor_name, message, now):
        """Return newly observed unexpected ContactState messages."""
        if sensor_name not in CONTACT_TOPICS:
            raise ValueError(f'unknown contact sensor: {sensor_name}')
        if not math.isfinite(now):
            raise ValueError('contact receive time must be finite')
        self.last_received[sensor_name] = now
        events = []
        for state in message.states:
            first = state.collision1_name
            second = state.collision2_name
            if not first or not second:
                continue
            if is_approved_ground_contact(sensor_name, first, second):
                continue
            pair = unordered_pair(first, second)
            if self.active_until.get(pair, -math.inf) < now:
                events.append(state)
                self.collision_count += 1
            self.active_until[pair] = now + CONTACT_RELEASE_GRACE_S
        expired = [
            pair for pair, deadline in self.active_until.items()
            if deadline < now
        ]
        for pair in expired:
            del self.active_until[pair]
        return tuple(events)

    def sensor_ages(self, now):
        """Return each sensor age, using None before its first message."""
        return {
            name: (
                max(0.0, now - self.last_received[name])
                if name in self.last_received else None
            )
            for name in CONTACT_TOPICS
        }
