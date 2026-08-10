import math

import pytest

from robot_perception.axis_aligned_follower_node import calculate_linear_speed
from robot_perception.axis_aligned_follower_node import calculate_turn_speed
from robot_perception.axis_aligned_follower_node import execution_plan_signature
from robot_perception.axis_aligned_follower_node import heading_is_settled
from robot_perception.axis_aligned_follower_node import plan_is_stable
from robot_perception.axis_aligned_follower_node import segment_heading
from robot_perception.axis_aligned_follower_node import validate_axis_aligned_waypoints


def make_waypoints():
    return [
        {'role': 'start', 'x': -0.9015, 'y': -0.845},
        {'role': 'cp1', 'x': -0.39, 'y': -0.845},
        {'role': 'lane_entry', 'x': -0.39, 'y': -0.115},
        {'role': 'cp2', 'x': 0.20, 'y': -0.115},
    ]


def test_path_segments_have_axis_aligned_headings():
    waypoints = validate_axis_aligned_waypoints(make_waypoints())
    headings = [
        segment_heading(previous, target)
        for previous, target in zip(waypoints, waypoints[1:])
    ]

    assert headings == [0.0, math.pi / 2.0, 0.0]


def test_diagonal_segment_is_rejected():
    waypoints = make_waypoints()
    waypoints[1]['y'] = -0.80

    with pytest.raises(ValueError, match='axis-aligned'):
        validate_axis_aligned_waypoints(waypoints)


def test_linear_speed_decreases_near_waypoint():
    far_speed = calculate_linear_speed(0.30, 0.015, 0.20, 0.02, 0.06)
    near_speed = calculate_linear_speed(0.10, 0.015, 0.20, 0.02, 0.06)
    stop_speed = calculate_linear_speed(0.01, 0.015, 0.20, 0.02, 0.06)

    assert math.isclose(far_speed, 0.06)
    assert math.isclose(near_speed, 0.03)
    assert math.isclose(stop_speed, 0.0)


def test_turn_speed_is_limited_and_preserves_direction():
    left_speed = calculate_turn_speed(1.0, 1.2, 0.08, 0.30)
    right_speed = calculate_turn_speed(-0.01, 1.2, 0.08, 0.30)

    assert math.isclose(left_speed, 0.30)
    assert math.isclose(right_speed, -0.08)


def test_cp1_commissioning_ignores_downstream_lane_changes():
    first_plan = validate_axis_aligned_waypoints(make_waypoints())
    second_plan = make_waypoints()
    second_plan[2]['y'] = 0.10
    second_plan[3]['y'] = 0.10
    second_plan = validate_axis_aligned_waypoints(second_plan)

    assert execution_plan_signature(first_plan, True) == (
        execution_plan_signature(second_plan, True)
    )
    assert execution_plan_signature(first_plan, False) != (
        execution_plan_signature(second_plan, False)
    )


def test_heading_must_be_accurate_and_stopped_before_driving():
    assert heading_is_settled(0.01, 0.01, 0.02, 0.02) is True
    assert heading_is_settled(0.03, 0.01, 0.02, 0.02) is False
    assert heading_is_settled(0.01, 0.03, 0.02, 0.02) is False


def test_plan_must_remain_unchanged_for_stable_duration():
    assert plan_is_stable(10.59, 10.0, 0.6) is False
    assert plan_is_stable(10.60, 10.0, 0.6) is True
    assert plan_is_stable(10.60, None, 0.6) is False
