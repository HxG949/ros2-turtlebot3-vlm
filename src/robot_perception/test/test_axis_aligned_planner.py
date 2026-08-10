import math

from robot_perception.axis_aligned_planner_node import PlanningGeometry
from robot_perception.axis_aligned_planner_node import is_planning_obstacle_range
from robot_perception.axis_aligned_planner_node import is_robot_self_point
from robot_perception.axis_aligned_planner_node import plan_axis_aligned_path


def make_geometry():
    return PlanningGeometry(
        start_x=-0.9015,
        start_y=-0.845,
        cp1_x=-0.39,
        cp1_y=-0.845,
        cp2_x=0.20,
        field_y_min=-1.05,
        field_y_max=1.05,
        robot_radius=0.105,
        safety_margin=0.070,
        candidate_spacing=0.02,
    )


def test_minimum_range_returns_are_excluded_from_planning_obstacles():
    assert not is_planning_obstacle_range(0.12, 0.12, 0.015)
    assert not is_planning_obstacle_range(0.135, 0.12, 0.015)
    assert is_planning_obstacle_range(0.136, 0.12, 0.015)


def test_points_inside_robot_envelope_are_excluded_from_planning():
    assert is_robot_self_point(0.125, 0.0, 0.105, 0.02)
    assert not is_robot_self_point(0.126, 0.0, 0.105, 0.02)


def test_current_lane_is_used_when_clear():
    result = plan_axis_aligned_path([], make_geometry())

    assert result['valid'] is True
    assert result['reason'] == 'current_lane_safe'
    assert math.isclose(result['selected_y'], -0.845)
    assert [point['role'] for point in result['waypoints']] == [
        'start',
        'cp1',
        'cp2',
    ]
    start, cp1, cp2 = result['waypoints']
    assert start['y'] == cp1['y'] == cp2['y']


def test_blocked_current_lane_selects_axis_aligned_detour():
    result = plan_axis_aligned_path(
        [(-0.10, -0.845)],
        make_geometry(),
    )

    assert result['valid'] is True
    assert result['reason'] == 'nearest_left_lane_selected'
    assert math.isclose(result['selected_y'], -0.655)
    assert result['minimum_clearance'] >= 0.070
    assert [point['role'] for point in result['waypoints']] == [
        'start',
        'cp1',
        'lane_entry',
        'cp2',
    ]

    start, cp1, lane_entry, cp2 = result['waypoints']
    assert start['y'] == cp1['y']
    assert cp1['x'] == lane_entry['x']
    assert lane_entry['y'] == cp2['y']


def test_new_lane_uses_selection_buffer():
    result = plan_axis_aligned_path(
        [(-0.10, -0.845)],
        make_geometry(),
        lane_selection_buffer=0.05,
    )

    assert result['valid'] is True
    assert math.isclose(result['selected_y'], -0.615)
    assert result['minimum_clearance'] >= 0.120


def test_no_safe_lane_returns_invalid_plan():
    obstacle_points = [
        (-0.10, -0.875 + index * 0.10)
        for index in range(18)
    ]
    result = plan_axis_aligned_path(obstacle_points, make_geometry())

    assert result == {
        'valid': False,
        'reason': 'no_safe_lane',
        'selected_y': None,
        'minimum_clearance': None,
        'waypoints': [],
    }


def test_safe_committed_lane_is_held_during_small_scan_change():
    geometry = make_geometry()
    first_result = plan_axis_aligned_path(
        [(-0.10, -0.845)],
        geometry,
    )
    second_result = plan_axis_aligned_path(
        [(-0.10, -0.835)],
        geometry,
        committed_y=first_result['selected_y'],
    )

    assert second_result['valid'] is True
    assert second_result['reason'] == 'committed_lane_held'
    assert second_result['selected_y'] == first_result['selected_y']


def test_unsafe_committed_lane_is_replaced_immediately():
    result = plan_axis_aligned_path(
        [(-0.10, -0.845)],
        make_geometry(),
        committed_y=-0.845,
    )

    assert result['valid'] is True
    assert result['reason'] == 'unsafe_committed_lane_replaced'
    assert math.isclose(result['selected_y'], -0.655)
