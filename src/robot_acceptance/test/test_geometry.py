"""Tests for parking-envelope and CP2 geometry."""

from dataclasses import FrozenInstanceError
import math

import pytest

from robot_acceptance.geometry import check_cp2_crossing
from robot_acceptance.geometry import normalize_angle
from robot_acceptance.geometry import parking_margins
from robot_acceptance.geometry import ParkingSpace
from robot_acceptance.geometry import Pose2D
from robot_acceptance.geometry import pose_errors


def baseline_space(yaw=0.0):
    """Return the frozen baseline inner parking space."""
    return ParkingSpace(Pose2D(0.8015, 0.0, yaw), 0.297, 0.210)


def test_normalize_angle_has_stable_pi_boundary_and_rejects_non_finite():
    """Use the documented half-open range at both pi representations."""
    assert normalize_angle(math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(-math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    with pytest.raises(ValueError, match='finite'):
        normalize_angle(math.inf)


def test_centered_baseline_margins_match_specification():
    """Reproduce the frozen 43.5 mm and 16 mm theoretical margins."""
    pose = Pose2D(0.8015, 0.0, math.pi)
    margins = parking_margins(pose, 0.210, 0.178,
                              baseline_space(math.pi))

    assert margins.front == pytest.approx(0.0435)
    assert margins.rear == pytest.approx(0.0435)
    assert margins.left == pytest.approx(0.016)
    assert margins.right == pytest.approx(0.016)
    assert margins.minimum == pytest.approx(0.016)
    assert margins.contained()


def test_rotated_footprint_uses_actual_four_corners():
    """A 90-degree robot swaps projected length and width extents."""
    pose = Pose2D(0.8015, 0.0, math.pi / 2.0)
    margins = parking_margins(pose, 0.210, 0.178, baseline_space())

    assert margins.front == pytest.approx((0.297 - 0.178) / 2.0)
    assert margins.left == pytest.approx((0.210 - 0.210) / 2.0)
    assert margins.minimum == pytest.approx(0.0, abs=1e-15)


def test_touching_edge_passes_but_physical_crossing_fails():
    """Apply only the 1e-9 floating-point epsilon at an exact edge."""
    touching = parking_margins(
        Pose2D(0.8015, 0.016, 0.0), 0.210, 0.178, baseline_space()
    )
    crossing = parking_margins(
        Pose2D(0.8015, 0.016000002, 0.0), 0.210, 0.178,
        baseline_space(),
    )

    assert touching.left == pytest.approx(0.0, abs=1e-15)
    assert touching.contained()
    assert crossing.left < -1e-9
    assert not crossing.contained()


def test_pose_errors_use_distance_and_shortest_pi_boundary():
    """Measure Euclidean position and wrap yaw through the pi boundary."""
    errors = pose_errors(
        Pose2D(0.01, 0.0, math.pi - 0.01),
        Pose2D(0.0, 0.0, -math.pi + 0.01),
    )
    assert errors.position == pytest.approx(0.01)
    assert errors.yaw == pytest.approx(0.02)


def test_cp2_accepts_directed_sample_at_all_inclusive_thresholds():
    """Accept threshold equality while requiring positive route progress."""
    result = check_cp2_crossing(
        Pose2D(0.38, 0.03, 0.02),
        Pose2D(0.385, 0.03, 0.02),
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.4, 0.0, 0.0),
        Pose2D(0.8, 0.0, 0.0),
        0.02,
    )
    assert result.passed
    assert result.longitudinal_error == pytest.approx(0.015)
    assert result.cross_track_error == pytest.approx(0.03)


@pytest.mark.parametrize(
    'sample,angular_speed',
    [
        (Pose2D(0.3849, 0.0, 0.0), 0.0),
        (Pose2D(0.4, 0.0301, 0.0), 0.0),
        (Pose2D(0.4, 0.0, 0.0201), 0.0),
        (Pose2D(0.4, 0.0, 0.0), 0.0201),
    ],
)
def test_cp2_rejects_each_out_of_threshold_measurement(sample, angular_speed):
    """Reject independently exceeded CP2 limits."""
    result = check_cp2_crossing(
        Pose2D(0.38, 0.0, 0.0), sample,
        Pose2D(0.0, 0.0, 0.0), Pose2D(0.4, 0.0, 0.0),
        Pose2D(0.8, 0.0, 0.0), angular_speed,
    )
    assert not result.passed


def test_cp2_rejects_reverse_motion_and_non_collinear_plan():
    """Make crossing direction and frozen plan geometry explicit."""
    reverse = check_cp2_crossing(
        Pose2D(0.41, 0.0, 0.0), Pose2D(0.4, 0.0, 0.0),
        Pose2D(0.0, 0.0, 0.0), Pose2D(0.4, 0.0, 0.0),
        Pose2D(0.8, 0.0, 0.0), 0.0,
    )
    assert not reverse.passed
    with pytest.raises(ValueError, match='collinear'):
        check_cp2_crossing(
            Pose2D(0.39, 0.0, 0.0), Pose2D(0.4, 0.0, 0.0),
            Pose2D(0.0, 0.0, 0.0), Pose2D(0.4, 0.01, 0.0),
            Pose2D(0.8, 0.0, 0.0), 0.0,
        )


def test_geometry_models_are_immutable():
    """Prevent mutation of measurements after evidence capture."""
    pose = Pose2D(0.0, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        pose.x = 1.0
