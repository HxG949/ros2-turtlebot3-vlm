from robot_perception.lidar_safety_node import LidarSafetyNode


class ScanMetadata:
    ranges = [1.0]
    angle_min = 0.0
    angle_increment = 0.1
    range_min = 0.12
    range_max = 3.5


def test_sector_coverage_requires_minimum_valid_fraction():
    assert LidarSafetyNode.coverage_is_valid(8, 10, 0.8) is True
    assert LidarSafetyNode.coverage_is_valid(7, 10, 0.8) is False


def test_empty_sector_is_invalid():
    assert LidarSafetyNode.coverage_is_valid(0, 0, 0.8) is False


def test_front_emergency_distance_uses_strict_threshold():
    assert LidarSafetyNode.emergency_stop_required(
        0.174,
        True,
        0.175,
    ) is True
    assert LidarSafetyNode.emergency_stop_required(
        0.175,
        True,
        0.175,
    ) is False
    assert LidarSafetyNode.emergency_stop_required(
        1.0,
        False,
        0.175,
    ) is True


def test_non_finite_scan_metadata_is_invalid():
    message = ScanMetadata()
    message.range_max = float('nan')

    assert LidarSafetyNode.scan_metadata_is_valid(message) is False


def test_normal_scan_metadata_is_valid():
    assert LidarSafetyNode.scan_metadata_is_valid(ScanMetadata()) is True


def test_single_short_rotation_ray_is_treated_as_noise():
    distances = [0.30, 0.17, 0.30, 0.30]

    assert LidarSafetyNode.rotation_is_safe(
        distances,
        True,
        0.18,
        3,
    ) is True


def test_three_consecutive_short_rays_block_rotation():
    distances = [0.30, 0.17, 0.16, 0.17, 0.30]

    assert LidarSafetyNode.rotation_is_safe(
        distances,
        True,
        0.18,
        3,
    ) is False


def test_three_consecutive_unknown_rays_block_rotation():
    distances = [0.30, None, None, None, 0.30]

    assert LidarSafetyNode.rotation_is_safe(
        distances,
        True,
        0.18,
        3,
    ) is False


def test_invalid_scan_blocks_rotation():
    assert LidarSafetyNode.rotation_is_safe(
        [0.30, 0.30, 0.30],
        False,
        0.18,
        3,
    ) is False


def test_rotation_distance_uses_robot_center_not_laser_origin():
    distance = LidarSafetyNode.distance_from_robot_center(
        0.12,
        3.141592653589793,
        -0.032,
        0.0,
    )

    assert round(distance, 3) == 0.152


def test_rotation_requires_five_unsafe_scans_to_confirm():
    streak = 0
    for expected_streak in range(1, 5):
        streak, caution, confirmed = (
            LidarSafetyNode.update_rotation_confirmation(
                False,
                streak,
                5,
            )
        )
        assert streak == expected_streak
        assert caution is True
        assert confirmed is False

    streak, caution, confirmed = (
        LidarSafetyNode.update_rotation_confirmation(False, streak, 5)
    )
    assert streak == 5
    assert caution is False
    assert confirmed is True


def test_safe_rotation_scan_clears_unsafe_streak():
    assert LidarSafetyNode.update_rotation_confirmation(
        True,
        4,
        5,
    ) == (0, False, False)
