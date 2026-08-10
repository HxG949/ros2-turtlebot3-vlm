import math

from robot_perception.safety_arbiter_node import SafetyArbiterLogic


def make_logic():
    return SafetyArbiterLogic(
        desired_timeout=0.25,
        safety_timeout=0.6,
    )


def safe_status():
    return {'valid': True, 'emergency_stop': False}


def test_disabled_arbiter_always_returns_zero():
    logic = make_logic()
    result = logic.evaluate(
        1.0,
        False,
        (0.06, 0.1),
        1.0,
        True,
        safe_status(),
        1.0,
    )

    assert result == ('DISABLED', 'motion_disabled', 0.0, 0.0)


def test_safe_fresh_command_is_forwarded():
    logic = make_logic()
    logic.evaluate(
        0.9,
        True,
        (0.0, 0.0),
        0.9,
        True,
        safe_status(),
        0.9,
    )
    state, reason, linear_x, angular_z = logic.evaluate(
        1.0,
        True,
        (0.06, -0.1),
        1.0,
        True,
        safe_status(),
        1.0,
    )

    assert state == 'ACTIVE'
    assert reason == 'command_allowed'
    assert math.isclose(linear_x, 0.06)
    assert math.isclose(angular_z, -0.1)
    assert logic.armed is True


def test_desired_timeout_latches_and_does_not_auto_resume():
    logic = make_logic()
    logic.evaluate(
        0.9,
        True,
        (0.0, 0.0),
        0.9,
        True,
        safe_status(),
        0.9,
    )
    logic.evaluate(
        1.0,
        True,
        (0.06, 0.0),
        1.0,
        True,
        safe_status(),
        1.0,
    )
    timed_out = logic.evaluate(
        1.30,
        True,
        (0.06, 0.0),
        1.0,
        True,
        safe_status(),
        1.30,
    )
    recovered = logic.evaluate(
        1.31,
        True,
        (0.06, 0.0),
        1.31,
        True,
        safe_status(),
        1.31,
    )

    assert timed_out == (
        'LATCHED',
        'desired_velocity_timeout',
        0.0,
        0.0,
    )
    assert recovered == timed_out


def test_emergency_stop_latches_after_arming():
    logic = make_logic()
    logic.evaluate(
        0.9,
        True,
        (0.0, 0.0),
        0.9,
        True,
        safe_status(),
        0.9,
    )
    logic.evaluate(
        1.0,
        True,
        (0.04, 0.0),
        1.0,
        True,
        safe_status(),
        1.0,
    )
    emergency = logic.evaluate(
        1.1,
        True,
        (0.04, 0.0),
        1.1,
        True,
        {'valid': True, 'emergency_stop': True},
        1.1,
    )

    assert emergency == ('LATCHED', 'emergency_stop', 0.0, 0.0)


def test_incomplete_safety_status_cannot_allow_motion():
    logic = make_logic()
    result = logic.evaluate(
        1.0,
        True,
        (0.0, 0.0),
        1.0,
        True,
        {'valid': True},
        1.0,
    )

    assert result == ('BLOCKED', 'safety_invalid', 0.0, 0.0)
    assert logic.armed is False


def test_non_boolean_emergency_status_cannot_allow_motion():
    logic = make_logic()
    result = logic.evaluate(
        1.0,
        True,
        (0.0, 0.0),
        1.0,
        True,
        {'valid': True, 'emergency_stop': 'false'},
        1.0,
    )

    assert result == ('BLOCKED', 'safety_invalid', 0.0, 0.0)


def test_nonzero_command_cannot_arm_a_fresh_arbiter():
    logic = make_logic()
    result = logic.evaluate(
        1.0,
        True,
        (0.04, 0.0),
        1.0,
        True,
        safe_status(),
        1.0,
    )

    assert result == (
        'BLOCKED',
        'waiting_for_zero_command',
        0.0,
        0.0,
    )
    assert logic.armed is False


def test_zero_command_is_remembered_before_safety_arrives():
    logic = make_logic()
    logic.record_desired_velocity((0.0, 0.0), True)
    result = logic.evaluate(
        1.0,
        True,
        (0.04, 0.0),
        1.0,
        True,
        safe_status(),
        1.0,
    )

    assert result == ('ACTIVE', 'command_allowed', 0.04, 0.0)
    assert logic.zero_command_seen is True
    assert logic.armed is True
