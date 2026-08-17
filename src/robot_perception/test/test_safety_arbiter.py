import math

from robot_perception.safety_arbiter_node import SafetyArbiterLogic


def make_logic():
    return SafetyArbiterLogic(
        desired_timeout=0.25,
        safety_timeout=0.6,
        startup_confirmation_duration=0.5,
    )


def safe_status():
    return {'valid': True, 'emergency_stop': False}


def arm_logic(logic, start=0.0):
    for step in range(7):
        now = start + step * 0.1
        result = logic.evaluate(
            now,
            True,
            (0.0, 0.0),
            now,
            True,
            safe_status(),
            now,
        )
    assert result == ('ACTIVE', 'command_allowed', 0.0, 0.0)


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
    arm_logic(logic)
    state, reason, linear_x, angular_z = logic.evaluate(
        0.7,
        True,
        (0.06, -0.1),
        0.7,
        True,
        safe_status(),
        0.7,
    )

    assert state == 'ACTIVE'
    assert reason == 'command_allowed'
    assert math.isclose(linear_x, 0.06)
    assert math.isclose(angular_z, -0.1)
    assert logic.armed is True


def test_desired_timeout_latches_and_does_not_auto_resume():
    logic = make_logic()
    arm_logic(logic)
    logic.evaluate(
        0.7,
        True,
        (0.06, 0.0),
        0.7,
        True,
        safe_status(),
        0.7,
    )
    timed_out = logic.evaluate(
        1.0,
        True,
        (0.06, 0.0),
        0.7,
        True,
        safe_status(),
        1.0,
    )
    recovered = logic.evaluate(
        1.01,
        True,
        (0.06, 0.0),
        1.01,
        True,
        safe_status(),
        1.01,
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
    arm_logic(logic)
    logic.evaluate(
        0.7,
        True,
        (0.04, 0.0),
        0.7,
        True,
        safe_status(),
        0.7,
    )
    emergency = logic.evaluate(
        0.8,
        True,
        (0.04, 0.0),
        0.8,
        True,
        {'valid': True, 'emergency_stop': True},
        0.8,
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


def test_startup_requires_continuously_fresh_inputs():
    logic = make_logic()
    confirming = logic.evaluate(
        0.0,
        True,
        (0.0, 0.0),
        0.0,
        True,
        safe_status(),
        0.0,
    )
    stale = logic.evaluate(
        0.3,
        True,
        (0.0, 0.0),
        0.0,
        True,
        safe_status(),
        0.3,
    )
    fresh_again = logic.evaluate(
        1.0,
        True,
        (0.0, 0.0),
        1.0,
        True,
        safe_status(),
        1.0,
    )

    assert confirming == (
        'WAITING',
        'confirming_safe_startup',
        0.0,
        0.0,
    )
    assert stale == ('WAITING', 'desired_velocity_timeout', 0.0, 0.0)
    assert fresh_again == confirming
    assert logic.armed is False
    assert logic.fault_reason is None


def test_fresh_nonzero_stream_can_arm_after_initial_zero_command():
    logic = make_logic()
    result = None
    for step in range(7):
        now = step * 0.1
        result = logic.evaluate(
            now,
            True,
            (0.0, 0.0) if step == 0 else (0.04, 0.0),
            now,
            True,
            safe_status(),
            now,
        )

    assert result == ('ACTIVE', 'command_allowed', 0.04, 0.0)
    assert logic.armed is True
