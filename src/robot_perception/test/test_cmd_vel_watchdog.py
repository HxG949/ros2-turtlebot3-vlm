from robot_perception.cmd_vel_watchdog_node import HeartbeatWatchdogLogic


def test_watchdog_is_idle_before_first_heartbeat():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
        startup_confirmation_duration=0.2,
    )

    assert logic.evaluate(1.0) == (
        False,
        'waiting_for_arbiter_heartbeat',
    )


def test_missing_initial_heartbeat_latches_emergency_takeover():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
        startup_confirmation_duration=0.2,
    )
    logic.evaluate(1.0)

    assert logic.evaluate(1.51) == (
        True,
        'arbiter_startup_timeout',
    )


def test_startup_requires_heartbeats_spanning_confirmation_duration():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=1.0,
        startup_confirmation_duration=0.2,
    )
    logic.evaluate(1.0)
    logic.record_heartbeat(1.0)

    assert logic.evaluate(1.30) == (
        False,
        'confirming_arbiter_heartbeat',
    )
    logic.record_heartbeat(1.21)
    assert logic.evaluate(1.21) == (False, 'arbiter_heartbeat_fresh')


def test_heartbeat_timeout_latches_after_startup_confirmation():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=1.0,
        startup_confirmation_duration=0.2,
    )
    logic.evaluate(1.0)
    logic.record_heartbeat(1.0)
    logic.record_heartbeat(1.21)

    timed_out = logic.evaluate(1.47)
    logic.record_heartbeat(1.48)
    recovered = logic.evaluate(1.48)

    assert timed_out == (True, 'arbiter_heartbeat_timeout')
    assert recovered == timed_out


def test_invalid_status_latches_after_monitoring_starts():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
        startup_confirmation_duration=0.2,
    )
    logic.record_heartbeat(1.0)
    logic.record_heartbeat(1.21)
    logic.record_invalid_status()

    assert logic.evaluate(1.22) == (
        True,
        'arbiter_status_invalid',
    )
