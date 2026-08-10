from robot_perception.cmd_vel_watchdog_node import HeartbeatWatchdogLogic


def test_watchdog_is_idle_before_first_heartbeat():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
    )

    assert logic.evaluate(1.0) == (
        False,
        'waiting_for_arbiter_heartbeat',
    )


def test_missing_initial_heartbeat_latches_emergency_takeover():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
    )
    logic.evaluate(1.0)

    assert logic.evaluate(1.51) == (
        True,
        'arbiter_startup_timeout',
    )


def test_heartbeat_timeout_latches_emergency_takeover():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
    )
    logic.record_heartbeat(1.0)

    assert logic.evaluate(1.20) == (
        False,
        'arbiter_heartbeat_fresh',
    )
    timed_out = logic.evaluate(1.26)
    logic.record_heartbeat(1.27)
    recovered = logic.evaluate(1.27)

    assert timed_out == (True, 'arbiter_heartbeat_timeout')
    assert recovered == timed_out


def test_invalid_status_latches_after_monitoring_starts():
    logic = HeartbeatWatchdogLogic(
        heartbeat_timeout=0.25,
        startup_timeout=0.5,
    )
    logic.record_heartbeat(1.0)
    logic.record_invalid_status()

    assert logic.evaluate(1.1) == (
        True,
        'arbiter_status_invalid',
    )
