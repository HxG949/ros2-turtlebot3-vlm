"""Tests for non-shell child process supervision and escalation."""

import os
from pathlib import Path
import signal
import sys
import time

from robot_acceptance.process_supervisor import ProcessSupervisor


def test_short_python_process_records_argv_pid_logs_and_returncode(tmp_path):
    """Supervise a harmless Python child without invoking ROS or a shell."""
    supervisor = ProcessSupervisor(tmp_path)
    child = supervisor.start(
        'short',
        [sys.executable, '-c', 'print("supervised")'],
    )
    assert child.argv[0] == sys.executable
    assert child.pid > 0
    assert os.getpgid(child.pid) == child.pid
    assert supervisor.wait_for_exit('short', 2.0) == 0
    assert child.stdout_path.read_text().strip() == 'supervised'
    assert supervisor.records()[0]['returncode'] == 0


def test_stop_sends_sigint_to_process_group_first(tmp_path):
    """Allow a cooperative recorder-like process to close on SIGINT."""
    script = (
        'import signal,time,sys; '
        'signal.signal(signal.SIGINT, lambda *_: sys.exit(23)); '
        'time.sleep(30)'
    )
    supervisor = ProcessSupervisor(
        tmp_path,
        interrupt_timeout=1.0,
        terminate_timeout=0.2,
        kill_timeout=0.2,
    )
    supervisor.start('recorder', [sys.executable, '-c', script])
    time.sleep(0.05)
    assert supervisor.stop('recorder') == 23


def test_start_passes_shell_false_and_new_session(monkeypatch, tmp_path):
    """Lock down Popen flags that prevent shell injection and shared signals."""
    captured = {}

    class FakeProcess:
        pid = 12345

        def poll(self):
            return 0

    def fake_popen(argv, **kwargs):
        captured['argv'] = argv
        captured.update(kwargs)
        return FakeProcess()

    monkeypatch.setattr('subprocess.Popen', fake_popen)
    supervisor = ProcessSupervisor(tmp_path)
    supervisor.start('safe', ['program', 'argument with spaces'])
    assert captured['argv'] == ('program', 'argument with spaces')
    assert captured['shell'] is False
    assert captured['start_new_session'] is True


def test_signal_constants_use_expected_escalation_order():
    """Document the required graceful-to-forceful signal sequence."""
    assert (signal.SIGINT, signal.SIGTERM, signal.SIGKILL) == (2, 15, 9)
    assert Path(sys.executable).exists()
