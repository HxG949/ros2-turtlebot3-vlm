"""Independent process-group supervision with captured evidence logs."""

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import time


@dataclass
class ManagedProcess:
    """Track one child process and its immutable launch arguments."""

    name: str
    argv: tuple
    process: subprocess.Popen
    stdout_path: Path
    stderr_path: Path
    stdout_stream: object
    stderr_stream: object
    returncode: int | None = None

    @property
    def pid(self):
        """Return the operating-system process ID."""
        return self.process.pid

    def record(self, base_path=None):
        """Return JSON-compatible process lifecycle data."""
        stdout_path = self.stdout_path
        stderr_path = self.stderr_path
        if base_path is not None:
            stdout_path = stdout_path.relative_to(base_path)
            stderr_path = stderr_path.relative_to(base_path)
        return {
            'name': self.name,
            'argv': list(self.argv),
            'pid': self.pid,
            'returncode': self.returncode,
            'stdout': str(stdout_path),
            'stderr': str(stderr_path),
        }


class ProcessSupervisor:
    """Launch children in dedicated sessions and stop their process groups."""

    def __init__(self, logs_path, interrupt_timeout=5.0,
                 terminate_timeout=3.0, kill_timeout=1.0):
        self.logs_path = Path(logs_path)
        self.logs_path.mkdir(parents=True, exist_ok=True)
        self.interrupt_timeout = interrupt_timeout
        self.terminate_timeout = terminate_timeout
        self.kill_timeout = kill_timeout
        self.processes = {}

    def start(self, name, argv, environment=None, cwd=None):
        """Start a non-shell child in a new process session."""
        if name in self.processes:
            raise ValueError(f'process name already registered: {name}')
        if isinstance(argv, (str, bytes)) or not argv:
            raise TypeError('argv must be a non-empty sequence, not a string')
        command = tuple(str(value) for value in argv)
        stdout_path = self.logs_path / f'{name}.stdout.log'
        stderr_path = self.logs_path / f'{name}.stderr.log'
        stdout_stream = stdout_path.open('xb')
        try:
            stderr_stream = stderr_path.open('xb')
        except Exception:
            stdout_stream.close()
            raise
        try:
            process = subprocess.Popen(
                command,
                cwd=cwd,
                env=environment,
                shell=False,
                start_new_session=True,
                stdout=stdout_stream,
                stderr=stderr_stream,
            )
        except Exception:
            stdout_stream.close()
            stderr_stream.close()
            raise
        managed = ManagedProcess(
            name=name,
            argv=command,
            process=process,
            stdout_path=stdout_path,
            stderr_path=stderr_path,
            stdout_stream=stdout_stream,
            stderr_stream=stderr_stream,
        )
        self.processes[name] = managed
        return managed

    def poll(self, name):
        """Poll one child and retain its return code when it exits."""
        managed = self.processes[name]
        returncode = managed.process.poll()
        if returncode is not None:
            managed.returncode = returncode
            self._close_streams(managed)
        return returncode

    def stop(self, name):
        """Stop a process group using SIGINT, SIGTERM, then SIGKILL."""
        managed = self.processes.get(name)
        if managed is None:
            return None
        if managed.process.poll() is None:
            for sig, timeout in (
                (signal.SIGINT, self.interrupt_timeout),
                (signal.SIGTERM, self.terminate_timeout),
                (signal.SIGKILL, self.kill_timeout),
            ):
                self._signal_group(managed, sig)
                if self._wait(managed, timeout):
                    break
        managed.returncode = managed.process.poll()
        self._close_streams(managed)
        return managed.returncode

    def stop_many(self, names):
        """Stop process groups in the exact supplied order."""
        for name in names:
            self.stop(name)

    def records(self):
        """Return stable process records in launch order."""
        return [
            managed.record(self.logs_path.parent)
            for managed in self.processes.values()
        ]

    @staticmethod
    def _signal_group(managed, sig):
        try:
            os.killpg(managed.pid, sig)
        except ProcessLookupError:
            pass

    @staticmethod
    def _wait(managed, timeout):
        try:
            managed.process.wait(timeout=max(0.0, timeout))
            return True
        except subprocess.TimeoutExpired:
            return False

    @staticmethod
    def _close_streams(managed):
        for stream in (managed.stdout_stream, managed.stderr_stream):
            if not stream.closed:
                stream.close()

    def wait_for_exit(self, name, timeout):
        """Wait for a child for tests or bounded orchestration."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            returncode = self.poll(name)
            if returncode is not None:
                return returncode
            time.sleep(0.01)
        raise TimeoutError(f'process did not exit: {name}')
