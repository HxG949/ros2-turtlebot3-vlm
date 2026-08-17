"""Atomic artifact-directory management for acceptance runs."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


ENVIRONMENT_NAMES = {
    'AMENT_PREFIX_PATH',
    'COLCON_PREFIX_PATH',
    'GAZEBO_MODEL_PATH',
    'PATH',
    'PYTHONPATH',
    'RMW_IMPLEMENTATION',
    'ROS_DOMAIN_ID',
    'ROS_LOCALHOST_ONLY',
    'ROS_VERSION',
    'TURTLEBOT3_MODEL',
}


def utc_timestamp(now=None):
    """Return a stable UTC timestamp suitable for JSON and run IDs."""
    value = now or datetime.now(timezone.utc)
    return value.astimezone(timezone.utc).isoformat().replace('+00:00', 'Z')


def make_run_id(now=None):
    """Build a filesystem-safe run ID from a UTC instant."""
    value = now or datetime.now(timezone.utc)
    return value.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')


def git_state(repository):
    """Return the repository commit and dirty state without changing git."""
    root = str(Path(repository).resolve())
    try:
        commit = subprocess.run(
            ['git', '-C', root, 'rev-parse', 'HEAD'],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            ['git', '-C', root, 'status', '--porcelain'],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return None, None
    return commit, bool(status)


def environment_snapshot(environment=None):
    """Capture ROS-relevant environment values without recording secrets."""
    source = os.environ if environment is None else environment
    return {
        name: source[name]
        for name in sorted(source)
        if name in ENVIRONMENT_NAMES or name.startswith('ROS_')
    }


class ArtifactRun:
    """Own one exclusive staging directory and atomically publish it."""

    def __init__(self, root, run_id=None):
        self.root = Path(root).expanduser().resolve()
        self.run_id = run_id or make_run_id()
        self.final_path = self.root / self.run_id
        self.staging_path = self.root / f'.{self.run_id}.inprogress'
        self.root.mkdir(parents=True, exist_ok=True)
        if self.final_path.exists():
            raise FileExistsError(f'artifact run already exists: {self.final_path}')
        self.staging_path.mkdir(mode=0o755, exist_ok=False)
        self.logs_path = self.staging_path / 'logs'
        self.logs_path.mkdir()
        self._finalized = False

    def path(self, relative):
        """Return a path inside staging and reject directory traversal."""
        result = (self.staging_path / relative).resolve()
        if self.staging_path != result and self.staging_path not in result.parents:
            raise ValueError('artifact path must remain inside the run directory')
        return result

    def write_json(self, relative, value):
        """Create one JSON artifact atomically without replacing a file."""
        payload = json.dumps(
            value,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        ) + '\n'
        self._write_once(relative, payload)

    def write_jsonl(self, relative, values):
        """Create a complete JSONL artifact atomically."""
        lines = [
            json.dumps(value, allow_nan=False, sort_keys=True)
            for value in values
        ]
        payload = ''.join(f'{line}\n' for line in lines)
        self._write_once(relative, payload)

    def write_text(self, relative, value):
        """Create one UTF-8 text artifact atomically."""
        self._write_once(relative, value)

    def _write_once(self, relative, payload):
        target = self.path(relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(
            f'.{target.name}.tmp.{os.getpid()}.{id(payload)}'
        )
        try:
            with temporary.open('x', encoding='utf-8', newline='') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)

    def manifest(self, argv, repository, started_at, ended_at=None):
        """Build the reproducibility manifest for this run."""
        commit, dirty = git_state(repository)
        return {
            'run_id': self.run_id,
            'started_at_utc': started_at,
            'ended_at_utc': ended_at,
            'git_commit': commit,
            'git_dirty': dirty,
            'argv': list(argv),
            'environment': environment_snapshot(),
            'platform': platform.platform(),
            'python': sys.version,
        }

    def finalize(self):
        """Atomically rename staging to its permanent, non-overwritten path."""
        if self._finalized:
            raise RuntimeError('artifact run was already finalized')
        if self.final_path.exists():
            raise FileExistsError(f'artifact run already exists: {self.final_path}')
        self.staging_path.rename(self.final_path)
        self._finalized = True
        return self.final_path
