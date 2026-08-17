"""Tests for atomic artifact creation and reproducibility metadata."""

from datetime import datetime, timezone
import json

import pytest

from robot_acceptance.artifacts import ArtifactRun
from robot_acceptance.artifacts import environment_snapshot
from robot_acceptance.artifacts import make_run_id


def test_run_id_is_utc_and_staging_is_exclusive(tmp_path):
    """Use a UTC ID and refuse a second owner of the same staging path."""
    instant = datetime(2026, 8, 17, 12, 30, tzinfo=timezone.utc)
    run_id = make_run_id(instant)
    assert run_id == '20260817T123000.000000Z'

    ArtifactRun(tmp_path, run_id)
    with pytest.raises(FileExistsError):
        ArtifactRun(tmp_path, run_id)


def test_json_and_jsonl_are_strict_atomic_and_never_overwritten(tmp_path):
    """Reject NaN and existing destinations while leaving no temp files."""
    artifacts = ArtifactRun(tmp_path, 'strict')
    artifacts.write_json('result.json', {'value': 1.0})
    artifacts.write_jsonl('events.jsonl', [{'event': 'created'}])

    with pytest.raises(FileExistsError):
        artifacts.write_json('result.json', {'value': 2.0})
    with pytest.raises(ValueError):
        artifacts.write_json('nan.json', {'value': float('nan')})

    result = json.loads(artifacts.path('result.json').read_text())
    assert result == {'value': 1.0}
    assert not list(artifacts.staging_path.glob('*.tmp.*'))


def test_finalize_refuses_existing_run_and_renames_atomically(tmp_path):
    """Publish a completed run once and never replace an existing run."""
    first = ArtifactRun(tmp_path, 'first')
    first.write_text('proof.txt', 'complete\n')
    final = first.finalize()
    assert (final / 'proof.txt').read_text() == 'complete\n'
    with pytest.raises(RuntimeError):
        first.finalize()

    second = ArtifactRun(tmp_path, 'second')
    second.final_path.mkdir()
    with pytest.raises(FileExistsError):
        second.finalize()


def test_environment_manifest_excludes_unrelated_secrets():
    """Capture relevant runtime context without copying arbitrary secrets."""
    snapshot = environment_snapshot({
        'ROS_DOMAIN_ID': '7',
        'PATH': '/bin',
        'TOKEN': 'secret',
    })
    assert snapshot == {'PATH': '/bin', 'ROS_DOMAIN_ID': '7'}
