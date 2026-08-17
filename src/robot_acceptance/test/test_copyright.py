"""Retain the repository-standard deferred copyright check."""

from ament_copyright.main import main
import pytest


@pytest.mark.skip(reason='Copyright headers are deferred in this repository.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Check copyright headers when the repository enables the check."""
    assert main(argv=['.', 'test']) == 0
