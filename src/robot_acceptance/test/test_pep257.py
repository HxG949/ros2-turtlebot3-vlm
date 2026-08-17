"""Run the repository-standard ament pep257 check."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check Python docstring style."""
    assert main(argv=['.', 'test']) == 0
