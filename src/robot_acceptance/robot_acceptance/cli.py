"""Command-line interface for conservative P0 acceptance orchestration."""

import argparse
from pathlib import Path
import sys

from robot_acceptance.runner import AcceptanceRunner
from robot_acceptance.runner import RunOptions


def build_parser():
    """Build the public runner parser with motion disabled by default."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--enable-motion',
        action='store_true',
        help='explicitly allow production navigation to command motion',
    )
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--gui',
        action='store_true',
        help='show Gazebo; headless is the default',
    )
    parser.add_argument(
        '--artifacts-root',
        type=Path,
        default=Path('artifacts'),
    )
    return parser


def run_main(argv=None):
    """Run one acceptance attempt and return its stable result exit code."""
    arguments_list = list(sys.argv[1:] if argv is None else argv)
    arguments = build_parser().parse_args(arguments_list)
    options = RunOptions(
        enable_motion=arguments.enable_motion,
        seed=arguments.seed,
        headless=not arguments.gui,
        artifacts_root=arguments.artifacts_root,
    )
    exit_code, artifact_directory = AcceptanceRunner(
        options,
        argv=['robot_acceptance_run', *arguments_list],
    ).run()
    print(f'Acceptance artifacts: {artifact_directory}')
    return exit_code
