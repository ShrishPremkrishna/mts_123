"""A failed web rebuild must not report old assets as the newly built app."""
import os
from pathlib import Path
import subprocess

import pytest

SETUP = Path(__file__).resolve().parents[1] / 'robot/setup.sh'


@pytest.mark.parametrize('build_exit', [0, 17])
def test_setup_build_preserves_npm_failure_even_with_existing_dist(tmp_path, build_exit):
    web = tmp_path / 'web'
    (web / 'dist').mkdir(parents=True)
    (web / 'dist/index.html').write_text('previous build')
    commands = tmp_path / 'commands'
    commands.mkdir()
    npm = commands / 'npm'
    npm.write_text('#!/bin/sh\nif [ "$1" = run ]; then\n  echo "build attempt"\n  exit "$BUILD_EXIT"\nfi\n')
    npm.chmod(0o755)
    source = SETUP.read_text()
    block = source[source.index('echo "[2/4]'):source.index('echo "[3/4]')]
    # Execute the real build section only: no Node downloads, user setup or key access.
    result = subprocess.run(['/bin/bash', '-euo', 'pipefail', '-c', block],
        env={**os.environ, 'REPO': str(tmp_path), 'BUILD_EXIT': str(build_exit),
             'PATH': str(commands) + os.pathsep + os.environ['PATH']},
        text=True, capture_output=True)
    assert result.returncode == build_exit
    assert ('built web/dist/' in result.stdout) == (build_exit == 0)
    assert (web / 'dist/index.html').read_text() == 'previous build'
