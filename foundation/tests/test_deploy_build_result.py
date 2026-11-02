"""Require fresh complete package outcomes before the installer claims a build.

Only disposable fixture shell scripts run; no ROS, colcon, installation, network
or robot access. A final successful command deliberately masks a simulated build
failure just as the course build.sh can mask colcon with local_setup.bash.
"""
import importlib.util
import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def installer():
    spec = importlib.util.spec_from_file_location('deploy_build_result', ROOT / 'pupper_gait_deploy/install.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ended(package, rc=0):
    return f"[0.123456] ({package}) JobEnded: {{'identifier': {package!r}, 'rc': {rc!r}}}"


def fixture_workspace(tmp_path, events=None, *, stale=False, colcon_exit=0, exit_code=0, log_mtime=None):
    workspace = tmp_path / 'ros2_ws'
    workspace.mkdir()
    script = '#!/bin/bash\n'
    if stale:
        log = workspace / 'log/latest_build/events.log'
        log.parent.mkdir(parents=True)
        log.write_text(events + '\n')
        os.utime(log, (log_mtime or 1, log_mtime or 1))
    elif events is not None:
        source = workspace / 'fixture-events'
        source.write_text(events + '\n')
        if log_mtime is not None:
            os.utime(source, (log_mtime, log_mtime))
        script += "mkdir -p log/latest_build\ncp -p fixture-events log/latest_build/events.log\n"
    script += f'# Model the build status, then the potentially masking final source.\n(exit {colcon_exit})\n'
    script += f'exit {exit_code}\n'
    (workspace / 'build.sh').write_text(script)
    return tmp_path


@pytest.mark.parametrize('events,stale', [
    (None, False),
    ('not a colcon events log', False),
    (ended('unrelated_package'), False),
    (ended('neural_controller'), False),
    (ended('joy_utils'), False),
    (ended('neural_controller') + '\n' + ended('joy_utils'), True),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobEnded: {'identifier': 'joy_utils', 'rc': 'zero'}", False),
    (ended('neural_controller') + '\n' + ended('joy_utils', .5), False),
    (ended('neural_controller') + '\n' + ended('joy_utils', False), False),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobEnded: {'identifier': 'other_package', 'rc': 0}", False),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobAborted: {'identifier': 'joy_utils'}", False),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobEnded: {'identifier': 'joy_utils',", False),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobEnded: ['joy_utils', 0]", False),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobEnded: {'identifier': 'joy_utils'}", False),
    (ended('neural_controller') + "\n[0.2] (joy_utils) JobEnded: {[]: 0}", False),
])
def test_rebuild_refuses_unverified_required_package_results(tmp_path, capsys, events, stale):
    repo = fixture_workspace(tmp_path, events, stale=stale, colcon_exit=17)
    assert installer().rebuild(repo, dry_run=False) is False
    output = capsys.readouterr().out
    assert 'Build succeeded.' not in output


@pytest.mark.parametrize('failed', ['neural_controller', 'joy_utils', 'unrelated_package'])
def test_rebuild_catches_masked_colcon_failure(tmp_path, capsys, failed):
    packages = {'neural_controller', 'joy_utils', failed}
    events = '\n'.join(ended(package, 17 if package == failed else 0) for package in sorted(packages))
    assert installer().rebuild(fixture_workspace(tmp_path, events, colcon_exit=17), dry_run=False) is False
    output = capsys.readouterr().out
    assert 'Build FAILED for:' in output and failed in output


def test_fresh_complete_success_keeps_standard_path(tmp_path, capsys):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils'), ended('unrelated_package')])
    assert installer().rebuild(fixture_workspace(tmp_path, events), dry_run=False) is True
    output = capsys.readouterr().out
    assert 'Verified successful build results for neural_controller and joy_utils' in output
    assert 'Build succeeded.' not in output


@pytest.mark.parametrize('line', [
    '[0.3] (neural_controller) JobEnded',
    "(neural_controller) JobEnded: {'identifier': 'neural_controller', 'rc': 1}",
])
def test_malformed_jobended_line_cannot_hide_behind_valid_successes(tmp_path, capsys, line):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils'), line])
    assert installer().rebuild(fixture_workspace(tmp_path, events), dry_run=False) is False
    assert 'UNVERIFIED' in capsys.readouterr().out


def test_literal_recursion_error_is_unverified(tmp_path, monkeypatch, capsys):
    module = installer()
    def too_deep(_payload):
        raise RecursionError('malformed nested literal')
    monkeypatch.setattr(module.ast, 'literal_eval', too_deep)
    events = '\n'.join([ended('neural_controller'), ended('joy_utils')])
    assert module.rebuild(fixture_workspace(tmp_path, events), dry_run=False) is False
    assert 'UNVERIFIED' in capsys.readouterr().out


def test_stdout_jobended_text_is_not_an_event(tmp_path):
    spoof = ended('joy_utils')
    output = f"[0.2] (neural_controller) StdoutLine: {{'line': {spoof!r}}}"
    assert installer().rebuild(fixture_workspace(tmp_path, ended('neural_controller') + '\n' + output), dry_run=False) is False


def test_valid_stdout_jobended_text_does_not_poison_actual_results(tmp_path):
    output = "[0.2] (neural_controller) StdoutLine: {'line': ' JobEnded is only text'}"
    events = '\n'.join([ended('neural_controller'), ended('joy_utils'), output])
    assert installer().rebuild(fixture_workspace(tmp_path, events), dry_run=False) is True


def test_future_dated_unchanged_log_cannot_mask_failure(tmp_path, capsys):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils')])
    repo = fixture_workspace(tmp_path, events, stale=True, log_mtime=4102444800, colcon_exit=17)
    assert installer().rebuild(repo, dry_run=False) is False
    assert 'UNVERIFIED' in capsys.readouterr().out


def test_new_log_does_not_depend_on_clock_or_timestamp_granularity(tmp_path):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils')])
    assert installer().rebuild(fixture_workspace(tmp_path, events, log_mtime=1), dry_run=False) is True


def test_changed_contents_at_same_path_inode_and_timestamp_are_current(tmp_path):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils')])
    repo = fixture_workspace(tmp_path, events, log_mtime=1)
    log = repo / 'ros2_ws/log/latest_build/events.log'
    log.parent.mkdir(parents=True)
    log.write_text(ended('neural_controller', 17) + '\n')
    os.utime(log, (1, 1))
    before = log.stat()
    assert installer().rebuild(repo, dry_run=False) is True
    after = log.stat()
    assert (after.st_dev, after.st_ino, after.st_mtime_ns) == (before.st_dev, before.st_ino, before.st_mtime_ns)


def test_identical_in_place_rewrite_is_conservatively_unverified(tmp_path):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils')])
    repo = fixture_workspace(tmp_path, events, log_mtime=1)
    log = repo / 'ros2_ws/log/latest_build/events.log'
    log.parent.mkdir(parents=True)
    log.write_text(events + '\n')
    assert installer().rebuild(repo, dry_run=False) is False


def test_new_colcon_directory_with_identical_contents_is_current(tmp_path):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils')])
    repo = fixture_workspace(tmp_path, events, log_mtime=1)
    workspace = repo / 'ros2_ws'
    old = workspace / 'log/build_previous/events.log'
    old.parent.mkdir(parents=True)
    old.write_text(events + '\n')
    os.utime(old, (1, 1))
    latest = workspace / 'log/latest_build'
    latest.symlink_to('build_previous', target_is_directory=True)
    (workspace / 'build.sh').write_text(
        '#!/bin/bash\nmkdir log/build_current\n'
        'cp -p fixture-events log/build_current/events.log\n'
        'rm log/latest_build\nln -s build_current log/latest_build\nexit 0\n')
    assert installer().rebuild(repo, dry_run=False) is True
    assert (latest / 'events.log').read_bytes() == old.read_bytes()
    assert (latest / 'events.log').stat().st_mtime_ns == old.stat().st_mtime_ns


@pytest.mark.parametrize('appended,expected', [
    ("[0.2] (neural_controller) StderrLine: {'line': 'build could not start'}", False),
    (ended('neural_controller'), False),
    (ended('neural_controller') + '\n' + ended('joy_utils'), True),
])
def test_same_inode_append_requires_new_required_outcomes(tmp_path, appended, expected):
    repo = fixture_workspace(tmp_path, appended, colcon_exit=0 if expected else 17)
    workspace = repo / 'ros2_ws'
    log = workspace / 'log/latest_build/events.log'
    log.parent.mkdir(parents=True)
    log.write_text(ended('neural_controller') + '\n' + ended('joy_utils') + '\n')
    before = log.stat()
    script = workspace / 'build.sh'
    script.write_text(script.read_text().replace(
        'cp -p fixture-events log/latest_build/events.log',
        'cat fixture-events >> log/latest_build/events.log'))
    assert installer().rebuild(repo, dry_run=False) is expected
    assert log.stat().st_ino == before.st_ino


def test_only_required_package_outcomes_are_claimed(tmp_path, capsys):
    events = '\n'.join([ended('neural_controller'), ended('joy_utils'),
                        "[0.2] (unfinished_package) JobStarted: {'identifier': 'unfinished_package'}"])
    assert installer().rebuild(fixture_workspace(tmp_path, events), dry_run=False) is True
    output = capsys.readouterr().out
    assert 'Verified successful build results for neural_controller and joy_utils' in output
    assert 'Build succeeded.' not in output


def test_nonzero_build_script_exit_still_fails_with_successful_events(tmp_path):
    events = ended('neural_controller') + '\n' + ended('joy_utils')
    assert installer().rebuild(fixture_workspace(tmp_path, events, exit_code=23), dry_run=False) is False


def test_dry_run_never_executes_build_or_requires_results(tmp_path):
    repo = fixture_workspace(tmp_path)
    script = repo / 'ros2_ws/build.sh'
    script.write_text('#!/bin/bash\ntouch should-not-exist\nexit 19\n')
    before = script.read_bytes()
    assert installer().rebuild(repo, dry_run=True) is True
    assert script.read_bytes() == before
    assert not (repo / 'ros2_ws/should-not-exist').exists()
    assert not (repo / 'ros2_ws/log').exists()


def test_missing_build_script_still_refuses(tmp_path):
    assert installer().rebuild(tmp_path, dry_run=False) is False
