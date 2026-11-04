"""Check the actual status CLI in a new Git checkout with no machine-local assets."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


SCRIPT = Path('foundation/scripts/session_status.py')


@pytest.fixture
def checkout(tmp_path):
    (tmp_path / SCRIPT.parent).mkdir(parents=True)
    shutil.copyfile(Path(__file__).resolve().parents[2] / SCRIPT, tmp_path / SCRIPT)
    results = tmp_path / 'foundation/results'
    results.mkdir()
    (results / 'report.json').write_text('{}')
    (results / 'history.json').write_text('{}')
    state = {
        'schema': 'pupper_current_session_v2', 'branch': 'main', 'legacy_branches': ['pupper-foundation'],
        'current_report': 'foundation/results/report.json', 'evidence': {},
        'historical_recovery': {'path': 'foundation/results/history.json', 'sha256': hashlib.sha256(b'{}').hexdigest()},
        'environment': {'python': '.venv-api/bin/python', 'frozen_hardware_inputs': 'captures.tar',
                        'frozen_hardware_sha256': hashlib.sha256(b'captured input').hexdigest()},
        'mode': 'test', 'activity': {'automation': 'none'}, 'hardware_session': {'status': 'closed'},
        'next': [], 'updated_utc': '',
    }
    (results / 'interruption-recovery.json').write_text(json.dumps(state))
    for args in (['init', '-q', '-b', 'main'], ['add', '.'],
                 ['-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '--no-gpg-sign', '-qm', 'test']):
        subprocess.run(['git', '-c', 'core.hooksPath=/dev/null', *args], cwd=tmp_path, check=True)
    return tmp_path


def status(root, *flags):
    result = subprocess.run([sys.executable, str(SCRIPT), '--json', *flags], cwd=root, capture_output=True, text=True)
    return result.returncode, json.loads(result.stdout)


def test_clean_clone_can_read_handoff_before_strict_local_verification(checkout):
    code, report = status(checkout)
    assert code == 0 and report['local_references_ok']
    assert len(report['missing_local']) == 2 and not report['verify_inputs_requested']
    code, report = status(checkout, '--verify-inputs')
    assert code == 1 and len(report['errors']) == 2
    (checkout / 'captures.tar').write_bytes(b'captured input')
    interpreter = checkout / '.venv-api/bin/python'
    interpreter.parent.mkdir(parents=True)
    interpreter.symlink_to(sys.executable)
    code, report = status(checkout, '--verify-inputs')
    assert code == 0 and not report['missing_local']
    (checkout / 'captures.tar').write_bytes(b'wrong capture')
    code, report = status(checkout, '--verify-inputs')
    assert code == 1 and not report['missing_local'] and 'checksum mismatch' in report['errors'][0]


def test_legacy_checkout_still_checks_tracked_reports_and_branch(checkout):
    subprocess.run(['git', 'switch', '-qc', 'pupper-foundation'], cwd=checkout, check=True)
    assert status(checkout)[0] == 0
    (checkout / 'foundation/results/report.json').unlink()
    code, report = status(checkout)
    assert code == 1 and any('report.json' in error for error in report['errors'])


def test_diverged_feature_branch_can_use_handoff_with_local_or_remote_main(checkout):
    def git(*args):
        subprocess.run(['git', '-c', 'core.hooksPath=/dev/null', '-c', 'user.name=Test',
                        '-c', 'user.email=test@example.invalid', *args], cwd=checkout, check=True)
    git('branch', 'soccer/replay')
    git('commit', '--allow-empty', '--no-gpg-sign', '-qm', 'advance main')
    git('switch', '-q', 'soccer/replay')
    git('commit', '--allow-empty', '--no-gpg-sign', '-qm', 'advance replay')
    code, report = status(checkout)
    assert code == 0 and 'refs/heads/main' in report['branch_note']
    git('update-ref', 'refs/remotes/origin/main', 'main')
    git('branch', '-D', 'main')
    code, report = status(checkout)
    assert code == 0 and 'refs/remotes/origin/main' in report['branch_note']
    assert status(checkout, '--verify-inputs')[0] == 1  # still missing local assets


def test_detached_and_unrelated_histories_do_not_pass_readiness(checkout):
    subprocess.run(['git', 'switch', '--detach', '-q'], cwd=checkout, check=True)
    code, report = status(checkout)
    assert code == 1 and 'Detached HEAD' in report['errors'][0]
    # Keep the tracked handoff in the tree, but give this branch an unrelated root.
    subprocess.run(['git', 'checkout', '-q', '--orphan', 'separate'], cwd=checkout, check=True)
    subprocess.run(['git', '-c', 'core.hooksPath=/dev/null', '-c', 'user.name=Test',
                    '-c', 'user.email=test@example.invalid', 'commit', '--no-gpg-sign', '-qm',
                    'independent history'], cwd=checkout, check=True)
    code, report = status(checkout)
    assert code == 1 and 'Cannot establish shared history' in report['errors'][0]
