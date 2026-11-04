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
    subprocess.run(['git', 'switch', '-qc', 'unrelated'], cwd=checkout, check=True)
    assert status(checkout)[0] == 1
    subprocess.run(['git', 'switch', '-q', 'main'], cwd=checkout, check=True)
    (checkout / 'foundation/results/report.json').unlink()
    code, report = status(checkout)
    assert code == 1 and any('report.json' in error for error in report['errors'])
