"""Backup/transfer correctness without SSH, ROS or real robot paths."""
import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/robot_artifact_backup.py'
spec = importlib.util.spec_from_file_location('robot_artifact_backup', SCRIPT)
backup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(backup)


@pytest.fixture
def capture(tmp_path):
    source = tmp_path / 'approved-policy.json'
    source.write_text('{"weights":[1,2,3],"version":"existing robot baseline"}\n')
    source.chmod(0o751)
    plan = tmp_path / 'reviewed-plan.json'
    plan.write_text(json.dumps({'schema': backup.SCHEMA, 'purpose': 'robot-baseline',
                               'files': [{'label': 'approved-policy', 'path': str(source), 'role': 'policy'}]}))
    return source, plan, tmp_path / 'captured'


def test_exact_bytes_and_original_mode_preserved_without_executable_copy(capture):
    source, plan, output = capture
    before = source.read_bytes()
    result = backup.create_artifact(plan, output)
    assert result['verified'] and result['identity_pinned'] and result['files'] == 1
    assert source.read_bytes() == before and stat.S_IMODE(source.stat().st_mode) == 0o751
    assert (output / 'files/approved-policy').read_bytes() == before
    assert stat.S_IMODE((output / 'files/approved-policy').stat().st_mode) == 0o600
    assert json.loads((output / 'manifest.json').read_text())['files'][0]['source_mode'] == 0o751
    assert backup.verify_artifact(output, expected_manifest_sha256=result['manifest_sha256']) == result
    assert backup.verify_artifact(output)['identity_pinned'] is False


def test_existing_destination_is_never_overwritten(capture):
    _, plan, output = capture
    backup.create_artifact(plan, output)
    before = (output / 'manifest.json').read_bytes()
    with pytest.raises(FileExistsError): backup.create_artifact(plan, output)
    assert (output / 'manifest.json').read_bytes() == before


@pytest.mark.parametrize('fault', ['changed_file', 'missing_file', 'extra_file', 'incomplete', 'path_escape', 'manifest_identity'])
def test_corrupt_or_incomplete_transfer_cannot_verify(capture, fault):
    _, plan, output = capture
    original = backup.create_artifact(plan, output)
    if fault == 'changed_file': (output / 'files/approved-policy').write_text('corrupted')
    elif fault == 'missing_file': (output / 'files/approved-policy').unlink()
    elif fault == 'extra_file': (output / 'files/extra').write_text('extra')
    elif fault == 'incomplete': (output / 'INCOMPLETE').touch()
    else:
        path = output / 'manifest.json'; data = json.loads(path.read_text())
        if fault == 'path_escape': data['files'][0]['stored_path'] = '../approved-policy.json'
        else: data['files'][0]['source_mode'] = 0o777
        path.write_text(json.dumps(data))
    with pytest.raises((ValueError, OSError)):
        backup.verify_artifact(output, expected_manifest_sha256=original['manifest_sha256'] if fault == 'manifest_identity' else None)


@pytest.mark.parametrize('kind', ['file_link', 'ancestor_link', 'fifo', 'sensitive_path', 'sensitive_contents'])
def test_unreviewed_link_special_file_or_credential_is_refused_without_leaking_value(capture, tmp_path, kind):
    source, plan, output = capture
    credential = 'private-credential-value-that-must-not-be-printed'
    selected = source
    if kind == 'file_link':
        selected = tmp_path / 'linked-policy.json'; selected.symlink_to(source)
    elif kind == 'ancestor_link':
        linked = tmp_path / 'linked'; linked.symlink_to(tmp_path, target_is_directory=True)
        selected = linked / source.name
    elif kind == 'fifo':
        selected = tmp_path / 'policy.pipe'; os.mkfifo(selected)
    elif kind == 'sensitive_path':
        selected = tmp_path / '.env'; selected.write_text(credential)
    else: source.write_text(json.dumps({'api_key': credential}))
    data = json.loads(plan.read_text()); data['files'][0]['path'] = str(selected); plan.write_text(json.dumps(data))
    with pytest.raises((ValueError, OSError)) as error: backup.create_artifact(plan, output)
    assert credential not in str(error.value)
    if output.exists():
        assert (output / 'INCOMPLETE').exists()
        with pytest.raises(ValueError): backup.verify_artifact(output)


def test_secret_pattern_across_read_boundary_is_refused(capture):
    source, plan, output = capture
    source.write_bytes(b'x' * (1024 * 1024 - 10) + b'"api_key":"a-long-private-secret-value"')
    with pytest.raises(ValueError, match='credential'): backup.create_artifact(plan, output)
    assert (output / 'INCOMPLETE').exists()


def test_source_mutation_during_capture_is_refused(capture):
    source, _, _ = capture
    class MutatingCopy:
        def write(self, block):
            with source.open('ab') as stream: stream.write(b'changed')
    # Only mutate once; otherwise the deliberately growing source would never end.
    target = MutatingCopy()
    original = target.write
    def once(block):
        original(block); target.write = lambda block: None
    target.write = once
    with pytest.raises(ValueError, match='changed during capture'):
        backup.fingerprint_file(source, destination=target)


@pytest.mark.parametrize('mutation', ['duplicate_label', 'duplicate_source', 'relative_source', 'traversal_label', 'directory_source'])
def test_plan_requires_exact_unique_reviewed_regular_files(capture, mutation):
    source, plan, output = capture
    data = json.loads(plan.read_text())
    if mutation == 'duplicate_label': data['files'].append(dict(data['files'][0], path=str(source.parent / 'other')))
    elif mutation == 'duplicate_source': data['files'].append(dict(data['files'][0], label='other'))
    elif mutation == 'relative_source': data['files'][0]['path'] = source.name
    elif mutation == 'traversal_label': data['files'][0]['label'] = '../escape'
    else: data['files'][0]['path'] = str(source.parent)
    plan.write_text(json.dumps(data))
    with pytest.raises((ValueError, OSError)): backup.create_artifact(plan, output)


def test_manifest_symlink_and_duplicate_json_keys_are_refused(capture, tmp_path):
    _, plan, output = capture
    backup.create_artifact(plan, output)
    original = (output / 'manifest.json').read_bytes()
    target = tmp_path / 'manifest-copy.json'; target.write_bytes(original)
    (output / 'manifest.json').unlink(); (output / 'manifest.json').symlink_to(target)
    with pytest.raises(OSError): backup.verify_artifact(output)
    plan.write_text('{"schema":"x","schema":"y"}')
    with pytest.raises(ValueError, match='duplicate JSON key'): backup.read_json(plan)


def test_stage_transfer_is_explicit_and_not_a_policy_promotion(capture):
    _, plan, output = capture
    data = json.loads(plan.read_text()); data['purpose'] = 'staged-transfer'
    data['files'][0]['expected_sha256'] = backup.fingerprint_file(data['files'][0]['path'])['sha256']
    plan.write_text(json.dumps(data))
    result = backup.create_artifact(plan, output)
    assert result['purpose'] == 'staged-transfer'
    assert 'not hardware readiness' in result['scope']


def test_inventory_uses_explicit_fixture_paths_and_refuses_linked_policy(tmp_path):
    repo = tmp_path / 'baseline'; repo.mkdir()
    mono = tmp_path / 'ros-workspace'; mono.mkdir()
    gemini = tmp_path / 'old-gemini'; gemini.mkdir()
    follower = tmp_path / 'follower'; follower.mkdir()
    deploy = tmp_path / 'deploy'; deploy.mkdir()
    tricks = gemini / 'tricks/example'; tricks.mkdir(parents=True)
    source = tricks / 'policy.json'; source.write_text('{"weights":[1]}')
    secret = tmp_path / '.env'; secret.write_text('DO_NOT_READ_ME')
    (tricks / 'linked.json').symlink_to(secret)
    launch = mono / 'ros2_ws/install/neural_controller/share/neural_controller/launch'; launch.mkdir(parents=True)
    (launch / 'launch.py').write_text('# approved launch fixture\n')
    library = mono / 'ros2_ws/install/neural_controller/lib'; library.mkdir(parents=True)
    (library / 'libneural_controller.so').write_bytes(b'ELF fixture, not executable')
    args = ['bash', str(SCRIPT.with_name('robot_inventory.sh')), '--repo-root', str(repo), '--monorepo-dir', str(mono),
            '--gemini-dir', str(gemini), '--follower-dir', str(follower), '--deployment-dir', str(deploy)]
    result = subprocess.run(args, capture_output=True, text=True, timeout=30, check=True)
    data = json.loads(result.stdout)
    assert data['ros']['sampled'] is False
    assert all(data['components'][name]['path_source'] == 'explicit argument' for name in ('monorepo','gemini','follower','deployment'))
    assert data['components']['monorepo']['active_installation_verified'] is False
    files = {Path(item['path']).name: item for item in data['file_hashes']}
    assert 'sha256' in files['policy.json'] and 'sha256' in files['launch.py'] and 'sha256' in files['libneural_controller.so']
    assert 'error' in files['linked.json'] and 'sha256' not in files['linked.json']
    assert 'DO_NOT_READ_ME' not in result.stdout


def test_atomic_source_path_replacement_cannot_capture_stale_inode(capture):
    source, _, _ = capture
    class ReplacingCopy:
        def write(self, block):
            replacement = source.with_name('replacement.json')
            replacement.write_bytes(b'new installed file')
            replacement.replace(source)
    with pytest.raises(ValueError, match='source (changed|path was replaced)'):
        backup.fingerprint_file(source, destination=ReplacingCopy())


@pytest.mark.parametrize('fault', ['missing_pin', 'wrong_pin'])
def test_stage_transfer_refuses_missing_or_mismatched_reviewed_source_hash(capture, fault):
    _, plan, output = capture
    data = json.loads(plan.read_text()); data['purpose'] = 'staged-transfer'
    if fault == 'wrong_pin': data['files'][0]['expected_sha256'] = '0' * 64
    plan.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='expected'):
        backup.create_artifact(plan, output)
    if output.exists():
        assert (output / 'INCOMPLETE').exists()
        with pytest.raises(ValueError): backup.verify_artifact(output)
