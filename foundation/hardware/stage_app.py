#!/usr/bin/env python3
"""Stage the reviewed app delta only. No install, service call or process launch."""
from pathlib import Path, PurePosixPath
from datetime import datetime, timezone
import argparse
import hashlib
import re
import json
import shutil
import tarfile



def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_path(root, relative):
    name = PurePosixPath(relative)
    if name.is_absolute() or '..' in name.parts or str(name) != relative or not name.parts:
        raise ValueError('Invalid selected path: ' + relative)
    target = root
    for part in name.parts:
        target = target / part
        if target.is_symlink():
            raise ValueError('Selected path traverses symlink: ' + relative)
    if not target.resolve().is_relative_to(root.resolve()):
        raise ValueError('Selected path escapes checkout: ' + relative)
    return target


def verify_selected(root, entries):
    for entry in entries:
        path = checked_path(root, entry['path'])
        if not path.is_file() or sha(path) != entry['sha256']:
            raise ValueError('Selected file missing or different: ' + str(path))


def entries_by_path(entries):
    if not isinstance(entries, list) or not entries:
        raise ValueError('Selected files must be a nonempty list')
    result = {}
    for entry in entries:
        name = entry['path']
        if name in result or not re.fullmatch('[0-9a-f]{64}', entry['sha256']):
            raise ValueError('Duplicate selected path or malformed hash')
        result[name] = entry
    return result


def stage(manifest_path, expected_manifest_sha256, archive):
    manifest_bytes = manifest_path.read_bytes()
    if not re.fullmatch('[0-9a-f]{64}', expected_manifest_sha256) or hashlib.sha256(manifest_bytes).hexdigest() != expected_manifest_sha256:
        raise ValueError('Input manifest hash mismatch')
    manifest = json.loads(manifest_bytes)
    if sha(archive) != manifest['archive']['sha256']:
        raise ValueError('Input archive hash mismatch')
    OLD, NEW = Path(manifest['baseline_remote_path']), Path(manifest['destination_remote_path'])
    if not OLD.is_absolute() or not NEW.is_absolute() or NEW.resolve().is_relative_to(OLD.resolve()):
        raise ValueError('Release paths must be absolute and destination outside baseline')
    if not OLD.is_dir() or OLD.is_symlink():
        raise ValueError('Preserved baseline checkout is unavailable')
    if NEW.exists() or NEW.is_symlink():
        raise ValueError('Destination exists; inspect it, do not overwrite or retry automatically')
    baseline = manifest['required_baseline_files']
    final = manifest['final_selected_files']
    before, after = entries_by_path(baseline), entries_by_path(final)
    dist_prefix = 'gemini_pupper/web/dist/'
    if {n for n in before if not n.startswith(dist_prefix)} - set(after):
        raise ValueError('Final selected identities omit preserved baseline files')
    required = {'foundation/hardware/floor_stock/shutdown_coordinator.py', 'foundation/hardware/lab6-session.yaml'}
    if not required <= set(before):
        raise ValueError('Required combined-checkout helpers missing from baseline manifest')
    verify_selected(OLD, baseline)  # Every precondition is checked before cloning.
    for name in ('APP-SELECTED-SHA256SUMS', 'app-stage-manifest.json', 'app-stage-receipt.json'):
        checked_path(OLD, name)  # Metadata writes must not follow inherited symlinks.
    dist_before = checked_path(OLD, 'gemini_pupper/web/dist')
    if not dist_before.is_dir():
        raise ValueError('Baseline browser dist directory is unavailable')
    if manifest['verification_file'].get('path') != 'SHA256SUMS':
        raise ValueError('Verification member must be SHA256SUMS')
    expected = entries_by_path(manifest['payload_files'])
    for name, entry in expected.items():
        parts = PurePosixPath(name).parts
        maintained = {
            'foundation/hardware/' + file for file in ('stage_app.py', 'persistent_session.py',
            'lab6_session.launch.py')}
        unchanged_urdf = (name == 'foundation/hardware/floor_stock/baseline-expanded.urdf' and
                          entry['sha256'] == '4858c01f6ce722455b2c9fa554e158bef2215bc1c3e75b2b005fdf9b89aaed44')
        follower = name.startswith(('follow_me_pupper/robot/', 'follow_me_pupper/lab/')) and (
            name.endswith('.py') or name in {'follow_me_pupper/robot/coco.txt',
            'follow_me_pupper/robot/camera_params.yaml', 'follow_me_pupper/robot/yolov8n_seg.hef'})
        if (name.startswith('gemini_pupper/tricks/') or (not name.startswith('gemini_pupper/') and name not in maintained and not follower and not unchanged_urdf) or
                PurePosixPath(name).suffix in {'.so', '.node', '.ko', '.a', '.onnx', '.pt', '.pth', '.pkl'} or
                entry.get('mode') not in (0o644, 0o755) or any(
                part.startswith('.env') or part in {'node_modules', '__pycache__', '.git', 'api_key', 'credentials.json'}
                or part.endswith(('.pem', '.key')) for part in parts)):
            raise ValueError('Nonapplication or credential/cache payload path refused: ' + name)
        if name not in after or after[name]['sha256'] != entry['sha256']:
            raise ValueError('Payload absent from final selected identities: ' + name)
    for name, entry in after.items():
        source = expected.get(name) or before.get(name)
        if source is None or entry['sha256'] != source['sha256']:
            raise ValueError('Final identity does not match baseline/overlay: ' + name)
        if name.startswith(dist_prefix) and name not in expected:
            raise ValueError('Final dist member absent from complete replacement payload: ' + name)
    if 'gemini_pupper/web/dist/index.html' not in expected:
        raise ValueError('Complete browser dist including index.html is required')
    expected['SHA256SUMS'] = {**manifest['verification_file'], 'mode': 0o644}
    with tarfile.open(archive, 'r:gz') as tf:
        members = tf.getmembers()
        if len(members) != len(expected) or {m.name for m in members} != set(expected):
            raise ValueError('Unexpected, missing or duplicate archive members')
        for member in members:
            checked_path(OLD, member.name)  # Also rejects existing baseline symlink parents.
            entry = expected[member.name]
            target = checked_path(OLD, member.name)
            if target.is_dir() or any(parent.is_file() for parent in target.parents if parent != OLD):
                raise ValueError('Payload path collides with an existing directory/file parent')
            if not member.isfile() or member.mode != entry['mode']:
                raise ValueError('Nonregular member or unexpected mode: ' + member.name)
            if 'bytes' in entry and member.size != entry['bytes']:
                raise ValueError('Member size mismatch: ' + member.name)
            with tf.extractfile(member) as stream:
                content = stream.read()
                if hashlib.sha256(content).hexdigest() != entry['sha256']:
                    raise ValueError('Member hash mismatch: ' + member.name)
                if re.search(rb'AIza[0-9A-Za-z_-]{35}|-----BEGIN [A-Z ]*PRIVATE KEY-----', content):
                    raise ValueError('Credential marker in payload; content withheld')
        # Only now create a separate checkout. copytree preserves unselected
        # inherited symlinks; selected paths above cannot traverse them.
        shutil.copytree(OLD, NEW, symlinks=True)
        dist = checked_path(NEW, 'gemini_pupper/web/dist')
        if not dist.is_dir():
            raise ValueError('Copied dist directory is unavailable')
        shutil.rmtree(dist)  # Only the new copy's fully replaced browser build.
        for member in members:
            target = checked_path(NEW, member.name)
            target.parent.mkdir(parents=True, exist_ok=True)
            with tf.extractfile(member) as source, target.open('wb') as dest:
                shutil.copyfileobj(source, dest)
            target.chmod(member.mode)
    verify_selected(NEW, [{'path': name, 'sha256': entry['sha256']} for name, entry in expected.items()])
    verify_selected(NEW, final)
    actual_dist = {str(p.relative_to(NEW)) for p in dist.rglob('*') if p.is_file() or p.is_symlink()}
    expected_dist = {name for name in expected if name.startswith('gemini_pupper/web/dist/')}
    if actual_dist != expected_dist:
        raise ValueError('Staged dist is not the exact complete payload')
    verify_selected(OLD, baseline)
    # OLD retains its immutable receipts. These generic metadata names in NEW
    # describe this stage; dated inherited evidence keeps its existing names.
    checked_path(NEW, 'APP-SELECTED-SHA256SUMS').write_text(''.join(e['sha256'] + '  ' + e['path'] + '\n' for e in final))
    checked_path(NEW, 'app-stage-manifest.json').write_bytes(manifest_bytes)
    receipt = {'staged_utc': datetime.now(timezone.utc).isoformat(), 'baseline_preserved': str(OLD),
               'new': str(NEW), 'archive_sha256': manifest['archive']['sha256'], 'manifest_sha256': expected_manifest_sha256,
               'stage_script_sha256': sha(Path(__file__)), 'verified_archive_members': len(expected),
               'verified_baseline_files': len(baseline), 'verified_final_selected_files': len(final),
               'native_driver_policy_or_gains_changed': False, 'started': False,
               'scope': 'Separate inherited checkout plus reviewed application overlay; no install/build/start/Stop/profile/homing action.'}
    checked_path(NEW, 'app-stage-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(stage(args.manifest, args.manifest_sha256, args.archive), indent=2))


if __name__ == '__main__':
    main()
