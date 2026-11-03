#!/usr/bin/env python3
"""Package a frozen Pupper app delta for unchanged stage_app.py. Never builds, stages or deploys."""
from pathlib import Path, PurePosixPath
import argparse
import gzip
import hashlib
import io
import json
import os
import re
import stat
import subprocess
import tarfile

DIST = 'gemini_pupper/web/dist/'
TRICKS = 'gemini_pupper/tricks/'
STAGE_SCRIPT = 'foundation/hardware/stage_app.py'
HEX64, HEX40 = re.compile('[0-9a-f]{64}'), re.compile('[0-9a-f]{40}')
SECRET = re.compile(rb'AIza[0-9A-Za-z_-]{35}|-----BEGIN [A-Z ]*PRIVATE KEY-----')
REQUIRED_BASELINE = {'foundation/hardware/floor_stock/shutdown_coordinator.py', 'foundation/hardware/lab6-session.yaml'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def git_head(repo):
    """Read-only HEAD lookup."""
    return subprocess.run(['git', '-C', str(repo), 'rev-parse', '--verify', 'HEAD'],
                          check=True, capture_output=True, text=True).stdout.strip()


def safe_relative(relative):
    name = PurePosixPath(relative) if isinstance(relative, str) else None
    if name is None or name.is_absolute() or '..' in name.parts or str(name) != relative or not name.parts:
        raise ValueError('Invalid relative path: %r' % (relative,))
    return relative


def checked_path(root, relative):
    """Same rules as stage_app.checked_path: no traversal, no symlink component."""
    target = root
    for part in PurePosixPath(safe_relative(relative)).parts:
        target = target / part
        if target.is_symlink():
            raise ValueError('Path traverses symlink: ' + relative)
    if not target.resolve().is_relative_to(root.resolve()):
        raise ValueError('Path escapes checkout: ' + relative)
    return target


def read_regular(root, relative):
    path = checked_path(root, relative)
    if not path.is_file():
        raise ValueError('Missing or nonregular source: ' + relative)
    return path.read_bytes()


def pinned_entries(entries, label):
    if not isinstance(entries, list) or not entries:
        raise ValueError(label + ' must be a nonempty list')
    result = {}
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get('sha256'), str) or not HEX64.fullmatch(entry['sha256']):
            raise ValueError(label + ' entry malformed')
        name = safe_relative(entry.get('path'))
        if name in result:
            raise ValueError(label + ' duplicate or conflicting path: ' + name)
        result[name] = entry['sha256']
    return result


def stage_refuses(name, digest, mode=0o644):
    """Mirror of stage_app's payload path policy; must never be wider."""
    parts = PurePosixPath(name).parts
    maintained = {'foundation/hardware/' + f for f in ('stage_app.py', 'persistent_session.py', 'lab6_session.launch.py')}
    unchanged_urdf = (name == 'foundation/hardware/floor_stock/baseline-expanded.urdf' and
                      digest == '4858c01f6ce722455b2c9fa554e158bef2215bc1c3e75b2b005fdf9b89aaed44')
    follower = name.startswith(('follow_me_pupper/robot/', 'follow_me_pupper/lab/')) and (
        name.endswith('.py') or name in {'follow_me_pupper/robot/coco.txt',
        'follow_me_pupper/robot/camera_params.yaml', 'follow_me_pupper/robot/yolov8n_seg.hef'})
    return (name.startswith(TRICKS) or
            (not name.startswith('gemini_pupper/') and name not in maintained and not follower and not unchanged_urdf) or
            PurePosixPath(name).suffix in {'.so', '.node', '.ko', '.a', '.onnx', '.pt', '.pth', '.pkl'} or
            mode not in (0o644, 0o755) or any(
                part.startswith('.env') or part in {'node_modules', '__pycache__', '.git', 'api_key', 'credentials.json'}
                or part.endswith(('.pem', '.key')) for part in parts))


def enumerate_dist(repo):
    root = checked_path(repo, DIST.rstrip('/'))
    if not root.is_dir():
        raise ValueError('Current browser dist directory is unavailable')
    found = set()
    for current, dirs, files in os.walk(root):
        for name in dirs + files:
            path = Path(current) / name
            mode = path.lstat().st_mode
            if stat.S_ISLNK(mode):
                raise ValueError('Symlink in browser dist: ' + path.relative_to(repo).as_posix())
            if name in files:
                if not stat.S_ISREG(mode):
                    raise ValueError('Nonregular browser dist entry')
                found.add(path.relative_to(repo).as_posix())
    return found


def check_inputs(args):
    for value, label in ((args.baseline_manifest_sha256, 'baseline manifest'), (args.freeze_manifest_sha256, 'freeze manifest')):
        if not HEX64.fullmatch(value):
            raise ValueError('Malformed expected %s sha256' % label)
    if not HEX40.fullmatch(args.expected_head):
        raise ValueError('Expected HEAD must be a full 40-hex commit')
    if not re.fullmatch(r'index-[A-Za-z0-9_-]+\.js', args.client_bundle):
        raise ValueError('Expected one built client filename index-*.js')
    if not PurePosixPath(args.destination).is_absolute() or '..' in PurePosixPath(args.destination).parts:
        raise ValueError('Destination must be an absolute remote path')
    if args.output.exists() or args.output.is_symlink():
        raise ValueError('Output exists; refusing to overwrite')


def plan(args):
    """Validate everything and compute the package without writing anything."""
    check_inputs(args)
    repo = args.repo.resolve()
    if args.output.resolve().is_relative_to(repo):
        raise ValueError('Output must be outside the source checkout')
    if git_head(repo) != args.expected_head:
        raise ValueError('Checkout HEAD differs from expected head')
    baseline_bytes, freeze_bytes = args.baseline_manifest.read_bytes(), args.freeze_manifest.read_bytes()
    if sha(baseline_bytes) != args.baseline_manifest_sha256:
        raise ValueError('Baseline manifest hash mismatch')
    if sha(freeze_bytes) != args.freeze_manifest_sha256:
        raise ValueError('Freeze manifest hash mismatch')
    old, freeze = json.loads(baseline_bytes), json.loads(freeze_bytes)
    if 'source_checkout_head' in freeze and freeze['source_checkout_head'] != args.expected_head:
        raise ValueError('Freeze source_checkout_head differs from expected head')
    pins = pinned_entries(freeze.get('files'), 'Freeze files')
    current = {}
    for name, digest in pins.items():
        current[name] = read_regular(repo, name)
        if sha(current[name]) != digest:
            raise ValueError('Current source differs from final freeze: ' + name)

    baseline = pinned_entries(old['final_selected_files'], 'Baseline final_selected_files')
    if not REQUIRED_BASELINE <= set(baseline):
        raise ValueError('Baseline lacks required combined-checkout helpers')
    old_dest = old['destination_remote_path']
    if args.output.resolve().is_relative_to(Path(old_dest).resolve()):
        raise ValueError('Output must be outside the preserved baseline release')
    new_dest = PurePosixPath(args.destination)
    if new_dest == PurePosixPath(old_dest) or PurePosixPath(old_dest) in new_dest.parents:
        raise ValueError('Destination must be outside the baseline release')
    inherited = old['inherited_skill_files_unchanged']
    inherited_pins = pinned_entries(inherited, 'Inherited skill files')
    if any(baseline.get(n) != d for n, d in inherited_pins.items()):
        raise ValueError('Inherited skill identities differ from baseline selection')
    kept = {n: d for n, d in baseline.items() if n.startswith(TRICKS) or n in inherited_pins}

    adds = []
    for name in args.add_file:
        safe_relative(name)
        if name in adds or name in baseline:
            raise ValueError('Added file duplicated or already selected: ' + name)
        if name.startswith((DIST, TRICKS)) or name in inherited_pins or stage_refuses(name, pins.get(name)):
            raise ValueError('Added file outside permitted application payload: ' + name)
        if name not in pins:
            raise ValueError('Added file is not pinned by final freeze: ' + name)
        adds.append(name)

    runtime = sorted({n for n in baseline if not n.startswith(DIST) and n not in kept} | set(adds))
    unpinned = [n for n in runtime if n not in pins]
    if unpinned:
        raise ValueError('Selected runtime source not pinned by final freeze: ' + ', '.join(unpinned))
    changed = [n for n in runtime if baseline.get(n) != pins[n]]

    dist = enumerate_dist(repo)
    frozen_dist = {n for n in pins if n.startswith(DIST)}
    if dist != frozen_dist:
        raise ValueError('Current dist differs from frozen dist set (missing %d, unpinned %d)'
                         % (len(frozen_dist - dist), len(dist - frozen_dist)))
    index, bundle = DIST + 'index.html', DIST + 'assets/' + args.client_bundle
    if index not in dist or bundle not in dist or ('assets/' + args.client_bundle).encode() not in current[index]:
        raise ValueError('index.html does not reference an existing requested client bundle')

    payload = sorted(set(changed) | dist)
    for name in payload:
        if stage_refuses(name, pins[name]):
            raise ValueError('Policy/binary/credential payload refused: ' + name)
        if SECRET.search(current[name]):
            raise ValueError('Credential marker in payload; content withheld: ' + name)

    build = {n for n in old['current_browser_build_inputs'] if not n.startswith(DIST)}
    build |= {n for n in adds if n.startswith('gemini_pupper/web/')}
    if any(n not in pins for n in build):
        raise ValueError('Browser build input not pinned by final freeze')
    if STAGE_SCRIPT not in pins:
        raise ValueError(STAGE_SCRIPT + ' must be pinned by final freeze')

    final = {n: pins[n] for n in runtime} | {n: pins[n] for n in dist} | kept
    if {n for n in baseline if not n.startswith(DIST)} - set(final):
        raise ValueError('Final selection omits preserved baseline files')
    return dict(repo=repo, old=old, baseline_bytes=baseline_bytes, freeze=freeze, freeze_bytes=freeze_bytes,
                pins=pins, current=current, changed=changed, adds=adds, payload=payload, dist=dist,
                build={n: pins[n] for n in sorted(build)}, final=final, inherited=inherited)


def write_archive(path, members):
    """Deterministic gzip tar: fixed mtime/owner, 0644 regular members only."""
    raw = io.BytesIO()
    with tarfile.open(fileobj=raw, mode='w', format=tarfile.PAX_FORMAT) as tf:
        for name, data in members:
            info = tarfile.TarInfo(name)
            info.size, info.mode, info.mtime = len(data), 0o644, 0
            tf.addfile(info, io.BytesIO(data))
    with open(path, 'xb') as fh, gzip.GzipFile(fileobj=fh, mode='wb', mtime=0, filename='') as gz:
        gz.write(raw.getvalue())


def verify_archive(path, members):
    expected = dict(members)
    with tarfile.open(path, 'r:gz') as tf:
        found = tf.getmembers()
        if len(found) != len(expected) or {m.name for m in found} != set(expected):
            raise ValueError('Archive member mismatch')
        for m in found:
            if not m.isfile() or m.mode != 0o644 or tf.extractfile(m).read() != expected[m.name]:
                raise ValueError('Archive content mismatch: ' + m.name)


def package(args):
    p = plan(args)
    repo, pins, current, old = p['repo'], p['pins'], p['current'], p['old']
    out = args.output
    out.mkdir(exist_ok=False)  # Parent must exist; never replaces anything.
    payload_dir = out / 'payload'
    entries, members = [], []
    for name in p['payload']:
        data = current[name]
        target = payload_dir / name
        target.parent.mkdir(parents=True, exist_ok=True)
        with open(target, 'xb') as fh:
            fh.write(data)
        target.chmod(0o644)
        entries.append({'path': name, 'sha256': sha(data), 'bytes': len(data), 'mode': 0o644})
        members.append((name, data))
    sums = ''.join(e['sha256'] + '  ' + e['path'] + '\n' for e in entries).encode()
    (payload_dir / 'SHA256SUMS').write_bytes(sums)
    members.append(('SHA256SUMS', sums))
    archive = out / 'lab6-application.tar.gz'
    write_archive(archive, members)
    verify_archive(archive, members)
    archive_bytes = archive.read_bytes()
    freeze_sha = sha(p['freeze_bytes'])
    manifest = {
        'schema': old['schema'],
        'source_checkout_head': args.expected_head,
        'source_identity': 'Final freeze %s at HEAD %s; client %s. Packaged by package_app.py; no build.'
                           % (freeze_sha, args.expected_head, args.client_bundle),
        'baseline_remote_path': old['destination_remote_path'],
        'destination_remote_path': args.destination,
        'baseline_manifest_sha256': sha(p['baseline_bytes']),
        'required_baseline_files': old['final_selected_files'],
        'final_selected_files': [{'path': n, 'sha256': d} for n, d in sorted(p['final'].items())],
        'payload_files': entries,
        'verification_file': {'path': 'SHA256SUMS', 'sha256': sha(sums)},
        'archive': {'name': archive.name, 'sha256': sha(archive_bytes), 'bytes': len(archive_bytes),
                    'regular_members': len(members), 'symlinks': 0, 'strip_components': 0},
        'current_browser_build_inputs': p['build'],
        'selected_native_unchanged': old['selected_native_unchanged'],
        'inherited_skill_files_unchanged': p['inherited'],
        'compatibility': old.get('compatibility', {}),
        'revision': {'changed_baseline_selected_source': [n for n in p['changed'] if n not in p['adds']],
                     'added_files': p['adds'], 'final_freeze_sha256': freeze_sha,
                     'final_freeze_source_checkout_head': p['freeze'].get('source_checkout_head'),
                     'reviewed_source_pins': dict(sorted(pins.items())),
                     'native_imu_model_build_changes': False, 'hardware_validated': False},
    }
    manifest_bytes = (json.dumps(manifest, indent=2) + '\n').encode()
    (out / 'manifest.json').write_bytes(manifest_bytes)
    stage_bytes = current[STAGE_SCRIPT]
    (out / 'stage_app.py').write_bytes(stage_bytes)

    # Post-creation identity checks: nothing the package depends on moved.
    if sha(args.baseline_manifest.read_bytes()) != args.baseline_manifest_sha256 or \
            sha(args.freeze_manifest.read_bytes()) != args.freeze_manifest_sha256:
        raise ValueError('Baseline or freeze manifest changed during packaging')
    for name, digest in pins.items():
        if sha(read_regular(repo, name)) != digest:
            raise ValueError('Source changed during packaging: ' + name)
    if enumerate_dist(repo) != p['dist']:
        raise ValueError('Browser dist changed during packaging')
    if git_head(repo) != args.expected_head:
        raise ValueError('HEAD changed during packaging')
    verify_archive(archive, members)
    if (sha(archive.read_bytes()) != manifest['archive']['sha256']
            or sha((out / 'stage_app.py').read_bytes()) != pins[STAGE_SCRIPT]
            or (out / 'manifest.json').read_bytes() != manifest_bytes):
        raise ValueError('Written artifact changed after creation')

    receipt = {'complete': True, 'manifest_sha256': sha(manifest_bytes), 'archive': manifest['archive'],
               'stage_app_sha256': sha(stage_bytes), 'final_freeze_sha256': freeze_sha,
               'baseline_manifest_sha256': manifest['baseline_manifest_sha256'],
               'source_checkout_head': args.expected_head,
               'changed_baseline_selected_source': manifest['revision']['changed_baseline_selected_source'],
               'added_files': p['adds'], 'payload_source_files': len(p['changed']), 'full_dist_files': len(p['dist']),
               'final_selected_files': len(p['final']), 'inherited_skill_pins': len(p['inherited']),
               'all_members_content_verified': True, 'source_before_after_unchanged': True,
               'no_build_stage_deploy_or_launch': True, 'packager_sha256': sha(Path(__file__).read_bytes())}
    # Written last: its absence means generation did not complete.
    (out / 'packaging-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, required=True)
    ap.add_argument('--baseline-manifest', type=Path, required=True)
    ap.add_argument('--baseline-manifest-sha256', required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--destination', required=True)
    ap.add_argument('--expected-head', required=True)
    ap.add_argument('--freeze-manifest', type=Path, required=True)
    ap.add_argument('--freeze-manifest-sha256', required=True)
    ap.add_argument('--client-bundle', required=True)
    ap.add_argument('--add-file', action='append', default=[])
    return ap.parse_args(argv)


def main(argv=None):
    receipt = package(parse(argv))
    print(json.dumps(receipt, indent=2))
    return receipt


if __name__ == '__main__':
    main()
