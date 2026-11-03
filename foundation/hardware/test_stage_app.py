#!/usr/bin/env python3
"""Synthetic, temp-only tests for stage_app.py. No processes, network, ROS or hardware."""
from pathlib import Path
from unittest import mock
import hashlib
import importlib.util
import io
import json
import os
import shutil
import stat
import tarfile
import tempfile
import unittest

_SPEC = importlib.util.spec_from_file_location('stage_app', Path(__file__).resolve().parent / 'stage_app.py')
stage_app = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(stage_app)

COORDINATOR = 'foundation/hardware/floor_stock/shutdown_coordinator.py'
SESSION_YAML = 'foundation/hardware/lab6-session.yaml'
DIST = 'gemini_pupper/web/dist/'

BASELINE = {
    COORDINATOR: b'# synthetic coordinator\n',
    SESSION_YAML: b'synthetic: true\n',
    'gemini_pupper/lab/basics.py': b'# basics\n',
    'gemini_pupper/robot/server.py': b'print("old server")\n',
    DIST + 'index.html': b'<script src="assets/index-OLDaaaa1.js"></script>\n',
    DIST + 'assets/index-OLDaaaa1.js': b'console.log("old");\n',
    DIST + 'receipt-audio/okay.pcm': b'\x00\x01pcm',
}
# Old and new hashed JS names differ, as in the real manifest.
PAYLOAD = {
    'gemini_pupper/robot/server.py': b'print("new server")\n',
    DIST + 'index.html': b'<script src="assets/index-NEWbbbb2.js"></script>\n',
    DIST + 'assets/index-NEWbbbb2.js': b'console.log("new");\n',
    DIST + 'receipt-audio/okay.pcm': b'\x00\x01pcm',
}


def h(data):
    return hashlib.sha256(data).hexdigest()


def snapshot(root):
    """Map of every entry under root without following symlinks."""
    out = {}
    for current, dirs, files in os.walk(root):
        for name in dirs + files:
            path = Path(current) / name
            rel = path.relative_to(root).as_posix()
            if path.is_symlink():
                out[rel] = ('link', os.readlink(path))
            elif path.is_file():
                out[rel] = ('file', path.read_bytes(), stat.S_IMODE(path.lstat().st_mode))
            else:
                out[rel] = ('dir',)
    return out


def drop(entries, path):
    entries[:] = [e for e in entries if e['path'] != path]


class Fixture:
    def __init__(self, tmp):
        self.root = Path(tmp).resolve()
        self.old = self.root / 'releases' / 'baseline'
        self.new = self.root / 'releases' / 'destination'
        self.inputs = self.root / 'inputs'
        self.outside = self.root / 'outside'
        for path in (self.inputs, self.outside):
            path.mkdir(parents=True)
        self.baseline = dict(BASELINE)
        self.payload = dict(PAYLOAD)
        for rel, data in self.baseline.items():
            target = self.old / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        # Unselected inherited content that must be carried over untouched.
        (self.old / 'UNLISTED-NOTES.txt').write_bytes(b'inherited\n')
        os.symlink('server.py', self.old / 'gemini_pupper/robot/latest-server')

    def members(self):
        sums = ''.join(h(d) + '  ' + n + '\n' for n, d in sorted(self.payload.items())).encode()
        result = [{'name': n, 'data': d, 'mode': 0o644, 'type': tarfile.REGTYPE, 'linkname': ''}
                  for n, d in sorted(self.payload.items())]
        result.append({'name': 'SHA256SUMS', 'data': sums, 'mode': 0o644, 'type': tarfile.REGTYPE, 'linkname': ''})
        return result

    def build(self, manifest_hook=None, member_hook=None):
        members = self.members()
        sums = members[-1]['data']
        if member_hook:
            member_hook(members)
        archive = self.inputs / 'app.tar.gz'
        with tarfile.open(archive, 'w:gz') as tf:
            for m in members:
                info = tarfile.TarInfo(m['name'])
                info.mode, info.type, info.linkname = m['mode'], m['type'], m['linkname']
                if m['type'] == tarfile.REGTYPE:
                    info.size = len(m['data'])
                    tf.addfile(info, io.BytesIO(m['data']))
                else:
                    tf.addfile(info)
        final = {n: h(d) for n, d in self.baseline.items() if not n.startswith(DIST)}
        final.update({n: h(d) for n, d in self.payload.items()})
        manifest = {
            'schema': 'ws1_application_delta_v1',
            'baseline_remote_path': str(self.old),
            'destination_remote_path': str(self.new),
            'payload_files': [{'path': n, 'sha256': h(d), 'bytes': len(d), 'mode': 0o644}
                              for n, d in sorted(self.payload.items())],
            'verification_file': {'path': 'SHA256SUMS', 'sha256': h(sums)},
            'archive': {'name': archive.name, 'sha256': h(archive.read_bytes())},
            'required_baseline_files': [{'path': n, 'sha256': h(d)} for n, d in sorted(self.baseline.items())],
            'final_selected_files': [{'path': n, 'sha256': s} for n, s in sorted(final.items())],
        }
        if manifest_hook:
            manifest_hook(manifest)
        manifest_path = self.inputs / 'manifest.json'
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
        return manifest_path, h(manifest_path.read_bytes()), archive

    def stage(self, **kwargs):
        return stage_app.stage(*self.build(**kwargs))


class StageAppTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.fx = Fixture(self._tmp.name)

    def assertRefused(self, pattern, destination_absent=True, **kwargs):
        before_old = snapshot(self.fx.old)
        before_outside = snapshot(self.fx.outside)
        with self.assertRaisesRegex(ValueError, pattern):
            self.fx.stage(**kwargs)
        if destination_absent:
            self.assertFalse(self.fx.new.exists() or self.fx.new.is_symlink(), 'destination created before refusal')
        self.assertEqual(snapshot(self.fx.old), before_old)
        self.assertEqual(snapshot(self.fx.outside), before_outside)

    def test_selected_helpers_and_follower_payload_are_supported(self):
        for name in ['foundation/hardware/persistent_session.py', 'foundation/hardware/lab6_session.launch.py',
                     'follow_me_pupper/robot/follow_service.py', 'follow_me_pupper/lab/target_lock.py',
                     'follow_me_pupper/robot/yolov8n_seg.hef']:
            self.fx.payload[name] = b'reviewed fixture'
        receipt = self.fx.stage()
        self.assertFalse(receipt['started'])
        for name, value in self.fx.payload.items():
            self.assertEqual((self.fx.new / name).read_bytes(), value)

    def test_changed_urdf_refused_despite_valid_manifest(self):
        self.fx.payload['foundation/hardware/floor_stock/baseline-expanded.urdf'] = b'changed geometry'
        self.assertRefused('Nonapplication')

    def test_policy_json_refused_even_with_valid_manifest(self):
        self.fx.payload['gemini_pupper/tricks/future/policy.json'] = b'{"weights": [1, 2, 3]}'
        self.assertRefused('Nonapplication')

    def test_native_binary_inside_application_refused(self):
        self.fx.payload['gemini_pupper/robot/libdriver.so'] = b'fixture native binary'
        self.assertRefused('Nonapplication')

    def test_pinned_setuid_mode_refused(self):
        def change(manifest): manifest['payload_files'][0]['mode'] = 0o4755
        self.assertRefused('Nonapplication', manifest_hook=change)

    # --- success -----------------------------------------------------------------

    def test_full_dist_replacement_and_preserved_rollback(self):
        before_old = snapshot(self.fx.old)
        manifest_path, manifest_sha, archive = self.fx.build()
        forbidden = AssertionError('stage must not launch, signal or use the network')
        with mock.patch('subprocess.Popen', side_effect=forbidden), \
                mock.patch('os.system', side_effect=forbidden), \
                mock.patch('os.kill', side_effect=forbidden), \
                mock.patch('os.posix_spawn', side_effect=forbidden, create=True), \
                mock.patch('socket.socket', side_effect=forbidden):
            receipt = stage_app.stage(manifest_path, manifest_sha, archive)

        self.assertEqual(snapshot(self.fx.old), before_old, 'baseline checkout must stay byte-identical')
        new = self.fx.new
        actual_dist = {p.relative_to(new).as_posix() for p in (new / DIST).rglob('*') if p.is_file()}
        self.assertEqual(actual_dist, {n for n in PAYLOAD if n.startswith(DIST)})
        self.assertFalse((new / DIST / 'assets/index-OLDaaaa1.js').exists())
        for rel, data in PAYLOAD.items():
            self.assertEqual((new / rel).read_bytes(), data)
            self.assertEqual(stat.S_IMODE((new / rel).stat().st_mode), 0o644)
        for rel in (COORDINATOR, SESSION_YAML, 'gemini_pupper/lab/basics.py'):
            self.assertEqual((new / rel).read_bytes(), BASELINE[rel])
        self.assertEqual((new / 'UNLISTED-NOTES.txt').read_bytes(), b'inherited\n')
        self.assertEqual(os.readlink(new / 'gemini_pupper/robot/latest-server'), 'server.py')
        self.assertTrue((new / 'SHA256SUMS').is_file())

        manifest = json.loads(manifest_path.read_text())
        listed = (new / 'APP-SELECTED-SHA256SUMS').read_text().splitlines()
        self.assertEqual(listed, [e['sha256'] + '  ' + e['path'] for e in manifest['final_selected_files']])
        self.assertEqual((new / 'app-stage-manifest.json').read_bytes(), manifest_path.read_bytes())
        self.assertEqual(json.loads((new / 'app-stage-receipt.json').read_text()), receipt)
        self.assertIs(receipt['started'], False)
        self.assertIs(receipt['native_driver_policy_or_gains_changed'], False)
        self.assertEqual(receipt['baseline_preserved'], str(self.fx.old))
        self.assertEqual(receipt['manifest_sha256'], manifest_sha)
        self.assertEqual(receipt['verified_archive_members'], len(PAYLOAD) + 1)
        self.assertEqual(receipt['verified_baseline_files'], len(BASELINE))

    # --- input hashes ------------------------------------------------------------

    def test_manifest_hash_refusals(self):
        manifest_path, manifest_sha, archive = self.fx.build()
        for bad in ('0' * 64, manifest_sha.upper(), manifest_sha[:-1], ''):
            with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, 'manifest hash mismatch'):
                stage_app.stage(manifest_path, bad, archive)
        self.assertFalse(self.fx.new.exists())

    def test_archive_hash_refusal(self):
        def hook(m):
            m['archive']['sha256'] = '0' * 64
        self.assertRefused('archive hash mismatch', manifest_hook=hook)

    def test_member_hash_refusal(self):
        def same_size_tamper(members):
            m = next(m for m in members if m['name'] == 'gemini_pupper/robot/server.py')
            m['data'] = b'print("BAD server")\n'
            self.assertEqual(len(m['data']), len(PAYLOAD[m['name']]))
        self.assertRefused('Member hash mismatch', member_hook=same_size_tamper)

    def test_member_size_and_mode_refusals(self):
        def grow(members):
            members[0]['data'] += b'#'
        self.assertRefused('Member size mismatch', member_hook=grow)
        for mode in (0o755, 0o4755, 0o666):
            def chmod(members, mode=mode):
                members[0]['mode'] = mode
            with self.subTest(mode=oct(mode)):
                self.assertRefused('unexpected mode', member_hook=chmod)

    # --- archive member shape ----------------------------------------------------

    def test_duplicate_members_and_entries(self):
        def dup_member(members):
            members.append(dict(members[0]))
        self.assertRefused('missing or duplicate archive members', member_hook=dup_member)

        def missing_member(members):
            del members[0]
        self.assertRefused('missing or duplicate archive members', member_hook=missing_member)

        for key in ('payload_files', 'required_baseline_files', 'final_selected_files'):
            def dup_entry(m, key=key):
                m[key].append(dict(m[key][0]))
            with self.subTest(key=key):
                self.assertRefused('Duplicate selected path', manifest_hook=dup_entry)

    def test_traversal_members(self):
        def extra(name):
            def hook(members):
                members.append({'name': name, 'data': b'x', 'mode': 0o644, 'type': tarfile.REGTYPE, 'linkname': ''})
            return hook
        for name in ('../escape.txt', '/tmp/escape.txt', 'gemini_pupper/../escape.txt'):
            with self.subTest(unlisted=name):
                self.assertRefused('Unexpected, missing or duplicate', member_hook=extra(name))
        # Listed in the manifest too, so only checked_path stands in the way.
        self.fx.payload['gemini_pupper/../escape.txt'] = b'escape\n'
        self.assertRefused('Invalid selected path')
        self.assertFalse((self.fx.root / 'releases' / 'escape.txt').exists())

    def test_nonregular_members(self):
        name = DIST + 'assets/extra.js'
        self.fx.payload[name] = b'extra\n'
        for kind, linkname in ((tarfile.SYMTYPE, '/etc/passwd'), (tarfile.LNKTYPE, DIST + 'index.html'),
                               (tarfile.DIRTYPE, ''), (tarfile.FIFOTYPE, '')):
            def hook(members, kind=kind, linkname=linkname):
                m = next(m for m in members if m['name'] == name)
                m['type'], m['linkname'] = kind, linkname
            with self.subTest(kind=kind):
                self.assertRefused('Nonregular member', member_hook=hook)

    def test_symlinks_in_baseline_are_not_followed(self):
        # Selected baseline file replaced by a symlink with identical content.
        target = self.fx.outside / 'basics.py'
        target.write_bytes(BASELINE['gemini_pupper/lab/basics.py'])
        (self.fx.old / 'gemini_pupper/lab/basics.py').unlink()
        os.symlink(target, self.fx.old / 'gemini_pupper/lab/basics.py')
        self.assertRefused('traverses symlink')

    def test_symlink_parent_of_new_payload_path(self):
        # Remove baseline web entries so only the payload walk reaches the symlink.
        for rel in [r for r in self.fx.baseline if r.startswith('gemini_pupper/web/')]:
            del self.fx.baseline[rel]
        shutil.rmtree(self.fx.old / 'gemini_pupper/web')
        os.symlink(self.fx.outside, self.fx.old / 'gemini_pupper/web')
        self.assertRefused('traverses symlink')

    def test_symlinked_metadata_target(self):
        os.symlink(self.fx.outside / 'receipt.json', self.fx.old / 'app-stage-receipt.json')
        self.assertRefused('traverses symlink')

    # --- destination and release paths --------------------------------------------

    def test_existing_destination(self):
        self.fx.new.mkdir()
        (self.fx.new / 'keep.txt').write_bytes(b'keep\n')
        self.assertRefused('Destination exists', destination_absent=False)
        self.assertEqual(sorted(p.name for p in self.fx.new.iterdir()), ['keep.txt'])
        self.assertEqual((self.fx.new / 'keep.txt').read_bytes(), b'keep\n')
        (self.fx.new / 'keep.txt').unlink()
        self.fx.new.rmdir()
        os.symlink(self.fx.root / 'missing-target', self.fx.new)
        self.assertRefused('Destination exists', destination_absent=False)
        self.assertEqual(os.readlink(self.fx.new), str(self.fx.root / 'missing-target'))

    def test_release_path_refusals(self):
        def inside(m):
            m['destination_remote_path'] = str(self.fx.old / 'nested')
        def relative(m):
            m['destination_remote_path'] = 'releases/destination'
        for hook in (inside, relative):
            with self.subTest(hook=hook.__name__):
                self.assertRefused('absolute and destination outside baseline', manifest_hook=hook)
        self.assertFalse((self.fx.old / 'nested').exists())

    # --- baseline dependencies ----------------------------------------------------

    def test_required_sibling_coordinator_and_yaml(self):
        for rel in (COORDINATOR, SESSION_YAML):
            def unlisted(m, rel=rel):
                drop(m['required_baseline_files'], rel)
                drop(m['final_selected_files'], rel)
            with self.subTest(unlisted=rel):
                self.assertRefused('Required combined-checkout helpers missing', manifest_hook=unlisted)

        original = (self.fx.old / SESSION_YAML).read_bytes()
        (self.fx.old / SESSION_YAML).write_bytes(b'synthetic: false\n')  # manifest still pins original
        self.assertRefused('Selected file missing or different')
        (self.fx.old / SESSION_YAML).write_bytes(original)

        (self.fx.old / COORDINATOR).unlink()
        self.assertRefused('Selected file missing or different')

    def test_different_baseline_dependency(self):
        (self.fx.old / 'gemini_pupper/lab/basics.py').write_bytes(b'# drifted\n')
        self.assertRefused('Selected file missing or different')

    # --- final selection ----------------------------------------------------------

    def test_incomplete_or_inconsistent_final_selection(self):
        def omit_preserved(m):
            drop(m['final_selected_files'], 'gemini_pupper/lab/basics.py')
        def omit_payload(m):
            drop(m['final_selected_files'], DIST + 'assets/index-NEWbbbb2.js')
        def keep_old_dist(m):
            m['final_selected_files'].append({'path': DIST + 'assets/index-OLDaaaa1.js',
                                              'sha256': h(BASELINE[DIST + 'assets/index-OLDaaaa1.js'])})
        def wrong_hash(m):
            next(e for e in m['final_selected_files'] if e['path'] == 'gemini_pupper/lab/basics.py')['sha256'] = '1' * 64
        def unknown_file(m):
            m['final_selected_files'].append({'path': 'gemini_pupper/robot/unknown.py', 'sha256': '2' * 64})
        def empty(m):
            m['final_selected_files'] = []
        cases = ((omit_preserved, 'omit preserved baseline files'),
                 (omit_payload, 'Payload absent from final'),
                 (keep_old_dist, 'absent from complete replacement payload'),
                 (wrong_hash, 'does not match baseline/overlay'),
                 (unknown_file, 'does not match baseline/overlay'),
                 (empty, 'nonempty list'))
        for hook, pattern in cases:
            with self.subTest(case=hook.__name__):
                self.assertRefused(pattern, manifest_hook=hook)

    def test_dist_without_index_refused(self):
        del self.fx.payload[DIST + 'index.html']
        self.assertRefused('index.html is required')

    # --- nonapplication / credential payload --------------------------------------

    def test_nonapplication_and_credential_paths_refused(self):
        for rel in (SESSION_YAML, 'foundation/native/driver.so', 'SHA256SUMS-extra', 'gemini_pupper/.env',
                    'gemini_pupper/robot/.env.local', 'gemini_pupper/web/node_modules/x.js',
                    'gemini_pupper/robot/__pycache__/server.cpython-311.pyc', 'gemini_pupper/.git/config',
                    'gemini_pupper/robot/api_key', 'gemini_pupper/credentials.json',
                    'gemini_pupper/robot/tls.pem', 'gemini_pupper/robot/tls.key'):
            def hook(m, rel=rel):
                m['payload_files'].append({'path': rel, 'sha256': '3' * 64, 'bytes': 1, 'mode': 0o644})
            with self.subTest(path=rel):
                self.assertRefused('Nonapplication or credential/cache payload path refused', manifest_hook=hook)

    def test_credential_content_refused(self):
        for content in (b'KEY = "AIza' + b'S' * 35 + b'"\n', b'-----BEGIN RSA PRIVATE KEY-----\nsynthetic\n'):
            with self.subTest(content=content[:12]):
                self.fx.payload['gemini_pupper/robot/server.py'] = content
                self.assertRefused('Credential marker in payload')


if __name__ == '__main__':
    unittest.main()
