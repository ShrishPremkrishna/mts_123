#!/usr/bin/env python3
"""Synthetic temp-only tests for package_app.py; roundtrips through unchanged stage_app.py."""
from pathlib import Path
from unittest import mock
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import tempfile
import unittest

HERE = Path(__file__).resolve().parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, HERE / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


package_app, stage_app = load('package_app'), load('stage_app')
DIST = 'gemini_pupper/web/dist/'
HEAD = 'a' * 40
SKILL = 'gemini_pupper/tricks/swim/policy.onnx'
NATIVE = {'foundation/hardware/native/driver.so': 'b' * 64}
ADD = 'gemini_pupper/web/services/follow.ts'


def h(data):
    return hashlib.sha256(data).hexdigest()


BASELINE = {
    'foundation/hardware/floor_stock/shutdown_coordinator.py': b'# coordinator\n',
    'foundation/hardware/lab6-session.yaml': b'synthetic: true\n',
    'foundation/hardware/stage_app.py': (HERE / 'stage_app.py').read_bytes(),
    'gemini_pupper/robot/server.py': b'print("old")\n',
    'gemini_pupper/web/App.tsx': b'export default 1\n',
    SKILL: b'\x00baseline-policy',
    DIST + 'index.html': b'<script src="assets/index-OLD1.js"></script>\n',
    DIST + 'assets/index-OLD1.js': b'old();\n',
}
REPO = {k: v for k, v in BASELINE.items() if not k.startswith(DIST)} | {
    'gemini_pupper/robot/server.py': b'print("new")\n',
    SKILL: b'\x00DIFFERENT-current-policy',  # must never be read into the release
    ADD: b'export const follow = 1\n',
    'tests/test_extra.py': b'# extra frozen test\n',
    DIST + 'index.html': b'<script src="assets/index-NEW2.js"></script>\n',
    DIST + 'assets/index-NEW2.js': b'neu();\n',
    DIST + 'receipt-audio/ok.pcm': b'\x00\x01',
}


def write_tree(root, files):
    for rel, data in files.items():
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / rel).write_bytes(data)


def snapshot(root):
    return {p.relative_to(root).as_posix(): (os.readlink(p) if p.is_symlink() else p.read_bytes() if p.is_file() else None)
            for p in sorted(root.rglob('*'))}


class PackageAppTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name).resolve()
        self.repo, self.old = self.root / 'repo', self.root / 'releases' / 'r1'
        self.new, self.out = self.root / 'releases' / 'r2', self.root / 'out'
        write_tree(self.repo, REPO)
        write_tree(self.old, BASELINE)
        self.baseline = {
            'schema': 'ws1_application_delta_v1', 'destination_remote_path': str(self.old),
            'final_selected_files': [{'path': n, 'sha256': h(d)} for n, d in sorted(BASELINE.items())],
            'current_browser_build_inputs': {'gemini_pupper/web/App.tsx': h(BASELINE['gemini_pupper/web/App.tsx'])},
            'selected_native_unchanged': NATIVE,
            'inherited_skill_files_unchanged': [{'path': SKILL, 'sha256': h(BASELINE[SKILL])}],
            'compatibility': {'motion_mode': 'class_run'},
        }
        self.freeze = {'source_checkout_head': HEAD,
                       'files': [{'path': n, 'sha256': h(d)} for n, d in sorted(REPO.items())]}
        self.adds = [ADD]
        head = mock.patch.object(package_app, 'git_head', return_value=HEAD)
        self.git = head.start()
        self.addCleanup(head.stop)

    def run_packager(self):
        bpath, fpath = self.root / 'baseline.json', self.root / 'freeze.json'
        bpath.write_text(json.dumps(self.baseline))
        fpath.write_text(json.dumps(self.freeze))
        argv = ['--repo', str(self.repo), '--baseline-manifest', str(bpath),
                '--baseline-manifest-sha256', h(bpath.read_bytes()), '--output', str(self.out),
                '--destination', str(self.new), '--expected-head', HEAD, '--freeze-manifest', str(fpath),
                '--freeze-manifest-sha256', h(fpath.read_bytes()), '--client-bundle', 'index-NEW2.js']
        for name in self.adds:
            argv += ['--add-file', name]
        with contextlib.redirect_stdout(io.StringIO()):
            return package_app.main(argv)

    def assertRefused(self, pattern):
        before = snapshot(self.repo)
        with self.assertRaisesRegex(ValueError, pattern):
            self.run_packager()
        self.assertFalse(self.out.exists())
        self.assertEqual(snapshot(self.repo), before)

    def test_roundtrip_through_unchanged_stage_app(self):
        receipt = self.run_packager()
        manifest_path = self.out / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        self.assertTrue(receipt['complete'])
        self.assertEqual((self.out / 'stage_app.py').read_bytes(), (HERE / 'stage_app.py').read_bytes())
        self.assertEqual({e['path'] for e in manifest['payload_files']},
                         {'gemini_pupper/robot/server.py', ADD} | {n for n in REPO if n.startswith(DIST)})
        self.assertEqual(manifest['selected_native_unchanged'], NATIVE)
        self.assertEqual(manifest['inherited_skill_files_unchanged'], self.baseline['inherited_skill_files_unchanged'])
        self.assertEqual(manifest['required_baseline_files'], self.baseline['final_selected_files'])
        self.assertEqual(set(manifest['current_browser_build_inputs']), {'gemini_pupper/web/App.tsx', ADD})
        self.assertEqual([e for e in manifest['final_selected_files'] if e['path'] == SKILL],
                         [{'path': SKILL, 'sha256': h(BASELINE[SKILL])}])
        old_before = snapshot(self.old)

        stage_app.stage(manifest_path, h(manifest_path.read_bytes()), self.out / 'lab6-application.tar.gz')

        self.assertEqual(snapshot(self.old), old_before)
        staged_dist = {p.relative_to(self.new).as_posix(): p.read_bytes()
                       for p in (self.new / DIST).rglob('*') if p.is_file()}
        self.assertEqual(staged_dist, {n: d for n, d in REPO.items() if n.startswith(DIST)})
        for name in ('foundation/hardware/lab6-session.yaml', SKILL, 'gemini_pupper/web/App.tsx'):
            self.assertEqual((self.new / name).read_bytes(), BASELINE[name])
        self.assertEqual((self.new / ADD).read_bytes(), REPO[ADD])

    def test_freeze_duplicate_and_conflicting_entries(self):
        entry = self.freeze['files'][0]
        for dup in (dict(entry), {'path': entry['path'], 'sha256': 'c' * 64}):
            with self.subTest(dup=dup):
                self.freeze['files'].append(dup)
                self.assertRefused('duplicate or conflicting')
                self.freeze['files'].pop()

    def test_bad_hash_and_head(self):
        original_hash = self.freeze['files'][0]['sha256']
        self.freeze['files'][0]['sha256'] = 'd' * 64
        self.assertRefused('differs from final freeze')
        self.freeze['files'][0]['sha256'] = 'XYZ'
        self.assertRefused('malformed')
        self.freeze['files'][0]['sha256'] = original_hash
        self.freeze['source_checkout_head'] = 'e' * 40
        self.assertRefused('source_checkout_head')
        self.freeze['source_checkout_head'] = HEAD
        self.git.side_effect = [HEAD, 'f' * 40]
        with self.assertRaisesRegex(ValueError, 'HEAD changed'):
            self.run_packager()
        self.assertFalse((self.out / 'packaging-receipt.json').exists())

    def test_unpinned_changed_unchanged_or_new_source(self):
        for name in ('gemini_pupper/robot/server.py', 'gemini_pupper/web/App.tsx', ADD):
            with self.subTest(name=name):
                saved = list(self.freeze['files'])
                self.freeze['files'] = [e for e in saved if e['path'] != name]
                self.assertRefused('not pinned')
                self.freeze['files'] = saved

    def test_missing_or_extra_dist(self):
        (self.repo / DIST / 'extra.js').write_bytes(b'x')
        self.assertRefused('unpinned 1')
        (self.repo / DIST / 'extra.js').unlink()
        self.freeze['files'].append({'path': DIST + 'gone.js', 'sha256': h(b'gone')})
        self.assertRefused('Missing or nonregular')

    def test_symlinks_and_traversal(self):
        os.symlink('index-NEW2.js', self.repo / DIST / 'assets/alias.js')
        self.assertRefused('Symlink in browser dist')
        (self.repo / DIST / 'assets/alias.js').unlink()
        self.freeze['files'].append({'path': '../outside.py', 'sha256': 'a' * 64})
        self.assertRefused('Invalid relative path')
        self.freeze['files'].pop()
        services = self.repo / 'gemini_pupper/web/services'
        services.rename(self.root / 'elsewhere')
        os.symlink(self.root / 'elsewhere', services)
        self.assertRefused('traverses symlink')

    def test_trick_or_nonapp_addition_refused_before_output(self):
        for name in ('gemini_pupper/tricks/new/policy.json', 'foundation/hardware/motor.py', 'gemini_pupper/.env'):
            with self.subTest(name=name):
                write_tree(self.repo, {name: b'x'})
                self.freeze['files'].append({'path': name, 'sha256': h(b'x')})
                self.adds = [name]
                self.assertRefused('outside permitted application payload')
                self.freeze['files'].pop()

    def test_secret_marker_in_payload_refused(self):
        secret = b'key = "AIza' + b'A' * 35 + b'"\n'
        write_tree(self.repo, {ADD: secret})
        self.freeze['files'] = [e if e['path'] != ADD else {'path': ADD, 'sha256': h(secret)} for e in self.freeze['files']]
        with self.assertRaises(ValueError) as caught:
            self.run_packager()
        self.assertNotIn('AIza', str(caught.exception))
        self.assertFalse(self.out.exists())

    def test_output_cannot_modify_source_or_preserved_baseline(self):
        for parent in (self.repo, self.old):
            with self.subTest(parent=parent):
                self.out = parent / 'new-package'
                old_before = snapshot(self.old)
                self.assertRefused('Output must be outside')
                self.assertEqual(snapshot(self.old), old_before)

    def test_existing_output_unchanged(self):
        self.out.mkdir()
        (self.out / 'keep.txt').write_bytes(b'keep')
        before = snapshot(self.out)
        with self.assertRaisesRegex(ValueError, 'Output exists'):
            self.run_packager()
        self.assertEqual(snapshot(self.out), before)


if __name__ == '__main__':
    unittest.main()
