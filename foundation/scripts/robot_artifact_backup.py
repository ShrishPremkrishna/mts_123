#!/usr/bin/env python3
"""Copy an explicit reviewed file list into a new local, verifiable artifact.

Never discovers whole home directories, invokes SSH/ROS, installs, restores,
changes source permissions, or executes a policy. Secret screens reduce accidental
capture; they cannot certify that an arbitrary user-selected file is secret-free.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat

SCHEMA = 'pupper_explicit_artifact_v1'
ROLES = frozenset({'policy', 'configuration', 'calibration', 'launch', 'controller_library',
                   'driver_library', 'source', 'model_metadata', 'inventory', 'build_identity'})
LABEL = re.compile(r'[a-zA-Z0-9][a-zA-Z0-9_.-]{0,95}\Z')
SHA256 = re.compile(r'[0-9a-f]{64}\Z')
SECRET_PARTS = frozenset({'.ssh', '.aws', '.azure', '.gnupg', '.kube', '.netrc',
    '.bash_history', '.zsh_history', '.bashrc', '.zshrc', '.profile', '.bash_profile',
    '.git-credentials', 'credentials', 'credentials.json',
    'api_key', 'api_key.txt', 'secrets', 'secrets.json', 'secrets.yaml', 'token.json'})
SECRET_BYTES = re.compile(
    rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|'
    rb'AIza[0-9A-Za-z_-]{30,}|\bsk-[0-9A-Za-z_-]{20,}|'
    rb'(?i:["\']?(?:api[_-]?key|client[_-]?secret|access[_-]?token|refresh[_-]?token|password)["\']?\s*[:=]\s*["\'][^"\'\r\n]{4,}["\'])')


def _safe_source_path(path):
    path = Path(path)
    if not path.is_absolute() or '..' in path.parts or path.name in ('', '.', '..'):
        raise ValueError('source path must be absolute, explicit and without traversal')
    for part in path.parts:
        lower = part.lower()
        if lower in SECRET_PARTS or lower.startswith('.env'):
            raise ValueError(f'refusing sensitive path component: {part}')
    return path


def _open_directory(path):
    path = Path(path)
    if not path.is_absolute() or '..' in path.parts:
        raise ValueError('directory path must be absolute and without traversal')
    directory = os.open(path.anchor, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for part in path.parts[1:]:
            next_directory = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
            os.close(directory)
            directory = next_directory
        return directory
    except BaseException:
        os.close(directory)
        raise


def _open_regular(path, *, screen_path=True):
    """Anchor each path component with openat/O_NOFOLLOW; reject special files."""
    path = _safe_source_path(path) if screen_path else Path(path)
    if not path.is_absolute() or '..' in path.parts:
        raise ValueError('file path must be absolute and without traversal')
    directory = _open_directory(path.parent)
    try:
        fd = os.open(path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
    finally:
        os.close(directory)
    if not stat.S_ISREG(os.fstat(fd).st_mode):
        os.close(fd)
        raise ValueError('only regular files may be captured or verified')
    return os.fdopen(fd, 'rb')


def fingerprint_file(path, *, destination=None, screen_secrets=True):
    """Hash/copy one stable regular file; reject links and common secret payloads."""
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    with _open_regular(path, screen_path=screen_secrets) as source:
        before = os.fstat(source.fileno())
        digest = hashlib.sha256()
        total, tail = 0, b''
        for block in iter(lambda: source.read(1024 * 1024), b''):
            if screen_secrets and SECRET_BYTES.search(tail + block):
                raise ValueError('possible credential material; exclude and review this file locally')
            tail = block[-512:]
            total += len(block)
            digest.update(block)
            if destination is not None:
                destination.write(block)
        after = os.fstat(source.fileno())
        identity = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
        if identity(before) != identity(after) or total != before.st_size:
            raise ValueError('source changed during capture; retry only after its writer is quiet')
        # A writer can atomically replace the path while our old open inode stays
        # unchanged. Reopen through the same no-follow path walk before accepting.
        with _open_regular(path, screen_path=screen_secrets) as current:
            if identity(before) != identity(os.fstat(current.fileno())):
                raise ValueError('source path was replaced during capture; wait for writers to be quiet')
    return {'bytes': total, 'sha256': digest.hexdigest(),
            'source_mode': stat.S_IMODE(before.st_mode), 'source_mtime_ns': before.st_mtime_ns,
            'capture_started_utc': started, 'capture_finished_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}


def _json_no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _read_json_bytes(path):
    class BoundedBuffer(io.BytesIO):
        def write(self, block):
            if self.tell() + len(block) > 20 * 1024 * 1024:
                raise ValueError('manifest/plan exceeds the 20 MiB limit')
            return super().write(block)
    buffer = BoundedBuffer()
    fingerprint_file(path, destination=buffer)
    data = buffer.getvalue()
    parsed = json.loads(data, object_pairs_hook=_json_no_duplicates,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON value')))
    return data, parsed


def read_json(path):
    return _read_json_bytes(path)[1]


def validate_plan(plan):
    if not isinstance(plan, dict) or set(plan) != {'schema', 'purpose', 'files'} or plan['schema'] != SCHEMA:
        raise ValueError('plan requires schema, purpose and files only')
    if plan['purpose'] not in ('robot-baseline', 'staged-transfer'):
        raise ValueError('purpose must be robot-baseline or staged-transfer')
    if not isinstance(plan['files'], list) or not 1 <= len(plan['files']) <= 10000:
        raise ValueError('plan must explicitly name 1..10000 files; directories/globs are unsupported')
    labels, paths = set(), set()
    for entry in plan['files']:
        if (not isinstance(entry, dict) or not {'label', 'path', 'role'} <= set(entry)
                or set(entry) - {'label', 'path', 'role', 'expected_sha256'}):
            raise ValueError('each plan file requires label, path, role and optional expected_sha256')
        label, path, role = entry['label'], entry['path'], entry['role']
        if not isinstance(label, str) or not LABEL.fullmatch(label) or label in labels:
            raise ValueError('labels must be unique safe single path components')
        if not isinstance(path, str) or str(_safe_source_path(path)) in paths:
            raise ValueError('source paths must be unique explicit absolute paths')
        expected = entry.get('expected_sha256')
        if expected is not None and (not isinstance(expected, str) or not SHA256.fullmatch(expected)):
            raise ValueError('expected source SHA256 must be 64 lowercase hexadecimal digits')
        if plan['purpose'] == 'staged-transfer' and expected is None:
            raise ValueError('staged-transfer files must each pin a reviewed expected_sha256')
        if role not in ROLES:
            raise ValueError('unsupported file role')
        labels.add(label); paths.add(str(Path(path)))
    return plan


def create_artifact(plan_path, output):
    plan_path, output = Path(plan_path).absolute(), Path(output).absolute()
    plan = validate_plan(read_json(plan_path))
    # Resolve only the chosen output parent; anchor every subsequent write to
    # opened directories. A racing symlink must never redirect even marker writes.
    output = output.parent.resolve(strict=True) / output.name
    if output.name in ('', '.', '..'):
        raise ValueError('output must name a new directory')
    parent_fd = _open_directory(output.parent)
    output_fd = files_fd = None
    try:
        os.mkdir(output.name, mode=0o700, dir_fd=parent_fd)
        output_fd = os.open(output.name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent_fd)
        os.fchmod(output_fd, 0o700)
        def mark_incomplete():
            try:
                fd = os.open('INCOMPLETE', os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=output_fd)
            except FileExistsError:
                return
            with os.fdopen(fd, 'w') as stream:
                stream.write('Capture has not completed and must not be used.\n')
        mark_incomplete()
        os.mkdir('files', mode=0o700, dir_fd=output_fd)
        files_fd = os.open('files', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=output_fd)
        os.fchmod(files_fd, 0o700)
        files = []
        for entry in plan['files']:
            stored = 'files/' + entry['label']
            fd = os.open(entry['label'], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=files_fd)
            with os.fdopen(fd, 'wb') as target:
                os.fchmod(target.fileno(), 0o600)
                fingerprint = fingerprint_file(entry['path'], destination=target)
                if entry.get('expected_sha256') is not None and fingerprint['sha256'] != entry['expected_sha256']:
                    raise ValueError('source differs from the reviewed expected SHA256: ' + entry['label'])
                target.flush(); os.fsync(target.fileno())
            copied = fingerprint_file(output / stored)
            if any(copied[key] != fingerprint[key] for key in ('bytes', 'sha256')):
                raise ValueError('copied bytes do not match source')
            files.append({'label': entry['label'], 'source_path': str(Path(entry['path'])),
                          'stored_path': stored, 'role': entry['role'],
                          **({'reviewed_source_sha256': entry['expected_sha256']} if 'expected_sha256' in entry else {}),
                          **fingerprint})
        manifest = {'schema': SCHEMA, 'purpose': plan['purpose'],
                    'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    'scope': 'Explicit files only. No installation, restore, policy selection or hardware approval.',
                    'files': files}
        encoded = (json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()
        fd = os.open('manifest.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=output_fd)
        with os.fdopen(fd, 'wb') as stream:
            os.fchmod(stream.fileno(), 0o600)
            stream.write(encoded); stream.flush(); os.fsync(stream.fileno())
        # The publicly named output must still be the private directory we made.
        for path, original_fd in ((output, output_fd), (output / 'files', files_fd)):
            current_fd = _open_directory(path)
            try:
                original, current = os.fstat(original_fd), os.fstat(current_fd)
                if (original.st_dev, original.st_ino) != (current.st_dev, current.st_ino):
                    raise ValueError('output directory was replaced during capture')
            finally:
                os.close(current_fd)
        os.unlink('INCOMPLETE', dir_fd=output_fd)
        try:
            return verify_artifact(output, expected_manifest_sha256=hashlib.sha256(encoded).hexdigest())
        except BaseException:
            mark_incomplete()
            raise
    finally:
        for fd in (files_fd, output_fd, parent_fd):
            if fd is not None:
                os.close(fd)


def verify_artifact(directory, *, expected_manifest_sha256=None):
    directory = Path(directory).absolute()
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError('artifact must be a real directory')
    if set(p.name for p in directory.iterdir()) != {'manifest.json', 'files'}:
        raise ValueError('artifact is incomplete or contains unexpected entries')
    if (directory / 'files').is_symlink() or not (directory / 'files').is_dir():
        raise ValueError('artifact files directory must not be a link')
    for path in (directory, directory / 'files'):
        fd = _open_directory(path)
        try:
            if stat.S_IMODE(os.fstat(fd).st_mode) != 0o700:
                raise ValueError('artifact directories must retain private mode 0700')
        finally:
            os.close(fd)
    with _open_regular(directory / 'manifest.json') as stream:
        if stat.S_IMODE(os.fstat(stream.fileno()).st_mode) != 0o600:
            raise ValueError('artifact manifest must retain private nonexecutable mode 0600')
    manifest_bytes, manifest = _read_json_bytes(directory / 'manifest.json')
    manifest_hash = hashlib.sha256(manifest_bytes).hexdigest()
    if expected_manifest_sha256 is not None:
        if not isinstance(expected_manifest_sha256, str) or not SHA256.fullmatch(expected_manifest_sha256):
            raise ValueError('expected manifest SHA256 must be 64 lowercase hexadecimal digits')
        if manifest_hash != expected_manifest_sha256:
            raise ValueError('manifest identity differs from the independently recorded SHA256')
    if not isinstance(manifest, dict) or manifest.get('schema') != SCHEMA:
        raise ValueError('unsupported artifact manifest')
    files = manifest.get('files')
    if not isinstance(files, list) or not 1 <= len(files) <= 10000:
        raise ValueError('invalid artifact file list')
    validate_plan({'schema': SCHEMA, 'purpose': manifest.get('purpose'), 'files': [
        {'label': e.get('label'), 'path': e.get('source_path'), 'role': e.get('role'),
         **({'expected_sha256': e['reviewed_source_sha256']} if 'reviewed_source_sha256' in e else {})} for e in files]})
    expected_names = set()
    for entry in files:
        if entry.get('stored_path') != 'files/' + entry['label']:
            raise ValueError('stored path must match its explicit file label')
        if type(entry.get('bytes')) is not int or entry['bytes'] < 0 or not SHA256.fullmatch(str(entry.get('sha256', ''))):
            raise ValueError('invalid manifest file size or digest')
        if 'reviewed_source_sha256' in entry and entry['reviewed_source_sha256'] != entry['sha256']:
            raise ValueError('captured file differs from its reviewed source digest')
        actual = fingerprint_file(directory / entry['stored_path'])
        if actual['source_mode'] != 0o600:
            raise ValueError('artifact files must retain private nonexecutable mode 0600')
        if any(actual[key] != entry[key] for key in ('bytes', 'sha256')):
            raise ValueError(f'file content mismatch: {entry["label"]}')
        expected_names.add(entry['label'])
    if set(p.name for p in (directory / 'files').iterdir()) != expected_names:
        raise ValueError('artifact files are missing or contain unexpected additions')
    return {'verified': True, 'directory': str(directory), 'manifest_sha256': manifest_hash,
            'identity_pinned': expected_manifest_sha256 is not None, 'purpose': manifest['purpose'],
            'files': len(files), 'bytes': sum(entry['bytes'] for entry in files),
            'scope': 'Recorded file integrity only; not hardware readiness or an executable rollback.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    create = sub.add_parser('create', help='copy an explicit reviewed plan into a new local directory')
    create.add_argument('--plan', type=Path, required=True)
    create.add_argument('--output', type=Path, required=True)
    verify = sub.add_parser('verify', help='verify copied bytes; never execute/install/restore')
    verify.add_argument('directory', type=Path)
    verify.add_argument('--expected-manifest-sha256')
    args = parser.parse_args(argv)
    try:
        report = (create_artifact(args.plan, args.output) if args.command == 'create' else
                  verify_artifact(args.directory, expected_manifest_sha256=args.expected_manifest_sha256))
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
        parser.exit(1, f'Artifact refused: {type(exc).__name__}: {exc}\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
