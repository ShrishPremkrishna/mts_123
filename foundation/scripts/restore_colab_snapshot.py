"""Verify a periodic snapshot offline, or restore it into a new directory.

Verification covers only the current per-file index, not historical receipt
versions, a coherent training-step snapshot, or completion of a remote run.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import stat


def validate_item(name, item):
    if not isinstance(name, str) or not name or not isinstance(item, dict):
        raise ValueError('Invalid snapshot path or identity')
    relative = Path(name)
    digest = item.get('sha256')
    if (relative.is_absolute() or '..' in relative.parts or relative.as_posix() != name
            or name == '.' or not isinstance(digest, str)
            or not re.fullmatch('[0-9a-f]{64}', digest)
            or type(item.get('bytes')) is not int or item['bytes'] < 0
            or item.get('path', name) != name):
        raise ValueError('Invalid snapshot path or identity')
    return relative, digest


def verify_file(source, digest, size):
    """Stream bytes; refuse symlinks, special files and detectable mutation."""
    if source.is_symlink():
        raise ValueError(f'Snapshot verification failed: symlink {source}')
    before = source.stat()
    if not stat.S_ISREG(before.st_mode) or before.st_size != size:
        raise ValueError(f'Snapshot verification failed: size/type {source}')
    actual = hashlib.sha256()
    with source.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            actual.update(chunk)
    after = source.stat()
    identity = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
    if source.is_symlink() or identity(before) != identity(after) or actual.hexdigest() != digest:
        raise ValueError(f'Snapshot verification failed: hash/mutation {source}')


def snapshot_plan(archive):
    index_path = archive/'index.json'
    if index_path.is_symlink() or (archive/'objects').is_symlink():
        raise ValueError('Snapshot index/objects must not be symlinks')
    raw = index_path.read_bytes()
    index = json.loads(raw)
    if not isinstance(index, dict) or not isinstance(index.get('files'), dict):
        raise ValueError('Invalid snapshot index')
    plan = []
    for name, item in index['files'].items():
        relative, digest = validate_item(name, item)
        if relative.parts[0] == 'recovery-snapshot.json':
            raise ValueError('Snapshot path collides with restoration receipt')
        source = archive/'objects'/digest
        verify_file(source, digest, item['bytes'])
        plan.append((source, relative, item))
    names = set(index['files'])
    if any(str(parent) in names for _, relative, _ in plan for parent in relative.parents if str(parent) != '.'):
        raise ValueError('Snapshot paths collide as file and directory')
    return index, plan, hashlib.sha256(raw).hexdigest()


def verify(archive):
    index, plan, index_sha = snapshot_plan(archive)
    return dict(verified_files=len(plan), bytes=sum(item['bytes'] for _, _, item in plan),
                index_sha256=index_sha, session=index.get('session'), remote_root=index.get('remote_root'),
                scope='Current indexed objects only; no run-completion or historical-version claim')


def restore(archive, output):
    index, plan, _ = snapshot_plan(archive)
    output.mkdir(parents=True, exist_ok=False)
    for source, relative, item in plan:
        destination = output/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        # Detect a source/copy failure after preflight; do not publish a success receipt.
        verify_file(destination, item['sha256'], item['bytes'])
    (output/'recovery-snapshot.json').write_text(json.dumps(index, indent=2)+'\n')
    return len(plan)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path, nargs='?')
    parser.add_argument('--verify', action='store_true', help='Read/hash current indexed objects only; write nothing')
    args = parser.parse_args()
    if args.verify and args.output is not None:
        parser.error('--verify takes no output directory')
    if not args.verify and args.output is None:
        parser.error('output is required unless --verify is used')
    print(json.dumps(verify(args.archive) if args.verify else {'restored_files': restore(args.archive, args.output)}))
