"""Bounded local checkpoint collector for an explicitly selected Colab output tree.

Downloads only training artifacts. Never changes remote jobs. Each received file
is verified against the remote manifest; previous versions remain content-addressed.
Keep the full final run archive as well: periodic recovery is intentionally partial.
A cycle's ok/verified counts cover only its returned remote manifest. --once exits
nonzero on any manifest/file error; corrupt existing objects are reported, never
silently replaced. The lock file records identity, not ongoing process liveness.
Keep this script beside restore_colab_snapshot.py. Large final archives use
download_colab_archive.py; periodic CLI requests still have a 120s per-file bound,
and --hours is checked between cycles, not a hard mid-transfer stop.
"""
import argparse
from contextlib import contextmanager
import fcntl
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time

from restore_colab_snapshot import validate_item, verify_file


def safe_text(text):
    return re.sub(r'([?&]colab-runtime-proxy-token=)[^&\s]+', r'\1[redacted]', text)


@contextmanager
def collector_lock(out, session, remote_root):
    """One writer per archive. The OS releases the lock if the collector dies."""
    path = out/'.collector.lock'
    if path.is_symlink() or (path.exists() and not path.is_file()):
        raise ValueError('Refusing non-regular collector lock')
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try: fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another collector owns this archive; no concurrent index writes') from None
        os.ftruncate(fd, 0)
        os.write(fd, json.dumps(dict(pid=os.getpid(), session=session, remote_root=remote_root,
                                    started_utc=datetime.now(timezone.utc).isoformat())).encode())
        os.fsync(fd)
        yield
    finally:
        os.close(fd)  # Keep the lock inode: unlinking would allow a second lock.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colab', type=Path, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--remote-root', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--interval', type=float, default=120)
    parser.add_argument('--hours', type=float, default=6)
    parser.add_argument('--once', action='store_true', help='Run one verified collection cycle and exit')
    args = parser.parse_args()
    if not args.remote_root.startswith('/content/pupper-') or '..' in Path(args.remote_root).parts:
        parser.error('remote root must be an explicit /content/pupper-* output directory')
    if not 30 <= args.interval <= 600 or not 0 < args.hours <= 12:
        parser.error('invalid bounded interval/duration')
    args.out.mkdir(parents=True, exist_ok=True)
    if args.out.is_symlink():
        parser.error('archive output must not be a symlink')
    with collector_lock(args.out, args.session, args.remote_root):
        return collect(args)


def collect(args):
    versions = args.out/'objects'; versions.mkdir(exist_ok=True)
    index_path = args.out/'index.json'
    if versions.is_symlink() or index_path.is_symlink():
        raise ValueError('Archive objects/index must not be symlinks')
    index = json.loads(index_path.read_text()) if index_path.exists() else dict(session=args.session, remote_root=args.remote_root, files={})
    if (index['session'], index['remote_root']) != (args.session, args.remote_root):
        raise ValueError('collector identity differs from existing archive')
    deadline = time.monotonic() + args.hours*3600
    remote = '''from pathlib import Path
import hashlib,json,time
root=Path(ROOT);files=[]
if not root.is_dir() or root.is_symlink():raise ValueError('Selected remote output directory is absent or is a symlink')
if root.exists():
 paths=[p for p in root.rglob('*') if p.is_file() and not p.is_symlink() and p.suffix in ('.json','.yaml','.yml','.pt','.npz','.onnx','.log') and time.time()-p.stat().st_mtime>3]
 checkpoints={}
 for p in paths:
  if p.suffix=='.pt':checkpoints.setdefault(p.parent,[]).append(p)
 kept={p for group in checkpoints.values() for p in sorted(group,key=lambda q:q.stat().st_mtime)[-2:]}
 for p in paths:
  if p.suffix=='.pt' and p not in kept:continue
  stat=p.stat();digest=hashlib.sha256()
  with p.open('rb') as f:
   for chunk in iter(lambda:f.read(1048576),b''):digest.update(chunk)
  after=p.stat()
  if (after.st_ino,after.st_size,after.st_mtime_ns)!=(stat.st_ino,stat.st_size,stat.st_mtime_ns):continue
  files.append(dict(path=str(p.relative_to(root)),bytes=stat.st_size,sha256=digest.hexdigest()))
print('RECOVERY_MANIFEST='+json.dumps(files))
'''.replace('ROOT', repr(args.remote_root))
    with tempfile.TemporaryDirectory(prefix='pupper-checkpoint-collector-') as temporary:
        script = Path(temporary)/'manifest.py';script.write_text(remote)
        def call(argv, timeout):
            result = subprocess.run([str(args.colab), *argv], capture_output=True, text=True, timeout=timeout)
            if result.returncode:
                raise RuntimeError(safe_text(result.stdout+result.stderr)[-1500:])
            return result.stdout
        while time.monotonic() < deadline:
            try:
                output = call(['exec','-s',args.session,'-f',str(script),'--timeout','60'],90)
                lines = [line for line in output.splitlines() if line.startswith('RECOVERY_MANIFEST=')]
                if len(lines)!=1:raise ValueError('remote manifest unavailable; no completion inferred')
                files = json.loads(lines[0].split('=',1)[1]); received=0; verified=0; failures=[]
                if not isinstance(files,list):raise ValueError('invalid remote manifest')
                # Recover small configuration files and checkpoints before large rollouts.
                files.sort(key=lambda item: (0 if Path(item['path']).suffix=='.pt' else 1 if Path(item['path']).suffix in ('.json','.yaml','.yml') else 2, item['bytes']))
                for item in files:
                    try:
                        relative,digest=validate_item(item['path'],item)
                        destination=versions/digest
                        # Even an unchanged index entry is not proof that its local object survived.
                        if destination.exists() or destination.is_symlink():
                            verify_file(destination,digest,item['bytes'])
                        else:
                            # Same filesystem, unique file, never reused after a failed download.
                            with tempfile.NamedTemporaryFile(dir=versions,prefix='download.',suffix='.partial',delete=False) as stream:
                                pending=Path(stream.name)
                            try:
                                call(['download','-s',args.session,str(Path(args.remote_root)/relative),str(pending)],120)
                                verify_file(pending,digest,item['bytes'])
                                with pending.open('rb') as stream:os.fsync(stream.fileno())
                                pending.replace(destination)
                            finally:pending.unlink(missing_ok=True)
                        verified+=1
                        if index['files'].get(item['path'],{}).get('sha256')==digest and index['files'][item['path']].get('bytes')==item['bytes']:continue
                        receipt=dict(item,previous_sha256=index['files'].get(item['path'],{}).get('sha256'),
                                     received_utc=datetime.now(timezone.utc).isoformat())
                        with (args.out/'receipts.jsonl').open('a') as log:
                            log.write(json.dumps(receipt)+'\n');log.flush();os.fsync(log.fileno())
                        next_index=dict(index,files={**index['files'],item['path']:item},updated_utc=datetime.now(timezone.utc).isoformat())
                        staged=args.out/'index.tmp'
                        if staged.is_symlink():raise ValueError('Refusing symlink index staging file')
                        with staged.open('w') as stream:
                            stream.write(json.dumps(next_index,indent=2)+'\n');stream.flush();os.fsync(stream.fileno())
                        staged.replace(index_path)
                        index=next_index;received+=1
                    except Exception as error:
                        failures.append(dict(path=item.get('path'),error=safe_text(str(error))))
                        continue  # One unavailable trace must not discard other recoverable files.
                event=dict(utc=datetime.now(timezone.utc).isoformat(),pid=os.getpid(),seen=len(files),verified=verified,received=received,preserved=len(index['files']),file_errors=failures,ok=not failures)
            except (Exception, subprocess.TimeoutExpired) as error:
                event=dict(utc=datetime.now(timezone.utc).isoformat(),pid=os.getpid(),ok=False,error=safe_text(str(error)))
            with (args.out/'collector.jsonl').open('a') as log:log.write(json.dumps(event)+'\n')
            print(json.dumps(event),flush=True)
            if args.once:return 0 if event['ok'] else 1
            time.sleep(min(args.interval,max(0,deadline-time.monotonic())))
        return 0 if event['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
