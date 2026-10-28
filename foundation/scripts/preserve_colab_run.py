"""Bounded local checkpoint collector for an explicitly selected Colab output tree.

Downloads only training artifacts. Never changes remote jobs. Each received file
is verified against the remote manifest; previous versions remain content-addressed.
Keep the full final run archive as well: periodic recovery is intentionally partial.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import time


def safe_text(text):
    return re.sub(r'([?&]colab-runtime-proxy-token=)[^&\s]+', r'\1[redacted]', text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colab', type=Path, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--remote-root', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--interval', type=float, default=120)
    parser.add_argument('--hours', type=float, default=6)
    args = parser.parse_args()
    if not args.remote_root.startswith('/content/pupper-') or '..' in Path(args.remote_root).parts:
        parser.error('remote root must be an explicit /content/pupper-* output directory')
    if not 30 <= args.interval <= 600 or not 0 < args.hours <= 12:
        parser.error('invalid bounded interval/duration')
    args.out.mkdir(parents=True, exist_ok=True)
    versions = args.out/'objects'; versions.mkdir(exist_ok=True)
    index_path = args.out/'index.json'
    index = json.loads(index_path.read_text()) if index_path.exists() else dict(session=args.session, remote_root=args.remote_root, files={})
    if (index['session'], index['remote_root']) != (args.session, args.remote_root):
        raise ValueError('collector identity differs from existing archive')
    deadline = time.monotonic() + args.hours*3600
    remote = '''from pathlib import Path
import hashlib,json,time
root=Path(ROOT);files=[]
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
  if p.stat().st_mtime_ns!=stat.st_mtime_ns:continue
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
                files = json.loads(lines[0].split('=',1)[1]); received=0
                for item in files:
                    relative=Path(item['path'])
                    if relative.is_absolute() or '..' in relative.parts:raise ValueError('invalid manifest path')
                    digest=item['sha256']
                    if not re.fullmatch('[0-9a-f]{64}',digest):raise ValueError('invalid hash')
                    if index['files'].get(item['path'],{}).get('sha256')==digest:continue
                    destination=versions/digest
                    if not destination.exists():
                        pending=Path(temporary)/'download.partial'
                        call(['download','-s',args.session,str(Path(args.remote_root)/relative),str(pending)],120)
                        if hashlib.sha256(pending.read_bytes()).hexdigest()!=digest:
                            continue  # A running log/status may have changed during download.
                        pending.replace(destination)
                    receipt=dict(item,previous_sha256=index['files'].get(item['path'],{}).get('sha256'),
                                 received_utc=datetime.now(timezone.utc).isoformat())
                    with (args.out/'receipts.jsonl').open('a') as log:log.write(json.dumps(receipt)+'\n')
                    index['files'][item['path']]=item;received+=1
                    index['updated_utc']=datetime.now(timezone.utc).isoformat()
                    staged=args.out/'index.tmp';staged.write_text(json.dumps(index,indent=2)+'\n');staged.replace(index_path)
                event=dict(utc=datetime.now(timezone.utc).isoformat(),seen=len(files),received=received,preserved=len(index['files']))
            except (Exception, subprocess.TimeoutExpired) as error:
                event=dict(utc=datetime.now(timezone.utc).isoformat(),error=safe_text(str(error)))
            with (args.out/'collector.jsonl').open('a') as log:log.write(json.dumps(event)+'\n')
            print(json.dumps(event),flush=True)
            time.sleep(min(args.interval,max(0,deadline-time.monotonic())))


if __name__=='__main__':main()
