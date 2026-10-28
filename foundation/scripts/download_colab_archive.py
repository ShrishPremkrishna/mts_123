"""Download a large Colab archive in bounded, resumable, verified chunks.

The official Contents API returns a whole base64 JSON response without a timeout.
Keep its authentication/CLI, but bound each request and verify the complete file.
This reads one explicit archive and writes transfer chunks, never changes jobs.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import tempfile


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):digest.update(block)
    return digest.hexdigest()


def safe_text(value):
    return re.sub(r'([?&]colab-runtime-proxy-token=)[^&\s]+',r'\1[redacted]',value)


def regular_or_absent(path):
    """Never follow cache/output symlinks or write through special files."""
    if path.is_symlink() or (path.exists() and not path.is_file()):
        raise ValueError(f'Refusing non-regular transfer file: {path}')


def atomic_json(path, value):
    regular_or_absent(path)
    with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,prefix=path.name+'.',suffix='.partial',delete=False) as stream:
        pending=Path(stream.name)
        try:
            json.dump(value,stream,indent=2);stream.write('\n');stream.flush()
            os.fsync(stream.fileno());pending.replace(path)
        finally:pending.unlink(missing_ok=True)


@contextmanager
def transfer_lock(cache):
    # An OS lock releases automatically on crash. Leave its inode in place so
    # a second process cannot lock an unlinked replacement while we still run.
    if cache.is_symlink() or (cache.exists() and not cache.is_dir()):
        raise ValueError('Refusing non-directory or symlink transfer cache')
    cache.mkdir(exist_ok=True)
    lock=cache/'.download.lock';regular_or_absent(lock)
    fd=os.open(lock,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):raise ValueError('Invalid transfer lock file')
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Another download owns this output/cache; resume after it exits') from None
        yield
    finally:os.close(fd)


def validate_manifest(manifest, expected_sha, expected_bytes, chunk_bytes=32*1024*1024):
    if not isinstance(manifest,dict) or manifest.get('sha256')!=expected_sha or type(manifest.get('bytes')) is not int or manifest['bytes']!=expected_bytes:
        raise ValueError('Archive identity differs from independent receipt')
    parts=manifest.get('parts')
    if not isinstance(parts,list) or not parts:raise ValueError('Incomplete chunk manifest')
    folder=f'/content/pupper-transfer-chunks/{expected_sha}/chunks-{chunk_bytes}'
    for i,part in enumerate(parts):
        if (not isinstance(part,dict) or part.get('name')!=f'part-{i:05d}.bin'
                or type(part.get('bytes')) is not int or not 0<part['bytes']<=chunk_bytes
                or not isinstance(part.get('sha256'),str) or not re.fullmatch('[0-9a-f]{64}',part['sha256'])
                or part.get('path')!=f'{folder}/{part["name"]}'):
            raise ValueError('Invalid chunk identity/path')
    if sum(p['bytes'] for p in parts)!=expected_bytes:raise ValueError('Incomplete chunk manifest')


def split_script(source, expected, expected_bytes, chunk_size):
    settings=json.dumps(dict(source=source,expected=expected,expected_bytes=expected_bytes,chunk_size=chunk_size))
    # A chunk-size namespace prevents concurrent downloads with different sizes
    # from replacing each other's remote parts. Unique temporary files make
    # identical concurrent split operations safe as well.
    return '''from pathlib import Path
import hashlib,json,os,tempfile
settings=json.loads(SETTINGS_JSON)
source=Path(settings['source']);expected=settings['expected'];expected_bytes=settings['expected_bytes'];chunk_size=settings['chunk_size']
assert source.is_file() and source.stat().st_size==expected_bytes
folder=Path('/content/pupper-transfer-chunks')/expected/f'chunks-{chunk_size}';folder.mkdir(parents=True,exist_ok=True)
parts=[];full=hashlib.sha256()
with source.open('rb') as stream:
 while True:
  data=stream.read(chunk_size)
  if not data:break
  full.update(data);digest=hashlib.sha256(data).hexdigest();p=folder/f'part-{len(parts):05d}.bin'
  assert not p.is_symlink(),'Refusing symlink chunk'
  if not p.exists() or p.stat().st_size!=len(data) or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
   with tempfile.NamedTemporaryFile(dir=folder,prefix=p.name+'.',suffix='.partial',delete=False) as temp:
    pending=Path(temp.name)
    try:temp.write(data);temp.flush();os.fsync(temp.fileno());pending.replace(p)
    finally:pending.unlink(missing_ok=True)
  parts.append(dict(name=p.name,path=str(p),bytes=len(data),sha256=digest))
assert full.hexdigest()==expected,'Source archive changed'
print('COLAB_ARCHIVE_PARTS='+json.dumps(dict(sha256=expected,bytes=expected_bytes,parts=parts)))
'''.replace('SETTINGS_JSON',repr(settings))


def download_archive(args):
    regular_or_absent(args.out)
    if args.out.exists():
        if args.out.stat().st_size==args.bytes and sha(args.out)==args.sha256:
            print(json.dumps({'verified_existing':str(args.out),'sha256':args.sha256}));return
        raise ValueError('Refusing to overwrite an existing unverified archive')
    cache=args.out.with_name(args.out.name+'.parts')
    def call(command, timeout):
        result=subprocess.run([str(args.colab),*command],capture_output=True,text=True,timeout=timeout)
        if result.returncode:raise RuntimeError(safe_text(result.stdout+result.stderr)[-2000:])
        return result.stdout
    remote=split_script(args.remote_archive,args.sha256,args.bytes,args.chunk_mib*1024*1024)
    with tempfile.TemporaryDirectory(prefix='pupper-archive-transfer-') as temporary:
        script=Path(temporary)/'split.py';script.write_text(remote)
        result=call(['exec','-s',args.session,'-f',str(script),'--timeout','60'],90)
        matches=[line.split('=',1)[1] for line in result.splitlines() if line.startswith('COLAB_ARCHIVE_PARTS=')]
        if len(matches)!=1:raise ValueError('Missing explicit chunk manifest')
        manifest=json.loads(matches[0]);validate_manifest(manifest,args.sha256,args.bytes,args.chunk_mib*1024*1024)
        atomic_json(cache/'manifest.json',manifest)
        def download(part):
            target=cache/part['name'];regular_or_absent(target)
            if target.exists() and target.stat().st_size==part['bytes'] and sha(target)==part['sha256']:return part['name'],'cached'
            with tempfile.NamedTemporaryFile(dir=cache,prefix=target.name+'.',suffix='.partial',delete=False) as temp:
                pending=Path(temp.name)
            try:
                call(['download','-s',args.session,part['path'],str(pending)],120)
                regular_or_absent(pending)
                if pending.stat().st_size!=part['bytes'] or sha(pending)!=part['sha256']:raise ValueError('Chunk hash/size mismatch: '+part['name'])
                pending.replace(target);return part['name'],'downloaded'
            finally:pending.unlink(missing_ok=True)
        errors=[];completed=[]
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures={pool.submit(download,p):p for p in manifest['parts']}
            for future in as_completed(futures):
                try:
                    name,kind=future.result();completed.append(name);print(json.dumps({'part':name,'state':kind,'completed':len(completed),'total':len(manifest['parts'])}),flush=True)
                except Exception as error:errors.append({'part':futures[future]['name'],'error':safe_text(str(error))})
                atomic_json(cache/'progress.json',{'completed':completed,'errors':errors})
        if errors:raise RuntimeError(f'{len(errors)} chunks unavailable; verified chunks retained for resumption')
        with tempfile.NamedTemporaryFile(dir=args.out.parent,prefix=args.out.name+'.',suffix='.partial',delete=False) as destination:
            pending=Path(destination.name)
            try:
                for part in manifest['parts']:
                    source_path=cache/part['name'];regular_or_absent(source_path)
                    with source_path.open('rb') as source:
                        for block in iter(lambda:source.read(1048576),b''):destination.write(block)
                destination.flush();os.fsync(destination.fileno())
                if pending.stat().st_size!=args.bytes or sha(pending)!=args.sha256:raise ValueError('Combined archive does not match independent receipt')
                # Publish the verified inode without replacing a file created
                # after our initial check. A regular rename would overwrite it.
                os.link(pending,args.out)
            finally:pending.unlink(missing_ok=True)
        atomic_json(cache/'verified.json',{'archive':str(args.out),'bytes':args.bytes,'sha256':args.sha256})
        print(json.dumps({'verified_archive':str(args.out),'bytes':args.bytes,'sha256':args.sha256}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colab',type=Path,required=True);parser.add_argument('--session',required=True)
    parser.add_argument('--remote-archive',required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--sha256',required=True);parser.add_argument('--bytes',type=int,required=True)
    parser.add_argument('--chunk-mib',type=int,default=32);parser.add_argument('--workers',type=int,default=2)
    args=parser.parse_args();args.colab=args.colab.resolve();args.out=args.out.parent.resolve()/args.out.name
    if (not args.remote_archive.startswith('/content/pupper-') or '..' in Path(args.remote_archive).parts
            or not re.fullmatch('[0-9a-f]{64}',args.sha256) or args.bytes<=0
            or not 4<=args.chunk_mib<=64 or not 1<=args.workers<=3):
        parser.error('Invalid explicit archive, hash, size, chunk size or concurrency')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with transfer_lock(args.out.with_name(args.out.name+'.parts')):download_archive(args)


if __name__=='__main__':main()
