"""Download a large Colab archive in bounded, resumable, verified chunks.

The official Contents API returns a whole base64 JSON response without a timeout.
Keep its authentication/CLI, but bound each request and verify the complete file.
This reads one explicit archive and writes transfer chunks, never changes jobs.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):digest.update(block)
    return digest.hexdigest()


def safe_text(value):
    return re.sub(r'([?&]colab-runtime-proxy-token=)[^&\s]+',r'\1[redacted]',value)


def validate_manifest(manifest, expected_sha, expected_bytes):
    if manifest.get('sha256')!=expected_sha or manifest.get('bytes')!=expected_bytes:
        raise ValueError('Archive identity differs from independent receipt')
    parts=manifest.get('parts',[])
    if not parts or sum(p['bytes'] for p in parts)!=expected_bytes:
        raise ValueError('Incomplete chunk manifest')
    for i,part in enumerate(parts):
        if (part['name']!=f'part-{i:05d}.bin' or not 0<part['bytes']<=64*1024*1024
                or not re.fullmatch('[0-9a-f]{64}',part['sha256'])
                or part['path']!=f'/content/pupper-transfer-chunks/{expected_sha}/{part["name"]}'):
            raise ValueError('Invalid chunk identity/path')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--colab',type=Path,required=True);parser.add_argument('--session',required=True)
    parser.add_argument('--remote-archive',required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--sha256',required=True);parser.add_argument('--bytes',type=int,required=True)
    parser.add_argument('--chunk-mib',type=int,default=32);parser.add_argument('--workers',type=int,default=2)
    args=parser.parse_args();args.colab=args.colab.resolve();args.out=args.out.resolve()
    if (not args.remote_archive.startswith('/content/pupper-') or '..' in Path(args.remote_archive).parts
            or not re.fullmatch('[0-9a-f]{64}',args.sha256) or args.bytes<=0
            or not 4<=args.chunk_mib<=64 or not 1<=args.workers<=3):
        parser.error('Invalid explicit archive, hash, size, chunk size or concurrency')
    if args.out.exists():
        if args.out.stat().st_size==args.bytes and sha(args.out)==args.sha256:
            print(json.dumps({'verified_existing':str(args.out),'sha256':args.sha256}));return
        raise ValueError('Refusing to overwrite an existing unverified archive')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    cache=args.out.with_name(args.out.name+'.parts');cache.mkdir(exist_ok=True)
    def call(command, timeout):
        result=subprocess.run([str(args.colab),*command],capture_output=True,text=True,timeout=timeout)
        if result.returncode:raise RuntimeError(safe_text(result.stdout+result.stderr)[-2000:])
        return result.stdout
    remote='''from pathlib import Path
import hashlib,json
source=Path(SOURCE);expected=EXPECTED;expected_bytes=EXPECTED_BYTES;chunk_size=CHUNK_SIZE
assert source.stat().st_size==expected_bytes
folder=Path('/content/pupper-transfer-chunks')/expected;folder.mkdir(parents=True,exist_ok=True)
parts=[];full=hashlib.sha256()
with source.open('rb') as stream:
 while True:
  data=stream.read(chunk_size)
  if not data:break
  full.update(data);digest=hashlib.sha256(data).hexdigest();p=folder/f'part-{len(parts):05d}.bin'
  if not p.exists() or p.stat().st_size!=len(data) or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
   temp=p.with_suffix('.partial');temp.write_bytes(data);temp.replace(p)
  parts.append(dict(name=p.name,path=str(p),bytes=len(data),sha256=digest))
assert full.hexdigest()==expected,'Source archive changed'
print('COLAB_ARCHIVE_PARTS='+json.dumps(dict(sha256=expected,bytes=expected_bytes,parts=parts)))
'''.replace('SOURCE',repr(args.remote_archive)).replace('EXPECTED_BYTES',str(args.bytes)).replace('EXPECTED',repr(args.sha256)).replace('CHUNK_SIZE',str(args.chunk_mib*1024*1024))
    with tempfile.TemporaryDirectory(prefix='pupper-archive-transfer-') as temporary:
        script=Path(temporary)/'split.py';script.write_text(remote)
        result=call(['exec','-s',args.session,'-f',str(script),'--timeout','60'],90)
        matches=[line.split('=',1)[1] for line in result.splitlines() if line.startswith('COLAB_ARCHIVE_PARTS=')]
        if len(matches)!=1:raise ValueError('Missing explicit chunk manifest')
        manifest=json.loads(matches[0]);validate_manifest(manifest,args.sha256,args.bytes)
        (cache/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        def download(part):
            target=cache/part['name']
            if target.exists() and target.stat().st_size==part['bytes'] and sha(target)==part['sha256']:return part['name'],'cached'
            pending=target.with_suffix('.partial')
            call(['download','-s',args.session,part['path'],str(pending)],120)
            if pending.stat().st_size!=part['bytes'] or sha(pending)!=part['sha256']:raise ValueError('Chunk hash/size mismatch: '+part['name'])
            pending.replace(target);return part['name'],'downloaded'
        errors=[];completed=[]
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures={pool.submit(download,p):p for p in manifest['parts']}
            for future in as_completed(futures):
                try:
                    name,kind=future.result();completed.append(name);print(json.dumps({'part':name,'state':kind,'completed':len(completed),'total':len(manifest['parts'])}),flush=True)
                except Exception as error:errors.append({'part':futures[future]['name'],'error':safe_text(str(error))})
                (cache/'progress.json').write_text(json.dumps({'completed':completed,'errors':errors},indent=2)+'\n')
        if errors:raise RuntimeError(f'{len(errors)} chunks unavailable; verified chunks retained for resumption')
        pending=args.out.with_name(args.out.name+'.partial')
        with pending.open('wb') as destination:
            for part in manifest['parts']:
                with (cache/part['name']).open('rb') as source:
                    for block in iter(lambda:source.read(1048576),b''):destination.write(block)
        if pending.stat().st_size!=args.bytes or sha(pending)!=args.sha256:raise ValueError('Combined archive does not match independent receipt')
        pending.replace(args.out)
        (cache/'verified.json').write_text(json.dumps({'archive':str(args.out),'bytes':args.bytes,'sha256':args.sha256},indent=2)+'\n')
        print(json.dumps({'verified_archive':str(args.out),'bytes':args.bytes,'sha256':args.sha256}),flush=True)


if __name__=='__main__':main()
