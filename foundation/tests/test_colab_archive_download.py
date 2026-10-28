import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[2]
SCRIPT=ROOT/'foundation/scripts/download_colab_archive.py'


def setup_cli(tmp_path, corrupt=False):
    chunks=[b'hello',b' world'];whole=b''.join(chunks);digest=hashlib.sha256(whole).hexdigest()
    manifest=dict(sha256=digest,bytes=len(whole),parts=[dict(name=f'part-{i:05d}.bin',
        path=f'/content/pupper-transfer-chunks/{digest}/part-{i:05d}.bin',bytes=len(data),
        sha256=hashlib.sha256(data).hexdigest()) for i,data in enumerate(chunks)])
    fake=tmp_path/'colab';flag=tmp_path/'failed-once';events=tmp_path/'downloads'
    fake.write_text(f'''#!{sys.executable}
import pathlib,sys
if sys.argv[1]=='exec':print('COLAB_ARCHIVE_PARTS='+{json.dumps(manifest)!r})
else:
 name=pathlib.Path(sys.argv[-2]).name
 with pathlib.Path({str(events)!r}).open('a') as f:f.write(name+'\\n')
 flag=pathlib.Path({str(flag)!r})
 if name=='part-00001.bin' and not flag.exists():flag.touch();sys.exit(1)
 data={chunks!r}[int(name[5:10])]
 if {corrupt!r}:data=b'bad'
 pathlib.Path(sys.argv[-1]).write_bytes(data)
''');fake.chmod(0o700)
    output=tmp_path/'run.tar.gz'
    command=[sys.executable,str(SCRIPT),'--colab',str(fake),'--session','test',
             '--remote-archive','/content/pupper-test.tar.gz','--out',str(output),
             '--sha256',digest,'--bytes',str(len(whole))]
    return command,output,events,whole,manifest


def test_failed_chunk_resumes_and_combined_archive_is_verified(tmp_path):
    command,output,events,whole,_=setup_cli(tmp_path)
    first=subprocess.run(command,capture_output=True,text=True)
    assert first.returncode!=0 and not output.exists()
    second=subprocess.run(command,capture_output=True,text=True)
    assert second.returncode==0,second.stderr
    assert output.read_bytes()==whole
    names=events.read_text().splitlines()
    assert names.count('part-00000.bin')==1 and names.count('part-00001.bin')==2
    third=subprocess.run(command,capture_output=True,text=True)
    assert third.returncode==0 and 'verified_existing' in third.stdout
    assert events.read_text().splitlines()==names


def test_corrupted_chunk_never_publishes_archive(tmp_path):
    command,output,_,_,_=setup_cli(tmp_path,corrupt=True)
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode!=0 and not output.exists()
    progress=json.loads(output.with_name(output.name+'.parts').joinpath('progress.json').read_text())
    assert len(progress['errors'])==2


def test_manifest_cannot_redirect_local_chunks_or_change_archive_identity(tmp_path):
    _,_,_,whole,manifest=setup_cli(tmp_path)
    spec=importlib.util.spec_from_file_location('archive_download',SCRIPT)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.validate_manifest(manifest,manifest['sha256'],len(whole))
    manifest['parts'][0]['name']='../escape'
    with pytest.raises(ValueError,match='chunk'):module.validate_manifest(manifest,manifest['sha256'],len(whole))
