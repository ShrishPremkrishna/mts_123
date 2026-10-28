import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import pytest

ROOT=Path(__file__).resolve().parents[2]
SCRIPT=ROOT/'foundation/scripts/download_colab_archive.py'


def module():
    spec=importlib.util.spec_from_file_location('archive_download',SCRIPT)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def setup_cli(tmp_path, corrupt=False, fail_once=True, wrong_whole_hash=False, concurrent_output=False):
    chunks=[b'hello',b' world'];whole=b''.join(chunks)
    digest='0'*64 if wrong_whole_hash else hashlib.sha256(whole).hexdigest()
    manifest=dict(sha256=digest,bytes=len(whole),parts=[dict(name=f'part-{i:05d}.bin',
        path=f'/content/pupper-transfer-chunks/{digest}/chunks-33554432/part-{i:05d}.bin',bytes=len(data),
        sha256=hashlib.sha256(data).hexdigest()) for i,data in enumerate(chunks)])
    fake=tmp_path/'colab';flag=tmp_path/'failed-once';events=tmp_path/'downloads';output=tmp_path/'run.tar.gz'
    fake.write_text(f'''#!{sys.executable}
import pathlib,sys
if sys.argv[1]=='exec':print('COLAB_ARCHIVE_PARTS='+{json.dumps(manifest)!r})
else:
 name=pathlib.Path(sys.argv[-2]).name
 with pathlib.Path({str(events)!r}).open('a') as f:f.write(name+'\\n')
 flag=pathlib.Path({str(flag)!r})
 if {fail_once!r} and name=='part-00001.bin' and not flag.exists():flag.touch();sys.exit(1)
 data={chunks!r}[int(name[5:10])]
 if {corrupt!r}:data=b'bad'
 if {concurrent_output!r} and name=='part-00000.bin':pathlib.Path({str(output)!r}).write_bytes(b'other owner')
 pathlib.Path(sys.argv[-1]).write_bytes(data)
''');fake.chmod(0o700)
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


def test_corrupt_cached_chunk_is_repaired_on_resume(tmp_path):
    command,output,events,whole,_=setup_cli(tmp_path)
    assert subprocess.run(command,capture_output=True).returncode!=0
    output.with_name(output.name+'.parts').joinpath('part-00000.bin').write_bytes(b'wrong')
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert output.read_bytes()==whole
    assert events.read_text().splitlines().count('part-00000.bin')==2


def test_individually_valid_chunks_cannot_override_independent_whole_file_receipt(tmp_path):
    command,output,_,_,_=setup_cli(tmp_path,fail_once=False,wrong_whole_hash=True)
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode!=0 and 'Combined archive does not match' in result.stderr
    assert not output.exists()
    cache=output.with_name(output.name+'.parts')
    assert all((cache/f'part-{i:05d}.bin').exists() for i in range(2))
    assert not (cache/'verified.json').exists()


def test_manifest_cannot_redirect_local_chunks_or_change_archive_identity(tmp_path):
    _,_,_,whole,manifest=setup_cli(tmp_path)
    helper=module();helper.validate_manifest(manifest,manifest['sha256'],len(whole))
    manifest['parts'][0]['name']='../escape'
    with pytest.raises(ValueError,match='chunk'):helper.validate_manifest(manifest,manifest['sha256'],len(whole))


@pytest.mark.parametrize('change',[{'bytes':True},{'bytes':5.0},{'sha256':None},{'path':'/content/pupper-transfer-chunks/wrong/part-00000.bin'}])
def test_malformed_manifest_is_explicitly_rejected(tmp_path,change):
    _,_,_,whole,manifest=setup_cli(tmp_path);manifest['parts'][0].update(change)
    with pytest.raises(ValueError,match='chunk'):module().validate_manifest(manifest,manifest['sha256'],len(whole))


@pytest.mark.parametrize('location',['output','cache','part','lock'])
def test_transfer_does_not_follow_output_or_cache_symlinks(tmp_path,location):
    command,output,_,_,_=setup_cli(tmp_path,fail_once=False)
    cache=output.with_name(output.name+'.parts');outside=tmp_path/'unrelated';outside.mkdir()
    victim=outside/'keep';victim.write_bytes(b'unrelated data')
    if location=='output':output.symlink_to(victim)
    elif location=='cache':cache.symlink_to(outside,target_is_directory=True)
    else:
        cache.mkdir();(cache/('part-00000.bin' if location=='part' else '.download.lock')).symlink_to(victim)
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode!=0
    assert victim.read_bytes()==b'unrelated data'
    assert output.is_symlink() if location=='output' else not output.exists()
    assert sorted(p.name for p in outside.iterdir())==['keep']


def test_second_process_cannot_mutate_active_output_cache(tmp_path):
    command,output,events,_,_=setup_cli(tmp_path,fail_once=False)
    cache=output.with_name(output.name+'.parts');cache.mkdir()
    with (cache/'.download.lock').open('w') as held:
        fcntl.flock(held,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=subprocess.run(command,capture_output=True,text=True,timeout=5)
        assert result.returncode!=0 and 'Another download owns' in result.stderr
        assert not events.exists() and not output.exists()
    # Closing the lock permits a retry; no stale lock cleanup is necessary.
    assert subprocess.run(command,capture_output=True).returncode==0


def test_publication_does_not_replace_output_created_during_download(tmp_path):
    command,output,_,_,_=setup_cli(tmp_path,fail_once=False,concurrent_output=True)
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode!=0 and output.read_bytes()==b'other owner'
    assert not output.with_name(output.name+'.parts').joinpath('verified.json').exists()


def test_remote_split_is_safe_for_same_and_different_chunk_sizes(tmp_path):
    helper=module();source=tmp_path/"SOURCE EXPECTED ' CHUNK_SIZE.tar";data=b'actual source data'*100
    source.write_bytes(data);digest=hashlib.sha256(data).hexdigest();remote=tmp_path/'remote'
    def split(chunk_bytes):
        script=helper.split_script(str(source),digest,len(data),chunk_bytes)
        # Execute the actual remote program with an isolated local remote root.
        script=script.replace("Path('/content/pupper-transfer-chunks')",f'Path({str(remote)!r})')
        result=subprocess.run([sys.executable,'-c',script],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        manifest=json.loads(result.stdout.split('COLAB_ARCHIVE_PARTS=',1)[1])
        assert b''.join(Path(p['path']).read_bytes() for p in manifest['parts'])==data
        return Path(manifest['parts'][0]['path']).parent
    with ThreadPoolExecutor(max_workers=3) as pool:folders=list(pool.map(split,[256,256,512]))
    assert folders[0]==folders[1] and folders[1]!=folders[2]
    assert not list(remote.rglob('*.partial'))
