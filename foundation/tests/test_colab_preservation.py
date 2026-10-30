"""A failed remote file must not prevent recovery of the other artifacts."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]


def test_file_failure_does_not_abort_verified_recovery_and_restore(tmp_path):
    data = b'{"checkpoint_identity":"kept"}'
    manifest = [dict(path='unavailable.pt',bytes=4,sha256='a'*64),
                dict(path='status.json',bytes=len(data),sha256=hashlib.sha256(data).hexdigest())]
    fake = tmp_path/'colab'
    fake.write_text(f'''#!{sys.executable}
import sys,pathlib
if sys.argv[1]=='exec':
 print('RECOVERY_MANIFEST='+{json.dumps(manifest)!r})
elif sys.argv[1]=='download':
 if sys.argv[-2].endswith('.pt'):
  print('simulated unavailable remote checkpoint',file=sys.stderr);sys.exit(1)
 pathlib.Path(sys.argv[-1]).write_bytes({data!r})
else:sys.exit(2)
''')
    fake.chmod(0o700)
    archive = tmp_path/'archive'
    result = subprocess.run([sys.executable, str(ROOT/'foundation/scripts/preserve_colab_run.py'),
        '--colab',str(fake),'--session','test','--remote-root','/content/pupper-test',
        '--out',str(archive),'--once'],capture_output=True,text=True)
    assert result.returncode == 1, result.stderr
    index = json.loads((archive/'index.json').read_text())
    assert set(index['files']) == {'status.json'}
    event = json.loads((archive/'collector.jsonl').read_text())
    assert event['received'] == 1 and event['file_errors'][0]['path'] == 'unavailable.pt'
    spec = importlib.util.spec_from_file_location('restore', ROOT/'foundation/scripts/restore_colab_snapshot.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    recovered = tmp_path/'recovered'
    assert module.restore(archive,recovered) == 1
    assert (recovered/'status.json').read_bytes() == data
    # Corruption must be detected before a fresh restoration directory is made.
    (archive/'objects'/index['files']['status.json']['sha256']).write_bytes(b'corrupt')
    with pytest.raises(ValueError, match='verification failed'):
        module.restore(archive,tmp_path/'corrupt-restore')
    assert not (tmp_path/'corrupt-restore').exists()


def load_restore():
    spec = importlib.util.spec_from_file_location('restore', ROOT/'foundation/scripts/restore_colab_snapshot.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def make_fake(tmp_path, manifest, bodies, failures=(), manifest_failure=False):
    fake = tmp_path/'fake-colab'
    fake.write_text(f'''#!{sys.executable}
import sys,pathlib
if sys.argv[1]=='exec':
 if {manifest_failure!r}:print('manifest failed');sys.exit(1)
 print('RECOVERY_MANIFEST='+{json.dumps(manifest)!r})
elif sys.argv[1]=='download':
 name=pathlib.Path(sys.argv[-2]).name
 body={bodies!r}.get(name)
 if body is not None:pathlib.Path(sys.argv[-1]).write_bytes(body)
 if name in {failures!r}:sys.exit(1)
else:sys.exit(2)
''')
    fake.chmod(0o700)
    return fake


def collect(fake, archive):
    return subprocess.run([sys.executable, str(ROOT/'foundation/scripts/preserve_colab_run.py'),
        '--colab',str(fake),'--session','test','--remote-root','/content/pupper-test',
        '--out',str(archive),'--once'],capture_output=True,text=True)


def item(name, data):
    return dict(path=name,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())


@pytest.mark.parametrize('indexed', [False, True])
def test_corrupt_cached_object_is_never_trusted_even_if_hash_unchanged(tmp_path, indexed):
    expected=item('old.json',b'original')
    archive=tmp_path/'archive';(archive/'objects').mkdir(parents=True)
    corrupt=archive/'objects'/expected['sha256'];corrupt.write_bytes(b'corrupt!')
    index=dict(session='test',remote_root='/content/pupper-test',files={'old.json':expected} if indexed else {})
    (archive/'index.json').write_text(json.dumps(index))
    good=item('good.json',b'good')
    fake=make_fake(tmp_path,[expected,good],{'old.json':b'original','good.json':b'good'})
    result=collect(fake,archive)
    assert result.returncode==1
    event=json.loads(result.stdout)
    assert not event['ok'] and event['verified']==1 and event['received']==1
    assert event['file_errors'][0]['path']=='old.json'
    assert corrupt.read_bytes()==b'corrupt!'  # Preserve evidence; no silent overwrite.
    assert json.loads((archive/'index.json').read_text())['files']['good.json']==good


@pytest.mark.parametrize('failure', ['hash','size','partial','no_write','manifest'])
def test_bad_download_and_manifest_fail_one_shot_without_poisoning_next_file(tmp_path, failure):
    bad=item('bad.json',b'expected');good=item('good.json',b'good')
    if failure=='size':bad['bytes']+=1
    body=b'changed' if failure=='hash' else b'expected'
    bodies={'bad.json':body,'good.json':b'good'}
    if failure=='no_write':bodies.pop('bad.json')
    fake=make_fake(tmp_path,[bad,good],bodies,failures=('bad.json',) if failure=='partial' else (),manifest_failure=failure=='manifest')
    archive=tmp_path/'archive';result=collect(fake,archive)
    assert result.returncode==1,result.stderr
    event=json.loads(result.stdout);assert not event['ok']
    assert not list((archive/'objects').glob('*.partial'))
    assert not (archive/'objects'/bad['sha256']).exists()
    if failure!='manifest':
        assert event['file_errors'][0]['path']=='bad.json'
        assert set(json.loads((archive/'index.json').read_text())['files'])=={'good.json'}


def test_missing_cached_object_is_downloaded_and_verified_without_losing_index(tmp_path):
    entry=item('checkpoint.pt',b'weights');archive=tmp_path/'archive';archive.mkdir()
    (archive/'index.json').write_text(json.dumps(dict(session='test',remote_root='/content/pupper-test',files={entry['path']:entry})))
    fake=make_fake(tmp_path,[entry],{'checkpoint.pt':b'weights'})
    result=collect(fake,archive)
    assert result.returncode==0,result.stderr
    assert json.loads(result.stdout)['verified']==1
    assert load_restore().verify(archive)['verified_files']==1


def test_second_collector_refuses_shared_archive_before_calling_cli(tmp_path):
    import fcntl
    archive=tmp_path/'archive';archive.mkdir()
    fake=make_fake(tmp_path,[],{})
    with (archive/'.collector.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        result=collect(fake,archive)
    assert result.returncode!=0 and 'Another collector owns' in result.stderr
    assert not (archive/'index.json').exists() and not (archive/'collector.jsonl').exists()


def test_offline_verify_writes_nothing_and_detects_corruption(tmp_path):
    entry=item('status.json',b'correct');archive=tmp_path/'archive'
    assert collect(make_fake(tmp_path,[entry],{'status.json':b'correct'}),archive).returncode==0
    before={p.relative_to(archive):p.read_bytes() for p in archive.rglob('*') if p.is_file()}
    command=[sys.executable,str(ROOT/'foundation/scripts/restore_colab_snapshot.py'),str(archive),'--verify']
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==0 and json.loads(result.stdout)['verified_files']==1
    after={p.relative_to(archive):p.read_bytes() for p in archive.rglob('*') if p.is_file()}
    assert before==after
    (archive/'objects'/entry['sha256']).write_bytes(b'corrupt')
    assert subprocess.run(command,capture_output=True).returncode!=0


def test_restore_detects_copy_corruption_and_never_emits_success_receipt(tmp_path, monkeypatch):
    entry=item('status.json',b'correct');archive=tmp_path/'archive'
    assert collect(make_fake(tmp_path,[entry],{'status.json':b'correct'}),archive).returncode==0
    module=load_restore()
    monkeypatch.setattr(module.shutil,'copyfile',lambda src,dst:dst.write_bytes(b'corrupt'))
    with pytest.raises(ValueError,match='verification failed'):module.restore(archive,tmp_path/'restored')
    assert not (tmp_path/'restored/recovery-snapshot.json').exists()


@pytest.mark.parametrize('name', ['.', '../outside', '/absolute', 'recovery-snapshot.json'])
def test_invalid_or_receipt_colliding_paths_refuse_before_restore_output(tmp_path,name):
    entry=item(name,b'valid');archive=tmp_path/'archive';(archive/'objects').mkdir(parents=True)
    (archive/'objects'/entry['sha256']).write_bytes(b'valid')
    (archive/'index.json').write_text(json.dumps(dict(files={name:entry})))
    with pytest.raises(ValueError):load_restore().restore(archive,tmp_path/'output')
    assert not (tmp_path/'output').exists()
