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
    assert result.returncode == 0, result.stderr
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
