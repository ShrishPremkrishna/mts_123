"""Queue the reviewed tracking experiments after the StableGait finetune.

Run detached on the existing experiment VM. Preserve all inputs and failed outputs.
The generated motion must be rechecked with corrected body contacts before this launch.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

repo = Path('/content/pupper-foundation')
project = repo/'pupper-mjlab'
python = project/'.venv/bin/python'
out = Path('/content/pupper-tracking-followups')
out.mkdir(exist_ok=False)
spider = Path('/content/recovered-policies/spider_clips')
generated = Path('/content/pupper-generated-results/standing-dance-candidates/sample_03/secret_dance.npz')
expected = {'model_2999.pt':'fdc3dd10f879178db2d49872c6c244efd8f41edd53d6f3add6ac02c21c007b39',
            'spider_walk.npz':'2a51d40c03dbbd373ed09b0b60ee7b5c7f6864f48e77d8a5153a4d5d1758cf6f'}
for name, digest in expected.items():
    assert hashlib.sha256((spider/name).read_bytes()).hexdigest() == digest
selection = json.loads((repo/'foundation/results/generated-candidate-selection.json').read_text())
assert hashlib.sha256(generated.read_bytes()).hexdigest() == selection['motion_sha256']
assert selection.get('corrected_contacts_training_eligible') is True
os.environ['MUJOCO_GL'] = os.environ['PYOPENGL_PLATFORM'] = 'egl'
os.environ['PYTHONUNBUFFERED'] = '1'
status = {'stage':'waiting_for_stablegait_finetune','jobs':{},'hardware_validated':False,
          'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'git_revision_at_launch':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()}

def save():
    temporary=out/'status.tmp'
    temporary.write_text(json.dumps(status,indent=2)+'\n')
    temporary.replace(out/'status.json')

def execute(name, command):
    status['stage']=name
    item={'command':list(map(str,command)),'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
    status['jobs'][name]=item;save()
    with (out/f'{name}.log').open('w') as log:
        result=subprocess.run(item['command'],cwd=project,stdout=log,stderr=subprocess.STDOUT)
    item.update(returncode=result.returncode,finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    save()
    if result.returncode:raise RuntimeError(f'{name} failed; see saved log')

save()
try:
    dependency=Path('/content/pupper-clamp-results/stablegait-deployclamp-original-rewards-s42/provenance.json')
    deadline=time.monotonic()+7200
    while True:
        if dependency.exists() and json.loads(dependency.read_text()).get('status')=='training_finished_not_evaluated':break
        if time.monotonic()>deadline:raise TimeoutError('StableGait finetune did not finish within two hours; no additional training started')
        time.sleep(15)
    for name,profile,motion,iterations,extra in [
        ('spider-corrected-contacts','spider_clips',spider/'spider_walk.npz',1000,['--initial-checkpoint',str(spider/'model_2999.pt')]),
        ('generated-composed','generated_trick',generated,3000,['--compose-com-randomization'])]:
        try:
            destination=out/name
            execute(name,[python,'-m','mjlab.scripts.pupper_foundation','train',profile,
                '--motion',motion,'--output',destination,'--deployment-target-clamp','--tracking-ground-hulls',
                '--iterations',iterations,'--num-envs','4096','--seed','42','--logger','tensorboard',*extra])
            manifests=list((destination/'runs').glob('*/foundation-run.json'))
            assert len(manifests)==1,manifests
            record=json.loads(manifests[0].read_text());run=manifests[0].parent
            assert record['state']=='trained_unvalidated',record['state']
            command=[python,'-m','mjlab.scripts.pupper_foundation','score','--motion',record['motion_file'],
                '--checkpoint',run/record['checkpoint'],'--output',destination/'evaluation',
                '--deployment-target-clamp','--tracking-ground-hulls','--suite']
            if profile=='spider_clips':command+=['--stress-profile','spider_clips']
            execute(name+'-evaluation',command)
        except Exception as error:
            status.setdefault('errors',{})[name]=f'{type(error).__name__}: {error}';save()
    status['stage']='finished_with_errors' if status.get('errors') else 'finished_requires_inspection'
except Exception as error:
    status.update(stage='failed',error=f'{type(error).__name__}: {error}')
    raise
finally:save()
