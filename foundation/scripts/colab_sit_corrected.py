"""Run the corrected sit continuation on the original Lab6 runtime after its batch.

Uses the exact saved reference/checkpoint and all three verified simulator fixes.
No robot connection or controller operation occurs.
"""
import datetime
import json
import os
from pathlib import Path
import subprocess

repo=Path('/content/pupper-foundation')
project=repo/'pupper-mjlab'
python=project/'.venv/bin/python'
original=Path('/content/foundation-output/lab6/runs/foundation_sit_stay_shake_s42_20261027T221651Z_e259b8f5')
out=Path('/content/pupper-sit-corrected')
out.mkdir(exist_ok=False)
os.environ['MUJOCO_GL']=os.environ['PYOPENGL_PLATFORM']='egl'
os.environ['PYTHONUNBUFFERED']='1'
state={'stage':'starting','hardware_validated':False,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'jobs':[]}

def save():
    p=out/'status.tmp';p.write_text(json.dumps(state,indent=2)+'\n');p.replace(out/'status.json')

def run(name,command):
    state['stage']=name
    record={'name':name,'command':list(map(str,command))};state['jobs'].append(record);save()
    with (out/f'{name}.log').open('w') as log:
        result=subprocess.run(record['command'],cwd=project,stdout=log,stderr=subprocess.STDOUT)
    record['returncode']=result.returncode;save()
    if result.returncode:raise RuntimeError(f'{name} failed; see saved log')

try:
    run('training',[python,'-m','mjlab.scripts.pupper_foundation','train','sit_stay_shake',
        '--motion',original/'sit_stay_shake.npz','--initial-checkpoint',original/'model_2999.pt',
        '--output',out/'training','--deployment-target-clamp','--tracking-ground-hulls',
        '--compose-com-randomization','--iterations','1000','--num-envs','4096',
        '--seed','42','--logger','tensorboard'])
    manifests=list((out/'training/runs').glob('*/foundation-run.json'))
    assert len(manifests)==1,manifests
    manifest=json.loads(manifests[0].read_text());run_dir=manifests[0].parent
    assert manifest['state']=='trained_unvalidated',manifest['state']
    run('evaluation',[python,'-m','mjlab.scripts.pupper_foundation','score',
        '--motion',manifest['motion_file'],'--checkpoint',run_dir/manifest['checkpoint'],
        '--output',out/'evaluation','--deployment-target-clamp','--tracking-ground-hulls',
        '--suite','--stress-profile','sit_stay_shake'])
    state['stage']='finished_requires_inspection'
except Exception as error:
    state.update(stage='failed',error=f'{type(error).__name__}: {error}')
    raise
finally:save()
