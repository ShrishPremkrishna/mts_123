"""Continue reviewed Sit with the explicit source-class driver target limits.

Run only after earlier workers on this checkout finish. Existing runs remain
immutable; this uses the same motion/rewards and adds the verified second clamp.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

repo=Path('/content/pupper-foundation');project=repo/'pupper-mjlab'
python=project/'.venv/bin/python'
original=Path('/content/pupper-sit-corrected/training/runs/foundation_sit_stay_shake_deployclamp_composedcom_groundhulls_s42_20261027T232555Z_49d040ff')
checkpoint=original/'model_3998.pt';motion=original/'sit_stay_shake.npz'
assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()=='ee256ec97d1a4b63122c9658c13f1d91b19ef047f0b3033460f7197c08846e14'
assert hashlib.sha256(motion.read_bytes()).hexdigest()=='5e7129e92ee10c3305e13ef33d973d5682664c813d7c2df5397bc9b85926d137'
limits=repo/'foundation/configs/class-driver-joint-limits.json'
out=Path('/content/pupper-sit-driver-corrected');out.mkdir(exist_ok=False)
os.environ['MUJOCO_GL']=os.environ['PYOPENGL_PLATFORM']='egl';os.environ['PYTHONUNBUFFERED']='1'
state=dict(stage='starting',created_utc=datetime.now(timezone.utc).isoformat(),jobs=[],hardware_validated=False,
           installed_robot_limits_verified=False)

def save():
    p=out/'status.tmp';p.write_text(json.dumps(state,indent=2)+'\n');p.replace(out/'status.json')

def run(name,command):
    state['stage']=name;job=dict(name=name,command=list(map(str,command)))
    state['jobs'].append(job);save()
    with (out/f'{name}.log').open('w') as log:
        result=subprocess.run(job['command'],cwd=project,stdout=log,stderr=subprocess.STDOUT)
    job['returncode']=result.returncode;save()
    if result.returncode:raise RuntimeError(f'{name} failed; see saved log')

try:
    run('training',[python,'-m','mjlab.scripts.pupper_foundation','train','sit_stay_shake',
        '--motion',motion,'--initial-checkpoint',checkpoint,'--output',out/'training',
        '--deployment-target-clamp','--driver-limits',limits,'--tracking-ground-hulls',
        '--compose-com-randomization','--iterations','1000','--num-envs','4096','--seed','42','--logger','tensorboard'])
    paths=list((out/'training/runs').glob('*/foundation-run.json'));assert len(paths)==1
    manifest=json.loads(paths[0].read_text());assert manifest['state']=='trained_unvalidated'
    run('evaluation',[python,'-m','mjlab.scripts.pupper_foundation','score',
        '--motion',manifest['motion_file'],'--checkpoint',paths[0].parent/manifest['checkpoint'],
        '--output',out/'evaluation','--deployment-target-clamp','--driver-limits',limits,
        '--tracking-ground-hulls','--suite','--stress-profile','sit_stay_shake'])
    state['stage']='finished_requires_inspection'
except Exception as error:
    state.update(stage='failed',error=f'{type(error).__name__}: {error}')
    raise
finally:save()
