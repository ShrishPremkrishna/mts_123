"""Train/evaluate SpiderWalk with verified torso contact, target clamp and COM.

Preserve the original profile rewards and exact completed baseline checkpoint.
All command sweeps are simulation only; limits/promotion require later review.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess

repo=Path('/content/pupper-foundation');project=repo/'pupper-mjlab'
python=project/'.venv/bin/python'
original=Path('/content/foundation-output/lab6/runs/foundation_spider_walk_s42_20261027T225728Z_c9662620/model_3999.pt')
assert hashlib.sha256(original.read_bytes()).hexdigest()=='ddd69001c7a56c4467b2e57ce859d8e33d81f7c651f3fa2f93c5675c9bd33bbc'
out=Path('/content/pupper-spider-walk-corrected');out.mkdir(exist_ok=False)
os.environ['MUJOCO_GL']=os.environ['PYOPENGL_PLATFORM']='egl';os.environ['PYTHONUNBUFFERED']='1'
state={'stage':'starting','jobs':[],'hardware_validated':False}

def save():
    p=out/'status.tmp';p.write_text(json.dumps(state,indent=2)+'\n');p.replace(out/'status.json')

def run(name,command):
    state['stage']=name
    item={'name':name,'command':list(map(str,command))};state['jobs'].append(item);save()
    with (out/f'{name}.log').open('w') as log:
        result=subprocess.run(item['command'],cwd=project,stdout=log,stderr=subprocess.STDOUT)
    item['returncode']=result.returncode;save()
    if result.returncode:raise RuntimeError(f'{name} failed; see saved log')

try:
    run('training',[python,'-m','mjlab.scripts.pupper_foundation','train','spider_walk',
        '--initial-checkpoint',original,'--output',out/'training',
        '--deployment-target-clamp','--compose-com-randomization','--locomotion-torso-contact',
        '--iterations','1500','--num-envs','4096','--seed','42','--logger','tensorboard'])
    manifests=list((out/'training/runs').glob('*/foundation-run.json'));assert len(manifests)==1
    manifest=json.loads(manifests[0].read_text());assert manifest['state']=='trained_unvalidated'
    checkpoint=manifests[0].parent/manifest['checkpoint']
    for seed in (101,202,303):
        for label,model in [('original',original),('corrected',checkpoint)]:
            run(f'{label}-evaluation-s{seed}',[python,'-m','mjlab.scripts.pupper_locomotion','evaluate',
                '--task','Mjlab-SpiderWalk-Flat-Pupper-v3','--checkpoint',model,
                '--commands',project/'configs/foundation/spider_commands.json',
                '--out',out/f'{label}-evaluation-s{seed}','--deployment-target-clamp',
                '--compose-com-randomization','--locomotion-torso-contact','--geometry-audit',
                '--num-envs','32','--seconds','10','--warmup','2','--seed',seed,'--device','cuda:0'])
    state['stage']='finished_requires_inspection'
except Exception as error:
    state.update(stage='failed',error=f'{type(error).__name__}: {error}')
    raise
finally:save()
