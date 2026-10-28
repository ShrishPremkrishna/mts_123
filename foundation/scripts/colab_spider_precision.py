"""Two isolated reward-width continuations; root dispatches only on a free runtime.

Run --arm baseline and --arm precision separately for an explicitly authorized
parallel run, or --arm both for sequential execution. This never pulls source,
allocates a runtime or changes an existing run. Failed arms retain their logs.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

CHECKPOINT_SHA='853db60aa5ba265b9e0c226b20d30c538887bc7267313004bbe5bd47161082f4'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plan(repo,checkpoint,output,arm,revision):
    if sha(checkpoint)!=CHECKPOINT_SHA:raise ValueError('requires the exact corrected SpiderWalk5498 checkpoint')
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
    if actual!=revision:raise ValueError(f'expected source revision {revision}, got {actual}')
    project=repo/'pupper-mjlab';python=project/'.venv/bin/python';out=output/arm
    driver=repo/'foundation/configs/class-driver-joint-limits.json';std=math.sqrt(.1) if arm=='baseline' else .08
    common=['--deployment-target-clamp','--driver-limits',driver,'--compose-com-randomization','--locomotion-torso-contact']
    train=[python,'-m','mjlab.scripts.pupper_foundation','train','spider_walk','--initial-checkpoint',checkpoint,
           '--output',out/'training','--linear-tracking-std',repr(std),*common,'--iterations','1000','--num-envs','4096',
           '--seed','42','--logger','tensorboard']
    return dict(arm=arm,std=std,checkpoint_sha256=CHECKPOINT_SHA,source_revision=actual,driver_limits_sha256=sha(driver),
                output=str(out),training_command=list(map(str,train)),hardware_validated=False),common


def run_arm(args,arm):
    project=args.repo/'pupper-mjlab';python=project/'.venv/bin/python';record,common=plan(args.repo,args.checkpoint,args.output,arm,args.expected_revision)
    if args.plan_only:print(json.dumps(record,indent=2));return
    out=Path(record['output']);out.mkdir(parents=True,exist_ok=False);state=dict(stage='starting',plan=record,jobs=[],hardware_validated=False)
    def save():
        tmp=out/'status.tmp';tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(out/'status.json')
    def run(name,command):
        item=dict(name=name,command=list(map(str,command)));state['stage']=name;state['jobs'].append(item);save()
        with (out/f'{name}.log').open('w') as log:result=subprocess.run(item['command'],cwd=project,stdout=log,stderr=subprocess.STDOUT)
        item['returncode']=result.returncode;save()
        if result.returncode:raise RuntimeError(f'{name} failed; see saved log')
    os.environ['MUJOCO_GL']=os.environ['PYOPENGL_PLATFORM']='egl';os.environ['PYTHONUNBUFFERED']='1'
    try:
        run('training',record['training_command'])
        manifests=list((out/'training/runs').glob('*/foundation-run.json'))
        if len(manifests)!=1:raise RuntimeError('expected exactly one isolated training run')
        manifest=json.loads(manifests[0].read_text())
        if manifest['state']!='trained_unvalidated':raise RuntimeError('training did not finish')
        override=manifest['reward_parameter_overrides']['track_linear_velocity.std']
        if override['value']!=record['std'] or manifest['initial_checkpoint_sha256']!=CHECKPOINT_SHA:raise RuntimeError('training contract mismatch')
        model=manifests[0].parent/manifest['checkpoint']
        if sha(model)!=manifest['checkpoint_sha256']:raise RuntimeError('checkpoint hash mismatch')
        for label,commands,seconds in [('grid',project/'configs/foundation/spider_commands.json',10),
                                       ('stops',args.repo/'foundation/configs/spider-precision-stops.json',8)]:
            frozen=out/f'{label}-commands.json';frozen.write_bytes(commands.read_bytes())
            for seed in (101,202,303):
                name=f'{label}-s{seed}';destination=out/name
                run(name,[python,'-m','mjlab.scripts.pupper_locomotion','evaluate','--task','Mjlab-SpiderWalk-Flat-Pupper-v3',
                    '--checkpoint',model,'--commands',frozen,'--out',destination,*common,'--geometry-audit',
                    '--num-envs','32','--seconds',seconds,'--warmup','2','--seed',seed,'--device','cuda:0'])
                if label=='stops':
                    run(f'{name}-analysis',[python,args.repo/'foundation/scripts/analyze_spider_stops.py',
                        '--trace',destination/'trace.npz','--output',destination/'stop-measurements.json'])
        state['stage']='finished_requires_inspection'
    except Exception as error:
        state.update(stage='failed',error=f'{type(error).__name__}: {error}');raise
    finally:save()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=Path('/content/pupper-foundation'))
    parser.add_argument('--checkpoint',type=Path,default=Path('/content/pupper-spider-walk-corrected/training/runs/foundation_spider_walk_deployclamp_composedcom_torsocontact_s42_20261027T234440Z_21b50aa9/model_5498.pt'))
    parser.add_argument('--output',type=Path,default=Path('/content/pupper-spider-precision'))
    parser.add_argument('--expected-revision',required=True)
    parser.add_argument('--arm',choices=['baseline','precision','both'],default='both')
    parser.add_argument('--plan-only',action='store_true');args=parser.parse_args()
    args.repo=args.repo.resolve();args.checkpoint=args.checkpoint.resolve();args.output=args.output.resolve()
    for arm in (['baseline','precision'] if args.arm=='both' else [args.arm]):run_arm(args,arm)


if __name__=='__main__':main()
