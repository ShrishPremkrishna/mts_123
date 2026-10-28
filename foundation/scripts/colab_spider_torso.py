"""Controlled torso-load intervention from the exact reviewed slew baseline6497.

Probe nominal mechanics versus randomized mechanics before training. Control and
penalty arms share1000iterations4096worldsseed42; only torso-support weight differs.
No source fetching, policy installation, limit relaxation or hardware approval.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess

import colab_spider_precision as original

RATE = 15.
SEEDS = (101, 202, 303)


def verify_source(repo, revision):
    actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    if actual != revision:
        raise ValueError(f'Expected frozen revision {revision}, got {actual}')
    if subprocess.run(['git', 'diff', '--quiet', 'HEAD', '--'], cwd=repo).returncode:
        raise ValueError('Tracked source changed during the experiment')


CHECKPOINT_SHA = '3889a2be7d883f188e25fdb7c30ab62da2bdc3e4bf38c4129af4c5fd27519ee5'


def plan(repo, checkpoint, output, arm, revision):
    if arm not in ('probe', 'control', 'torso'):
        raise ValueError('Unknown matched torso arm')
    verify_source(repo, revision)
    if original.sha(checkpoint) != CHECKPOINT_SHA:
        raise ValueError('Requires exact reviewed slew baseline6497 checkpoint')
    project=repo/'pupper-mjlab';python=project/'.venv/bin/python';driver=repo/'foundation/configs/class-driver-joint-limits.json'
    weight=None if arm=='probe' else 0. if arm=='control' else -2.
    common=['--deployment-target-clamp','--driver-limits',str(driver),'--compose-com-randomization','--locomotion-torso-contact','--target-slew-rad-s','15.0']
    train=None if arm=='probe' else list(map(str,[python,'-m','mjlab.scripts.pupper_foundation','train','spider_walk','--initial-checkpoint',checkpoint,'--output',output/arm/'training','--linear-tracking-std',repr(.1**.5),*common,'--torso-support-weight',str(weight),'--iterations','1000','--num-envs','4096','--seed','42','--logger','tensorboard']))
    return dict(arm=arm,output=str(output/arm),std=.1**.5,torso_support_weight=weight,checkpoint_sha256=CHECKPOINT_SHA,source_revision=revision,driver_limits_sha256=original.sha(driver),training_command=train,training_iterations=0 if arm=='probe' else 1000,target_slew_rad_s=RATE,evaluation_seeds=[101] if arm=='probe' else list(SEEDS),hardware_validated=False,automatic_promotion=False,source_frozen=True,scope='Same pose/reference/controller/driver/slew physical contract; only bounded actual torso-load reward weight differs.'),common


def check_contract(block, driver_sha):
    if (not isinstance(block, dict) or block.get('contract') != 'pupper_position_target_slew_v1'
            or block.get('max_rate_rad_s') != RATE or block.get('max_step_s') != .02
            or block.get('driver_limits_sha256') != driver_sha):
        raise RuntimeError('Missing or mismatched slew physical contract')


def run_arm(args, arm):
    record, common = plan(args.repo, args.checkpoint, args.output, arm, args.expected_revision)
    if args.plan_only:
        print(json.dumps(record, indent=2)); return
    out = Path(record['output']); out.mkdir(parents=True, exist_ok=False)
    project = args.repo/'pupper-mjlab'; python = project/'.venv/bin/python'
    state = dict(stage='starting', plan=record, jobs=[], hardware_validated=False)
    def save():
        temp = out/'status.tmp'; temp.write_text(json.dumps(state, indent=2)+'\n'); temp.replace(out/'status.json')
    def run(name, command):
        verify_source(args.repo, args.expected_revision)
        item = dict(name=name, command=list(map(str, command)))
        state['stage'] = name; state['jobs'].append(item); save()
        with (out/f'{name}.log').open('x') as log:
            result = subprocess.run(item['command'], cwd=project, stdout=log, stderr=subprocess.STDOUT)
        item['returncode'] = result.returncode; save()
        if result.returncode:
            raise RuntimeError(f'{name} failed; retained log contains diagnostics')
        verify_source(args.repo, args.expected_revision)
    os.environ['MUJOCO_GL'] = os.environ['PYOPENGL_PLATFORM'] = 'egl'
    os.environ['PYTHONUNBUFFERED'] = '1'
    try:
        model = args.checkpoint
        if arm != 'probe':
            run('training', record['training_command'])
            manifests = list((out/'training/runs').glob('*/foundation-run.json'))
            if len(manifests) != 1: raise RuntimeError('Expected one isolated training run')
            manifest = json.loads(manifests[0].read_text())
            if (manifest['state'] != 'trained_unvalidated'
                    or manifest['initial_checkpoint_sha256'] != CHECKPOINT_SHA
                    or manifest['reward_parameter_overrides']['track_linear_velocity.std']['value'] != record['std']
                    or manifest['torso_support_contract']['reward_weight'] != record['torso_support_weight']
                    or manifest['iterations'] != 1000 or manifest['num_envs'] != 4096 or manifest['seed'] != 42):
                raise RuntimeError('Training input, reward width, budget or state mismatch')
            check_contract(manifest['position_target_slew_contract'], record['driver_limits_sha256'])
            model = manifests[0].parent/manifest['checkpoint']
            if original.sha(model) != manifest['checkpoint_sha256']:
                raise RuntimeError('Completed checkpoint hash mismatch')
            policy = manifests[0].parent/'policy.json'
            if original.sha(policy) != manifest['policy_sha256']:
                raise RuntimeError('Completed policy hash mismatch')
            check_contract(json.loads(policy.read_text()).get('position_target_slew'), record['driver_limits_sha256'])
            state['checkpoint'] = str(model); state['checkpoint_sha256'] = original.sha(model)
            state['policy'] = str(policy); state['policy_sha256'] = original.sha(policy); save()
        suites=[('grid',project/'configs/foundation/spider_commands.json',10),('stops',args.repo/'foundation/configs/spider-precision-stops.json',8)]
        if arm=='probe':suites=suites[:1]
        for label,commands,seconds in suites:
            frozen=out/f'{label}-commands.json';frozen.write_bytes(commands.read_bytes())
            for seed in ([101] if arm=='probe' else SEEDS):
                for nominal in ([False,True] if label=='grid' and seed==101 else [False]):
                    name=f'{label}-s{seed}'+('-nominal-mechanics' if nominal else '');destination=out/name
                    extra=['--nominal-mechanics'] if nominal else []
                    run(name,[python,'-m','mjlab.scripts.pupper_locomotion','evaluate','--task','Mjlab-SpiderWalk-Flat-Pupper-v3','--checkpoint',model,'--commands',frozen,'--out',destination,*common,'--geometry-audit','--torso-contact-audit',*extra,'--num-envs','32','--seconds',str(seconds),'--warmup','2','--seed',str(seed),'--device','cuda:0'])
                    metrics=json.loads((destination/'metrics.json').read_text())
                    check_contract(metrics['provenance'].get('position_target_slew_contract'),record['driver_limits_sha256'])
                    support=json.loads((destination/'torso_support_metrics.json').read_text())
                    if support['contract']!='flat_torso_support_audit_v1':raise RuntimeError('Missing torso-force audit')
                    if label=='stops':
                        run(f'{name}-analysis',[python,args.repo/'foundation/scripts/analyze_spider_stops.py','--trace',destination/'trace.npz','--output',destination/'stop-measurements.json'])
        state['stage'] = 'finished_requires_inspection'
    except BaseException as error:
        state.update(stage='failed', error=f'{type(error).__name__}: {error}'); raise
    finally:
        save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path('/content/pupper-foundation'))
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('/content/pupper-spider-torso'))
    parser.add_argument('--expected-revision', required=True)
    parser.add_argument('--arm', choices=['probe', 'control', 'torso'], required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    args.repo = args.repo.resolve(); args.checkpoint = args.checkpoint.resolve(); args.output = args.output.resolve()
    run_arm(args,args.arm)


if __name__ == '__main__':
    main()
