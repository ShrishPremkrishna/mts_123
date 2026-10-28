"""Matched Spider continuations with an explicit, opt-in command slew contract.

All arms share the exact5498 initial actor and physical contract. `prefiltered`
measures that actor under the new physics before any adaptation. Training arms
change only reward width; prior non-slew continuations supply budget controls.
This driver never fetches source, installs a policy, or grants hardware approval.
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


def plan(repo, checkpoint, output, arm, revision):
    if arm not in ('prefiltered', 'baseline', 'precision'):
        raise ValueError('Unknown matched slew arm')
    verify_source(repo, revision)
    record, common = original.plan(repo, checkpoint, output,
                                   'baseline' if arm == 'prefiltered' else arm, revision)
    common = [*common, '--target-slew-rad-s', str(RATE)]
    record.update(arm=arm, output=str(output/arm), target_slew_rad_s=RATE,
                  evaluation_seeds=list(SEEDS), source_frozen=True,
                  initial_physics_transition='legacy5498 weights -> explicit15rad/s contract',
                  hardware_validated=False, automatic_promotion=False,
                  native_handoff='required separately after export and local artifact recovery')
    if arm == 'prefiltered':
        record.update(std=None, training_command=None, training_iterations=0)
    else:
        record['training_command'].extend(['--target-slew-rad-s', str(RATE)])
        record['training_iterations'] = 1000
    return record, list(map(str, common))


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
        if arm != 'prefiltered':
            run('training', record['training_command'])
            manifests = list((out/'training/runs').glob('*/foundation-run.json'))
            if len(manifests) != 1: raise RuntimeError('Expected one isolated training run')
            manifest = json.loads(manifests[0].read_text())
            if (manifest['state'] != 'trained_unvalidated'
                    or manifest['initial_checkpoint_sha256'] != original.CHECKPOINT_SHA
                    or manifest['reward_parameter_overrides']['track_linear_velocity.std']['value'] != record['std']
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
        for label, commands, seconds in [('grid', project/'configs/foundation/spider_commands.json', 10),
                                         ('stops', args.repo/'foundation/configs/spider-precision-stops.json', 8)]:
            frozen = out/f'{label}-commands.json'; frozen.write_bytes(commands.read_bytes())
            for seed in SEEDS:
                name = f'{label}-s{seed}'; destination = out/name
                run(name, [python, '-m', 'mjlab.scripts.pupper_locomotion', 'evaluate',
                    '--task', 'Mjlab-SpiderWalk-Flat-Pupper-v3', '--checkpoint', model,
                    '--commands', frozen, '--out', destination, *common, '--geometry-audit',
                    '--num-envs', '32', '--seconds', str(seconds), '--warmup', '2', '--seed', str(seed), '--device', 'cuda:0'])
                metrics = json.loads((destination/'metrics.json').read_text())
                check_contract(metrics['provenance'].get('position_target_slew_contract'), record['driver_limits_sha256'])
                if label == 'stops':
                    run(f'{name}-analysis', [python, args.repo/'foundation/scripts/analyze_spider_stops.py',
                        '--trace', destination/'trace.npz', '--output', destination/'stop-measurements.json'])
        state['stage'] = 'finished_requires_inspection'
    except BaseException as error:
        state.update(stage='failed', error=f'{type(error).__name__}: {error}'); raise
    finally:
        save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path('/content/pupper-foundation'))
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('/content/pupper-spider-slew'))
    parser.add_argument('--expected-revision', required=True)
    parser.add_argument('--arm', choices=['prefiltered', 'baseline', 'precision', 'all'], default='all')
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    args.repo = args.repo.resolve(); args.checkpoint = args.checkpoint.resolve(); args.output = args.output.resolve()
    for arm in (['prefiltered', 'baseline', 'precision'] if args.arm == 'all' else [args.arm]):
        run_arm(args, arm)


if __name__ == '__main__':
    main()
