#!/usr/bin/env python3
"""Held-out Optional2 evaluation only; never train, launch remotely or promote.

Freeze a batch_status.json AFTER selected_flat is recorded. Supply exact completed
checkpoints and, for a non-stock selection, its original reference directory.
Use separate output roots for --scope flat and --scope bumpy as stages finish.
Both Bumpy policies use the selected flat policy's identical reference bytes.

The reviewed legacy Optional2 contract is intentional: deployed target clamp,
training-config randomization/noise, legacy COM composition, no torso collider.
Results cannot establish independent COM robustness or reliable Bumpy torso
clearance. Use --plan-only before execution; old failed attempts are preserved.
"""
from __future__ import annotations

import argparse
import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import time

_SPEC = importlib.util.spec_from_file_location('optional_validation_shared',
    Path(__file__).with_name('colab_gait_validation.py'))
shared = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(shared)

FLAT = 'Mjlab-MixedGaits-Flat-Pupper-v3'
BUMPY = 'Mjlab-MixedGaits-Bumpy-Pupper-v3'
CONTRACT = {'deployment_target_clamp': True, 'driver_limits': None, 'compose_com_randomization': False,
            'locomotion_torso_contact': False, 'randomized': True,
            'num_envs': 32, 'seconds': 10., 'warmup': 2.}


def _input(inputs, label, path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f'{label} missing: {path}')
    inputs[label] = {'path': str(path), 'sha256': shared.sha(path)}
    return path


def _source_run(path):
    """Relocated downloads retain the original run directory and exact filename."""
    path = Path(path)
    return path.parent.name, path.name


def _finished_checkpoint(inputs, label, path, task, references):
    path = _input(inputs, label, path)
    if not re.fullmatch(r'model_\d+\.pt', path.name):
        raise ValueError(f'{label}: supply an exact model_N.pt checkpoint, never a latest glob')
    source = _input(inputs, label + '_provenance', path.parent / 'provenance.json')
    provenance = json.loads(source.read_text())
    if (provenance.get('command') != 'train' or provenance.get('task') != task
            or provenance.get('status') != 'training_finished_not_evaluated'):
        raise ValueError(f'{label}: require completed exact-task training provenance')
    if Path(provenance.get('out', '')).name != path.parent.name:
        raise ValueError(f'{label}: checkpoint directory does not identify the source training run')
    for key in ('deployment_target_clamp', 'compose_com_randomization', 'locomotion_torso_contact'):
        if provenance.get(key, False) is not CONTRACT[key]:
            raise ValueError(f'{label}: training {key} differs from reviewed Optional2 contract')
    if provenance.get('driver_limits') is not None:
        raise ValueError(f'{label}: training driver limits differ from reviewed Optional2 contract')
    if not re.fullmatch(r'[0-9a-f]{64}', provenance.get('source_sha256', '')):
        raise ValueError(f'{label}: missing source identity')
    if provenance.get('references', {}) != references:
        raise ValueError(f'{label}: reference bytes differ from training provenance')
    if bool(provenance.get('reference_dir')) != bool(references):
        raise ValueError(f'{label}: stock/custom reference identity is inconsistent')
    for name in ('policy.json', 'params/env.yaml', 'params/agent.yaml'):
        _input(inputs, f'{label}_{name}', path.parent / name)
    return path, provenance


def _completed_stage(selection, source_directory):
    for stage in selection.get('stages', {}).values():
        command = stage.get('command', [])
        if 'train' not in command or '--out' not in command:
            continue
        target = command[command.index('--out') + 1]
        if target == source_directory:
            if stage.get('status') == 'complete' and stage.get('returncode') == 0:
                return
            break
    raise ValueError(f'Frozen selection snapshot has no completed training stage for {source_directory}')


def make_plan(args):
    repo = args.repo.expanduser().resolve()
    python = Path(os.path.abspath((args.python or repo / 'pupper-mjlab/.venv/bin/python').expanduser()))
    if not python.is_file():
        raise ValueError(f'Python environment missing: {python}')
    inputs = {}
    selection_path = _input(inputs, 'selection', args.selection_file)
    selection = json.loads(selection_path.read_text())
    chosen = selection.get('selected_flat')
    if not isinstance(chosen, dict) or not chosen.get('checkpoint') or 'reference_dir' not in chosen:
        raise ValueError('Freeze batch_status.json only after the flat candidate selection is complete')
    if selection.get('smoke') or selection.get('deployment_target_clamp', False) is not True:
        raise ValueError('Selection must be a non-smoke, deployment-clamped Optional2 batch')
    if selection.get('compose_com_randomization', False) is not False:
        raise ValueError('Selection must retain the explicit legacy COM contract')
    references = {}
    reference_dir = None
    if chosen['reference_dir']:
        if args.selected_reference_dir is None:
            raise ValueError('The selected candidate needs its exact --selected-reference-dir')
        reference_dir = args.selected_reference_dir.expanduser().resolve()
        if not reference_dir.is_dir():
            raise ValueError('Selected reference directory does not exist')
        for path in sorted(reference_dir.iterdir()):
            if path.suffix in ('.npz', '.json') and path.is_file():
                _input(inputs, 'reference_' + path.name, path)
                references[path.name] = shared.sha(path)
        if not {'captured_reach_gaits.npz', 'reference_manifest.json'} <= references.keys():
            raise ValueError('Selected references require both capture and manifest')
        manifest = json.loads((reference_dir / 'reference_manifest.json').read_text())
        frequency = manifest.get('frequency_mult')
        if (manifest.get('reference_sha256') != references['captured_reach_gaits.npz']
                or isinstance(frequency, bool) or not isinstance(frequency, (float, int))
                or not math.isfinite(frequency) or frequency <= 0):
            raise ValueError('Selected reference manifest does not match its capture/cadence')
    elif args.selected_reference_dir is not None:
        raise ValueError('Selected candidate uses stock references; do not substitute custom references')
    stock = repo / 'pupper-mjlab/src/mjlab/tasks/pupper_gait/mdp'
    if any((stock / name).exists() for name in ('captured_reach_gaits.npz', 'reference_manifest.json', 'captured_jump_gait.npz')):
        raise ValueError('Stock reference source is contaminated by external captures; use a clean source checkout')
    _input(inputs, 'stock_reference_source', stock / 'gait_reference.py')
    # Freeze the evaluator's default16 sequence without importing/registering tasks.
    evaluator = repo / 'pupper-mjlab/src/mjlab/scripts/pupper_locomotion.py'
    tree = ast.parse(evaluator.read_text())
    sequences = [ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'DEFAULT_SEQUENCE' for t in n.targets)]
    if len(sequences) != 1 or len(sequences[0]) != 16:
        raise ValueError('Expected reviewed evaluator default16 command sequence')
    commands = sequences[0]
    if any(not isinstance(c, list) or len(c) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in c) for c in commands):
        raise ValueError('Invalid default command sequence')
    selected, selected_source = _finished_checkpoint(inputs, 'selected', args.selected_checkpoint, FLAT, references)
    if _source_run(selected) != _source_run(chosen['checkpoint']):
        raise ValueError('Selected checkpoint differs from the frozen batch decision')
    if selected_source.get('reference_dir') != chosen['reference_dir']:
        raise ValueError('Selected reference directory differs from the frozen batch decision')
    if chosen.get('profile') not in ('baseline', 'fast') or selected_source.get('profile') != chosen['profile']:
        raise ValueError('Selected training profile differs from the frozen batch decision')
    _completed_stage(selection, selected_source['out'])
    entries = []
    if args.scope in ('flat', 'all'):
        if args.baseline_checkpoint is None:
            raise ValueError('--scope flat/all requires --baseline-checkpoint')
        baseline, baseline_source = _finished_checkpoint(inputs, 'baseline', args.baseline_checkpoint, FLAT, {})
        _completed_stage(selection, baseline_source['out'])
        if Path(baseline_source['out']).name != 'mixed-baseline':
            raise ValueError('Flat baseline must be the batch mixed-baseline stage')
        entries += [('flat-baseline-s404', 'flat', 'baseline', baseline, None, {}, 404, FLAT),
                    ('flat-selected-s404', 'flat', 'selected', selected, reference_dir, references, 404, FLAT)]
    if args.scope in ('bumpy', 'all'):
        if args.bumpy_checkpoint is None:
            raise ValueError('--scope bumpy/all requires --bumpy-checkpoint')
        bumpy, bumpy_source = _finished_checkpoint(inputs, 'bumpy', args.bumpy_checkpoint, BUMPY, references)
        _completed_stage(selection, bumpy_source['out'])
        if (bumpy_source.get('checkpoint_sha256') != shared.sha(selected)
                or _source_run(bumpy_source.get('checkpoint', '')) != _source_run(chosen['checkpoint'])):
            raise ValueError('Bumpy finetune must start from the exact selected flat checkpoint')
        for seed in (202, 303, 404):
            entries += [(f'bumpy-pre-s{seed}', 'bumpy', 'selected', selected, reference_dir, references, seed, BUMPY),
                        (f'bumpy-finetuned-s{seed}', 'bumpy', 'bumpy', bumpy, reference_dir, references, seed, BUMPY)]
    jobs, aliases, signatures = [], {}, {}
    for name, group, label, checkpoint, refdir, hashes, seed, task in entries:
        identity = (shared.sha(checkpoint), json.dumps(hashes, sort_keys=True), seed, task)
        if identity in signatures:
            aliases[name] = signatures[identity]
            continue
        signatures[identity] = name
        argv = [str(python), '-B', '-m', 'mjlab.scripts.pupper_locomotion', 'evaluate',
                '--task', task, '--checkpoint', str(checkpoint), '--device', args.device,
                '--seed', str(seed), '--num-envs', '32', '--seconds', '10.0', '--warmup', '2.0',
                '--randomized', '--deployment-target-clamp']
        if refdir is not None:
            argv += ['--reference-dir', str(refdir)]
        jobs.append({'name': name, 'group': group, 'label': label, 'seed': seed, 'task': task,
                     'checkpoint_sha256': shared.sha(checkpoint), 'references': hashes,
                     'commands': commands, 'argv_without_out': argv})
    return {'schema_version': 1, 'scope': args.scope, 'repo': str(repo), 'python': str(python),
            'device': args.device, 'contract': CONTRACT, 'inputs': inputs,
            'source_sha256': shared.source_identity(repo), 'runner_sha256': shared.sha(__file__),
            'helper_sha256': shared.sha(Path(__file__).with_name('colab_gait_validation.py')),
            'jobs': jobs, 'aliases': aliases,
            'workload': {'jobs': len(jobs), 'vector_control_steps': len(jobs) * 16 * 500,
                         'robot_control_steps': len(jobs) * 16 * 500 * 32,
                         'simulated_robot_seconds': len(jobs) * 16 * 10 * 32},
            'automatic_promotion': False, 'approved_for_hardware': False,
            'limitations': ['Legacy COM composition does not establish independent COM robustness.',
                           'Controller target limits only; the local class driver applies tighter downstream bounds, and installed Pi limits remain unknown.',
                           'No torso collider; Bumpy torso distance is not a reliable clearance claim.',
                           'Aliases reuse identical evidence and are not independent trials.']}


def verify_inputs(plan):
    shared.verify_inputs(plan)
    if shared.sha(__file__) != plan['runner_sha256'] or shared.sha(Path(__file__).with_name('colab_gait_validation.py')) != plan['helper_sha256']:
        raise ValueError('Runner/helper changed; preserve results and use a new output root')


def verify_result(plan, job, directory):
    metrics = json.loads((directory / 'metrics.json').read_text())
    provenance = json.loads((directory / 'provenance.json').read_text())
    if metrics.get('provenance') != provenance:
        raise ValueError('Metrics/provenance disagree')
    expected = {**CONTRACT, 'seed': job['seed'], 'task': job['task'],
                'source_sha256': plan['source_sha256'], 'checkpoint_sha256': job['checkpoint_sha256'],
                'zero_policy': False, 'reset_each_command': False, 'no_heading_hold': False,
                'allow_out_of_range': False}
    if any(provenance.get(key, False if key in ('compose_com_randomization', 'locomotion_torso_contact') else None) != value
           for key, value in expected.items()):
        raise ValueError('Reported evaluation protocol differs from the plan')
    if provenance.get('references', {}) != job['references']:
        raise ValueError('Evaluated reference bytes differ from plan')
    if [row['command'] for row in metrics['segments']] != job['commands']:
        raise ValueError('Incomplete or changed command sequence')
    return shared.artifact_hashes(directory)


def summarize(plan, status):
    import numpy as np
    summary = {'comparisons': [], 'incomplete_jobs': [], 'errors': [], 'aliases': plan['aliases'],
               'automatic_promotion': False, 'approved_for_hardware': False, 'limitations': plan['limitations']}
    results = {}
    for job in plan['jobs']:
        attempts = status['jobs'].get(job['name'], [])
        item = attempts[-1] if attempts else {}
        if item.get('state') != 'completed' or not shared.completed_artifacts_match(item):
            summary['incomplete_jobs'].append(job['name'])
        else:
            results[job['name']] = item
    for alias, target in plan['aliases'].items():
        if target in results:
            results[alias] = results[target]
    pairs = []
    if plan['scope'] in ('flat', 'all'):
        pairs.append(('flat', 404, 'flat-baseline-s404', 'flat-selected-s404'))
    if plan['scope'] in ('bumpy', 'all'):
        pairs += [('bumpy', seed, f'bumpy-pre-s{seed}', f'bumpy-finetuned-s{seed}') for seed in (202,303,404)]
    for group, seed, a, b in pairs:
        if a not in results or b not in results:
            continue
        left, right = (Path(results[k]['directory']) for k in (a,b))
        with np.load(left / 'randomization_snapshot.npz', allow_pickle=False) as x, np.load(right / 'randomization_snapshot.npz', allow_pickle=False) as y:
            paired = set(x.files) == set(y.files) and all(np.array_equal(x[k], y[k]) for k in x.files)
        same_environment = shared.sha(left / 'env.yaml') == shared.sha(right / 'env.yaml')
        comparison = {'group': group, 'seed': seed, 'left': a, 'right': b,
                      'same_evidence_not_independent_trial': left == right,
                      'realized_randomization_equal': paired, 'environment_config_equal': same_environment,
                      'left_artifacts': results[a]['artifact_sha256'], 'right_artifacts': results[b]['artifact_sha256']}
        if not paired or not same_environment:
            summary['errors'].append(f'{group} seed{seed}: environments/randomization differ; comparison withheld')
        else:
            comparison['left_metrics'] = json.loads((left / 'metrics.json').read_text())['segments']
            comparison['right_metrics'] = json.loads((right / 'metrics.json').read_text())['segments']
        summary['comparisons'].append(comparison)
    return summary


def run_plan(plan, out, *, resume=False, retry_incomplete=False, execute=shared.launch, check_runtime=shared.runtime_preflight):
    out = out.expanduser().resolve()
    plan_path, status_path = out / 'plan.json', out / 'status.json'
    if resume:
        if json.loads(plan_path.read_text()) != plan:
            raise ValueError('Resume requires identical frozen inputs/source/protocol; use a new output root')
        status = json.loads(status_path.read_text())
    else:
        out.mkdir(parents=True, exist_ok=False)
        shared.save(plan_path, plan)
        status = {'created_utc': shared.utc_now(), 'state': 'prepared', 'jobs': {}, 'automatic_promotion': False}
    shared.save(status_path, status)
    try:
        verify_inputs(plan)
        for job in plan['jobs']:
            attempts = status['jobs'].get(job['name'], [])
            prior = attempts[-1] if attempts else {}
            if prior.get('state') == 'completed' and not shared.completed_artifacts_match(prior):
                raise ValueError('Completed output changed; use a new output root')
            if prior.get('state') == 'running' and not retry_incomplete:
                raise ValueError('Prior worker may still be running; confirm stopped before --retry-incomplete')
        shared.save(out / 'runtime.json', check_runtime(plan, out / 'preflight.log'))
        for job in plan['jobs']:
            attempts = status['jobs'].setdefault(job['name'], [])
            prior = attempts[-1] if attempts else {}
            if prior.get('state') == 'completed':
                continue
            verify_inputs(plan)
            parent = out / 'jobs' / job['name']; parent.mkdir(parents=True, exist_ok=True)
            directory = parent / f'attempt-{len(attempts)+1:03d}'
            log = directory.with_suffix('.log')
            argv = [*job['argv_without_out'], '--out', str(directory)]
            item = {'state': 'running', 'directory': str(directory), 'log': str(log),
                    'command': argv, 'started_utc': shared.utc_now()}
            attempts.append(item); status.update(state='running', active_job=job['name'])
            shared.save(status_path, status)
            print(f'{job["name"]}: {directory}', flush=True)
            started = time.monotonic()
            try:
                item['returncode'] = execute(argv, Path(plan['repo']), log)
                if item['returncode']:
                    raise RuntimeError(f'evaluation exited {item["returncode"]}; see {log}')
                verify_inputs(plan)
                item['artifact_sha256'] = verify_result(plan, job, directory)
                item['state'] = 'completed'
            except (OSError, ValueError, KeyError, RuntimeError) as exc:
                item.update(state='failed', error=f'{type(exc).__name__}: {exc}')
            except BaseException as exc:
                item.update(state='interrupted', error=type(exc).__name__); raise
            finally:
                item.update(finished_utc=shared.utc_now(), elapsed_seconds=time.monotonic()-started)
                shared.save(status_path, status)
        report = summarize(plan, status); shared.save(out / 'summary.json', report)
        failed = bool(report['incomplete_jobs'] or report['errors'])
        status['state'] = 'finished_with_errors' if failed else 'finished_requires_review'
        status['summary'] = str(out / 'summary.json')
        return 1 if failed else 0
    except BaseException as exc:
        status.update(state='interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        status.pop('active_job', None); status['updated_utc'] = shared.utc_now()
        shared.save(status_path, status)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--python', type=Path)
    parser.add_argument('--selection-file', type=Path, required=True)
    parser.add_argument('--baseline-checkpoint', type=Path)
    parser.add_argument('--selected-checkpoint', type=Path, required=True)
    parser.add_argument('--bumpy-checkpoint', type=Path)
    parser.add_argument('--selected-reference-dir', type=Path)
    parser.add_argument('--scope', choices=('flat','bumpy','all'), default='all')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--retry-incomplete', action='store_true')
    args = parser.parse_args()
    if args.retry_incomplete and not args.resume:
        parser.error('--retry-incomplete requires --resume')
    plan = make_plan(args)
    if args.plan_only:
        print(json.dumps(plan, indent=2)); return 0
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    return run_plan(plan, args.out, resume=args.resume, retry_incomplete=args.retry_incomplete)


if __name__ == '__main__':
    raise SystemExit(main())
