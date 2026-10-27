#!/usr/bin/env python3
"""Run the reviewed GPU gait comparisons on an already prepared runtime.

No training, download, Git update, robot access, policy installation or automatic
promotion. Pass exact checkpoints: recovered StableGait model_2999.pt, clamp
finetune model_4498.pt, and the known SpiderWalk run's final model_3999.pt.
Use --skip-spider while its body-clearance/contact contract is under review;
then only the six paired StableGait jobs run and no Spider checkpoint is needed.
Use --plan-only to inspect the exact workload before execution. Start only after
the existing Lab 6/Sit training and scoring processes have finished; this script
does not interrupt them or assume ownership of the GPU.

Example (paths are supplied by the runtime operator):
  python foundation/scripts/colab_gait_validation.py --repo /content/pupper-foundation \
    --recovered-checkpoint /content/recovered-policies/4hkyvuxh/model_2999.pt \
    --finetuned-checkpoint /content/stablegait/model_4498.pt \
    --spider-checkpoint /content/spider_walk/model_3999.pt \
    --out /content/pupper-gait-validation

Resume with identical arguments plus --resume. Failed/interrupted attempts remain
on disk; retry uses a new attempt directory. A stale 'running' record requires
--retry-incomplete only after the prior worker has actually stopped.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ARTIFACTS = ('metrics.json', 'provenance.json', 'env.yaml', 'trace.npz', 'randomization_snapshot.npz')


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    temporary = Path(path).with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def source_identity(repo):
    package = repo / 'pupper-mjlab/src/mjlab'
    extensions = {'.py', '.xml', '.urdf', '.yaml', '.yml', '.json', '.csv', '.npz'}
    files = {str(p.relative_to(package)): sha(p) for p in sorted(package.rglob('*'))
             if p.is_file() and '__pycache__' not in p.parts and p.suffix in extensions}
    if not files:
        raise ValueError(f'No mjlab source found in {package}')
    return hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()


def commands(path):
    value = json.loads(path.read_text())
    if (not isinstance(value, list) or not value or any(
            not isinstance(row, list) or len(row) != 3 or any(
                isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
                for x in row) for row in value)):
        raise ValueError(f'{path}: expected finite [vx, vy, yaw_rad_s] commands')
    return value


def make_plan(args):
    repo = args.repo.resolve()
    python = (args.python or repo / 'pupper-mjlab/.venv/bin/python').resolve()
    if not python.is_file():
        raise ValueError(f'Python environment missing: {python}')
    if len(args.seeds) != len(set(args.seeds)) or not args.seeds:
        raise ValueError('Supply distinct evaluation seeds')
    if args.num_envs < 1 or not 0 <= args.warmup < args.seconds or not math.isfinite(args.seconds):
        raise ValueError('Require positive environments and finite seconds > warmup >= 0')
    stable_commands = (args.stable_commands or repo / 'pupper-mjlab/configs/foundation/stable_commands.json').resolve()
    spider_commands = (args.spider_commands or repo / 'pupper-mjlab/configs/foundation/spider_commands.json').resolve()
    inputs = {}
    skip_spider = getattr(args, 'skip_spider', False)
    selected_inputs = [('recovered', args.recovered_checkpoint), ('finetuned', args.finetuned_checkpoint),
                       ('stable_commands', stable_commands)]
    if not skip_spider:
        if args.spider_checkpoint is None:
            raise ValueError('Supply --spider-checkpoint or explicitly select --skip-spider')
        selected_inputs += [('spider', args.spider_checkpoint), ('spider_commands', spider_commands)]
    for name, path in selected_inputs:
        path = path.resolve()
        if not path.is_file():
            raise ValueError(f'{name} input missing: {path}; supply the exact completed run/checkpoint')
        inputs[name] = {'path': str(path), 'sha256': sha(path)}
    sequences = {'stable': commands(stable_commands)}
    if not skip_spider:
        sequences['spider'] = commands(spider_commands)
    jobs = []
    for seed in args.seeds:
        selected = [
            ('stable-recovered', args.recovered_checkpoint, 'Mjlab-StableGait-Flat-Pupper-v3', stable_commands, sequences['stable']),
            ('stable-finetuned', args.finetuned_checkpoint, 'Mjlab-StableGait-Flat-Pupper-v3', stable_commands, sequences['stable']),
        ]
        if not skip_spider:
            selected.append(('spider-baseline', args.spider_checkpoint, 'Mjlab-SpiderWalk-Flat-Pupper-v3', spider_commands, sequences['spider']))
        for label, checkpoint, task, command_path, sequence in selected:
            argv = [str(python), '-B', '-m', 'mjlab.scripts.pupper_locomotion', 'evaluate',
                    '--task', task, '--checkpoint', str(checkpoint.resolve()),
                    '--commands', str(command_path), '--device', args.device,
                    '--num-envs', str(args.num_envs), '--seed', str(seed),
                    '--seconds', str(args.seconds), '--warmup', str(args.warmup),
                    '--deployment-target-clamp', '--compose-com-randomization']
            jobs.append({'name': f'{label}-s{seed}', 'policy': label, 'seed': seed,
                         'task': task, 'commands': sequence, 'argv_without_out': argv})
    vector_steps = sum(len(j['commands']) * math.ceil(args.seconds / .02) for j in jobs)
    return {'schema_version': 1, 'repo': str(repo), 'python': str(python), 'device': args.device,
            'num_envs': args.num_envs, 'seconds_per_command': args.seconds, 'warmup_s': args.warmup,
            'seeds': args.seeds, 'skip_spider': skip_spider, 'inputs': inputs, 'source_sha256': source_identity(repo),
            'runner_sha256': sha(__file__), 'jobs': jobs,
            'workload': {'jobs': len(jobs), 'vector_control_steps': vector_steps,
                         'robot_control_steps': vector_steps * args.num_envs,
                         'robot_physics_steps': vector_steps * args.num_envs * 5,
                         'simulated_robot_seconds': vector_steps * .02 * args.num_envs,
                         'wall_time_estimate': 'Measure the first completed job; GPU type, compilation cache and CPU overhead determine elapsed time.'},
            'scope': 'Flat-terrain simulation; play configuration retains startup physics randomization and observation noise. No external-push or hardware evidence. No reset between command segments.',
            'automatic_promotion': False, 'automatic_walking_limits': False}


def verify_inputs(plan):
    for name, item in plan['inputs'].items():
        if sha(item['path']) != item['sha256']:
            raise ValueError(f'{name} bytes changed; preserve this run and use a new output directory')
    if source_identity(Path(plan['repo'])) != plan['source_sha256']:
        raise ValueError('Source changed during the sweep; preserve this run and use a new output directory')


def artifact_hashes(directory):
    return {name: sha(directory / name) for name in ARTIFACTS}


def completed_artifacts_match(item):
    try:
        return artifact_hashes(Path(item['directory'])) == item['artifact_sha256']
    except (OSError, KeyError):
        return False


def launch(command, repo, log):
    env = os.environ.copy()
    env.pop('PUPPER_REFERENCE_DIR', None)
    env['MUJOCO_GL'] = env['PYOPENGL_PLATFORM'] = 'egl'
    env['PYTHONUNBUFFERED'] = '1'
    with log.open('x') as stream:
        process = subprocess.Popen(command, cwd=repo, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return process.wait()
        except BaseException:
            # Own only this child process group; never stop another GPU job.
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            raise


def summarize(plan, status):
    """Expose measured tracking for review; never infer operating caps from survival."""
    import numpy as np

    summary = {'completed_jobs': [], 'incomplete_jobs': [], 'errors': [], 'stable_comparisons': [],
               'stable_pair_controls': [],
               'spider_commands': [], 'automatic_promotion': False, 'approved_for_hardware': False}
    results = {}
    for job in plan['jobs']:
        attempts = status['jobs'].get(job['name'], [])
        item = attempts[-1] if attempts else {}
        if item.get('state') != 'completed' or not completed_artifacts_match(item):
            summary['incomplete_jobs'].append(job['name'])
            continue
        directory = Path(item['directory'])
        metrics = json.loads((directory / 'metrics.json').read_text())
        results[job['name']] = metrics
        summary['completed_jobs'].append({'name': job['name'], 'directory': str(directory),
                                          'artifact_sha256': item['artifact_sha256']})
        if job['policy'] == 'spider-baseline':
            with np.load(directory / 'trace.npz', allow_pickle=False) as trace:
                for row in metrics['segments']:
                    mask = (trace['segment'] == row['segment'])[:, None] & trace['valid'].astype(bool)
                    velocities = trace['velocity'][mask]
                    summary['spider_commands'].append({
                        'seed': job['seed'], **row,
                        'mean_achieved_velocity_m_s_m_s_rad_s': velocities.mean(axis=0).tolist() if len(velocities) else None,
                    })
    keys = ('valid_samples', 'failures', 'timeouts', 'mean_xy_error_m_s', 'mean_abs_yaw_error_rad_s',
            'mean_torque_squared_nm2', 'mean_contact_slip_m_s', 'target_clipped_fraction',
            'maximum_target_range_excess_rad')
    for seed in plan['seeds']:
        a, b = (results.get(f'stable-{policy}-s{seed}') for policy in ('recovered', 'finetuned'))
        if a is None or b is None:
            continue
        dirs = [Path(status['jobs'][f'stable-{label}-s{seed}'][-1]['directory'])
                for label in ('recovered', 'finetuned')]
        with np.load(dirs[0] / 'randomization_snapshot.npz', allow_pickle=False) as left, \
             np.load(dirs[1] / 'randomization_snapshot.npz', allow_pickle=False) as right:
            same_randomization = set(left.files) == set(right.files) and all(
                np.array_equal(left[key], right[key]) for key in left.files)
        controls = {'seed': seed,
                    'source_equal': a['provenance']['source_sha256'] == b['provenance']['source_sha256'],
                    'environment_config_equal': sha(dirs[0] / 'env.yaml') == sha(dirs[1] / 'env.yaml'),
                    'realized_randomization_equal': same_randomization}
        summary['stable_pair_controls'].append(controls)
        if not all(controls[key] for key in ('source_equal', 'environment_config_equal', 'realized_randomization_equal')):
            summary['errors'].append(f'seed {seed}: StableGait environments/source were not paired; comparison withheld')
            continue
        for original, candidate in zip(a['segments'], b['segments'], strict=True):
            if original['command'] != candidate['command']:
                raise ValueError(f'seed {seed}: paired commands differ')
            summary['stable_comparisons'].append({'seed': seed, 'segment': original['segment'],
                'command': original['command'], 'recovered': {key: original.get(key) for key in keys},
                'finetuned': {key: candidate.get(key) for key in keys}})
    spider_complete = all(f'spider-baseline-s{seed}' in results for seed in plan['seeds'])
    zero_failures = spider_complete and all(row['failures'] == 0 and row['timeouts'] == 0
        and row['valid_samples'] > 0 for row in summary['spider_commands'])
    summary['spider_limit_review'] = {
        'evaluation_requested': not plan.get('skip_spider', False),
        'all_requested_runs_complete': spider_complete,
        'zero_failures_and_nonempty_measurements': zero_failures,
        'tracking_judgment': 'pending human review; no arbitrary tracking-error cutoff is encoded',
        'recommended_limits': None,
        'rule': ('Do not recommend limits when any run fails or measurements are missing. Even with zero failures, '
                 'inspect achieved forward/lateral/yaw velocities against requests across every seed and both directions, '
                 'plus slip, effort, stopping and transition traces. Only then propose conservative tested commands whose '
                 'tracking is reasonable for the intended use. Do not extrapolate above tested commands or treat training '
                 'caps as achieved speeds. A locomotion sweep does not validate the Spider entry/exit policy handoff.'),
    }
    return summary


def run_plan(plan, out, *, resume=False, retry_incomplete=False, execute=launch):
    out = out.resolve()
    plan_path, status_path = out / 'plan.json', out / 'status.json'
    if resume:
        if json.loads(plan_path.read_text()) != plan:
            raise ValueError('Resume requires identical checkpoints, commands, source and parameters; use a new output directory')
        status = json.loads(status_path.read_text())
    else:
        out.mkdir(parents=True, exist_ok=False)
        save(plan_path, plan)
        status = {'created_utc': utc_now(), 'state': 'prepared', 'jobs': {}, 'automatic_promotion': False}
    save(status_path, status)
    try:
        for job in plan['jobs']:
            attempts = status['jobs'].setdefault(job['name'], [])
            previous = attempts[-1] if attempts else {}
            if previous.get('state') == 'completed':
                if not completed_artifacts_match(previous):
                    raise ValueError(f"{job['name']}: completed outputs changed or disappeared; use a new output directory")
                continue
            if previous.get('state') == 'running' and not retry_incomplete:
                raise ValueError(f"{job['name']}: prior worker may still be running. Confirm it stopped before --retry-incomplete")
            verify_inputs(plan)
            job_dir = out / 'jobs' / job['name']
            job_dir.mkdir(parents=True, exist_ok=True)
            number = len(attempts) + 1
            directory = job_dir / f'attempt-{number:03d}'
            log = job_dir / f'attempt-{number:03d}.log'
            command = [*job['argv_without_out'], '--out', str(directory)]
            item = {'state': 'running', 'directory': str(directory), 'log': str(log),
                    'command': command, 'started_utc': utc_now()}
            attempts.append(item)
            status.update(state='running', active_job=job['name'])
            save(status_path, status)
            started = time.monotonic()
            print(f"{job['name']}: {directory}", flush=True)
            try:
                item['returncode'] = execute(command, Path(plan['repo']), log)
                if item['returncode']:
                    raise RuntimeError(f"evaluation exited {item['returncode']}; see {log}")
                item['artifact_sha256'] = artifact_hashes(directory)
                metrics = json.loads((directory / 'metrics.json').read_text())
                if len(metrics['segments']) != len(job['commands']):
                    raise ValueError('Incomplete command sweep in metrics')
                if [row['command'] for row in metrics['segments']] != job['commands']:
                    raise ValueError('Reported commands differ from the prepared sweep')
                provenance = metrics['provenance']
                if provenance['source_sha256'] != plan['source_sha256']:
                    raise ValueError('Evaluation source identity differs from prepared plan')
                expected = dict(seed=job['seed'], num_envs=plan['num_envs'], task=job['task'],
                                seconds=plan['seconds_per_command'], warmup=plan['warmup_s'],
                                deployment_target_clamp=True, compose_com_randomization=True)
                if any(provenance.get(key) != value for key, value in expected.items()):
                    raise ValueError('Evaluation protocol differs from the prepared plan')
                checkpoint = command[command.index('--checkpoint') + 1]
                if provenance['checkpoint_sha256'] != sha(checkpoint):
                    raise ValueError('Evaluated checkpoint bytes differ from the explicit input')
                item['state'] = 'completed'
            except (OSError, ValueError, KeyError, RuntimeError) as exc:
                item.update(state='failed', error=f'{type(exc).__name__}: {exc}')
            except BaseException as exc:
                item.update(state='interrupted', error=type(exc).__name__)
                raise
            finally:
                item.update(finished_utc=utc_now(), elapsed_seconds=time.monotonic() - started)
                save(status_path, status)
            print(f"{job['name']}: {item['state']} ({item['elapsed_seconds']:.1f}s)", flush=True)
        summary = summarize(plan, status)
        save(out / 'summary.json', summary)
        has_errors = bool(summary['incomplete_jobs'] or summary['errors'])
        status['state'] = 'finished_with_errors' if has_errors else 'finished_requires_review'
        status['summary'] = str(out / 'summary.json')
        return 1 if has_errors else 0
    except BaseException as exc:
        status.update(state='interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        status.pop('active_job', None)
        status['updated_utc'] = utc_now()
        save(status_path, status)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--python', type=Path, help='Defaults to <repo>/pupper-mjlab/.venv/bin/python')
    parser.add_argument('--recovered-checkpoint', type=Path, required=True)
    parser.add_argument('--finetuned-checkpoint', type=Path, required=True)
    parser.add_argument('--spider-checkpoint', type=Path)
    parser.add_argument('--skip-spider', action='store_true', help='Run only the six StableGait jobs; do not inspect or evaluate Spider inputs')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--stable-commands', type=Path)
    parser.add_argument('--spider-commands', type=Path)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--seeds', type=int, nargs='+', default=[101, 202, 303])
    parser.add_argument('--num-envs', type=int, default=32)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--warmup', type=float, default=2.)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--retry-incomplete', action='store_true')
    parser.add_argument('--plan-only', action='store_true', help='Print JSON plan; create no result directory or subprocess')
    args = parser.parse_args()
    if args.retry_incomplete and not args.resume:
        parser.error('--retry-incomplete requires --resume')
    plan = make_plan(args)
    if args.plan_only:
        print(json.dumps(plan, indent=2))
        return 0
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    return run_plan(plan, args.out, resume=args.resume, retry_incomplete=args.retry_incomplete)


if __name__ == '__main__':
    raise SystemExit(main())
