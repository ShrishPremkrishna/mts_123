"""The evaluation runner must preserve evidence and never promote from survival."""
import importlib.util
import json
import os
import subprocess
import venv
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

PATH = Path(__file__).parents[1] / 'scripts/colab_gait_validation.py'
spec = importlib.util.spec_from_file_location('gait_validation_runner', PATH)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.fixture
def plan(tmp_path):
    repo = tmp_path / 'repo'
    package = repo / 'pupper-mjlab/src/mjlab'
    package.mkdir(parents=True)
    (package / 'source.py').write_text('# fixed test source\n')
    inputs = {}
    for name in ('recovered', 'finetuned', 'spider'):
        inputs[name] = tmp_path / f'{name}.pt'
        inputs[name].write_bytes(name.encode())
    for name, count in (('stable', 22), ('spider', 18)):
        inputs[name + '_commands'] = tmp_path / f'{name}.json'
        inputs[name + '_commands'].write_text(json.dumps([[0.01 * i, 0, 0] for i in range(count)]))
    args = SimpleNamespace(repo=repo, python=Path(sys.executable), seeds=[101, 202, 303],
        num_envs=32, seconds=10., warmup=2., device='cuda:0',
        recovered_checkpoint=inputs['recovered'], finetuned_checkpoint=inputs['finetuned'],
        spider_checkpoint=inputs['spider'], stable_commands=inputs['stable_commands'],
        spider_commands=inputs['spider_commands'])
    return runner.make_plan(args)


def fake_runtime(plan, log):
    log.write_text('simulated runtime for evaluator orchestration tests\n')
    return {'test_runtime': True}


def fake_execute(plan, fail_name=None):
    def execute(argv, repo, log):
        assert argv[3:5] == ['mjlab.scripts.pupper_locomotion', 'evaluate']
        def arg(name):
            return argv[argv.index(name) + 1]
        directory = Path(arg('--out'))
        directory.mkdir()
        log.write_text('preserved evaluator output\n')
        if fail_name and fail_name in str(directory):
            (directory / 'partial.txt').write_text('preserve failed attempt')
            return 1
        sequence = json.loads(Path(arg('--commands')).read_text())
        seed = int(arg('--seed'))
        n = int(arg('--num-envs'))
        provenance = dict(source_sha256=plan['source_sha256'], checkpoint_sha256=runner.sha(arg('--checkpoint')),
                          seed=seed, num_envs=n, task=arg('--task'), seconds=float(arg('--seconds')),
                          warmup=float(arg('--warmup')), deployment_target_clamp=True, compose_com_randomization=True)
        rows = [dict(segment=i, command=c, valid_samples=n, failures=0, timeouts=0,
                     mean_xy_error_m_s=1.0) for i, c in enumerate(sequence)]
        (directory / 'metrics.json').write_text(json.dumps(dict(segments=rows, provenance=provenance)))
        (directory / 'provenance.json').write_text(json.dumps(provenance))
        (directory / 'env.yaml').write_text(f'seed: {seed}\n')
        np.savez(directory / 'randomization_snapshot.npz', mass=np.full(n, seed / 100))
        np.savez(directory / 'trace.npz', segment=np.arange(len(sequence)),
                 valid=np.ones((len(sequence), n), dtype=bool), velocity=np.zeros((len(sequence), n, 3)))
        return 0
    return execute


def test_default_plan_contains_exact_nine_reviewed_sweeps(plan):
    assert len(plan['jobs']) == 9
    assert plan['workload']['vector_control_steps'] == 93000
    assert plan['workload']['simulated_robot_seconds'] == 59520
    assert plan['workload']['robot_physics_steps'] == 14880000
    for job in plan['jobs']:
        argv = job['argv_without_out']
        assert '--deployment-target-clamp' in argv and '--compose-com-randomization' in argv
        assert 'train' not in argv and '--randomized' not in argv
    assert not plan['automatic_promotion'] and not plan['automatic_walking_limits']


def test_skip_spider_omits_all_spider_inputs_and_jobs(plan):
    args = SimpleNamespace(repo=Path(plan['repo']), python=Path(plan['python']), seeds=plan['seeds'],
        num_envs=32, seconds=10., warmup=2., device='cuda:0', skip_spider=True,
        recovered_checkpoint=Path(plan['inputs']['recovered']['path']),
        finetuned_checkpoint=Path(plan['inputs']['finetuned']['path']), spider_checkpoint=None,
        stable_commands=Path(plan['inputs']['stable_commands']['path']), spider_commands=Path('/missing/unused.json'))
    stable_only = runner.make_plan(args)
    assert len(stable_only['jobs']) == 6
    assert all(job['policy'].startswith('stable-') for job in stable_only['jobs'])
    assert 'spider' not in stable_only['inputs'] and 'spider_commands' not in stable_only['inputs']
    assert stable_only['workload']['vector_control_steps'] == 66000
    assert stable_only['workload']['simulated_robot_seconds'] == 42240
    args.skip_spider = False
    with pytest.raises(ValueError, match='spider-checkpoint'):
        runner.make_plan(args)


def test_zero_failures_with_poor_tracking_never_creates_speed_recommendations(plan, tmp_path):
    output = tmp_path / 'results'
    assert runner.run_plan(plan, output, check_runtime=fake_runtime, execute=fake_execute(plan)) == 0
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['spider_limit_review']['zero_failures_and_nonempty_measurements']
    assert summary['spider_limit_review']['recommended_limits'] is None
    assert not summary['automatic_promotion'] and not summary['approved_for_hardware']
    assert all(row['realized_randomization_equal'] for row in summary['stable_pair_controls'])
    assert json.loads((output / 'status.json').read_text())['state'] == 'finished_requires_review'


def test_resume_skips_verified_success_and_preserves_failed_attempts(plan, tmp_path):
    output = tmp_path / 'results'
    assert runner.run_plan(plan, output, check_runtime=fake_runtime, execute=fake_execute(plan, 'stable-finetuned-s101')) == 1
    old = output / 'jobs/stable-finetuned-s101/attempt-001/partial.txt'
    assert old.read_text() == 'preserve failed attempt'
    calls = []
    success = fake_execute(plan)
    def retry(argv, repo, log):
        calls.append(argv)
        return success(argv, repo, log)
    assert runner.run_plan(plan, output, resume=True, check_runtime=fake_runtime, execute=retry) == 0
    assert len(calls) == 1 and old.exists()
    assert (output / 'jobs/stable-finetuned-s101/attempt-002/metrics.json').is_file()


def test_changed_completed_output_is_not_silently_reused(plan, tmp_path):
    output = tmp_path / 'results'
    runner.run_plan(plan, output, check_runtime=fake_runtime, execute=fake_execute(plan))
    (output / 'jobs/stable-recovered-s101/attempt-001/metrics.json').write_text('{}')
    with pytest.raises(ValueError, match='changed or disappeared'):
        runner.run_plan(plan, output, resume=True, check_runtime=fake_runtime, execute=lambda *args: pytest.fail('must not launch'))


def test_changed_input_blocks_launch_and_resume(plan, tmp_path):
    Path(plan['inputs']['finetuned']['path']).write_bytes(b'other checkpoint')
    with pytest.raises(ValueError, match='bytes changed'):
        runner.run_plan(plan, tmp_path / 'results', check_runtime=fake_runtime, execute=lambda *args: pytest.fail('must not launch'))


def test_incomplete_worker_requires_explicit_retry_after_it_stops(plan, tmp_path):
    output = tmp_path / 'results'
    runner.run_plan(plan, output, check_runtime=fake_runtime, execute=fake_execute(plan))
    status = json.loads((output / 'status.json').read_text())
    status['jobs']['stable-recovered-s101'][-1]['state'] = 'running'
    runner.save(output / 'status.json', status)
    with pytest.raises(ValueError, match='may still be running'):
        runner.run_plan(plan, output, resume=True, check_runtime=fake_runtime, execute=lambda *args: pytest.fail('must not launch'))


def test_wrong_reported_protocol_cannot_be_marked_complete(plan, tmp_path):
    execute = fake_execute(plan)
    def wrong_seed(argv, repo, log):
        result = execute(argv, repo, log)
        path = Path(argv[argv.index('--out') + 1]) / 'metrics.json'
        record = json.loads(path.read_text())
        record['provenance']['seed'] = -1
        path.write_text(json.dumps(record))
        return result
    assert runner.run_plan(plan, tmp_path / 'results', check_runtime=fake_runtime, execute=wrong_seed) == 1
    status = json.loads((tmp_path / 'results/status.json').read_text())
    assert all(attempts[-1]['state'] == 'failed' for attempts in status['jobs'].values())


@pytest.mark.parametrize('explicit', [False, True])
def test_venv_interpreter_preserves_prefix_and_installed_package(plan, tmp_path, explicit):
    repo = Path(plan['repo'])
    environment = repo / 'pupper-mjlab/.venv'
    venv.EnvBuilder(with_pip=False, symlinks=True).create(environment)
    interpreter = environment / 'bin/python'
    assert interpreter.is_symlink()
    site = Path(subprocess.check_output([str(interpreter), '-c',
        'import sysconfig; print(sysconfig.get_path("purelib"))'], text=True).strip())
    (site / 'foundation_venv_probe.py').write_text('VALUE = "only-in-venv"\n')
    args = SimpleNamespace(repo=repo,
        python=Path(os.path.relpath(interpreter)) if explicit else None,
        seeds=[101], num_envs=1, seconds=1., warmup=0., device='cpu', skip_spider=True,
        recovered_checkpoint=Path(plan['inputs']['recovered']['path']),
        finetuned_checkpoint=Path(plan['inputs']['finetuned']['path']), spider_checkpoint=None,
        stable_commands=Path(plan['inputs']['stable_commands']['path']), spider_commands=None)
    prepared = runner.make_plan(args)
    executable = prepared['jobs'][0]['argv_without_out'][0]
    result = json.loads(subprocess.check_output([executable, '-B', '-c',
        'import json, sys, foundation_venv_probe; print(json.dumps([sys.prefix, foundation_venv_probe.VALUE]))'],
        text=True, cwd=repo))
    assert prepared['python'] == str(interpreter)
    assert result == [str(environment), 'only-in-venv']
    # The old resolve() behavior demonstrably cannot import the venv-only package.
    bypassed = subprocess.run([str(interpreter.resolve()), '-B', '-c',
        'import foundation_venv_probe'], text=True, capture_output=True, cwd=repo)
    assert bypassed.returncode != 0 and 'ModuleNotFoundError' in bypassed.stderr


def test_missing_runtime_dependency_fails_once_before_any_evaluation(plan, tmp_path):
    environment = tmp_path / 'empty-venv'
    venv.EnvBuilder(with_pip=False, symlinks=True).create(environment)
    plan['python'] = str(environment / 'bin/python')
    output = tmp_path / 'results'
    with pytest.raises(RuntimeError, match='no evaluation jobs launched'):
        runner.run_plan(plan, output, execute=lambda *args: pytest.fail('must not launch'))
    status = json.loads((output / 'status.json').read_text())
    assert status['state'] == 'failed' and status['jobs'] == {}
    assert 'ModuleNotFoundError' in (output / 'preflight.log').read_text()
    assert (output / 'plan.json').is_file()
