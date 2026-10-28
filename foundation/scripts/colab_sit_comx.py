"""Bounded Sit COM-distribution continuation, with matched prior-policy scores.

Dispatch only after the runtime's current workers finish. Source is pinned and
never pulled here. The original reference/rewards/limits and previous policies
remain untouched. All outputs need inspection; no policy is installed by this job.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

EXPECTED = {
    'retained3998.pt': 'ee256ec97d1a4b63122c9658c13f1d91b19ef047f0b3033460f7197c08846e14',
    'prior4997.pt': 'f141a03427da03ef5c3a693fca23b80290df9baa8e5e9dd4bed6976bbdbe6b85',
    'sit_stay_shake.npz': '5e7129e92ee10c3305e13ef33d973d5682664c813d7c2df5397bc9b85926d137',
    'prior-foundation-run.json': 'f50fa05663316ffb28a9142ae0a7df7a71f9b2fc3fa0679c1c7eba51b20e8aa2',
    'prior-env.yaml': '09e2458bfe43da71b64c84736e2a9a235bcbef351e745f5e4a3fc9623788bc62',
}
DRIVER_SHA = '733302896259e7177cd8f74cea1029542ea31dd224dc79af7426ba131c7974d3'
HELD_OUT = [('com_back_015', -.015), ('com_front_015', .015),
            ('com_back_025', -.025), ('com_front_025', .025)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_source(repo, revision):
    actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    if actual != revision:
        raise ValueError(f'expected source revision {revision}, got {actual}')
    if subprocess.run(['git', 'diff', '--quiet', 'HEAD', '--', 'pupper-mjlab',
                       'foundation/scripts/colab_sit_comx.py',
                       'foundation/configs/class-driver-joint-limits.json'], cwd=repo).returncode:
        raise ValueError('training source has uncommitted changes')
    return actual


def plan(repo, inputs, output, revision):
    actual = check_source(repo, revision)
    for name, digest in EXPECTED.items():
        if sha(inputs / name) != digest:
            raise ValueError(f'input hash mismatch: {name}')
    driver = repo / 'foundation/configs/class-driver-joint-limits.json'
    if sha(driver) != DRIVER_SHA:
        raise ValueError('class driver limits changed')
    project = repo / 'pupper-mjlab'
    prior = json.loads((inputs / 'prior-foundation-run.json').read_text())
    allowed_changes = {'src/mjlab/scripts/pupper_foundation.py', 'src/mjlab/tasks/pupper_gait/bootstrap.py'}
    differences = [name for name, digest in prior['source']['files'].items()
                   if not (project / name).is_file() or sha(project / name) != digest]
    if set(differences) - allowed_changes:
        raise ValueError(f'unreviewed source difference from prior Sit training: {differences}')
    common = ['--deployment-target-clamp', '--driver-limits', driver,
              '--tracking-ground-hulls']
    command = [project / '.venv/bin/python', '-m', 'mjlab.scripts.pupper_foundation',
               'train', 'sit_stay_shake', '--motion', output / 'inputs/sit_stay_shake.npz',
               '--initial-checkpoint', output / 'inputs/retained3998.pt',
               '--output', output / 'training', *common, '--compose-com-randomization',
               '--tracking-com-x-half-range', '.02', '--iterations', '1000',
               '--num-envs', '4096', '--seed', '42', '--logger', 'tensorboard']
    return dict(source_revision=actual, input_sha256=EXPECTED, driver_limits_sha256=DRIVER_SHA,
                differences_from_prior_source=differences,
                reviewed_source_change_scope='launcher explicit overrides plus unrelated locomotion bootstrap; other hashed prior sources identical',
                training_command=list(map(str, command)), x_training_half_range_m=.02,
                unchanged_yz_half_range_m=.01, iterations=1000, seed=42,
                comparisons=['candidate_comx02', 'prior_driver4997', 'retained3998'],
                held_out_cases=[dict(name=name, com_x_m=x,
                                    distribution='inside widened training support' if abs(x) < .02 else 'outside widened training support')
                                for name, x in HELD_OUT],
                course_front_case='x+0.02 m was outside original +/-0.01 m training support; now at widened support boundary',
                approved_for_hardware=False)


def yaml_data(path):
    """Read saved Python-tagged config as inert data, never import/execute tags."""
    import yaml
    class DataLoader(yaml.SafeLoader):
        pass
    def tagged(loader, suffix, node):
        if isinstance(node, yaml.MappingNode):
            return loader.construct_mapping(node)
        if isinstance(node, yaml.SequenceNode):
            return loader.construct_sequence(node)
        return {'python_tag': suffix, 'value': loader.construct_scalar(node)}
    DataLoader.add_multi_constructor('tag:yaml.org,2002:python/', tagged)
    return yaml.load(Path(path).read_text(), Loader=DataLoader)


def audit_training(manifest_path, inputs):
    record = json.loads(manifest_path.read_text())
    prior = json.loads((inputs / 'prior-foundation-run.json').read_text())
    if record['state'] != 'trained_unvalidated':
        raise ValueError('training did not finish')
    for key in ('profile_config', 'seed', 'num_envs', 'iterations', 'reward_parameter_overrides',
                'com_randomization_contract', 'collision_contract'):
        if record.get(key, {}) != prior.get(key, {}):
            raise ValueError(f'unexpected training contract change: {key}')
    # Paths may differ; all numerical command bounds and the source hash must match.
    candidate_contract = dict(record['actuation_contract'])
    prior_contract = dict(prior['actuation_contract'])
    candidate_source = dict(candidate_contract.pop('source'))
    prior_source = dict(prior_contract.pop('source'))
    candidate_source.pop('path'); prior_source.pop('path')
    if candidate_contract != prior_contract or candidate_source != prior_source:
        raise ValueError('actuation contract changed')
    if record['initial_checkpoint_sha256'] != EXPECTED['retained3998.pt']:
        raise ValueError('wrong initial checkpoint')
    if record['motion_sha256'] != EXPECTED['sit_stay_shake.npz']:
        raise ValueError('reference changed')
    ranges = record['domain_randomization_overrides']['base_com.ranges']
    if ranges['value'] != {'0': [-.02, .02], '1': [-.01, .01], '2': [-.01, .01]}:
        raise ValueError('wrong COM range')
    baseline = yaml_data(inputs / 'prior-env.yaml')
    trained = yaml_data(manifest_path.parent / 'params/env.yaml')
    if baseline['events']['base_com']['params']['ranges'] != {0: [-.01, .01], 1: [-.01, .01], 2: [-.01, .01]}:
        raise ValueError('prior COM ranges differ from expected course distribution')
    baseline['events']['base_com']['params']['ranges'][0] = [-.02, .02]
    baseline['commands']['motion']['motion_file'] = trained['commands']['motion']['motion_file']
    if baseline != trained:
        raise ValueError('saved env config differs beyond X COM range and frozen motion path')
    checkpoint = manifest_path.parent / record['checkpoint']
    if sha(checkpoint) != record['checkpoint_sha256']:
        raise ValueError('trained checkpoint hash mismatch')
    return checkpoint, dict(saved_env_match_except=['events.base_com.params.ranges.0', 'commands.motion.motion_file'],
                            prior_manifest_sha256=sha(inputs / 'prior-foundation-run.json'),
                            prior_env_sha256=sha(inputs / 'prior-env.yaml'),
                            candidate_manifest_sha256=sha(manifest_path),
                            candidate_env_sha256=sha(manifest_path.parent / 'params/env.yaml'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path('/content/pupper-foundation'))
    parser.add_argument('--inputs', type=Path, default=Path('/content/pupper-sit-comx-inputs'))
    parser.add_argument('--output', type=Path, default=Path('/content/pupper-sit-comx02'))
    parser.add_argument('--expected-revision', required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    repo, inputs, out = (path.resolve() for path in (args.repo, args.inputs, args.output))
    record = plan(repo, inputs, out, args.expected_revision)
    if args.plan_only:
        print(json.dumps(record, indent=2)); return
    out.mkdir(parents=True, exist_ok=False)
    shutil.copytree(inputs, out / 'inputs')
    inputs = out / 'inputs'
    state = dict(stage='starting', created_utc=datetime.now(timezone.utc).isoformat(),
                 plan=record, jobs=[], approved_for_hardware=False)
    project = repo / 'pupper-mjlab'; python = project / '.venv/bin/python'
    driver = repo / 'foundation/configs/class-driver-joint-limits.json'
    motion = inputs / 'sit_stay_shake.npz'
    def save():
        temporary = out / 'status.tmp'
        temporary.write_text(json.dumps(state, indent=2) + '\n')
        temporary.replace(out / 'status.json')
    def run(name, command):
        check_source(repo, args.expected_revision)
        job = dict(name=name, command=list(map(str, command)))
        state['stage'] = name; state['jobs'].append(job); save()
        with (out / f'{name}.log').open('w') as log:
            result = subprocess.run(job['command'], cwd=project, stdout=log, stderr=subprocess.STDOUT)
        job['returncode'] = result.returncode; save()
        if result.returncode:
            raise RuntimeError(f'{name} failed; see saved log')
    os.environ['MUJOCO_GL'] = os.environ['PYOPENGL_PLATFORM'] = 'egl'
    os.environ['PYTHONUNBUFFERED'] = '1'
    try:
        run('training', record['training_command'])
        manifests = list((out / 'training/runs').glob('*/foundation-run.json'))
        if len(manifests) != 1:
            raise ValueError('expected exactly one isolated training run')
        candidate, state['config_audit'] = audit_training(manifests[0], inputs)
        save()
        # Each policy gets the same eight course/stress cases and four fixed
        # offset tests, using the same seed/device/source and unmodified motion.
        for name, checkpoint in [('candidate_comx02', candidate),
                                 ('prior_driver4997', inputs / 'prior4997.pt'),
                                 ('retained3998', inputs / 'retained3998.pt')]:
            destination = out / 'evaluation' / name
            run(f'{name}-suite', [python, '-m', 'mjlab.scripts.pupper_foundation', 'score',
                '--motion', motion, '--checkpoint', checkpoint, '--output', destination,
                '--deployment-target-clamp', '--driver-limits', driver, '--tracking-ground-hulls',
                '--suite', '--stress-profile', 'sit_stay_shake'])
            for label, x in HELD_OUT:
                run(f'{name}-{label}', [python, '-m', 'mjlab.tasks.tracking.config.pupper.play_motion',
                    motion, '--checkpoint', checkpoint, '--deployment-target-clamp', 'True',
                    '--driver-limits', driver, '--tracking-ground-hulls', 'True', '--seed', '101',
                    '--com-x', x, '--report-file', destination / f'{label}-s101.json'])
        state['stage'] = 'finished_requires_inspection'
    except Exception as error:
        state.update(stage='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        save()


if __name__ == '__main__':
    main()
