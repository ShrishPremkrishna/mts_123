"""Start deployment-matched finetunes on the existing second Colab runtime.

Requires the original Mixed baseline and uploaded recovered StableGait checkpoint.
The old outputs are retained. This script creates no runtime and controls no robot.
"""
import json
import os
from pathlib import Path
import subprocess

repo = Path('/content/pupper-foundation')
project = repo / 'pupper-mjlab'
python = project / '.venv/bin/python'
root = Path('/content/pupper-clamp-results')
subprocess.run(['git', 'pull', '--ff-only'], cwd=repo, check=True)
env = os.environ.copy()
env['MUJOCO_GL'] = env['PYOPENGL_PLATFORM'] = 'egl'
env['PYTHONUNBUFFERED'] = '1'
stable = root / 'stablegait-deployclamp-original-rewards-s42'
recovered = Path('/content/recovered-policies/4hkyvuxh/model_2999.pt')
assert recovered.is_file(), recovered
root.mkdir(exist_ok=False)
jobs = []

def launch(name, command, cwd):
    with (root / f'{name}.log').open('w') as log:
        process = subprocess.Popen(list(map(str, command)), cwd=cwd, env=env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    jobs.append({'name': name, 'pid': process.pid, 'command': list(map(str, command))})
    (root / 'workers.json').write_text(json.dumps(jobs, indent=2))
    print(f'{name} started pid={process.pid}', flush=True)

launch('stablegait-training', [python, '-m', 'mjlab.scripts.pupper_locomotion', 'train',
    '--task', 'Mjlab-StableGait-Flat-Pupper-v3', '--checkpoint', recovered,
    '--out', stable, '--weights', repo/'foundation/configs/lab4-stablegait-original-rewards.json',
    '--deployment-target-clamp', '--iterations', '1500', '--num-envs', '4096',
    '--seed', '42', '--device', 'cuda:0', '--logger', 'tensorboard'], project)
launch('optional2-batch', [python, repo/'foundation/scripts/colab_optional.py',
    '--repo', repo, '--out', root/'optional2', '--mode', 'locomotion', '--no-wandb',
    '--deployment-target-clamp', '--baseline-checkpoint',
    '/content/pupper-optional-results/mixed-baseline/model_2999.pt',
    '--baseline-iterations', '1500'], repo)
