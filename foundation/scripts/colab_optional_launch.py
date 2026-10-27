"""Run with `colab exec`; installs optional dependencies and starts a logged worker."""
import json
import os
from pathlib import Path
import subprocess

repo = Path('/content/pupper-foundation')
output = Path('/content/pupper-optional-results')
output.mkdir(exist_ok=True)
python = repo / 'pupper-mjlab/.venv/bin/python'
subprocess.run(['git', 'pull', '--ff-only'], cwd=repo, check=True)
subprocess.run(['git', 'sparse-checkout', 'set', 'pupper-mjlab', 'pupper_gait_deploy',
                'foundation', 'implicit_compliance'], cwd=repo, check=True)
setup_log = output / 'setup.log'
with setup_log.open('w') as log:
    result = subprocess.run(['uv', 'pip', 'install', '--python', str(python),
                             'Pillow', 'gdown'], stdout=log, stderr=subprocess.STDOUT)
    if result.returncode == 0:
        # Match the locked Torch 2.9/CUDA 12.8 environment; never let an optional
        # image classifier silently replace the RL runtime's Torch installation.
        result = subprocess.run(['uv', 'pip', 'install', '--python', str(python),
            '--no-deps', '--index-url', 'https://download.pytorch.org/whl/cu128',
            'torchvision==0.24.0+cu128'], stdout=log, stderr=subprocess.STDOUT)
if result.returncode:
    print(setup_log.read_text()[-6000:], flush=True)
    raise RuntimeError('Optional dependencies failed to install')
subprocess.run([str(python), '-c',
    'import torch,torchvision; assert torch.cuda.is_available(); print(torch.__version__,torchvision.__version__)'], check=True)
env = os.environ.copy()
env['MUJOCO_GL'] = env['PYOPENGL_PLATFORM'] = 'egl'
env['PYTHONUNBUFFERED'] = '1'
command = [str(python), str(repo / 'foundation/scripts/colab_optional.py'),
           '--repo', str(repo), '--out', str(output), '--skip-implicit', '--no-wandb']
with (output / 'worker.log').open('a') as log:
    process = subprocess.Popen(command, cwd=repo, env=env, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
(output / 'worker.json').write_text(json.dumps({'pid':process.pid, 'command':command}, indent=2))
print(f'OPTIONAL_WORKER_STARTED pid={process.pid} logs={output}', flush=True)
