"""Run through the official Colab CLI; prepares this branch without starting motors."""
import os
import pathlib
import subprocess


def run(cmd, cwd=None):
    subprocess.run(cmd, cwd=cwd, check=True)

run(['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv'])
repo = pathlib.Path('/content/pupper-foundation')
os.environ['GIT_LFS_SKIP_SMUDGE'] = '1'
if not repo.exists():
    run(['git', 'clone', '--depth', '1', '--filter=blob:none', '--sparse', '--branch',
         'pupper-foundation', 'https://github.com/ShrishPremkrishna/mts_123.git', str(repo)])
    run(['git', 'sparse-checkout', 'set', 'pupper-mjlab', 'pupper_gait_deploy', 'foundation'], repo)
else:
    run(['git', 'pull', '--ff-only'], repo)
run(['git', 'rev-parse', 'HEAD'], repo)
run(['python', '-m', 'pip', 'install', 'uv'])
run(['uv', 'sync', '--extra', 'cu128'], repo/'pupper-mjlab')
run(['uv', 'run', '--no-sync', 'python', '-c', 'import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))'], repo/'pupper-mjlab')
print('PUPPER_BOOTSTRAP_READY', flush=True)
