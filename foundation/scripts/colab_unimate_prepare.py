"""Colab worker: generate and score candidates; training waits for inspection."""
import json
import os
from pathlib import Path
import subprocess

repo = Path('/content/pupper-foundation')
project = repo / 'pupper-mjlab'
python = project / '.venv/bin/python'
output = Path('/content/pupper-generated-results')
output.mkdir(exist_ok=True)
os.environ['MUJOCO_GL'] = os.environ['PYOPENGL_PLATFORM'] = 'egl'
os.environ['PYTHONUNBUFFERED'] = '1'
status = {'stage': 'setup', 'hardware_validated': False}


def run(stage, command):
    status['stage'] = stage
    (output / 'status.json').write_text(json.dumps(status, indent=2))
    with (output / f'{stage}.log').open('w') as log:
        result = subprocess.run(command, cwd=project, stdout=log, stderr=subprocess.STDOUT)
    status[stage] = {'returncode': result.returncode}
    (output / 'status.json').write_text(json.dumps(status, indent=2))
    if result.returncode:
        raise RuntimeError(f'{stage} failed; inspect its saved log and candidate reports')


try:
    run('install', ['bash', 'src/mjlab/tasks/tracking/config/pupper/unimate/install.sh', '/content/UniMate'])
    available = subprocess.check_output(['nvidia-smi', '--query-gpu=memory.free',
                        '--format=csv,noheader,nounits'], text=True).strip().splitlines()
    if len(available) != 1 or float(available[0]) < 20000:
        raise RuntimeError('Need at least 20 GB free GPU memory before concurrent generation; rerun after other jobs finish')
    status['free_gpu_memory_mib_before_generation'] = float(available[0])
    run('generate', [str(python), '-m', 'mjlab.scripts.pupper_generate', 'generate',
        '--unimate', '/content/UniMate', '--prompt',
        'An object gently sways its body from side to side while standing on all four legs.',
        '--samples', '4', '--seed', '42', '--output', str(output / 'standing-dance-generated')])
    run('candidates', [str(python), '-m', 'mjlab.scripts.pupper_unimate_candidates',
        '--raw', str(output / 'standing-dance-generated/samples.npz'),
        '--output', str(output / 'standing-dance-candidates'), '--videos'])
    status['stage'] = 'requires_candidate_inspection_before_training'
except Exception as error:
    status.update(stage='failed', error=str(error))
    raise
finally:
    (output / 'status.json').write_text(json.dumps(status, indent=2))
