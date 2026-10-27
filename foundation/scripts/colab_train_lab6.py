"""Execute inside the authenticated Colab G4 runtime; results stay in /content.

The orchestrator must retrieve /content/foundation-output before releasing it.
Each profile owns an immutable reference, configuration and source manifest.
"""
import json
import os
from pathlib import Path
import subprocess
import time

repo = Path('/content/pupper-foundation/pupper-mjlab')
output = Path('/content/foundation-output')
output.mkdir(exist_ok=True)
os.environ['MUJOCO_GL'] = 'egl'
os.environ['PYOPENGL_PLATFORM'] = 'egl'
results = []
for profile in ('sit_stay_shake', 'spider_clips', 'spider_walk'):
    log = output / f'train-{profile}.log'
    command = ['uv', 'run', '--no-sync', 'python', '-m',
               'mjlab.scripts.pupper_foundation', 'train', profile,
               '--output', str(output / 'lab6'), '--logger', 'tensorboard']
    started = time.time()
    print(f'START {profile}', flush=True)
    with log.open('w') as stream:
        result = subprocess.run(command, cwd=repo, stdout=stream,
                                stderr=subprocess.STDOUT)
    item = {'profile': profile, 'returncode': result.returncode,
            'elapsed_seconds': time.time() - started, 'log': str(log)}
    results.append(item)
    (output / 'lab6-jobs.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(item), flush=True)
    print('\n'.join(log.read_text().splitlines()[-65:]), flush=True)
    if result.returncode:
        raise RuntimeError(f'{profile} failed; see {log}')
print('LAB6_TRAINING_FINISHED_REQUIRES_EVALUATION', flush=True)
