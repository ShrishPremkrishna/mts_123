import os
import pathlib
import subprocess
import sys

repo=pathlib.Path('/content/pupper-foundation/pupper-mjlab')
output=pathlib.Path('/content/foundation-output')
output.mkdir(exist_ok=True)
os.environ['MUJOCO_GL']='egl'
os.environ['PYOPENGL_PLATFORM']='egl'
for profile in ('sit_stay_shake', 'spider_clips', 'spider_walk'):
    log=output/f'smoke-{profile}.log'
    command=['uv','run','--no-sync','python','-m','mjlab.scripts.pupper_foundation','train',profile,
             '--output',str(output/'smoke'),'--num-envs','64','--iterations','2','--logger','tensorboard']
    with log.open('w') as stream:
        result=subprocess.run(command,cwd=repo,stdout=stream,stderr=subprocess.STDOUT)
    print(profile, 'returncode',result.returncode,flush=True)
    print('\n'.join(log.read_text().splitlines()[-65:]),flush=True)
    if result.returncode:
        raise RuntimeError(f'{profile} smoke failed; see {log}')
print('ALL_GPU_SMOKES_PASSED',flush=True)
