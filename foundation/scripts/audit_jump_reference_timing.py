"""Audit one-shot capture timing through the actual loader and training/export runtime.

This read-only CPU diagnostic does not validate physical flight or approve hardware.
Pass the exact reference directory and default-pose policy that define the experiment.
Run in a fresh process because the class task resolves references at import time.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse,hashlib,json,os,subprocess
p=argparse.ArgumentParser();p.add_argument('references',type=Path);p.add_argument('--default-policy',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
if a.out.exists():raise FileExistsError(a.out)
if not a.references.is_dir() or not a.default_policy.is_file():raise FileNotFoundError('Exact references and default-pose policy are required')
os.environ['PUPPER_REFERENCE_DIR']=str(a.references.resolve())
import numpy as np
import torch
from mjlab.tasks.pupper_gait.mdp import gait_reference,gait,mixed_jump
from mjlab.tasks.pupper_gait import mystery_env_cfg as c
from mjlab.tasks.pupper_gait.bootstrap import robot_limits
from mjlab.tasks.pupper_gait.export import gait_reference_metadata,jump_slot_metadata,mixed_jump_reference_offset_numpy
repo=Path(__file__).resolve().parents[2]
policy=json.loads(a.default_policy.read_text())
with np.load(a.references/'captured_jump_gait.npz',allow_pickle=False) as z:
 raw=z['jump'];duration=float(z['duration_s']);touchdown=float(z['touchdown_s'])
loaded=gait_reference.build_joint_reference_table(100,'jump')
expected=np.stack([np.interp(np.linspace(0,1,100),np.linspace(0,1,len(raw)),raw[:,j]) for j in range(12)],axis=1)
assert np.max(np.abs(loaded-expected))<1e-7,'Loader changed values beyond nonperiodic resampling'
ticks=torch.arange(0,121);n=len(ticks);default=torch.tensor(policy['default_joint_pos'],dtype=torch.float32)
params=dict(command_name='twist',frequency=c.GAIT_FREQUENCY,blend_speed=c.GAIT_BLEND_SPEED,n_samples=c.GAIT_N_SAMPLES,gallop_speed=c.TROTGALLOP_ONSET,jump_duration_s=c.JUMP_PLAYBACK_S,slot_s=c.JUMP_ACTIVE_S,cross_fade_s=c.JUMP_CROSS_FADE_S)
commands=torch.zeros(n,3);commands[:,0]=.3
env=SimpleNamespace(device=torch.device('cpu'),num_envs=n,step_dt=.01,episode_length_buf=ticks,
 scene={'robot':SimpleNamespace(data=SimpleNamespace(default_joint_pos=default.expand(n,12),joint_pos_limits=torch.tensor(robot_limits(),dtype=torch.float32)[None]))},
 command_manager=SimpleNamespace(get_command=lambda _:commands),observation_manager=SimpleNamespace(get_term_cfg=lambda *_:SimpleNamespace(params=params)))
starts=torch.full((n,3),float('inf'));starts[:,0]=0;setattr(env,'_pupper_mixed_jump_slots',starts)
g=gait_reference_metadata(env);j=jump_slot_metadata(env)
actual=mixed_jump.mixed_jump_reference_offset(env,**params).numpy();t=(ticks.float()*.01).numpy();zero=np.zeros(n)
exported=mixed_jump_reference_offset_numpy(g,j,np.asarray(policy['default_joint_pos']),t,commands[:,0].numpy(),zero,zero,zero)
error=float(np.max(np.abs(actual-exported)));assert error<2e-6
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
r={'reference_sha256':sha(a.references/'captured_jump_gait.npz'),'script_sha256':sha(Path(__file__)),'raw_samples':len(raw),'raw_duration_s':duration,'input_touchdown_s':touchdown,'loader_samples':len(loaded),'loader_resample':'Linear, nonperiodic, endpoint-preserving interpolation from raw evenly spaced samples to 100 rows; model clamp has zero effect for this capture.',
 'playback_s':j['playback_s'],'active_s':j['active_s'],'cross_fade_s':j['cross_fade_s'],'busy_s':j['busy_s'],'reference_exit_fade_start_s':j['active_s']-j['cross_fade_s'],'reference_exit_complete_before_input_touchdown_s':touchdown-j['active_s'],'training_vs_export_max_error_rad':error,'parity_samples':n,'parity_command_m_s':[.3,0,0],
 'effective_jump_reference_source_time_limit_s':j['active_s']*len(loaded)/(len(loaded)-1),'interpretation':'Nominal playback retains input duration, but the course active reference returns to locomotion 40 ms before the input touchdown. Landing/recovery samples after that boundary are not commanded as the active jump tail. This timing audit does not itself establish that the source is a valid observed capture. N-based runtime lookup differs from N-1 endpoint resampling; this is not exact-time playback. The trained policy must be evaluated. No clipping, smoothing, or speed change was applied to the archived capture.',
 'hardware_approved':False, 'physical_flight_validated_by_this_audit':False,
 'inputs':{'reference_directory':str(a.references.resolve()),'default_policy':str(a.default_policy.resolve()),'default_policy_sha256':sha(a.default_policy)},
 'runtime_source_sha256':{str(Path(m.__file__).resolve().relative_to(repo)):sha(Path(m.__file__)) for m in (gait_reference,gait,mixed_jump,c)},
 'export_source_sha256':sha(Path(__import__('mjlab.tasks.pupper_gait.export',fromlist=['x']).__file__))}
a.out.parent.mkdir(parents=True,exist_ok=True)
with a.out.open('x') as f:f.write(json.dumps(r,indent=2)+'\n')
print(json.dumps(r,indent=2))
