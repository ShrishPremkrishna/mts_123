#!/usr/bin/env python3
"""Measure stopping from continuous traces; thresholds define a metric, not approval."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def analyze(trace, speed=.02, yaw=.05, hold=.5):
    if not np.isfinite([speed,yaw,hold]).all() or min(speed,yaw,hold)<=0:
        raise ValueError('stopping measurement thresholds must be finite and positive')
    with np.load(trace,allow_pickle=False) as data:
        arrays={k:data[k].copy() for k in ('segment','command','velocity','root_position','terminated','timeout','episode','torque','dt')}
    segment=arrays['segment'];command=arrays['command'];velocity=arrays['velocity'];dt=float(arrays['dt'])
    if dt<=0 or not np.isfinite(dt):raise ValueError('positive finite trace dt required')
    # Locomotion traces record state BEFORE each action: the first zero-command
    # row is t=0. N samples span (N-1)*dt, so include both ends of the hold window.
    n_hold=int(np.ceil(hold/dt))+1;stops=[]
    for s in np.unique(segment):
        ids=np.flatnonzero(segment==s)
        if ids[0]==0 or not np.allclose(command[ids],0,atol=1e-9):continue
        prior=command[ids[0]-1]
        if np.allclose(prior,0,atol=1e-9):continue
        for world in range(velocity.shape[1]):
            if np.allclose(prior[world],0,atol=1e-9):continue
            reset=bool(np.any(arrays['terminated'][ids,world]) or np.any(arrays['timeout'][ids,world]) or
                       np.any(arrays['episode'][ids,world]!=arrays['episode'][ids[0]-1,world]))
            planar=np.linalg.norm(velocity[ids,world,:2],axis=1)
            slow=(planar<=speed)&(np.abs(velocity[ids,world,2])<=yaw)
            starts=np.flatnonzero(np.convolve(slow.astype(int),np.ones(n_hold,dtype=int),'valid')==n_hold) if len(ids)>=n_hold else []
            first=None if reset or not len(starts) else int(starts[0])
            pos=arrays['root_position'][ids,world,:2];origin=pos[0]
            until=min(len(ids),int(np.floor(1/dt))+1)
            stops.append(dict(segment=int(s),world=world,previous_command=prior[world].tolist(),reset_or_timeout=reset,
                settled=first is not None,time_to_sustained_stop_s=None if first is None else first*dt,
                displacement_first_second_m=float(np.linalg.norm(pos[:until]-origin,axis=1).max()),
                displacement_full_segment_m=float(np.linalg.norm(pos-origin,axis=1).max()),
                final_planar_speed_m_s=float(planar[-1]),final_abs_yaw_rate_rad_s=float(abs(velocity[ids[-1],world,2])),
                mean_torque_squared_nm2=float(np.square(arrays['torque'][ids,world]).mean()),
                sampled_torque_at_3nm_limit_fraction=float(np.mean(np.abs(arrays['torque'][ids,world])>=2.99))))
    times=[r['time_to_sustained_stop_s'] for r in stops if r['settled']]
    return dict(protocol='continuous_zero_command_stop_measurement_v1',trace_sha256=hashlib.sha256(Path(trace).read_bytes()).hexdigest(),
                thresholds=dict(planar_speed_m_s=speed,abs_yaw_rate_rad_s=yaw,continuous_hold_s=hold),
                interpretation='Engineering measurement definition, not a course requirement or hardware pass criterion. Traces record state before each action, so the first zero-command row is time zero. A hold spans both endpoint samples; resets are censored failures, never successful stops.',
                total_stops=len(stops),settled_stops=len(times),reset_or_timeout_stops=sum(r['reset_or_timeout'] for r in stops),
                settled_only_median_time_s=float(np.median(times)) if times else None,
                settled_only_maximum_time_s=max(times) if times else None,stops=stops,hardware_validated=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--trace',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.write_text(json.dumps(analyze(args.trace),indent=2)+'\n')
