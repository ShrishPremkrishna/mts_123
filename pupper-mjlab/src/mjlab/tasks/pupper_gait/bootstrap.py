"""Optional Lab 2 capture hygiene and reference construction (NumPy-only core).

Traces contain physical-time joint trajectories, positive running reference phase,
reset/episode identity, commands and validity. A named gait is never proof of speed.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

JOINT_ORDER=[f"leg_{leg}_{joint}" for leg in ("front_r","front_l","back_r","back_l") for joint in (1,2,3)]


def file_hash(path):
    h=hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda:f.read(1024*1024),b""): h.update(chunk)
    return h.hexdigest()


def validate_reference(table, *, limits=None, cycle_s=.75, periodic=True, max_velocity=None):
    table=np.asarray(table,dtype=np.float64)
    if table.ndim!=2 or table.shape[1]!=12 or len(table)<8 or not np.isfinite(table).all():
        raise ValueError("Reference must be finite (N>=8,12) joint angles in robot order")
    if not np.isfinite(cycle_s) or cycle_s<=0: raise ValueError("Positive finite duration required")
    if limits is not None:
        limits=np.asarray(limits)
        if limits.shape!=(12,2) or np.any(table<limits[:,0]-1e-6) or np.any(table>limits[:,1]+1e-6):
            raise ValueError("Reference exceeds robot joint limits; repair it rather than silently clipping")
    delta=np.diff(np.vstack([table,table[:1]]) if periodic else table,axis=0)
    dt=cycle_s/(len(table) if periodic else len(table)-1)
    peak=float(np.max(np.abs(delta))/dt)
    if max_velocity is not None and peak>max_velocity: raise ValueError(f"Reference velocity {peak:.3f} exceeds {max_velocity} rad/s")
    return dict(samples=len(table),duration_s=cycle_s,periodic=periodic,peak_joint_velocity_rad_s=peak,
                wrap_step_rad=float(np.max(np.abs(table[-1]-table[0]))),
                joint_min=table.min(axis=0).tolist(),joint_max=table.max(axis=0).tolist())


def periodic_smooth(table,harmonics=6):
    if harmonics<1: raise ValueError("harmonics must be positive")
    spectrum=np.fft.rfft(table,axis=0)
    spectrum[harmonics+1:]=0
    return np.fft.irfft(spectrum,n=len(table),axis=0)


def mirror_average(table, phase_shift=.5):
    """Front/back left-right pairing. All three motor signs mirror in robot order.

    Preserve a requested inter-leg phase offset (half cycle for diagonal trot).
    Use only after checking that the desired gait really has this symmetry.
    """
    table=np.asarray(table,dtype=np.float64).copy()
    shift=int(round(phase_shift*len(table)))%len(table)
    for r,l in ((0,3),(6,9)):
        a=table[:,r:r+3]; b=table[:,l:l+3]
        mean=.5*(a-np.roll(b,-shift,axis=0))
        table[:,r:r+3]=mean
        table[:,l:l+3]=-np.roll(mean,shift,axis=0)
    return table


def fold_cycles(trace, command, *, samples=100, min_cycles=3, tolerance=.025, harmonics=6, symmetry=False):
    """Extract complete uninterrupted cycles; never fold resets/transitions together."""
    required=("joint_pos","phase","episode","command","valid")
    for key in required:
        if key not in trace: raise ValueError(f"Missing trace field {key}")
    q=np.asarray(trace["joint_pos"]); phase=np.asarray(trace["phase"])
    command=np.asarray(command,dtype=float)
    if q.ndim!=3 or q.shape[2]!=12 or phase.shape!=q.shape[:2]: raise ValueError("Trace shapes must be (time,env,12)/(time,env)")
    valid=np.asarray(trace["valid"],dtype=bool)&(np.abs(trace["command"]-command).max(axis=-1)<=tolerance)
    cycles=[]; origins=[]
    for env in range(q.shape[1]):
        wraps=np.flatnonzero(np.diff(phase[:,env])<-.5)+1
        for a,b in zip(wraps[:-1],wraps[1:]):
            # Include bracketing samples to interpolate exact phase boundaries.
            lo,hi=a-1,b+1
            if lo<0 or hi>len(q) or b-a<4 or not valid[lo:hi,env].all(): continue
            episodes=trace["episode"][lo:hi,env]
            if not np.all(episodes==episodes[0]): continue
            p=np.unwrap(phase[lo:hi,env]*2*np.pi)/(2*np.pi)
            start=np.ceil(p[0]+1e-8)
            if p[-1]<start+1 or np.any(np.diff(p)<=0): continue
            grid=start+np.arange(samples)/samples
            cycle=np.stack([np.interp(grid,p,q[lo:hi,env,j]) for j in range(12)],axis=1)
            if not np.isfinite(cycle).all(): continue
            cycles.append(cycle); origins.append(dict(env=env,start_step=int(a),end_step=int(b)))
    if len(cycles)<min_cycles:
        raise ValueError(f"Only {len(cycles)} clean complete cycles; need {min_cycles}. Increase rollout or repair policy.")
    stack=np.stack(cycles)
    # Robust representative: do not let one remaining atypical cycle dominate.
    table=periodic_smooth(np.median(stack,axis=0),harmonics)
    if symmetry: table=mirror_average(table)
    report=dict(cycles=len(cycles),cycle_rms_dispersion_rad=float(np.sqrt(np.mean((stack-table)**2))),
                origins=origins,command=command.tolist(),harmonics=harmonics,symmetry=symmetry)
    return table,report


def summarize_trace(trace,dt,warmup_s=1.):
    """Per commanded segment; failures count even during warmup and after resets."""
    result=[]
    for segment in np.unique(trace["segment"]):
        time_mask=trace["segment"]==segment
        mask=time_mask[:,None]&trace["valid"].astype(bool)
        command=trace["command"][time_mask][0,0]
        n=int(mask.sum())
        failure=int(np.asarray(trace["terminated"])[time_mask].sum())
        timeout=int(np.asarray(trace["timeout"])[time_mask].sum())
        record=dict(segment=int(segment),command=command.tolist(),valid_samples=n,failures=failure,timeouts=timeout,
                    simulated_robot_seconds=float(time_mask.sum()*trace["joint_pos"].shape[1]*dt))
        if n:
            error=trace["velocity"][mask]-trace["command"][mask]
            # Explicit names distinguish mean norm from RMS and component mean.
            record.update(mean_xy_error_m_s=float(np.linalg.norm(error[:,:2],axis=1).mean()),
                          rms_xy_error_m_s=float(np.sqrt((error[:,:2]**2).sum(axis=1).mean())),
                          mean_abs_yaw_error_rad_s=float(np.abs(error[:,2]).mean()),
                          mean_achieved_vx_m_s=float(trace["velocity"][mask,0].mean()),
                          mean_torque_squared_nm2=float((trace["torque"][mask]**2).mean()),
                          peak_abs_torque_nm=float(np.abs(trace["torque"][mask]).max()))
            contacts=trace["foot_contact"][mask].astype(bool)
            speeds=trace["foot_speed_xy"][mask]
            record["mean_contact_slip_m_s"]=float(speeds[contacts].mean()) if contacts.any() else None
            # Heading drift is reported only for requested zero yaw, and only within
            # contiguous episodes; never subtract headings across automatic resets.
            drift=[]
            if abs(command[2])<1e-6:
                for e in range(trace["joint_pos"].shape[1]):
                    for episode in np.unique(trace["episode"][mask[:,e],e]):
                        ids=np.flatnonzero(mask[:,e]&(trace["episode"][:,e]==episode))
                        if len(ids)>1:
                            angle=np.unwrap(trace["heading"][ids,e]); drift.append(abs(float(angle[-1]-angle[0])))
            record["max_abs_heading_drift_rad"]=max(drift) if drift else None
            if "height" in trace:
                record["peak_base_height_world_m"]=float(trace["height"][mask].max())
                record["minimum_base_height_world_m"]=float(trace["height"][mask].min())
            if "vertical_velocity" in trace:
                record["peak_downward_speed_m_s"]=float(np.maximum(-trace["vertical_velocity"][mask],0).max())
            if "foot_force_norm" in trace:
                forces=trace["foot_force_norm"][mask]
                record["peak_sampled_foot_contact_force_n"]=float(np.nanmax(forces)) if np.isfinite(forces).any() else None
            if "target_clipped" in trace:
                clipping=trace["target_clipped"][mask].astype(bool)
                record["target_clipped_fraction"]=float(clipping.mean())
                record["any_target_clipped_sample_fraction"]=float(clipping.any(axis=1).mean())
                record["target_clipped_fraction_by_joint"]=clipping.mean(axis=0).tolist()
                record["maximum_target_range_excess_rad"]=float(trace["target_range_excess_rad"][mask].max())
            record["airborne_sample_fraction"]=float((~contacts.any(axis=1)).mean())
        result.append(record)
    return dict(segments=result,dt=dt,warmup_s=warmup_s,
                scope="simulation only; measured velocities are simulator observations, not hardware speed")


def save_references(out, forward, backward, report, *, limits=None, frequency_mult=1.):
    if not np.isfinite(frequency_mult) or frequency_mult<=0: raise ValueError("Invalid frequency multiplier")
    out=Path(out); out.mkdir(parents=True,exist_ok=True)
    duration=.75/frequency_mult
    checks={name:validate_reference(table,limits=limits,cycle_s=duration) for name,table in (("reach_fwd",forward),("reach_back",backward))}
    filename=out/"captured_reach_gaits.npz"
    np.savez_compressed(filename,reach_fwd=forward,reach_back=backward,joint_names=np.asarray(JOINT_ORDER),
                        metadata_json=np.asarray(json.dumps(report)))
    manifest=dict(version=1,frequency_mult=frequency_mult,reference_sha256=file_hash(filename),joint_order=JOINT_ORDER,
                  checks=checks,provenance=report,status="candidate reference; not proof of successful trained or hardware behavior")
    (out/"reference_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    return manifest


def select_jump(trace, pre_s=.30, post_s=.30, min_flight_s=.08, *, limits=None):
    """Select a genuine flight with loading and landing, never a reset or a fall."""
    dt=float(trace["dt"])
    if dt<=0: raise ValueError("Invalid trace timestep")
    contacts=np.asarray(trace["foot_contact"],dtype=bool).sum(axis=-1)
    candidates=[]
    for env in range(contacts.shape[1]):
        airborne=contacts[:,env]==0
        edges=np.diff(np.r_[False,airborne,False].astype(int))
        starts,ends=np.flatnonzero(edges==1),np.flatnonzero(edges==-1)
        for flight_start,flight_end in zip(starts,ends):
            flight=(flight_end-flight_start)*dt
            if flight<min_flight_s: continue
            before=int(np.ceil(pre_s/dt)); after=int(np.ceil(post_s/dt))
            a=flight_start-before; b=flight_end+after
            # Match the current fixed touchdown scheduler: retain a slightly
            # longer settle rather than speed-stretching a captured motion.
            b=max(b,a+int(np.ceil(.80/dt))+1)
            if a<0 or b>len(contacts) or (b-a-1)*dt>=1.44: continue
            if not np.all(trace["valid"][a:b,env]) or not np.all(trace["episode"][a:b,env]==trace["episode"][a,env]): continue
            if np.any(trace["terminated"][a:b,env]) or contacts[a:flight_start,env].max()<2 or contacts[flight_end:b,env].max()<2: continue
            height=float(np.max(trace["height"][flight_start:flight_end,env]))
            rise=height-float(np.median(trace["height"][a:flight_start,env]))
            if rise<.02: continue
            if limits is not None:
                try:
                    validate_reference(trace["joint_pos"][a:b,env], limits=limits,
                                       cycle_s=(b-a-1)*dt, periodic=False)
                except ValueError:
                    continue  # Choose another observed window; never clip the best one.
            candidates.append(dict(env=env,start=a,end=b,flight_s=flight,touchdown_s=(flight_end-a)*dt,peak_height_m=height,rise_m=rise))
    if not candidates: raise ValueError("No clean jump with loading, flight and landing found; do not fabricate a capture")
    return max(candidates,key=lambda c:c["rise_m"])


def compare_evaluations(baseline, candidate, min_speed=.5, min_improvement=.05, max_cost_regression=.2):
    """A conservative per-seed promotion rule; multiple held-out seeds remain required."""
    def finite_number(value):
        return isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, (bool, np.bool_)) and np.isfinite(value)
    if (not all(finite_number(v) for v in (min_speed,min_improvement,max_cost_regression))
            or min_speed<=0 or min_improvement<0 or max_cost_regression<0):
        raise ValueError("Invalid promotion thresholds")
    keys=("task", "seed", "num_envs", "seconds", "warmup", "commands", "randomized", "reset_each_command", "no_heading_hold")
    for key in keys:
        if baseline["provenance"].get(key) != candidate["provenance"].get(key):
            raise ValueError(f"Cannot compare different evaluation protocols: {key}")
    if bool(baseline["provenance"].get("deployment_target_clamp",False)) != bool(candidate["provenance"].get("deployment_target_clamp",False)):
        raise ValueError("Cannot compare different evaluation protocols: deployment_target_clamp")
    if bool(baseline["provenance"].get("compose_com_randomization",False)) != bool(candidate["provenance"].get("compose_com_randomization",False)):
        raise ValueError("Cannot compare different evaluation protocols: compose_com_randomization")
    if bool(baseline["provenance"].get("locomotion_torso_contact",False)) != bool(candidate["provenance"].get("locomotion_torso_contact",False)):
        raise ValueError("Cannot compare different evaluation protocols: locomotion_torso_contact")
    def driver_identity(provenance):
        contract=provenance.get("driver_target_contract")
        if contract is None:return None
        return ({k:v for k,v in contract.items() if k not in ("source","scope")},
                {k:v for k,v in contract.get("source",{}).items() if k!="path"})
    if driver_identity(baseline["provenance"]) != driver_identity(candidate["provenance"]):
        raise ValueError("Cannot compare different evaluation protocols: driver_target_contract")
    if baseline['provenance'].get('position_target_slew_contract') != candidate['provenance'].get('position_target_slew_contract'):
        raise ValueError('Cannot compare different evaluation protocols: position_target_slew_contract')
    if any(r["provenance"].get("zero_policy") for r in (baseline,candidate)):
        raise ValueError("An infrastructure smoke test cannot be promoted as a trained policy")
    if len(baseline["segments"]) != len(candidate["segments"]):
        raise ValueError("Different command sequences")
    reasons=[]; gains=[]
    for base,new in zip(baseline["segments"],candidate["segments"]):
        for segment in (base,new):
            command=segment.get("command")
            if not isinstance(command,(list,tuple)) or len(command)!=3 or not all(finite_number(v) for v in command):
                raise ValueError("Promotion requires finite three-axis commands")
            for key in ("failures","valid_samples"):
                value=segment.get(key)
                if not finite_number(value) or value<0 or int(value)!=value:
                    raise ValueError(f"Promotion requires nonnegative integer {key}")
        if base["command"] != new["command"]: raise ValueError("Different command sequence")
        label=str(base["command"])
        if new["failures"] > base["failures"]: reasons.append(f"more failures at {label}")
        if min(base["valid_samples"],new["valid_samples"]) < 100:
            reasons.append(f"too few steady-state samples at {label}"); continue
        for segment in (base,new):
            for key in ("mean_xy_error_m_s","mean_torque_squared_nm2","mean_contact_slip_m_s","mean_abs_yaw_error_rad_s"):
                value=segment.get(key)
                if not finite_number(value) or value<0:
                    raise ValueError(f"Promotion requires finite nonnegative {key}")
        error=base["mean_xy_error_m_s"]; error_new=new["mean_xy_error_m_s"]
        if abs(base["command"][0]) >= min_speed:
            gains.append((error-error_new)/max(error,1e-6))
        elif error_new > max(.02,error)*(1+max_cost_regression):
            reasons.append(f"low-speed tracking regressed at {label}")
        for metric in ("mean_torque_squared_nm2","mean_contact_slip_m_s","mean_abs_yaw_error_rad_s"):
            old,new_value=base[metric],new[metric]
            if new_value>max(old,.01)*(1+max_cost_regression):
                reasons.append(f"{metric} regressed at {label}")
    if not gains: reasons.append("no high-speed segments")
    if gains and min(gains)<min_improvement: reasons.append("not every high-speed direction/segment improved enough")
    return dict(promotable_for_this_seed=not reasons,reasons=reasons,high_speed_relative_improvements=gains,
                thresholds=dict(min_speed=min_speed,min_improvement=min_improvement,max_cost_regression=max_cost_regression),
                next_step="repeat at three held-out seeds and randomized terrain before hardware" if not reasons else "retain baseline; inspect failure and revise or stop bootstrapping")


def robot_limits():
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import get_spec, JOINT_NAMES
    model=get_spec().compile()
    return np.array([model.joint(name).range for name in JOINT_NAMES])


def main():
    p=argparse.ArgumentParser(description=__doc__); sub=p.add_subparsers(dest="command",required=True)
    fold=sub.add_parser("fold"); fold.add_argument("trace",type=Path); fold.add_argument("out",type=Path)
    fold.add_argument("--speed",type=float,default=1.5); fold.add_argument("--samples",type=int,default=100)
    fold.add_argument("--min-cycles",type=int,default=3); fold.add_argument("--harmonics",type=int,default=6)
    fold.add_argument("--symmetry",action="store_true"); fold.add_argument("--frequency-mult",type=float,default=1.)
    design=sub.add_parser("design"); design.add_argument("out",type=Path)
    design.add_argument("--half-stride",type=float,default=.08); design.add_argument("--stance-z",type=float,default=-.12)
    design.add_argument("--swing-lift",type=float,default=.05); design.add_argument("--swing-keyframes",type=int,default=5)
    design.add_argument("--frequency-mult",type=float,default=2.)
    jump=sub.add_parser("jump"); jump.add_argument("trace",type=Path); jump.add_argument("out",type=Path)
    jump.add_argument("--env",type=int,default=0); jump.add_argument("--start",type=int); jump.add_argument("--end",type=int); jump.add_argument("--auto",action="store_true")
    compare=sub.add_parser("compare"); compare.add_argument("baseline",type=Path); compare.add_argument("candidate",type=Path); compare.add_argument("out",type=Path)
    args=p.parse_args()
    if args.command=="fold":
        with np.load(args.trace,allow_pickle=False) as f: trace={k:f[k] for k in f.files}
        forward,fr=fold_cycles(trace,[args.speed,0,0],samples=args.samples,min_cycles=args.min_cycles,harmonics=args.harmonics,symmetry=args.symmetry)
        backward,br=fold_cycles(trace,[-args.speed,0,0],samples=args.samples,min_cycles=args.min_cycles,harmonics=args.harmonics,symmetry=args.symmetry)
        meta=dict(method="median phase-aligned clean cycles, periodic Fourier smoothing",trace_sha256=file_hash(args.trace),forward=fr,backward=br)
        print(json.dumps(save_references(args.out,forward,backward,meta,limits=robot_limits(),frequency_mult=args.frequency_mult),indent=2))
    elif args.command=="design":
        from mjlab.tasks.pupper_gait.mdp import gait_reference as ref
        ref._GAIT_TOUCHDOWN["foundation_candidate"]=(0.,.5,.5,0.)
        ref._GAIT_STRIDE["foundation_candidate"]=args.half_stride
        ref._GAIT_STANCE_Z["foundation_candidate"]=args.stance_z
        ref._GAIT_SWING_LIFT["foundation_candidate"]=args.swing_lift
        ref._GAIT_SWING_KF["foundation_candidate"]=args.swing_keyframes
        table=ref.build_joint_reference_table(100,"foundation_candidate")
        backward=np.roll(table[::-1],1,axis=0)
        meta=dict(method="authored extended-reach trot; candidate, not measured improvement",parameters={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()})
        print(json.dumps(save_references(args.out,table,backward,meta,limits=robot_limits(),frequency_mult=args.frequency_mult),indent=2))
    elif args.command=="compare":
        result=compare_evaluations(json.loads(args.baseline.read_text()),json.loads(args.candidate.read_text()))
        args.out.write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps(result,indent=2))
    else:
        with np.load(args.trace,allow_pickle=False) as f: trace={k:f[k] for k in f.files}
        selection=select_jump(trace,limits=robot_limits()) if args.auto else None
        a,b,e=(selection["start"],selection["end"],selection["env"]) if selection else (args.start,args.end,args.env)
        if a is None or b is None: raise ValueError("Provide --auto or both --start and --end")
        if a<0 or b<=a+7 or b>len(trace["joint_pos"]) or e<0 or e>=trace["joint_pos"].shape[1]: raise ValueError("Invalid jump range")
        if not np.all(trace["episode"][a:b,e]==trace["episode"][a,e]) or np.any(trace["terminated"][a:b,e]): raise ValueError("Jump capture crosses a reset/failure")
        table=trace["joint_pos"][a:b,e]
        duration=float((b-a-1)*trace["dt"])
        report=validate_reference(table,limits=robot_limits(),cycle_s=duration,periodic=False)
        # The existing jump scheduler has a 1.5 s maximum reward window.
        if duration+.06>=1.5 or duration<=.2: raise ValueError("Course jump schedule requires 0.2 < captured duration < 1.44 seconds")
        if selection:
            touchdown=selection["touchdown_s"]
        else:
            airborne=np.asarray(trace["foot_contact"])[a:b,e].sum(axis=-1)==0
            landing=np.flatnonzero(airborne[:-1]&~airborne[1:])+1
            if not len(landing): raise ValueError("Selected interval has no measured flight-to-contact landing")
            touchdown=float(landing[-1]*trace["dt"])
        if not .15<touchdown<=duration: raise ValueError("Invalid measured touchdown within capture")
        args.out.mkdir(parents=True,exist_ok=True)
        report.update(selection=selection,touchdown_s=touchdown,trace_sha256=file_hash(args.trace),start_step=a,end_step=b,env=e,status="simulation candidate; first hardware run requires TA review")
        np.savez_compressed(args.out/"captured_jump_gait.npz",jump=table,duration_s=duration,touchdown_s=touchdown,joint_names=np.asarray(JOINT_ORDER),metadata_json=np.asarray(json.dumps(report)))
        (args.out/"jump_report.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__": main()
