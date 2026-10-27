#!/usr/bin/env python3
"""Describe a recorded tracking rollout without adding a new pass/fail threshold.

Run with the repository environment. Motion, physics report and recorded trajectory
must belong to the same rollout. All measurements are simulation evidence only.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.signal import find_peaks

from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.tricks import sit_stay_shake, spider


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(motion_path, trajectory_path, report_path, profile):
    with np.load(motion_path, allow_pickle=False) as source:
        motion = {key:source[key].copy() for key in source.files}
    with np.load(trajectory_path, allow_pickle=False) as source:
        trace = {key:source[key].copy() for key in source.files}
    physics=json.loads(report_path.read_text())
    frames=trace['reference_frame'].astype(int)
    fps=float(trace['fps'][0])
    if frames.ndim!=1 or len(frames)!=len(physics['samples']) or not len(frames):
        raise ValueError('Physics report and trajectory must have identical nonempty frame counts')
    if not np.array_equal(frames,[s['reference_frame'] for s in physics['samples']]):
        raise ValueError('Physics report and trajectory frame alignment differs')
    base=list(motion['body_names']).index('base_link')
    for key,expected in [('joint_pos',motion['joint_pos'][frames]),
                         ('root_pos',motion['body_pos_w'][frames,base]),
                         ('root_quat',motion['body_quat_w'][frames,base])]:
        np.testing.assert_allclose(trace['reference_'+key],expected,atol=1e-6)
    module=sit_stay_shake if profile=='sit_stay_shake' else spider
    clips={str(name):(int(start),int(end)) for name,start,end in zip(
        motion['clip_names'],motion['clip_starts'],motion['clip_ends'],strict=True)}
    phases=[];cursor=0
    for name in module.ORDER:
        lo,hi=clips[name];length=hi-lo
        # concat_clips retains the previous clip's shared boundary frame.
        # Compare the newly appended frames exactly; do not relax phase matching.
        np.testing.assert_array_equal(motion['joint_pos'][cursor+1:cursor+length],motion['joint_pos'][lo+1:hi])
        phases.append((name,cursor,cursor+length-1));cursor+=length-1
    if cursor+1!=len(motion['joint_pos']):
        raise ValueError('Named training order does not reproduce this motion; do not infer phase labels')
    k=md.kinematics();feet={}
    for tag in ['reference','actual']:
        positions=[]
        for p,q,j in zip(trace[tag+'_root_pos'],trace[tag+'_root_quat'],trace[tag+'_joint_pos'],strict=True):
            calculated=k.feet(p,q,j)
            positions.append([calculated[leg] for leg in md.LEGS])
        feet[tag]=np.asarray(positions)
    result={'profile':profile,'scope':'Behavior-specific descriptive metrics of one recorded simulation; no new pass threshold or hardware approval.',
            'approved_for_hardware':False,'inputs':{str(p):digest(p) for p in [motion_path,trajectory_path,report_path]},
            'analysis_source_sha256':digest(Path(__file__)),
            'fps':fps,'collision_contract':physics.get('collision_contract'),
            'course_tracking_verdict':physics['verdict'],'reference_resets':physics['reference_resets'],
            'tolerances':'None added. Paw peak detection uses half the programmed3cm shake amplitude as prominence and0.4s separation, below the reference0.6s peak interval. Counts and amplitudes are reported, not graded.',
            'observability':'The deployed69D actor has no measured root translation or linear velocity; world-position drift is distinct from pose and balance errors.',
            'phases':[]}
    samples=physics['samples']
    qerr=trace['actual_joint_pos']-trace['reference_joint_pos']
    for occurrence,(name,start,end) in enumerate(phases):
        selected=np.where((frames>=start)&(frames<=end))[0]
        end_window=selected[frames[selected]>=max(start,end-round(.5*fps))]
        ss=[samples[i] for i in selected]
        row={'occurrence':occurrence,'clip':name,'reference_interval_s':[start/fps,end/fps],
             'joint_tracking_rms_rad':float(np.sqrt(np.mean(qerr[selected]**2))),
             'joint_tracking_max_abs_rad':float(np.abs(qerr[selected]).max()),
             'peak_joint_speed_rad_s':max(s['joint_speed_max_rad_s'] for s in ss),
             'peak_motor_torque_nm':max(s['motor_torque_max_nm'] for s in ss),
             'peak_absolute_body_tilt_deg':max(s['body_tilt_from_upright_deg'] for s in ss),
             'peak_orientation_tracking_error_deg':max(s['orientation_error_deg'] for s in ss),
             'peak_position_tracking_error_m':max(s['position_error_m'] for s in ss),
             'peak_sampled_hull_vertex_penetration_m':max(0.,-min(s['lowest_sampled_hull_vertex_m'] for s in ss)),
             'minimum_foot_contact_count':min(s['foot_contact_count'] for s in ss),
             'final_half_second':{'joint_mean_error_rad':qerr[end_window].mean(axis=0).tolist(),
                                  'joint_max_abs_error_rad':float(np.abs(qerr[end_window]).max()),
                                  'root_mean_error_xyz_m':(trace['actual_root_pos'][end_window]-trace['reference_root_pos'][end_window]).mean(axis=0).tolist(),
                                  'actual_root_mean_height_m':float(trace['actual_root_pos'][end_window,2].mean()),
                                  'reference_root_mean_height_m':float(trace['reference_root_pos'][end_window,2].mean())}}
        if name.startswith('shake_'):
            intended='front_l' if name.endswith('left') else 'front_r'
            other='front_r' if intended=='front_l' else 'front_l'
            row['paw_offer']={'intended_leg':intended,'programmed_peak_center_height_m':sit_stay_shake.PAW_HEIGHT,
                              'programmed_vertical_shake_amplitude_m':sit_stay_shake.SHAKE}
            for tag in ['reference','actual']:
                h=feet[tag][selected,md.LEGS.index(intended),2]
                peaks,_=find_peaks(h,prominence=sit_stay_shake.SHAKE/2,distance=round(.4*fps))
                row['paw_offer'][tag]={'intended_paw_peak_center_height_m':float(h.max()),
                    'opposite_front_paw_peak_center_height_m':float(feet[tag][selected,md.LEGS.index(other),2].max()),
                    'prominent_peak_count':len(peaks),'prominent_peak_times_in_clip_s':((frames[selected[peaks]]-start)/fps).tolist()}
        result['phases'].append(row)
    last=frames>=frames[-1]-round(.5*fps)
    end=int(frames[-1]); window=slice(max(0,end-round(.5*fps)),end+1)
    terminal_start=end
    while terminal_start>0 and all(np.array_equal(array[terminal_start-1],array[end]) for array in
                                  (motion['joint_pos'],motion['body_pos_w'],motion['body_quat_w'])):
        terminal_start-=1
    result['return_to_stand']={'reference_exact_stationary_terminal_duration_s':(end-terminal_start)/fps,
        'reference_final_joint_speed_max_rad_s':float(np.abs(motion['joint_vel'][end]).max()),
        'reference_last_half_second_joint_speed_max_rad_s':float(np.abs(motion['joint_vel'][window]).max()),
        'reference_last_half_second_max_joint_motion_rad':float(np.ptp(motion['joint_pos'][window],axis=0).max()),
        'reference_final_joint_pose_rad':trace['reference_joint_pos'][-1].tolist(),
        'actual_final_joint_pose_rad':trace['actual_joint_pos'][-1].tolist(),
        'final_joint_error_rad':qerr[-1].tolist(),'last_half_second_joint_max_abs_error_rad':float(np.abs(qerr[last]).max()),
        'final_reported_joint_speed_max_rad_s':samples[-1]['joint_speed_max_rad_s'],
        'last_half_second_peak_orientation_error_deg':max(samples[i]['orientation_error_deg'] for i in np.where(last)[0]),
        'last_half_second_minimum_foot_contact_count':min(samples[i]['foot_contact_count'] for i in np.where(last)[0])}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion',type=Path,required=True)
    parser.add_argument('--trajectory',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--profile',choices=['sit_stay_shake','spider_clips'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze(args.motion,args.trajectory,args.report,args.profile)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(args.output)

if __name__=='__main__':main()
