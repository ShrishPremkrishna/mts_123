#!/usr/bin/env python3
"""Audit a saved flat-ground capture window without changing its motion.

Reconstruct source-identical geometry, verify every collision mask/body/type,
restore the recorded per-world mass/base-COM values, and query all active
robot/plane distances at the recorded poses. This is geometric replay, not a
new dynamics rollout or measured contact force. Original arrays are preserved.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco
import yaml

ROOT=Path(__file__).resolve().parents[2]
TOLERANCE_M=1e-6


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module);return module


def normalized_records(model):
    return [dict(name=model.geom(g).name or '',body=model.body(int(model.geom_bodyid[g])).name,
                 type=mujoco.mjtGeom(int(model.geom_type[g])).name,contype=int(model.geom_contype[g]),conaffinity=int(model.geom_conaffinity[g]))
            for g in range(model.ngeom) if model.geom_bodyid[g]!=0]


def verify_collision_identity(model,record):
    expected=[{k:(v.removeprefix('robot/') if k in ('name','body') else v) for k,v in row.items() if k not in ('id','torso')} for row in record['robot_geometries']]
    actual=normalized_records(model)
    if actual!=expected or record['terrain_heightfield_count']!=0:
        raise ValueError('Reconstructed geometry masks/names/body/type do not match the saved source model')
    return hashlib.sha256(json.dumps(actual,sort_keys=True).encode()).hexdigest()


def audit_poses(model,trace,body_masses,body_names,base_com_offset):
    model=copy.copy(model)  # Restore per-world inertial values without changing the caller's template.
    root=model.body('base_link').id
    for name,mass in zip(body_names,body_masses):model.body_mass[model.body(str(name).removeprefix('robot/')).id]=mass
    model.body_ipos[root]+=base_com_offset
    data=mujoco.MjData(model);mujoco.mj_setConst(model,data)
    joint_names=[f'leg_{leg}_{j}' for leg in ('front_r','front_l','back_r','back_l') for j in (1,2,3)]
    qadr=[model.jnt_qposadr[model.joint(n).id] for n in joint_names]
    free=int(np.flatnonzero(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)[0]);rq=int(model.jnt_qposadr[free])
    terrain=model.geom('terrain').id
    active=[g for g in range(model.ngeom) if model.geom_bodyid[g]!=0 and (model.geom_contype[g] or model.geom_conaffinity[g])]
    feet=[model.body(f'leg_{leg}_3').id for leg in ('front_r','front_l','back_r','back_l')]
    torso=[g for g in active if model.geom_bodyid[g]==root]
    if len(active)!=9 or len(torso)!=1:raise ValueError('This replay is scoped to original knee/foot spheres plus one torso hull')
    distances=[];height=[];foot=[]
    for i in range(len(trace['joint_pos'])):
        data.qpos[rq:rq+3]=trace['root_position'][i,0];data.qpos[rq+3:rq+7]=trace['root_quaternion'][i,0];data.qpos[qadr]=trace['joint_pos'][i,0]
        mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
        values=[mujoco.mj_geomDistance(model,data,g,terrain,10.,None) for g in active]
        distances.append(values);height.append(data.subtree_com[root,2])
        foot.append([any(v<=TOLERANCE_M for g,v in zip(active,values) if model.geom_bodyid[g]==body) for body in feet])
    distances=np.asarray(distances);height=np.asarray(height)
    if not np.isfinite(distances).all() or not np.isfinite(height).all():raise ValueError('Nonfinite replay geometry')
    return dict(robot_ground_contact_count=(distances<=TOLERANCE_M).sum(axis=1)[:,None],
        audited_foot_contact=np.asarray(foot)[:,None,:],robot_com_height_m=height[:,None],
        robot_com_vertical_velocity_m_s=np.gradient(height,float(trace['dt']),edge_order=2)[:,None],
        torso_ground_signed_distance_m=distances[:,active.index(torso[0]),None],
        robot_geom_ground_distance_m=distances[:,None,:]),[model.geom(g).name for g in active]


def run(args):
    source=args.source.resolve();provenance=json.loads((source/'provenance.json').read_text());revision=provenance['git_revision']
    snapshot=source/'randomization_snapshot.npz'
    if sha(snapshot)!=provenance['domain_randomization']['snapshot_sha256']:raise ValueError('Randomization snapshot hash differs from recorded source')
    cfg=yaml.load((source/'env.yaml').read_text(),Loader=yaml.BaseLoader)
    if cfg['scene']['terrain']['terrain_type']!='plane':raise ValueError('Only recorded flat plane geometry is supported')
    inertial=[x['function'] for x in provenance['domain_randomization']['event_order'] if x['name']=='inertial']
    if inertial!=['mjlab.tasks.pupper.deployment_contract.density_randomization_preserving_com']:
        raise ValueError('Need the reviewed COM-preserving density model to reconstruct all body COMs')
    sources=[ROOT/'pupper-mjlab/src/mjlab/tasks/pupper/locomotion_contact_contract.py',
        ROOT/'pupper-mjlab/src/mjlab/tasks/tracking/config/pupper/env_cfgs.py',ROOT/'pupper-mjlab/src/mjlab/tasks/tracking/config/pupper/contact_contract.py',
        *sorted(p for p in (ROOT/'pupper-mjlab/src/mjlab/asset_zoo/robots/pupper_v3').rglob('*') if p.is_file() and p.suffix in {'.py','.xml','.obj','.stl'})]
    source_hashes={}
    for p in sources:
        rel=str(p.relative_to(ROOT));expected=subprocess.check_output(['git','show',revision+':'+rel],cwd=ROOT)
        if hashlib.sha256(expected).hexdigest()!=sha(p):raise ValueError(f'Geometry source changed since captured run: {rel}')
        source_hashes[rel]=sha(p)
    helper=load(ROOT/'foundation/scripts/check_spider_handoff.py','capture_geometry_model')
    model,contract=helper.make_model('torso_only')
    geometry_sha=verify_collision_identity(model,provenance['collision_model'])
    with np.load(source/'trace.npz',allow_pickle=False) as archive:
        shape=archive['valid'].shape;a,b,e=args.start,args.end,args.env
        if not 1<=a<b<shape[0]-1 or not 0<=e<shape[1]:raise ValueError('Choose a bounded window with one neighboring frame on each side')
        # Neighbor frames make central-difference COM velocity available at both
        # saved endpoints. No sample spacing, joint value or timing is altered.
        expanded={}
        for key in archive.files:
            v=archive[key]
            expanded[key]=v[a-1:b+1,e:e+1].copy() if v.ndim>=2 and v.shape[:2]==shape else v[a-1:b+1].copy() if v.ndim and v.shape[0]==shape[0] else v.copy()
        original_q=archive['joint_pos'][a:b,e:e+1].copy()
    if not np.isfinite(expanded['joint_pos']).all() or not np.isfinite(expanded['root_position']).all() or not np.isfinite(expanded['root_quaternion']).all():raise ValueError('Nonfinite source pose')
    with np.load(snapshot,allow_pickle=False) as dr:
        arrays,geoms=audit_poses(model,expanded,dr['body_mass_kg'][e],dr['body_names'],dr['base_com_offset_m'][e])
    selected={key:(v[1:-1].copy() if v.ndim and v.shape[0]==b-a+2 else v.copy()) for key,v in expanded.items()}
    if 'torso_ground_signed_distance_m' in selected:selected['original_torso_ground_signed_distance_m']=selected['torso_ground_signed_distance_m'].copy()
    selected.update({key:value[1:-1].copy() for key,value in arrays.items()})
    np.testing.assert_array_equal(selected['joint_pos'],original_q)
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    selected['contact_audit_metadata_json']=np.asarray(json.dumps(dict(
        protocol='saved_flat_capture_full_contact_geometry_v1',source_revision=revision,
        source_trace_sha256=sha(source/'trace.npz'),source_provenance_sha256=sha(source/'provenance.json'),
        source_window=dict(start=a,end=b,env=e),geometry_identity_sha256=geometry_sha,
        randomization_snapshot_sha256=sha(snapshot),source_geometry_sha256=source_hashes,
        contact_tolerance_m=TOLERANCE_M,motion_unchanged=True,dynamics_replayed=False),sort_keys=True))
    np.savez_compressed(out/'audited_trace.npz',**selected)
    audit=dict(protocol='saved_flat_capture_full_contact_geometry_v1',source_revision=revision,
        source_directory=str(source),source_trace_sha256=sha(source/'trace.npz'),source_provenance_sha256=sha(source/'provenance.json'),
        source_env_sha256=sha(source/'env.yaml'),randomization_snapshot_sha256=sha(snapshot),source_geometry_sha256=source_hashes,
        geometry_contract=contract,normalized_geometry_sha256=geometry_sha,geom_order=geoms,contact_tolerance_m=TOLERANCE_M,
        source_window=dict(start=a,end=b,env=e),sample_dt_s=float(selected['dt']),motion_unchanged=True,interpolation=False,
        dynamics_replayed=False,hardware_validated=False,foot_contact_source='fresh geometric distance at aligned recorded pose; original foot_contact retained',
        com_reconstruction='Recorded per-world body masses and base COM offset; COM vertical velocity from central differences of recorded-pose COM at native trace20ms sampling.',
        limitations=['Sampled geometry cannot establish contact timing between recorded frames.','No support forces are reconstructed; geometric touching is conservative.','Compiled source uses knee/foot spheres plus torso hull; no inactive visual meshes are reclassified as active geometry.'],
        audited_trace_sha256=sha(out/'audited_trace.npz'))
    bootstrap=load(ROOT/'pupper-mjlab/src/mjlab/tasks/pupper_gait/bootstrap.py','capture_selector')
    try:
        audit['strict_selection']=bootstrap.select_jump(selected,limits=bootstrap.robot_limits());audit['accepted']=True
    except ValueError as error:audit['strict_selection']=None;audit['accepted']=False;audit['rejection']=str(error)
    audit['capture_gate']=dict(bootstrap.JUMP_CAPTURE_GATE)
    (out/'audit.json').write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:audit[k] for k in ('accepted','source_window','strict_selection')},indent=2))
    return audit


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--start',type=int,required=True);p.add_argument('--end',type=int,required=True);p.add_argument('--env',type=int,required=True)
    run(p.parse_args())
