"""Foot-plane body tilt from recorded joint angles (MuJoCo FK of the production model).
Writes <out>.npz: t, up_body (unit floor normal in body frame), feet spread, jv_rms."""
import sys, importlib.util, numpy as np, mujoco
from pathlib import Path
ROOT=Path('/Users/shrish/workspace/cs123')
spec=importlib.util.spec_from_file_location('hand', ROOT/'foundation/scripts/check_spider_handoff.py')
hand=importlib.util.module_from_spec(spec); sys.modules['hand']=hand; spec.loader.exec_module(hand)
model,_=hand.make_model('full_hulls'); d=mujoco.MjData(model)
base=model.body('base_link').id
D=np.load(sys.argv[1],allow_pickle=True); js=D['js']; jn=list(D['jn'])
qadr=[model.jnt_qposadr[model.joint(n).id] for n in jn]
sites=[model.site(f'leg_{l}_3_foot_site').id for l in ('front_r','front_l','back_r','back_l')]
rj=[j for j in range(model.njnt) if model.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE][0]; rq=model.jnt_qposadr[rj]
step=int(sys.argv[3]) if len(sys.argv)>3 else 10   # ~25 Hz from 250 Hz joint states
T=[];U=[];V=[]
for k in range(0,len(js),step):
    r=js[k]; d.qpos[:]=0; d.qpos[rq+3]=1
    for a,v in zip(qadr,r[1:13]): d.qpos[a]=v
    mujoco.mj_kinematics(model,d)
    R=d.xmat[base].reshape(3,3); p0=d.xpos[base]
    F=np.array([R.T@(d.site_xpos[s]-p0) for s in sites])
    # lowest-3 plane would need gravity; use all 4 (least squares)
    c=F.mean(0); u,s,vt=np.linalg.svd(F-c); n=vt[2]
    if n[2]>0: n=-n          # normal pointing from feet toward body = up
    n=-n if (c@n)>0 else n   # body origin is above the feet plane: up points from c toward 0
    T.append(r[0]); U.append(n); V.append(np.sqrt((r[13:25]**2).mean()))
np.savez(sys.argv[2],t=np.array(T),up=np.array(U),jv=np.array(V))
print('done',len(T))
