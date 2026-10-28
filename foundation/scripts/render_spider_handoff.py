#!/usr/bin/env python3
"""Render two saved continuous physics traces side by side; no simulation rerun."""
import argparse
import json
from pathlib import Path
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw


def render(left, right, output, left_label=None, right_label=None, shot_times=None):
    from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
    panels=[]
    inputs=[(left,left_label or 'Exported policy physics')] if right is None else [(left,left_label or 'Legacy activation'),(right,right_label or 'Selected clip activation')]
    if shot_times is None:shot_times=(.5,1.5,2.5,4.,6.,9.) if right is None else (22.6,23.1,23.6,24.1,27.,32.)
    if not np.isfinite(shot_times).all() or any(t<0 for t in shot_times):raise ValueError('Snapshot times must be finite and nonnegative')
    for folder,label in inputs:
        model=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'))
        # Presentation only: brighten the headlight and floor; qpos/model physics
        # remain the exact saved rollout. Dark mesh materials otherwise hide feet.
        model.vis.headlight.active=1
        model.vis.headlight.ambient[:]=[.7,.7,.7]
        model.vis.headlight.diffuse[:]=[.8,.8,.8]
        model.geom_rgba[model.geom('terrain').id]=[.85,.85,.85,1]
        data=mujoco.MjData(model);trace=json.loads((folder/'trajectory.json').read_text())
        qadr=[model.jnt_qposadr[model.joint(name).id] for name in JOINT_NAMES]
        root=int(model.jnt_qposadr[np.flatnonzero(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)[0]])
        camera=mujoco.MjvCamera();camera.distance=.75;camera.azimuth=120;camera.elevation=-22
        panels.append((model,data,trace,qadr,root,camera,mujoco.Renderer(model,height=480,width=640),label))
    count=min(len(panel[2]) for panel in panels);shots=[]
    try:
        with imageio.get_writer(output,fps=26) as writer:
            for i in range(0,count,2):
                images=[]
                for model,data,trace,qadr,root,camera,renderer,label in panels:
                    row=trace[i];data.qpos[root:root+3]=row['root_pos'];data.qpos[root+3:root+7]=row['root_quat'];data.qpos[qadr]=row['joint_pos']
                    mujoco.mj_forward(model,data)
                    camera.lookat[:]=[*row['root_pos'][:2],.075]
                    renderer.update_scene(data,camera=camera)
                    im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
                    draw.rectangle((0,0,640,57),fill='white')
                    draw.text((12,8),f'{label} | {row["time_s"]:.2f}s | {row["stage"]}',fill='black')
                    draw.text((12,29),f'{row["activation_stage"]} | torso {row["root_pos"][2]*100:.1f}cm | tilt {row["tilt_deg"]:.1f}deg',fill='black')
                    images.append(np.asarray(im))
                combined=np.concatenate(images,axis=1);writer.append_data(combined)
                t=panels[0][2][i]['time_s']
                if any(abs(t-target)<1/52 for target in shot_times):
                    shots.append(combined)
        if shots:
            imageio.imwrite(output.with_suffix('.contact-sheet.png'),np.concatenate(shots,axis=0))
    finally:
        for panel in panels:panel[6].close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('left',type=Path);parser.add_argument('right',type=Path,nargs='?');parser.add_argument('output',type=Path)
    parser.add_argument('--left-label');parser.add_argument('--right-label')
    parser.add_argument('--snapshot-times',type=float,nargs='+')
    args=parser.parse_args();render(args.left,args.right,args.output,args.left_label,args.right_label,args.snapshot_times)
