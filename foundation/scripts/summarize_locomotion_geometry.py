#!/usr/bin/env python3
"""Preserve measured, source-identified plane-clearance screens from rollout traces."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def summarize(path):
  metrics=json.loads((path/'metrics.json').read_text())
  audit=json.loads((path/'torso_geometry_audit.json').read_text())
  with np.load(path/'trace.npz') as trace:
    distance=trace['torso_ground_signed_distance_m']
    selected=trace['valid'].astype(bool)&np.isfinite(distance)
    values=distance[selected]
    raw=distance[np.isfinite(distance)]
    # A state is one policy-step observation in one simulated robot world.
    aggregate=dict(scored_states=int(selected.sum()),
                   penetrated_scored_states=int((values<-.0001).sum()),
                   penetrated_scored_fraction=float(np.mean(values<-.0001)),
                   minimum_scored_clearance_m=float(values.min()),
                   minimum_all_clearance_m=float(raw.min()),
                   failures=sum(s['failures'] for s in metrics['segments']),
                   timeouts=sum(s['timeouts'] for s in metrics['segments']))
  segments=[]
  for geometry,behavior in zip(audit['segments'],metrics['segments'],strict=True):
    assert geometry['segment']==behavior['segment']
    segments.append({**geometry,**{k:behavior[k] for k in (
      'command','mean_xy_error_m_s','mean_abs_yaw_error_rad_s','mean_achieved_vx_m_s',
      'peak_abs_torque_nm','mean_contact_slip_m_s','failures','timeouts')}})
  return dict(directory=str(path),provenance=metrics['provenance'],aggregate=aggregate,
              geometry_method={k:v for k,v in audit.items() if k!='segments'},segments=segments,
              files_sha256={name:hashlib.sha256((path/name).read_bytes()).hexdigest()
                            for name in ('metrics.json','provenance.json','torso_geometry_audit.json','trace.npz')})


def main():
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--run',type=Path,action='append',required=True)
  parser.add_argument('--out',type=Path,required=True)
  args=parser.parse_args()
  report=dict(scope='Single-seed continuous command screens with four simulated worlds, not hardware validation or a broad robustness claim.',
              method_validation='Signed plane distance matches lowest transformed vertex of actual Pupper torso mesh at upright and rotated poses; disabled geometry is still measured. Heightfields explicitly unsupported.',
              runs=[summarize(p) for p in args.run])
  args.out.parent.mkdir(parents=True,exist_ok=True)
  args.out.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
