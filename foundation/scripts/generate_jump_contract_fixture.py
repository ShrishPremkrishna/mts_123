#!/usr/bin/env python3
"""Regenerate the synthetic custom-touchdown Torch/JSON/C++ contract fixture.

This is a math test with invented bounded joint angles. It is not a captured jump,
trained policy, or motion to execute. Run in a fresh process: mjlab registers task
configs at import, so the fixture reference directory must exist first.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root/'pupper_gait_deploy/test/jump_timing_golden.json')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='pupper-contract-') as temporary:
        capture = Path(temporary)/'captured_jump_gait.npz'
        phase = np.linspace(0, 1, 56)[:, None]
        joints = .05*np.sin(2*np.pi*phase)*np.linspace(.5, 1, 12)[None, :]
        np.savez(capture, jump=joints, duration_s=1.1, touchdown_s=.66,
                 joint_names=[f'leg_{leg}_{i}' for leg in ('front_r','front_l','back_r','back_l') for i in (1,2,3)])
        os.environ['PUPPER_REFERENCE_DIR'] = temporary
        from mjlab.envs import ManagerBasedRlEnv
        from mjlab.tasks.registry import load_env_cfg
        from mjlab.tasks.pupper_gait.export import gait_reference_metadata, jump_slot_metadata
        import mjlab.scripts.export_pupper_policy as exporter
        cfg = load_env_cfg('Mjlab-Mystery-Flat-Pupper-v3', play=True)
        cfg.scene.num_envs = 64
        cfg.seed = 42
        env = ManagerBasedRlEnv(cfg, device='cpu')
        try:
            gait, slot = gait_reference_metadata(env), jump_slot_metadata(env)
            assert abs(slot['playback_s'] - 1.1) < 1e-9
            assert abs(slot['active_s'] - .62) < 1e-9
            assert abs(slot['busy_s'] - 1.16) < 1e-9
            fixture = exporter.make_gait_golden(env, gait)
            exporter._SLOT_CASE_T_IN = (-.02, 0., .02, .3, .5, .56, .6, .62, .64, 1.2)
            fixture['jump_slot'] = slot
            fixture['slot_cases'] = exporter.make_slot_golden(env, gait, slot)
            fixture['_comment'] = 'Synthetic custom-touchdown contract fixture; not a real jump, policy, or physical validation. Regenerate with foundation/scripts/generate_jump_contract_fixture.py.'
            fixture['synthetic_capture'] = {'duration_s':1.1, 'touchdown_s':.66,
                'pre_land_lead_s':.1, 'cross_fade_s':.06, 'source': 'bounded analytical sine joint table, not sensor data'}
            fixture['generation_revision'] = subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip()
            paths = ['pupper-mjlab/src/mjlab/tasks/pupper_gait/export.py',
                     'pupper-mjlab/src/mjlab/tasks/pupper_gait/mdp/gait_reference.py',
                     'pupper-mjlab/src/mjlab/tasks/pupper_gait/mdp/mixed_jump.py',
                     'pupper-mjlab/src/mjlab/tasks/pupper_gait/mystery_env_cfg.py']
            fixture['source_sha256'] = {p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in paths}
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(fixture, indent=2, allow_nan=False)+'\n')
            print(args.output)
        finally:
            env.close()


if __name__ == '__main__':
    main()
