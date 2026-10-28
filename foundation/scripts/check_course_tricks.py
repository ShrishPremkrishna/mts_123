"""Nominal native activation/hold screen of active, finite, supplied course tricks.

Runs no ROS, network call or physical robot command. This is a diagnostic, not
approval of the course policies or evidence of randomized/hardware robustness.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(ROOT/'gemini_pupper/robot'))
    from tricks import load_tricks
    available = load_tricks()
    selected = {name: trick for name, trick in available.items()
                if trick.kind == 'policy' and not trick.interactive and not trick.secret}
    if not selected or any(t.start_pose != 'standing' for t in selected.values()):
        raise ValueError('This screen requires finite standing-start policies')
    runner = ROOT/'foundation/scripts/check_spider_handoff.py'
    limits = ROOT/'foundation/configs/class-driver-joint-limits.json'
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(),
                  scope='One nominal native activation, finite clip and 2s terminal hold per active supplied finite policy. Full collision hulls, class-source driver bounds, exact policy and descriptor tilt limit. No randomized trials, ROS execution, walking handoff or hardware test.',
                  runner_sha256=sha(runner), driver_limits_sha256=sha(limits),
                  excluded={name: 'Reference, interactive, or generated secret policy has separate evidence.'
                            for name in available if name not in selected},
                  trials=[], hardware_validated=False)
    env = os.environ.copy()
    env['FORCE_CPU'] = '1'
    env['CUDA_VISIBLE_DEVICES'] = ''
    for key, relative in [('MPLCONFIGDIR','matplotlib'),('WARP_CACHE_PATH','warp'),('XDG_CACHE_HOME','general')]:
        env.setdefault(key, str(output/'cache'/relative))
    for name, trick in sorted(selected.items()):
        target = output/name
        descriptor = ROOT/'gemini_pupper/tricks'/name/'trick.yaml'
        policy_hash, descriptor_hash = sha(trick.path), sha(descriptor)
        command = [sys.executable, str(runner), '--single-motion', '--clip-anchor',
                   '--trick-policy', str(trick.path), '--output', str(target),
                   '--driver-limits', str(limits), '--contact-model', 'full_hulls',
                   '--max-body-angle-deg', str(math.degrees(trick.max_body_angle))]
        print(f'Screening {name}', flush=True)
        with (output/f'{name}.log').open('w') as log:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, check=False)
        trial = dict(name=name, command=command, returncode=result.returncode,
                     policy_sha256=policy_hash, descriptor_sha256=descriptor_hash,
                     inputs_changed=(sha(trick.path) != policy_hash or sha(descriptor) != descriptor_hash),
                     declared_end_pose=trick.end_pose,
                     max_body_angle_deg=math.degrees(trick.max_body_angle))
        saved = target/'report.json'
        if saved.is_file():
            native = json.loads(saved.read_text())
            trial.update(report_path=str(saved.relative_to(output)), report_sha256=sha(saved),
                         source_changed=native['source_changed'], stop=native['stop'],
                         completed_sequence=native['completed_sequence'],
                         stage_summaries=native['stage_summaries'])
            if native['trick_policy_sha256'] != policy_hash:
                trial['inputs_changed'] = True
        else:
            trial['error'] = 'No native report; inspect log. No completion inferred.'
        report['trials'].append(trial)
        (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    # Code/source consistency and successful execution are separate from a
    # physically completed sequence, which is recorded individually above.
    report['runner_changed'] = sha(runner) != report['runner_sha256']
    report['evidence_complete'] = (not report['runner_changed'] and all(
        t['returncode'] == 0 and 'report_sha256' in t and not t['source_changed'] and not t['inputs_changed']
        for t in report['trials']))
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    return 0 if report['evidence_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
