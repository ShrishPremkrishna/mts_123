#!/usr/bin/env python3
"""Run the foundation's offline checks and save their exact commands and outcomes.

Invoke with the repository's installed Python environment. This runs no training,
network request, browser operation or robot command. ROS is faked by the behavioral
tests; MuJoCo and PyTorch must be installed for the numerical and CPU physics tests.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output = (args.output or root/'artifacts/validation'/stamp).resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env['FORCE_CPU'] = '1'
    env['CUDA_VISIBLE_DEVICES'] = ''
    env.pop('PUPPER_REFERENCE_DIR', None)
    for key, relative in [('MPLCONFIGDIR','matplotlib'), ('WARP_CACHE_PATH','warp'), ('XDG_CACHE_HOME','general')]:
        env.setdefault(key, str(output/'cache'/relative))
    python = sys.executable
    checks = [
        ('classical', [python,'-m','pytest','foundation/tests','-q','-p','no:cacheprovider']),
        ('tool_queue', [python,'-m','pytest','gemini_pupper/tests','-q','-p','no:cacheprovider']),
        ('compliance', [python,'-m','pytest','implicit_compliance/tests','-q','-p','no:cacheprovider']),
    ]
    numerical = ['test_pupper_foundation.py','test_pupper_tracking.py','test_pupper_spider.py',
                 'test_pupper_unimate.py','test_unimate_repair.py','test_unimate_worker.py',
                 'test_unimate_candidates.py','test_foundation_evaluation.py','test_pupper_export.py',
                 'test_pupper_gait_export.py','test_pupper_latency.py','test_pupper_bootstrap.py',
                 'test_deployment_target_clamp.py','test_pupper_randomization_contract.py',
                 'test_foundation_training_contract.py','test_tracking_ground_hulls.py',
                 'test_locomotion_torso_contact.py','test_pupper_collision_audit.py',
                 'test_pupper_policy_audit.py']
    numerical += ['test_driver_target_clamp.py']
    checks += [('motion_export', [python,'-m','pytest',*[f'pupper-mjlab/tests/{name}' for name in numerical],'-q','-p','no:cacheprovider']),
               ('controller_cpp', ['bash','pupper_gait_deploy/test/run_host_test.sh'])]
    revision = subprocess.run(['git','rev-parse','HEAD'],cwd=root,text=True,capture_output=True,check=False)
    candidates = subprocess.run(['git','ls-files','--cached','--others','--exclude-standard','-z'],
                                cwd=root,capture_output=True,check=True).stdout.decode().split('\0')
    watched = ('foundation/','forward_kinematics_lab/','pd_control_lab/','ik_heuristic_walking_lab/',
               'pupper-gait-tuner/','follow_me_pupper/','gemini_pupper/','implicit_compliance/',
               'pupper_gait_deploy/','pupper-mjlab/src/','pupper-mjlab/tests/')
    files = sorted({name for name in candidates if name.startswith(watched) and (root/name).is_file()
                    and Path(name).suffix in ('.py','.cpp','.hpp','.sh','.json','.yaml','.xml')}
                   | {str(path.relative_to(root)) for path in (root/'foundation').rglob('*.py')})
    hashes = {name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in files}
    report = {'created_utc':stamp,'git_revision':revision.stdout.strip(),'python':sys.version,
              'scope':'offline regression and numerical contract checks; no hardware or trained-policy performance claim',
              'source_sha256':hashes,'checks':[],'passed':False}
    destination = output/'checks.json'
    for name, command in checks:
        start = time.monotonic()
        log = output/f'{name}.log'
        print(f'Running {name}; log: {log}', flush=True)
        with log.open('w') as stream:
            result = subprocess.run(command,cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,check=False)
        report['checks'].append({'name':name,'command':command,'returncode':result.returncode,
                                 'seconds':time.monotonic()-start,'log':str(log)})
        destination.write_text(json.dumps(report,indent=2)+'\n')
        print(f'{name}: {"PASS" if result.returncode == 0 else "FAIL"}', flush=True)
    report['passed'] = all(check['returncode'] == 0 for check in report['checks'])
    destination.write_text(json.dumps(report,indent=2)+'\n')
    print(destination)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
