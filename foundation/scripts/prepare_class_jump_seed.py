#!/usr/bin/env python3
"""Convert the identified shipped jump reference into a labeled training seed.

The loader's filename is captured_jump_gait.npz, but this is a course template,
not a new physical capture. No motion is invented, clipped, smoothed or scaled.
Run with the Pupper environment installed; existing output folders are refused.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
import numpy as np

POLICY_SHA256 = 'ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712'
REFERENCE_SHA256 = '0d98db362dfb4b80eef6378b3aa116075f7a63d493328617ab3b25e8fb6ebfdb'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(repo, out):
    repo, out = Path(repo).resolve(), Path(out).resolve()
    if out.exists():
        raise FileExistsError(f'Refusing to overwrite {out}')
    policy = repo/'pupper_gait_deploy/policies/default_policy.json'
    reference = repo/'gemini_pupper/tricks/jump/reference.json'
    if sha(policy) != POLICY_SHA256 or sha(reference) != REFERENCE_SHA256:
        raise ValueError('Class policy/reference identity changed; review the new source explicitly')
    p, r = json.loads(policy.read_text()), json.loads(reference.read_text())
    jump = p['jump_slot']
    if not np.allclose(np.asarray(jump['jump_table']), r['table'], rtol=0., atol=1e-6):
        raise ValueError('Policy and class reference tables differ')
    if any(jump[k] != r[k] for k in ('playback_s', 'active_s', 'cross_fade_s')):
        raise ValueError('Policy and class reference timing differ')
    spec = importlib.util.spec_from_file_location('class_seed_bootstrap', repo/'pupper-mjlab/src/mjlab/tasks/pupper_gait/bootstrap.py')
    bootstrap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bootstrap)
    if p['gait_reference']['joint_names'] != bootstrap.JOINT_ORDER:
        raise ValueError('Canonical robot joint order changed')
    absolute = np.asarray(jump['jump_table'], dtype=np.float64)
    check = bootstrap.validate_reference(absolute, limits=bootstrap.robot_limits(), cycle_s=jump['playback_s'], periodic=False)
    metadata = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'source_kind': 'imported_class_reference_template_not_measured_capture',
        'source_policy': str(policy.relative_to(repo)), 'source_policy_sha256': sha(policy),
        'source_reference': str(reference.relative_to(repo)), 'source_reference_sha256': sha(reference),
        'conversion': 'Exact absolute exported jump joint-angle table in canonical robot order. No default-pose addition, smoothing, scaling, clipping or invented motion.',
        'source_description_correction': 'The original course reference.json description incorrectly called these offsets; that prose is corrected in this branch without changing numerical data. export.jump_slot_metadata stores the absolute table and both training mixed_jump_reference_offset and native gait_reference.hpp subtract the default pose when producing observation offsets.',
        'absolute_joint_table_contract': True,
        'format_note': 'captured_jump_gait.npz is the existing loader socket; this is not an observed physical jump.',
        'nominal_timing': {'playback_s': jump['playback_s'], 'active_s': jump['active_s'], 'cross_fade_s': jump['cross_fade_s'], 'legacy_touchdown_fallback_s': .84, 'touchdown_is_measured': False},
        'kinematic_validation': check, 'hardware_approved': False, 'physical_flight_validated': False,
    }
    out.mkdir(parents=True)
    # Deliberately omit touchdown_s: .84 s is the existing nominal fallback,
    # not a measurement from a newly captured simulated or physical landing.
    np.savez_compressed(out/'captured_jump_gait.npz', jump=absolute, duration_s=jump['playback_s'],
                        joint_names=np.asarray(bootstrap.JOINT_ORDER), metadata_json=np.asarray(json.dumps(metadata)))
    metadata['reference_sha256'] = sha(out/'captured_jump_gait.npz')
    (out/'class_reference_manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.repo, args.out), indent=2))


if __name__ == '__main__':
    main()
