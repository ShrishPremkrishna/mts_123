"""Read and validate installed native policy files; never load ROS or execute motion."""
from __future__ import annotations

import argparse
import contextlib
from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    validator_path = root / 'pupper_gait_deploy/download_policy.py'
    spec = importlib.util.spec_from_file_location('installed_policy_validator', validator_path)
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    sys.path.insert(0, str(root / 'gemini_pupper/robot'))
    from tricks import load_tricks, reference_message_data, validate_locomotion_contract

    report = {'checked_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'Native policy files, descriptors and reference contracts only; no simulated execution or hardware capability claim.',
              'validator_sha256': hashlib.sha256(validator_path.read_bytes()).hexdigest(),
              'policies': [], 'errors': []}
    try:
        tricks = load_tricks()
        report['active_descriptors'] = sorted(tricks)
        for trick in tricks.values():
            if trick.kind == 'reference':
                reference_message_data(trick)
        validate_locomotion_contract(root / 'pupper_gait_deploy/policies/default_policy.json',
                                     [trick for trick in tricks.values() if trick.kind == 'reference'])
    except (ValueError, KeyError, OSError, SystemExit) as error:
        report['errors'].append(f'Descriptor/reference contract: {error}')
    paths = sorted(set((root / 'gemini_pupper/tricks').glob('*/policy.json'))
                   | set((root / 'gemini_pupper/tricks').glob('*/walk_policy.json'))
                   | {root / 'pupper_gait_deploy/policies/default_policy.json'})
    for path in paths:
        entry = {'path': str(path.relative_to(root)), 'native_contract_valid': False}
        try:
            entry['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
            with contextlib.redirect_stdout(io.StringIO()):
                validator.validate(path)
            entry['native_contract_valid'] = True
        except (ValueError, KeyError, OSError, SystemExit) as error:
            entry['error'] = str(error)
        report['policies'].append(entry)
    report['passed'] = not report['errors'] and all(item['native_contract_valid'] for item in report['policies'])
    payload = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(payload)
    else:
        print(payload, end='')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
