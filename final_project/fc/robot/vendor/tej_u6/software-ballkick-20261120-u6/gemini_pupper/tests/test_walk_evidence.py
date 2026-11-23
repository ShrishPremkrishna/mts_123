"""Operating limits must not silently travel to a different walking policy."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
from get_trick import validate_walk_evidence


def test_limits_bind_exact_export_bytes(tmp_path):
    policy = tmp_path / 'walk_policy.json'
    policy.write_text(json.dumps({'layers': ['evaluated model']}))
    limits = {'walk_policy_sha256': hashlib.sha256(policy.read_bytes()).hexdigest()}
    validate_walk_evidence(tmp_path, limits)
    policy.write_text(json.dumps({'layers': ['different model']}))
    with pytest.raises(ValueError, match='does not match'):
        validate_walk_evidence(tmp_path, limits)


@pytest.mark.parametrize('digest', [None, '', 'some evaluated run', 'a' * 63, 'g' * 64, True])
def test_missing_or_invalid_identity_refused(tmp_path, digest):
    (tmp_path / 'walk_policy.json').write_text('{}')
    with pytest.raises(ValueError, match='requires walk_policy_sha256'):
        validate_walk_evidence(tmp_path, {'walk_policy_sha256': digest})
