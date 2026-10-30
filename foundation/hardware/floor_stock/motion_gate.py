"""Canonical source is reviewable offline; no motor path is released by this adoption."""
MOTION_EXECUTION_ENABLED = False
BLOCKERS = [
    'Reviewed deployment identity pins and installed-runtime verification must be restored; source hash inventory is not environment validation.',
    'Failed native floor raw archive has not been recovered or independently reviewed.',
    'Exact joystick input-only preflight has not passed on the next session runtime.',
    'Detached-owner failure/fallback cause has not been reconciled with native receipts.',
]

def require_motion_enabled():
    if not MOTION_EXECUTION_ENABLED:
        raise RuntimeError('Canonical floor motion execution is disabled: ' + ' '.join(BLOCKERS))
