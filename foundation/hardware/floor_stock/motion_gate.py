"""Released for explicit supervised stock stand/R3 trials after native input review.

The supervisor still requires operator approval, verified deployed identities,
current physical input exercise and explicit floor placement. No automatic start.
"""
MOTION_EXECUTION_ENABLED = True
BLOCKERS = []

def require_motion_enabled():
    if not MOTION_EXECUTION_ENABLED:
        raise RuntimeError('Canonical floor motion execution is disabled: ' + ' '.join(BLOCKERS))
