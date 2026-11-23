#!/bin/bash
# Request controller deactivation, stop the app/server/robot stack, and restore
# the normal screen app. Request acceptance does not verify physical motor release.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=/tmp/gemini-pupper
lifecycle() { python3 "$HERE/session_processes.py" --directory "$LOG" "$@"; }
lifecycle check || exit 1
SESSION=$(lifecycle stopping) || exit 1
export GEMINI_PUPPER_SESSION=$SESSION
api_ok=1
api_success() {
  python3 -c 'import json,sys; sys.exit(0 if json.load(sys.stdin).get("success") is True else 1)' 2>/dev/null
}
if lifecycle api-owned; then
# Stop always bypasses the command-epoch requirement. Check HTTP and semantic
# success; never print server bodies, which may contain unrelated robot details.
if ! stop_reply=$(curl -fsS -m 3 -X POST localhost:8000/stop/immediate 2>/dev/null) ||
   ! printf '%s' "$stop_reply" | api_success; then
  api_ok=0
fi
# Acquire a fresh epoch after Stop. A later competing Stop can invalidate it;
# reject that response honestly rather than retrying an old deactivation intent.
if health_reply=$(curl -fsS -m 3 localhost:8000/health 2>/dev/null) &&
   command_epoch=$(printf '%s' "$health_reply" | python3 -c '
import json,sys
metadata=json.load(sys.stdin)
if metadata.get("command_epoch_required") is not True:
    raise SystemExit(1)
value=metadata.get("command_epoch")
if not isinstance(value,str) or not value or len(value)>256 or any(ord(c)<33 or ord(c)>126 for c in value):
    raise SystemExit(1)
print(value)
' 2>/dev/null); then
  if deactivate_reply=$(curl -fsS -m 3 -X POST -H "X-Pupper-Command-Epoch: $command_epoch" localhost:8000/walking/deactivate 2>/dev/null) &&
     printf '%s' "$deactivate_reply" | api_success; then
    sleep 2
  else
    api_ok=0
  fi
else
  api_ok=0
fi
else
  api_ok=0
fi
unset stop_reply health_reply command_epoch deactivate_reply
cleanup_ok=1
lifecycle cleanup || cleanup_ok=0
if [ "$api_ok" = 1 ] && [ "$cleanup_ok" = 1 ]; then
  echo "API Stop/deactivation requests accepted; owned process cleanup completed. Physical motor release is not verified."
else
  echo "Graceful API Stop/deactivation or owned process cleanup was not confirmed. Recovery receipt retained; physical motor release is not verified. Check the robot before further use." >&2
  exit 1
fi
