#!/bin/bash
# Request controller deactivation, stop the app/server/robot stack, and restore
# the normal screen app. Request acceptance does not verify physical motor release.
api_ok=1
api_success() {
  python3 -c 'import json,sys; sys.exit(0 if json.load(sys.stdin).get("success") is True else 1)' 2>/dev/null
}
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
unset stop_reply health_reply command_epoch deactivate_reply
free_port() {  # stop whatever is serving port 8000 (an older robot server)
  local pids; pids=$(ss -ltnpH "sport = :8000" 2>/dev/null | grep -oP "pid=\\K[0-9]+" | sort -u)
  [ -n "$pids" ] && kill $pids 2>/dev/null && sleep 1
  return 0
}
pkill -f "[r]obot/kiosk.py"; pkill -f "[g]emini-pupper-kiosk"; pkill -f "[r]obot/server.py"; free_port
pkill -INT -f "[g]emini_pupper.launch.py"
for i in $(seq 1 15); do pgrep -f "[r]os2_control_node" >/dev/null || break; sleep 1; done
pkill -9 -f "[g]emini_pupper.launch.py"; pkill -9 -f "[/]opt/ros/jazzy/lib/"; pkill -9 -f "[/]home/pi/pupperv3-monorepo/ros2_ws/install/"
# The person follower runs from the follow_me_pupper repo, outside the paths above.
# SIGTERM first so it releases the Hailo; a respawned copy can outlive the launch.
pkill -f "[f]ollow_service.py" && sleep 2; pkill -9 -f "[f]ollow_service.py"
sudo -n systemctl start pupper-gui 2>/dev/null
if [ "$api_ok" = 1 ]; then
  echo "API Stop/deactivation requests accepted; teardown requested. Completion and physical motor release not verified."
else
  echo "Teardown requested, but graceful API Stop/deactivation was not confirmed. Check the robot before further use." >&2
  exit 1
fi
