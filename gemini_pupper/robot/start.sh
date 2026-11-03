#!/bin/bash
# Start Gemini Pupper on the robot.
#   robot/start.sh            robot stack + robot server (use from a laptop through an SSH tunnel)
#   --kiosk is retired; robot/kiosk.py explicitly attaches to an existing API.
# Stop everything with robot/stop.sh.
set -uo pipefail
# Refuse the retired full-stack kiosk path before receipt creation or homing.
if [ "${1:-}" = "--kiosk" ]; then
  echo "--kiosk no longer starts a robot stack. Start voice explicitly against an existing healthy API:" >&2
  echo "python3 robot/kiosk.py --url http://localhost:<api-port> --bootstrap-dir <private-onboard-directory>" >&2
  exit 2
fi
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(dirname "$HERE")
LOG=/tmp/gemini-pupper
lifecycle() { python3 "$HERE/session_processes.py" --directory "$LOG" "$@"; }
# Ownership checks precede any launch (which may home). Receipt operations lock
# briefly; Stop can cancel startup while the shell waits for readiness.
lifecycle check || exit 1
KEY_FILE=${GEMINI_KEY_FILE:-$HOME/.config/gemini-pupper/api_key}
KIOSK=0; [ "${1:-}" = "--kiosk" ] && KIOSK=1
SESSION=$(lifecycle begin "${1:-}") || exit 1
export GEMINI_PUPPER_SESSION=$SESSION
trap 'code=$?; if [ "$code" != 0 ]; then echo "Startup incomplete; ownership/recovery receipt retained at $LOG/owned_session.json. No retry or rehome attempted." >&2; fi' EXIT
python3 "$HERE/preflight.py" || exit 1
MONOREPO=$(cd "$HERE" && python3 -c 'from runtime_config import component_dir; print(component_dir("pupperv3-monorepo", "PUPPER_MONOREPO_DIR"))')
printf -v ROS 'source %q; source %q; export ROS_LOCALHOST_ONLY=1' "/opt/ros/jazzy/setup.bash" "$MONOREPO/ros2_ws/install/local_setup.bash"

[ -f "$REPO/web/dist/index.html" ] || { echo "Missing $REPO/web/dist: run robot/setup.sh first (it builds the web app)"; exit 1; }
if [ $KIOSK = 1 ] && [ ! -s "$KEY_FILE" ]; then
  echo "Missing $KEY_FILE. Run robot/setup.sh, or create it with:  mkdir -p $(dirname $KEY_FILE) && (umask 077; cat > $KEY_FILE)"; exit 1
fi

if pgrep -f "[r]os2_control_node" >/dev/null; then
  lifecycle ros borrowed || exit 1
  echo "[1/3] checking the existing ROS stack without changing it"
else
  lifecycle ros created || exit 1
  echo "[1/3] starting robot stack (about a minute; log $LOG/robot.log)"
  lifecycle spawn launch "$HERE/launch/gemini_pupper.launch.py" bash -c "$ROS; exec ros2 launch \"\$1\"" _ "$HERE/launch/gemini_pupper.launch.py" || exit 1
  # spawn_controllers.py writes this once gemini_walk and every trick controller is loaded
  for i in $(seq 1 240); do lifecycle refresh || exit 1; [ -f $LOG/controllers_ready ] && break; sleep 1; done
  if [ -f $LOG/controllers_ready ]; then echo "      $(cat $LOG/controllers_ready)"
  else echo "      controllers still loading after 4 min; see $LOG/robot.log"; fi
fi

# Names alone do not prove a course-only stack has Gemini's mux, follower or
# matching policies. Inspect before replacing a server or claiming readiness.
bash -c "$ROS; exec python3 \"\$1\"" _ "$HERE/startup_readiness.py" || exit 1

echo "[2/3] starting robot server on localhost:8000 (log $LOG/server.log)"
lifecycle spawn server "$HERE/server.py" bash -c "$ROS; cd \"\$1\" && exec python3 -u \"\$1/server.py\"" _ "$HERE" || exit 1
for i in $(seq 1 30); do lifecycle api-owned >/dev/null 2>&1 && curl -fsS -m 3 -o /dev/null localhost:8000/health && break; sleep 1; done
lifecycle api-owned && curl -fsS -m 3 -o /dev/null localhost:8000/health || { echo "robot server did not start; see $LOG/server.log"; exit 1; }

if [ $KIOSK = 0 ]; then
  lifecycle ready || exit 1
  echo "[3/3] ready. From a laptop:  ssh -N -L 8000:localhost:8000 pi@<pupper>  then open http://localhost:8000"
  exit 0
fi
echo "[3/3] opening the app on the robot's screen"
export XDG_RUNTIME_DIR=/run/user/$(id -u) WAYLAND_DISPLAY=wayland-0
lifecycle stop-gui || exit 1
lifecycle spawn chromium "--user-data-dir=$HOME/.cache/gemini-pupper-kiosk" chromium --ozone-platform=wayland --kiosk --noerrdialogs --disable-infobars --no-first-run \
  --password-store=basic --use-fake-ui-for-media-stream --autoplay-policy=no-user-gesture-required \
  --remote-debugging-port=9222 --user-data-dir=$HOME/.cache/gemini-pupper-kiosk http://localhost:8000 \
  || exit 1
lifecycle spawn kiosk "$HERE/kiosk.py" python3 -u "$HERE/kiosk.py" "$KEY_FILE" || exit 1
sleep 12; tail -2 $LOG/kiosk.log
lifecycle ready || exit 1
echo "ready. Stop with robot/stop.sh"
