#!/usr/bin/env bash
# Camera kick release u6 (2026-11-20) = u5 + lab/ball_eye.py, lab/ball_lineup.py, lab/part10_kick_seen.py and
# robot/tool_registry.py (tools kick_seen and look_ball). Files only: no API, controller or motor action.
set -euo pipefail
B=${KICKSEEN_BASE:-/home/pi/pupper-foundation-session-20261029}
KIT=$(cd "$(dirname "$0")" && pwd)
OLDKIT=$B/kickfast-kit-20261119
U5=$B/software-ballkick-20261119-u5
U6=$B/software-ballkick-20261120-u6
PY=${KICKSEEN_PY:-$B/venv/bin/python3}
MSHA=eeaa21e52401cd001b44da22db69d9b40bb774618d4fa7406d05b7e178a87b95
OLD_MSHA=243baf716608d713fbaea1a797c9a932e62bfd08ea8263f52d9a6dbc8aac7a90
RUN=${KICKSEEN_RUN:-20261120-kickseen1}
export PYTHONDONTWRITEBYTECODE=1
test -d "$U5" || { echo "missing $U5" >&2; exit 1; }
test ! -e "$U6" || { echo "$U6 exists: not changed" >&2; exit 1; }
test ! -e "$B/supervised-console-runs/$RUN" || { echo "run $RUN exists" >&2; exit 1; }
grep -q "$OLD_MSHA" "$OLDKIT/kickfast4-prepare.sh" || { echo "kickfast4-prepare.sh has no manifest pin to replace" >&2; exit 1; }
echo "$MSHA  $KIT/u6-app-manifest.json" | sha256sum -c -
cp -a "$U5" "$U6"
cp -R "$KIT/files/." "$U6/"
cp "$KIT/u6-app-manifest.json" "$U6/ballkick-app-manifest.json"
cp "$KIT/u6-app-manifest.json" "$U6/app-stage-manifest.json"
for s in prepare session; do
  sed -e 's|software-ballkick-20261119-u5|software-ballkick-20261120-u6|g' -e "s|20261119-kickfast4|$RUN|g" \
      -e "s|$OLD_MSHA|$MSHA|g" "$OLDKIT/kickfast4-$s.sh" > "$KIT/kickseen-$s.sh"
done
cd "$U6"
test -f gemini_pupper/robot/camera_params.yaml || { echo "no camera_params.yaml in u6" >&2; exit 1; }
"$PY" - <<'P'
import hashlib, json, os
m = json.load(open("ballkick-app-manifest.json"))
digest = lambda path: hashlib.sha256(open(path, "rb").read()).hexdigest() if os.path.isfile(path) else "missing"
bad = [e["path"] for e in m["final_selected_files"] if digest(e["path"]) != e["sha256"]]
print("u6 pinned files", len(m["final_selected_files"]), "mismatched", bad)
raise SystemExit(1 if bad else 0)
P
"$PY" - <<'P'
import sys
sys.path[:0] = ["gemini_pupper/lab", "gemini_pupper/robot"]
import cv2, numpy, yaml
import ball_eye, ball_lineup
lens = ball_eye.load_lens()
print("imports ok: cv2", cv2.__version__, "numpy", numpy.__version__, "lens", lens)
P
echo "u6 installed at $U6 (manifest $MSHA), scripts $KIT/kickseen-prepare.sh and kickseen-session.sh for run $RUN. Nothing started."
