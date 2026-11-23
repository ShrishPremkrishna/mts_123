#!/usr/bin/env bash
# Ball-kick trial release (2026-11-18): a NEW folder = r6 release + branch tej/gemini-ball-kick overlay.
# Writes only the new folder. r6, optlab3 and the native install are not changed. No process is started.
set -euo pipefail
B=/home/pi/pupper-foundation-session-20261029
R6=$B/software-lab6-20261103-r6
NEW=$B/software-ballkick-20261118
IN=/tmp/ballkick
test ! -e "$NEW" || { echo "refuse: $NEW exists"; exit 1; }
cp -a "$R6" "$NEW"
tar -xzf "$IN/ballkick-overlay.tar.gz" -C "$NEW" --no-same-owner
while read -r p; do
  [ -n "$p" ] || continue
  if [ -e "$NEW/$p" ]; then mkdir -p "$NEW/parked/$(dirname "$p")"; mv "$NEW/$p" "$NEW/parked/$p"; fi
done < "$IN/ballkick-remove.txt"
rmdir "$NEW/gemini_pupper/tricks/secret_dance" 2>/dev/null || true
cp "$IN/ballkick-app-manifest.json" "$IN/ballkick-telemetry-topics.json" "$IN/ballkick-prepare.sh" "$IN/ballkick-install.sh" "$NEW/"
{ echo "Ball-kick trial overlay on software-lab6-20261103-r6 (2026-11-18, Tej): branch tej/gemini-ball-kick, uncommitted, from b03d0c8."
  echo "Overlay files: $(tar -tzf "$IN/ballkick-overlay.tar.gz" | wc -l). Parked (moved, not deleted): see parked/."
  sha256sum "$IN/ballkick-overlay.tar.gz" "$NEW/ballkick-app-manifest.json"; } > "$NEW/BALLKICK-CHANGES.txt"
cd "$NEW" && /home/pi/pupper-foundation-session-20261029/venv/bin/python3 - <<'PY'
import json, hashlib
m = json.load(open('ballkick-app-manifest.json'))
bad = [e['path'] for e in m['final_selected_files']
       if hashlib.sha256(open(e['path'], 'rb').read()).hexdigest() != e['sha256']]
print('selected files', len(m['final_selected_files']), 'mismatched', bad)
PY
echo "installed $NEW"
