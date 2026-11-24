#!/usr/bin/env bash
# stage.sh <host> [--dry-run] — copy final_project/fc/robot/ to <host>:~/fc-v1/robot/ and print what changed.
# Runs on the laptop. Touches only ~/fc-v1/robot/ on the Pi. vendor/ (archived copy of Tej's u6
# sources, already on the Pi), provision/ and sim/ (laptop-only) and caches are not staged.
# Also places the link auth files the bridge needs (both kept across re-stages):
#   ~/fc-v1/robot/link.key  (0600) copied from fc/config/link.key (python -m fc.robot.keygen)
#   ~/fc-v1/robot/robot_id  the id of the robots.json entry whose "host" is <host> (or $FC_ROBOT_ID)
set -euo pipefail
host=${1:?usage: stage.sh <host> [--dry-run]}
dry=${2:-}
src=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/
cfg=$src../config
key=$cfg/link.key
[ -f "$key" ] || { echo "missing $key: run 'python -m fc.robot.keygen' from final_project/ first" >&2; exit 1; }
[ -z "$(find "$key" -perm -g=r -o -perm -o=r -o -perm -g=w -o -perm -o=w 2>/dev/null)" ] ||
  { echo "$key is group/other accessible; chmod 600 it" >&2; exit 1; }
robot_id=${FC_ROBOT_ID:-$(python3 -c 'import json,sys; m=[r["id"] for r in json.load(open(sys.argv[1])) if r.get("host")==sys.argv[2]]; print(m[0] if len(m)==1 else "")' "$cfg/robots.json" "$host")}
[[ "$robot_id" =~ ^[A-Za-z0-9_.-]{1,32}$ ]] ||
  { echo "no unique robots.json entry with host '$host'; set its host or pass FC_ROBOT_ID=<id>" >&2; exit 1; }
ssh "$host" 'mkdir -p ~/fc-v1/robot && chmod 700 ~/fc-v1'
echo "staging $src -> $host:~/fc-v1/robot/ as robot '$robot_id' ${dry:+(dry run)}"
changes=$(rsync -rlptz --delete --itemize-changes ${dry:+--dry-run} \
  --exclude vendor/ --exclude __pycache__/ --exclude .pytest_cache/ --exclude '*.pyc' \
  --exclude /link.key --exclude /robot_id --exclude /provision/ --exclude /sim/ \
  "$src" "$host:fc-v1/robot/")
if [ -z "$changes" ]; then echo "no changes"; else echo "$changes"; fi
[ -n "$dry" ] && exit 0
# Key goes over stdin (never argv); written 0600 via umask, then renamed into place.
ssh "$host" "umask 077 && cd ~/fc-v1/robot && cat > link.key.tmp && chmod 600 link.key.tmp && mv link.key.tmp link.key &&
  printf '%s\n' '$robot_id' > robot_id" < "$key"
ssh "$host" 'chmod +x ~/fc-v1/robot/fcctl.sh ~/fc-v1/robot/drive_bridge.py && cd ~/fc-v1/robot &&
  echo "robot_id $(cat robot_id), link.key mode $(stat -c %a link.key)" && sha256sum drive_bridge.py fcctl.sh profiles/*.sh'
