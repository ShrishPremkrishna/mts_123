#!/usr/bin/env bash
# stage.sh <host> — copy final_project/fc/robot/ to <host>:~/fc-v1/robot/ and print what changed.
# Runs on the laptop. Touches only ~/fc-v1/robot/ on the Pi. vendor/ (archived copy of Tej's u6
# sources, already on the Pi) and caches are not staged.
set -euo pipefail
host=${1:?usage: stage.sh <host> [--dry-run]}
dry=${2:-}
src=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/
ssh "$host" 'mkdir -p ~/fc-v1/robot && chmod 700 ~/fc-v1'
echo "staging $src -> $host:~/fc-v1/robot/ ${dry:+(dry run)}"
changes=$(rsync -rlptz --delete --itemize-changes ${dry:+--dry-run} \
  --exclude vendor/ --exclude __pycache__/ --exclude .pytest_cache/ --exclude '*.pyc' \
  "$src" "$host:fc-v1/robot/")
if [ -z "$changes" ]; then echo "no changes"; else echo "$changes"; fi
[ -n "$dry" ] || ssh "$host" 'chmod +x ~/fc-v1/robot/fcctl.sh ~/fc-v1/robot/drive_bridge.py && cd ~/fc-v1/robot && sha256sum drive_bridge.py fcctl.sh profiles/*.sh'
