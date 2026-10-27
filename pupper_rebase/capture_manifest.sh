#!/bin/bash
# Record a Pupper's software state into manifests/<name>/ so two images can be compared.
#   ./capture_manifest.sh <ssh-host> <name>      e.g. ./capture_manifest.sh pupper test-image
set -euo pipefail
export LC_ALL=C
HOST=${1:?ssh host}; NAME=${2:?manifest name}
OUT="$(dirname "$0")/manifests/$NAME"; mkdir -p "$OUT/conf"
MONO=/home/pi/pupperv3-monorepo

ssh "$HOST" 'bash -s' > "$OUT/overview.txt" 2>&1 <<'REMOTE'
echo "== os"; cat /etc/os-release | head -4; uname -r; cat /proc/device-tree/model; echo
echo "== image"; cat /etc/rpi-issue 2>/dev/null | head -1
echo "== hailo"; hailortcli --version 2>&1 | head -1
echo "== python"; python3 --version
echo "== enabled services"; systemctl list-unit-files --state=enabled --no-pager --no-legend | grep -v "@" | awk '{print $1}'
echo "== custom units"; ls -l /etc/systemd/system | grep -E "\.service" | awk '{print $9, $10, $11}'
echo "== home git repos"
for d in ~/*/; do if [ -d "$d/.git" ]; then echo "$(basename $d) $(git -C $d branch --show-current) $(git -C $d rev-parse --short HEAD) dirty=$(git -C $d status --porcelain | wc -l) $(git -C $d remote get-url origin 2>/dev/null)"; fi; done
true
REMOTE

ssh "$HOST" 'dpkg-query -W -f="\${Package}\t\${Version}\n"' | LC_ALL=C sort > "$OUT/dpkg.tsv"
ssh "$HOST" 'pip list --format=freeze 2>/dev/null' | LC_ALL=C sort > "$OUT/pip.txt"
ssh "$HOST" 'for f in /opt/ros/jazzy/share/*/package.xml; do echo -e "$(basename $(dirname $f))\t$(grep -m1 -oP "(?<=<version>)[^<]+" $f)"; done' | LC_ALL=C sort > "$OUT/ros_pkgs.tsv"
ssh "$HOST" "cd $MONO && echo \"\$(git branch --show-current) \$(git rev-parse HEAD)\" && git log -1 --format='%ad %s' --date=short" > "$OUT/monorepo_head.txt"
ssh "$HOST" "cd $MONO && git status --porcelain" | LC_ALL=C sort > "$OUT/monorepo_status.txt"
ssh "$HOST" "cd $MONO && git diff" > "$OUT/monorepo_uncommitted.patch"
ssh "$HOST" "cd $MONO && find . -path ./.git -prune -o -path ./ros2_ws/build -prune -o -path ./ros2_ws/install -prune -o -path ./ros2_ws/log -prune -o -path ./pupper-rs/target -prune -o -path '*/node_modules' -prune -o -type f -print0 | xargs -0 sha1sum" | LC_ALL=C sort -k2 > "$OUT/monorepo_files.sha1"
ssh "$HOST" 'mkdir -p /tmp/pupper_conf && cd /tmp/pupper_conf && cp /boot/firmware/config.txt /boot/firmware/cmdline.txt . 2>/dev/null; rm -rf units && mkdir units && for f in /etc/systemd/system/*.service; do cp -L "$f" units/ 2>/dev/null; done; cp ~/.bashrc bashrc 2>/dev/null; true'
rsync -a "$HOST":/tmp/pupper_conf/ "$OUT/conf/"
echo "Saved manifest to $OUT"; wc -l "$OUT"/*.t* "$OUT"/*.sha1 | tail -1
