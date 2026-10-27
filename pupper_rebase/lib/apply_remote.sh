#!/bin/bash
# Runs ON the Pupper (copied to /tmp/cs123-rebase by rebase.sh).
# Arguments: [--dry-run] [--skip-wifi-pairing]
set -euo pipefail
DRY=0; SKIP_WIFI=0
for a in "$@"; do case $a in --dry-run) DRY=1;; --skip-wifi-pairing) SKIP_WIFI=1;; esac; done

BASE=b5b8c89ec5c71b18fe8bb3d3fa27d492441644b3     # class image: main, 2025-10-18
TARGET=0e50afbdc11ae066606b0290c23969f46a265626   # cs123-build: main 6f96c5e + wifi-pairing-mode 64c86d7
MONO=/home/pi/pupperv3-monorepo
A=/tmp/cs123-rebase
PIP_PKGS="fastapi==0.141.1 uvicorn==0.54.0 python-multipart==0.0.32 pyzbar==0.1.9 viser==1.0.30 yourdfpy==0.0.60"

run()  { echo "  + $*"; if [ $DRY = 0 ]; then bash -c "$*"; fi; }
step() { echo; echo "== $*"; }
die()  { echo "ABORT: $*" >&2; exit 1; }

step "1/9 preflight"
sudo -n true 2>/dev/null || die "passwordless sudo is required"
HEAD=$(git -C $MONO rev-parse HEAD)
case $HEAD in $BASE|$TARGET) echo "  monorepo at ${HEAD:0:7}";; *) die "monorepo is at ${HEAD:0:7}, expected ${BASE:0:7} or ${TARGET:0:7}";; esac
# Local changes we expect and keep: pupper_gait_deploy's controller, LFS artefacts, the common/ sources.
UNEXPECTED=$(git -C $MONO status --porcelain | grep -vE "ros2_ws/src/neural_controller/|infra/pupper_image_builder/resources/hailort|ros2_ws/src/common/|hailort.log" || true)
[ -z "$UNEXPECTED" ] || die "unexpected local changes in the monorepo:
$UNEXPECTED"
curl -fsS -m 10 -o /dev/null https://pypi.org/simple/ || die "no internet (pypi.org unreachable)"
echo "  sudo ok, internet ok, $(df -h / | awk 'NR==2{print $4}') free"

# The class image ships /home/pi/.config owned by root, which breaks apps that save settings there.
if [ "$(stat -c %U /home/pi/.config 2>/dev/null)" != pi ]; then run "sudo chown pi:pi /home/pi/.config"; fi

step "2/9 wipe API keys"
bash $A/wipe_keys_remote.sh $( [ $DRY = 1 ] && echo --dry-run )

step "3/9 monorepo to cs123-build (${TARGET:0:7})"
if [ "$HEAD" = "$TARGET" ]; then echo "  already there"; else
  run "git -C $MONO fetch -q $A/monorepo-cs123-build.bundle cs123-build:cs123-build"
  # Skip LFS downloads: only the bag recordings are LFS; the trick CSVs are plain files.
  run "GIT_LFS_SKIP_SMUDGE=1 git -C $MONO checkout -q cs123-build"
fi

step "4/9 system and Python packages"
run "sudo apt-get update -qq && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq libzbar0"
run "sudo pip install -q --break-system-packages $PIP_PKGS"

step "5/9 rebuild changed ROS packages (animation_controller_py, hailo)"
run "cd $MONO/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select animation_controller_py hailo --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 2>&1 | tail -3"

step "6/9 screen app (pupper-gui)"
BIN=$MONO/pupper-rs/target/aarch64-unknown-linux-gnu/release/pupper-rs
run "mkdir -p $(dirname $BIN) && install -m 0755 $A/pupper-rs $BIN"
if ! systemctl cat pupper-gui.service >/dev/null 2>&1; then run "bash $MONO/pupper-rs/install_service.sh"; fi
run "sudo systemctl daemon-reload && sudo systemctl enable -q pupper-gui.service && sudo systemctl restart pupper-gui.service"

step "7/9 Tailscale (installed, not logged in)"
if command -v tailscale >/dev/null; then echo "  already installed: $(tailscale version | head -1)"
else run "curl -fsSL https://tailscale.com/install.sh | sh >/dev/null"; fi
echo "  students log in with their own account: sudo tailscale up"

step "8/9 Claude Code (installed, not logged in)"
if [ -x /home/pi/.local/bin/claude ]; then echo "  already installed: $(/home/pi/.local/bin/claude --version 2>/dev/null | head -1)"
else run "curl -fsSL https://claude.ai/install.sh | bash >/dev/null"; fi
echo "  students sign in with their own account on first run: claude"

step "9/9 WiFi pairing (comitup)"
if [ $SKIP_WIFI = 1 ]; then echo "  skipped"
else
  # Runs detached: restarting comitup can drop WiFi for a moment and cut this SSH session.
  run "setsid nohup bash $MONO/infra/wifi-pairing/install_wifi_pairing.sh > /home/pi/cs123-wifi-pairing.log 2>&1 < /dev/null &"
  echo "  started in the background; log: ~/cs123-wifi-pairing.log"
fi

[ $DRY = 1 ] || echo "cs123-build ${TARGET} applied $(date -Iseconds)" > /home/pi/.cs123-build
echo; echo "apply done$( [ $DRY = 1 ] && echo ' (dry run: nothing changed)')"
