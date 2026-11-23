#!/usr/bin/env bash
# Starts the Pupper FC ball detector (the teammates' release, read only) in the environment of the kickseen session.
set -euo pipefail
eval "$(grep "^export " /home/pi/pupper-foundation-session-20261029/kickseen-kit-20261120/kickseen-session.sh)"
cd /home/pi/pupper-foundation-session-20261029/kickseen-kit-20261120/fcdet-run
exec /usr/bin/python3 -B -s /home/pi/pupper-fc-releases/fc-gamepad-free-ef65bb7af26f/follow_me_pupper/robot/viser_camera.py --source ros --detect --publish-detections --fc-headless --model /home/pi/pupper-fc-runs/front-camera-20261117-a/inputs/yolov8n_seg.hef
