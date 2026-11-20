#!/usr/bin/env bash
# Put a policy in the Square slot, install our controller into the monorepo, build.
#   ./install.sh <policy.json>        (MONO=~/pupperv3-monorepo by default)
set -eo pipefail
POLICY=$(realpath "${1:?usage: $0 <policy.json>}")
REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
DEPLOY=$REPO/pupper_gait_deploy
MONO=${MONO:-$HOME/pupperv3-monorepo}

python3 "$DEPLOY/download_policy.py" --validate-only "$POLICY"
cp "$POLICY" "$DEPLOY/policies/test_policy.json"
python3 "$DEPLOY/install.py" --monorepo "$MONO" --no-build

# Tests off: test_neural_controller_safety.cpp does not compile on the Pi (see 03-install.md).
cd "$MONO/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select neural_controller joy_utils \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_CXX_FLAGS="-g0"
echo "Square = $(sha256sum "$POLICY" | cut -c1-12)…  Next: ./launch.sh"
