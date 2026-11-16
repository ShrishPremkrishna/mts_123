#!/usr/bin/env bash
# Put the learned header-kick policy in the square-button slot (neural_controller_student)
# and rebuild, without launching. Run ON THE ROBOT, with the team's go: install.py
# reinstalls the course config, launch file, controller sources and all policies from
# DEPLOY_DIR into the monorepo and rebuilds. The previous slot policy is kept as a backup.
#
#   ./install_learned_policy.sh ~/kick/policies/header_kick_cpu_run2.json
#   DEPLOY_DIR=/other/pupper_gait_deploy ./install_learned_policy.sh POLICY_JSON
set -euo pipefail
POLICY=$(realpath "${1:?usage: $0 POLICY_JSON}")
DEPLOY_DIR=${DEPLOY_DIR:-/home/pi/mts_123/pupper_gait_deploy}  # the Pi's team checkout
cd "$DEPLOY_DIR"
[[ -f install.py && -f download_policy.py ]] || { echo "not a pupper_gait_deploy folder: $DEPLOY_DIR" >&2; exit 1; }
if [[ ! -f robot_trims.yaml ]]; then
  echo "WARNING: $DEPLOY_DIR has no robot_trims.yaml, so the install applies no per-robot motor trims." >&2
  echo "Stop unless the team confirms this checkout is the one the robot is built from." >&2
fi
python3 download_policy.py --validate-only "$POLICY"
echo ">>> python3 install.py --dry-run"
python3 install.py --dry-run
read -r -p "Install the above from $DEPLOY_DIR and rebuild? [y/N] " answer
[[ "$answer" == "y" ]] || { echo "nothing changed"; exit 1; }
if [[ -f policies/test_policy.json ]]; then
  backup="policies/test_policy.json.bak.$(date +%Y%m%d-%H%M%S)"
  cp policies/test_policy.json "$backup"
  echo "previous square-slot policy saved as $DEPLOY_DIR/$backup"
fi
cp "$POLICY" policies/test_policy.json
python3 install.py
echo "Done. Restart the course stack (it decides at launch whether the square controller exists), then:"
echo "  ros2 control list_controllers   # neural_controller_student must be inactive"
