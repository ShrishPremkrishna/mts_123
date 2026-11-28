#!/usr/bin/env bash
# Read-only: compare final_project/fc/robot (working tree) with mts:~/fc-v1/robot. No output + "MATCH" = identical.
# usage: bash sha_check.sh [host]   (run after fc/robot/stage.sh <host>)
set -euo pipefail
host=${1:-mts}
cd /Users/shrish/workspace/cs123/final_project/fc/robot
F="-type f ! -path './vendor/*' ! -path './provision/*' ! -path './sim/*' ! -name '*.pyc' ! -path '*/__pycache__/*' ! -path './.pytest_cache/*' ! -name link.key ! -name robot_id"
diff <(eval find . $F | LC_ALL=C sort | xargs shasum -a 256) \
     <(ssh -o BatchMode=yes -o ConnectTimeout=8 "$host" "cd ~/fc-v1/robot && find . $F | LC_ALL=C sort | xargs sha256sum") && echo "MATCH (files checked: $(eval find . $F | wc -l | tr -d ' '))"
