#!/usr/bin/env bash
# Download a policy.json from W&B and validate it.
#   ./get_policy.sh bumpy|flat|<entity>/<project>/<run-id> [out.json]
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
case "$1" in
  bumpy) RUN=photon_mz-stanford-university/mjlab/23pt1499 ;;
  flat)  RUN=photon_mz-stanford-university/mjlab/13m4vcia ;;
  "")    echo "usage: $0 bumpy|flat|<entity>/<project>/<run-id> [out.json]" >&2; exit 2 ;;
  *)     RUN=$1 ;;
esac
OUT=${2:-$HOME/policies/$(basename "$RUN").json}
mkdir -p "$(dirname "$OUT")"
python3 "$REPO/pupper_gait_deploy/download_policy.py" "$RUN" --output "$OUT"
sha256sum "$OUT"
