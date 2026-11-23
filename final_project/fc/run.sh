#!/usr/bin/env bash
# Pupper FC laptop app launcher: uses <repo>/.venv-fc (made from fc/requirements.txt if missing).
#
#   fc/run.sh                         # sim, Play screen at http://127.0.0.1:8800/  (ops: /ops)
#   fc/run.sh camera --robots blue --udp-host blue=10.19.203.20
#   fc/run.sh camera --video synthetic --no-link --dry-run-ops    # offline rehearsal
#
# Extra arguments go to `python -m fc.app`. FC_PYTHON=/path/to/python3.12 picks the base interpreter.
set -euo pipefail
FC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname "$FC_DIR")"                       # final_project/
REPO="$(git -C "$PROJECT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null | sed 's#/\.git$##')"
[ -n "$REPO" ] || REPO="$(dirname "$PROJECT")"
VENV="${FC_VENV:-$REPO/.venv-fc}"                    # shared by every worktree of this repo
REQ="$FC_DIR/requirements.txt"

if [ ! -x "$VENV/bin/python" ]; then
  base="${FC_PYTHON:-$(command -v python3.12 || true)}"
  [ -n "$base" ] || { echo "Python 3.12 not found: install it or set FC_PYTHON" >&2; exit 1; }
  echo "creating $VENV with $base" >&2
  "$base" -m venv "$VENV"
fi
# install/repair when a pinned package is missing or at another version
if ! "$VENV/bin/python" - "$REQ" <<'PY'
import importlib.metadata as m, sys
for line in open(sys.argv[1]):
    line = line.split("#")[0].strip()
    if "==" in line:
        name, ver = line.split("==")
        try:
            assert m.version(name) == ver
        except Exception:
            sys.exit(1)
PY
then
  echo "installing $REQ into $VENV" >&2
  "$VENV/bin/python" -m pip install --disable-pip-version-check -q -r "$REQ"
fi

source="sim"
case "${1:-}" in
  sim|camera) source="$1"; shift ;;
esac
cd "$PROJECT"
exec "$VENV/bin/python" -m fc.app --source "$source" "$@"
