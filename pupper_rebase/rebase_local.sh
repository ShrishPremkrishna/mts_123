#!/bin/bash
# Bring a class Pupper up to the CS 123 build, run ON the Pupper itself (no laptop SSH setup):
#   git clone https://github.com/cs123-stanford/pupper_rebase.git && cd pupper_rebase
#   ./rebase_local.sh [--dry-run] [--skip-wifi-pairing]
# Same steps and checks as rebase.sh. The work runs detached and logs to ~/cs123-rebase.log, so a
# dropped SSH session (the WiFi pairing step restarts networking) does not stop it: reconnect and
# run `tail -f ~/cs123-rebase.log`. The manifest capture is left to capture_manifest.sh on a PC.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=/home/pi/cs123-rebase.log
A=/tmp/cs123-rebase

[ "$(id -un)" = pi ] && [ -d /home/pi/pupperv3-monorepo ] || { echo "run this on the Pupper, as user pi"; exit 1; }
for f in artifacts/monorepo-cs123-build.bundle artifacts/pupper-rs lib/apply_remote.sh lib/wipe_keys_remote.sh lib/verify_remote.sh; do
  [ -f "$HERE/$f" ] || { echo "missing $f"; exit 1; }
done

if [ "${1:-}" != --detached ]; then
  setsid nohup bash "$HERE/$(basename "$0")" --detached "$@" > "$LOG" 2>&1 < /dev/null &
  echo "running in the background, log: $LOG"
  echo "if your connection drops, reconnect and run: tail -f $LOG"
  tail -n +1 -f --pid=$! "$LOG"
  [ "$(tail -1 "$LOG")" = "REBASE OK" ]
  exit
fi
shift

set +e
(
  set -e
  echo ">>> copying artifacts to $A"
  rm -rf $A && mkdir -p $A && cp -a "$HERE/artifacts/." "$HERE/lib/." $A/
  echo ">>> applying"
  bash $A/apply_remote.sh "$@"
  case " $* " in *" --dry-run "*) exit 0;; esac
  case " $* " in *" --skip-wifi-pairing "*) ;; *)
    echo ">>> waiting for WiFi pairing install (an SSH connection may drop briefly)"
    sleep 20
    for i in $(seq 1 30); do pgrep -f '[i]nstall_wifi_pairing.sh' >/dev/null || break; sleep 5; done
    tail -3 ~/cs123-wifi-pairing.log || true ;;
  esac
  echo ">>> verifying"
  bash $A/verify_remote.sh
)
rc=$?
[ $rc = 0 ] && echo "REBASE OK" || echo "REBASE FAILED"
