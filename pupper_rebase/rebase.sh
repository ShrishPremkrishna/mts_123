#!/bin/bash
# Bring a class Pupper (student image) up to the CS 123 build.
#   ./rebase.sh <ssh-host> [--dry-run] [--skip-wifi-pairing]
# Steps: wipe API keys, monorepo to cs123-build, packages, ROS rebuild of the two
# changed packages, screen app, Tailscale and Claude Code (both logged out),
# WiFi pairing. Then verify
# and record a manifest in manifests/<host>-after/.
set -euo pipefail
HOST=${1:?ssh host}; shift
HERE=$(cd "$(dirname "$0")" && pwd)
for f in artifacts/monorepo-cs123-build.bundle artifacts/pupper-rs lib/apply_remote.sh lib/wipe_keys_remote.sh lib/verify_remote.sh; do
  [ -f "$HERE/$f" ] || { echo "missing $f (run ./prepare.sh first)"; exit 1; }
done
echo ">>> copying artifacts to $HOST"
ssh "$HOST" "mkdir -p /tmp/cs123-rebase"
rsync -a "$HERE/artifacts/" "$HERE/lib/" "$HOST":/tmp/cs123-rebase/
echo ">>> applying"
ssh "$HOST" "bash /tmp/cs123-rebase/apply_remote.sh $*"
case " $* " in *" --dry-run "*) exit 0;; esac
case " $* " in *" --skip-wifi-pairing "*) ;; *)
  echo ">>> waiting for WiFi pairing install (the connection may drop briefly)"
  sleep 20
  for i in $(seq 1 30); do ssh -o ConnectTimeout=5 "$HOST" "pgrep -f '[i]nstall_wifi_pairing.sh'" >/dev/null 2>&1 || break; sleep 5; done
  ssh "$HOST" "tail -3 ~/cs123-wifi-pairing.log" || true ;;
esac
echo ">>> verifying"
ssh "$HOST" "bash /tmp/cs123-rebase/verify_remote.sh" || { echo "VERIFY FAILED"; exit 1; }
"$HERE/capture_manifest.sh" "$HOST" "$HOST-after" >/dev/null && echo ">>> manifest saved to manifests/$HOST-after"
