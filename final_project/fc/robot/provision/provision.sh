#!/usr/bin/env bash
# provision.sh — make a reachable Pupper Pi game-ready for Pupper FC. Runs on the laptop.
#
#   provision.sh [--check] [--apply-os] [--fix-clone-identity NEW_HOSTNAME] [--yes] <ssh-host> <robot-id> <team> <tag>
#
#   --check        inspect only: read-only probe + readiness report + the Pi OS items below (ok/missing).
#                  Writes nothing anywhere.
#   (default)      probe, fill this robot's empty `host` in fc/config/robots.json, stage fc/robot via
#                  stage.sh (rsync into ~/fc-v1/robot, link.key 0600, robot_id), write ~/fc-v1/identity.json,
#                  probe again and print the readiness report. On the Pi it touches only ~/fc-v1/.
#   --fix-clone-identity NEW_HOSTNAME
#                  ALSO change system files on a card cloned from another robot (needs passwordless sudo):
#                  hostname + /etc/hosts, a new machine-id, new ssh host keys, drop the copied Tailscale node
#                  state (never `tailscale logout`: that would log out the source robot too), clear the
#                  copied ~/fc-v1/run state. Done once (marker /var/lib/fc-provision/identity). Reboot after.
#   --apply-os     ALSO, after staging, apply the Pi OS/driver changes mts has and a fresh card lacks
#                  (provision/os_items.sh: Hailo off, fan curve, pupper-gui off, GT911 touch overlays off,
#                  v4 IMU driver built from this card's source + linked). Prints each missing item, what it
#                  will change and its undo, then asks you to type the Pi's hostname. Boot changes need a
#                  reboot (printed). Never copies imu_cal.json: run `fcctl.sh imu-level` on each robot.
#   --yes          do not ask before --fix-clone-identity or --apply-os.
#
# No default host: the target is only ever the <ssh-host> you type. robots.json guards against
# provisioning one robot's identity onto another (host/tag/team already owned by a different id).
# Exit: 0 ready, 1 not ready (see report), 2 usage/config refusal, 3 unreachable.
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
robot_dir=$(cd "$here/.." && pwd)
cfg=$(cd "$robot_dir/../config" && pwd)
robots=$cfg/robots.json
py=${PYTHON:-python3}
report() { "$py" "$here/report.py" "$@"; }

check=0 fix_host="" yes=0 apply_os=0
args=()
while [ $# -gt 0 ]; do
  case $1 in
    --check) check=1 ;;
    --fix-clone-identity) fix_host=${2:?--fix-clone-identity needs NEW_HOSTNAME}; shift ;;
    --apply-os) apply_os=1 ;;
    --yes) yes=1 ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    --*) echo "unknown option $1" >&2; exit 2 ;;
    *) args+=("$1") ;;
  esac
  shift
done
[ ${#args[@]} -eq 4 ] || { sed -n '4p' "$0" >&2; exit 2; }
host=${args[0]} rid=${args[1]} team=${args[2]} tag=${args[3]}
[[ "$tag" =~ ^[0-9]+$ ]] || { echo "tag must be an integer" >&2; exit 2; }
if [ -n "$fix_host" ]; then
  [ $check -eq 0 ] || { echo "--check and --fix-clone-identity exclude each other" >&2; exit 2; }
  [[ "$fix_host" =~ ^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$ ]] || { echo "bad hostname '$fix_host'" >&2; exit 2; }
fi
if [ $apply_os -eq 1 ]; then
  [ $check -eq 0 ] && [ -z "$fix_host" ] || { echo "--apply-os excludes --check and --fix-clone-identity" >&2; exit 2; }
fi

# 1. laptop config: robots.json must agree with the arguments (refuses cross-robot mix-ups).
action=$(report config --robots "$robots" --robot-id "$rid" --team "$team" --tag "$tag" --host "$host") || exit 2

# Host-key fingerprints the laptop already trusts for the OTHER robots (from known_hosts only; their hosts
# are never contacted). A target presenting one of them is an uncorrected clone.
fp_args=()
while IFS=$'\t' read -r oid ohost; do
  [ -n "$ohost" ] || continue
  addr=$(ssh -G "$ohost" 2>/dev/null | awk '$1=="hostname"{print $2; exit}')
  fp=$(ssh-keygen -F "${addr:-$ohost}" 2>/dev/null | awk '!/^#/ && $2=="ssh-ed25519"{print $2" "$3; exit}' |
       ssh-keygen -lf - 2>/dev/null | awk '{print $2}' || true)
  [ -n "$fp" ] && fp_args+=(--known-hosts-fp "$oid=$fp")
done < <("$py" -c 'import json,sys; [print(r["id"], r.get("host",""), sep="\t") for r in json.load(open(sys.argv[1])) if r["id"]!=sys.argv[2]]' "$robots" "$rid")

tmp=$(mktemp -d "${TMPDIR:-/tmp}/fc-provision.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
probe() {
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$host" bash -s < "$here/probe.sh" > "$tmp/probe.txt" 2> "$tmp/probe.err" || {
    echo "cannot reach '$host' over ssh (BatchMode; needs your key on the Pi: ssh-copy-id $host):" >&2
    sed 's/^/  /' "$tmp/probe.err" >&2; exit 3; }
}
# Pi OS items (os_items.sh runs on the Pi; check/plan are read-only, like probe.sh).
os_items() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$host" bash -s -- "$@" < "$here/os_items.sh"; }
os_section() {
  echo "== Pi OS items vs mts (fc/robot/README.md; not part of READY; apply with --apply-os)"
  os_items "$@" || true
}
show() {
  report report --robots "$robots" --robot-id "$rid" --team "$team" --tag "$tag" --host "$host" \
    --probe "$tmp/probe.txt" --local-dir "$robot_dir" ${fp_args[@]+"${fp_args[@]}"}
}

echo "== probe $host (read-only) as $rid / team $team / tag $tag"
probe
if [ $check -eq 1 ]; then
  [ "$action" = fill-host ] && echo "(robots.json: $rid.host is empty; a full run fills it with '$host')"
  set +e; show; rc=$?; set -e
  os_section check
  exit $rc
fi

# 2. optional: give a cloned card its own identity (system files; explicit flag only).
if [ -n "$fix_host" ]; then
  grep -q '^sudo_nopass=yes' "$tmp/probe.txt" || { echo "--fix-clone-identity needs passwordless sudo on $host" >&2; exit 2; }
  if [ $yes -eq 0 ]; then
    echo "About to change SYSTEM identity on $host ($(grep '^hostname=' "$tmp/probe.txt" | cut -d= -f2), lan" \
         "$(grep '^lan_ips=' "$tmp/probe.txt" | cut -d= -f2)) -> hostname '$fix_host', new machine-id, new ssh host keys," \
         "Tailscale state dropped. Type the new hostname to continue:"
    read -r answer
    [ "$answer" = "$fix_host" ] || { echo "not confirmed; nothing changed" >&2; exit 2; }
  fi
  ssh -o BatchMode=yes "$host" sudo -n bash -s -- "$fix_host" <<'REMOTE'
set -euo pipefail
new=$1
marker=/var/lib/fc-provision/identity
if [ -f "$marker" ] && grep -qx "$new" "$marker"; then echo "identity already fixed as $new (marker)"; exit 0; fi
if pgrep -f '[r]os2_control_node|[d]rive_bridge|[p]ersistent_session.py' >/dev/null; then
  echo "a robot stack is running; stop it first (fcctl.sh stop)" >&2; exit 1; fi
old=$(hostname)
hostnamectl set-hostname "$new"
if grep -q '^127\.0\.1\.1' /etc/hosts; then sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$new/" /etc/hosts
else printf '127.0.1.1\t%s\n' "$new" >> /etc/hosts; fi
rm -f /etc/machine-id && systemd-machine-id-setup >/dev/null
[ -L /var/lib/dbus/machine-id ] || { rm -f /var/lib/dbus/machine-id; ln -s /etc/machine-id /var/lib/dbus/machine-id; }
# New host keys are generated aside and swapped in only when complete (never leave sshd keyless).
kd=$(mktemp -d) && mkdir -p "$kd/etc/ssh" && ssh-keygen -A -f "$kd" >/dev/null
[ -s "$kd/etc/ssh/ssh_host_ed25519_key" ] || { echo "host key generation failed; keys unchanged" >&2; exit 1; }
rm -f /etc/ssh/ssh_host_* && mv "$kd"/etc/ssh/ssh_host_* /etc/ssh/ && rm -rf "$kd"
if [ -d /var/lib/tailscale ]; then
  systemctl stop tailscaled 2>/dev/null || true
  rm -f /var/lib/tailscale/tailscaled.state
  systemctl start tailscaled 2>/dev/null || true
fi
home=$(getent passwd pi | cut -d: -f6)
[ -n "$home" ] && rm -rf "$home/fc-v1/run"
mkdir -p "$(dirname "$marker")" && echo "$new" > "$marker"
echo "identity: $old -> $new; new machine-id + ssh host keys (active after reboot); tailscale state cleared"
REMOTE
  cat <<EOF
Clone identity fixed. Now, in order:
  1. ssh $host sudo reboot      (new host keys + machine-id take effect; DHCP may give a new LAN IP)
  2. On the laptop: ssh-keygen -R <old address>   (the host key changed, so ssh will warn once)
  3. ssh <new address> sudo tailscale up --hostname=$fix_host   (browser login, your account)
  4. Add an ssh alias for $fix_host in ~/.ssh/config, then rerun: $0 <alias> $rid $team $tag
EOF
  exit 0
fi

# 3. laptop robots.json: fill this robot's empty host (git-tracked, reviewable).
if [ "$action" = fill-host ]; then report set-host --robots "$robots" --robot-id "$rid" --host "$host"; fi

# 4. stage fc/robot (+ link.key, robot_id) with the maintained stage.sh; touches only ~/fc-v1/robot.
[ -f "$cfg/link.key" ] || { echo "no $cfg/link.key: run 'python -m fc.robot.keygen' once (from final_project/), then re-stage every robot" >&2; exit 2; }
FC_ROBOT_ID=$rid "$robot_dir/stage.sh" "$host"

# 5. ~/fc-v1/identity.json: who this Pi is and which laptop commit provisioned it (outside robot/, so a
#    later stage.sh --delete keeps it).
commit=$(git -C "$robot_dir" rev-parse --short HEAD 2>/dev/null || echo unknown)
dirty=$({ git -C "$robot_dir" status --porcelain -- . 2>/dev/null || true; } | wc -l | tr -d ' ')
"$py" -c 'import json,sys,time; print(json.dumps({"robot_id":sys.argv[1],"team":sys.argv[2],"tag":int(sys.argv[3]),
  "ssh_host":sys.argv[4],"laptop_commit":sys.argv[5],"laptop_dirty_files":int(sys.argv[6]),
  "provisioned_utc":time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime())}, sort_keys=True))' \
  "$rid" "$team" "$tag" "$host" "$commit" "$dirty" |
  ssh -o BatchMode=yes "$host" 'umask 077 && cat > ~/fc-v1/identity.json.tmp && mv ~/fc-v1/identity.json.tmp ~/fc-v1/identity.json'

# 5b. optional: Pi OS/driver items (system files; explicit flag, owner approves each run).
if [ $apply_os -eq 1 ]; then
  echo "== Pi OS items on $host: plan (read-only)"
  if os_items plan; then echo "(nothing to apply)"
  else
    grep -q '^sudo_nopass=yes' "$tmp/probe.txt" || echo "(note: no passwordless sudo on $host: hailo/fan/gui/touch will be refused)"
    if [ $yes -eq 0 ]; then
      pi_host=$(grep '^hostname=' "$tmp/probe.txt" | cut -d= -f2)
      echo "About to change the items above on $host (hostname '$pi_host'). Type that hostname to continue:"
      read -r answer
      [ -n "$pi_host" ] && [ "$answer" = "$pi_host" ] || { echo "not confirmed; nothing changed" >&2; exit 2; }
    fi
    patch_sha=$(shasum -a 256 "$robot_dir/imu/stock-imu-filter.patch" 2>/dev/null || sha256sum "$robot_dir/imu/stock-imu-filter.patch")
    echo "== Pi OS items on $host: apply"
    set +e; os_items apply "${patch_sha%% *}"; os_rc=$?; set -e
    [ $os_rc -eq 0 ] || echo "(some Pi OS items were not applied; see above)"
  fi
fi

# 6. probe again and report.
echo "== readiness ($host)"
probe
set +e; show; rc=$?; set -e
os_section check
cat <<EOF

Next (operator, robot ON THE STAND): ssh $host ~/fc-v1/robot/fcctl.sh status
  then start stock --confirm (homes the legs), bridge start, and the calibration in PROVISION.md.
EOF
exit $rc
