#!/usr/bin/env bash
# clone_live.sh — live file-level clone of THIS Pi's system onto a second USB drive plugged into it, with an
# offline identity reset, then a read-only verify. Runs on the source Pi (e.g. mts) as root; the source stays up.
# Nov 28: mts -> pupper-red, 25.6 GB in 13 min, booted first time, provision.sh --check passed.
#
#   sudo clone_live.sh --serial SERIAL --hostname NEW_HOST --robot-id ID [--dry-run] [--yes]
#                      [--exclude /PATTERN]... [--excludes-file FILE] [--log FILE] [--mount DIR]
#   clone_live.sh --print-excludes
#
#   --serial       target drive's serial (lsblk -dno SERIAL). Required: the target is only ever the one disk
#                  with this serial. Refused: no/several matches, the boot drive, the disk holding /, or any
#                  disk with a partition mounted anywhere (except a stale mount under --mount).
#   --hostname     new hostname (must differ from this Pi's). --robot-id: new ~/fc-v1/robot/robot_id (must differ).
#   --dry-run      print the plan (device, serial, model, sizes, excludes, exact rsync line) and change nothing.
#                  Needs no root, writes no log.
#   --exclude      add an rsync exclude (absolute, anchored at /). Repeatable.
#   --excludes-file  replace the default excludes (one per line, # comments). The mandatory ones always apply:
#                  pseudo filesystems, /mnt, /media, and ~/fc-v1/run/* (the IMU cal belongs to this Pi's chip).
#   --yes          do not ask before erasing the target (needed when stdin is not a terminal, e.g. nohup).
#   --log          log file (default /var/log/fc-clone-NEW_HOST.log). Exit code goes to LOG.done, pid to LOG.pid.
#
# Polling from another shell (do not `pgrep -f clone_live` over ssh: it matches the polling command itself):
#   ssh mts 'cat /var/log/fc-clone-pupper-red.log.done 2>/dev/null || tail -c 300 /var/log/fc-clone-pupper-red.log'
# Exit: 0 cloned and verified (or dry-run plan ok), 1 failed during copy/reset/verify, 2 refused/usage.
# Tests: fc/robot/tests/test_clone_live.py (guards + plan against fake lsblk/findmnt/df/du; no disk touched).
set -euo pipefail
set -f                            # no globbing: exclude patterns like /proc/* stay literal

ROOT=${FC_CLONE_ROOT:-/}          # source root. Tests point it at a temp tree; on a Pi leave it as /.
R=${ROOT%/}
PI_HOME=/home/pi
BOOT_SECTORS=1048576              # 512 MiB FAT32 boot partition at sector 8192, as on the stock image

MANDATORY_EXCLUDES="/proc/* /sys/* /dev/* /run/* /tmp/* /mnt/* /media/* /lost+found $PI_HOME/fc-v1/run/*"
DEFAULT_EXCLUDES="/swapfile /var/log/journal/* /var/tmp/*
$PI_HOME/pupper-foundation-session-20261029 $PI_HOME/pupper-fc-runs
$PI_HOME/.vscode-server $PI_HOME/.cursor-server $PI_HOME/fc-v1/runs/*"

die() { echo "REFUSE: $*" >&2; exit 2; }
usage() { sed -n '2,/^set -euo/p' "$0" 2>/dev/null | sed '$d;s/^# \{0,1\}//' >&2; exit 2; }
hb() { awk -v b="$1" 'BEGIN{split("B KB MB GB TB",u);i=1;while(b>=1000&&i<5){b/=1000;i++}
  if(i==1)printf "%d %s",b,u[i];else printf "%.1f %s",b,u[i]}'; }
part() { case $1 in *[0-9]) echo "${1}p$2" ;; *) echo "$1$2" ;; esac; }

serial="" newhost="" robot_id="" dry=0 yes=0 log="" mnt=/mnt/fc-clone excl_file="" extra=""
while [ $# -gt 0 ]; do
  case $1 in
    --serial) serial=${2-}; shift ;;
    --hostname) newhost=${2-}; shift ;;
    --robot-id) robot_id=${2-}; shift ;;
    --dry-run) dry=1 ;;
    --yes) yes=1 ;;
    --exclude) extra="$extra ${2-}"; shift ;;
    --excludes-file) excl_file=${2-}; shift ;;
    --log) log=${2-}; shift ;;
    --mount) mnt=${2-}; shift ;;
    --print-excludes) echo "# mandatory"; printf '%s\n' $MANDATORY_EXCLUDES; echo "# default"; printf '%s\n' $DEFAULT_EXCLUDES; exit 0 ;;
    -h|--help) usage ;;
    *) die "unknown argument '$1' (see --help)" ;;
  esac
  shift || die "missing value"
done

# ---- arguments (before anything else) ---------------------------------------------------------------
[ -n "$serial" ] || die "--serial is required: the target drive is chosen only by its serial (lsblk -dno NAME,SERIAL,MODEL)"
[ -n "$newhost" ] || die "--hostname is required"
[ -n "$robot_id" ] || die "--robot-id is required"
echo "$serial" | grep -Eq '^[A-Za-z0-9_.-]{4,64}$' || die "bad serial '$serial'"
echo "$newhost" | grep -Eq '^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$' || die "bad hostname '$newhost'"
echo "$robot_id" | grep -Eq '^[a-z0-9][a-z0-9_-]{0,31}$' || die "bad robot id '$robot_id'"
case $mnt in /mnt/?*) ;; *) die "--mount must be a directory under /mnt (got '$mnt')" ;; esac

if [ -n "$excl_file" ]; then
  [ -f "$excl_file" ] || die "excludes file '$excl_file' not found"
  base_excl=$(sed 's/#.*//' "$excl_file" | tr -s ' \t' '\n' | sed '/^$/d')
else
  base_excl=$DEFAULT_EXCLUDES
fi
excludes=""
for p in $MANDATORY_EXCLUDES $base_excl $extra; do
  case $p in /*) ;; *) die "exclude '$p' must be absolute (anchored at /), e.g. $PI_HOME/foo" ;; esac
  case " $excludes " in *" $p "*) ;; *) excludes="$excludes $p" ;; esac
done

# ---- log + done marker (real runs only) ---------------------------------------------------------------
if [ "$dry" = 0 ]; then
  log=${log:-/var/log/fc-clone-$newhost.log}
  rm -f "$log.done" 2>/dev/null || true
  : >> "$log" 2>/dev/null || die "cannot write log $log (run with sudo, or pass --log)"
  echo $$ > "$log.pid"
  exec > >(tee -a "$log") 2>&1
  echo "=== clone_live $(date -u +%FT%TZ) serial=$serial hostname=$newhost robot_id=$robot_id"
fi
mounted=0
finish() {
  local code=$?
  if [ "$mounted" = 1 ]; then umount -R "$mnt" 2>/dev/null || true; fi
  if [ "$dry" = 0 ]; then
    echo "FC_CLONE_EXIT $code $(date -u +%FT%TZ)"
    echo "$code" > "$log.done"; rm -f "$log.pid"
  fi
}
trap finish EXIT

# ---- source identity ----------------------------------------------------------------------------------
cur_host=$(cat "$R/etc/hostname" 2>/dev/null || hostname)
cur_id=$(cat "$R$PI_HOME/fc-v1/robot/robot_id" 2>/dev/null || true)
[ "$newhost" != "$cur_host" ] || die "--hostname $newhost is this Pi's own hostname"
[ "$robot_id" != "$cur_id" ] || die "--robot-id $robot_id is this Pi's own robot id"
oldid=$(sed -n 's/.*root=PARTUUID=\([0-9a-f]\{8\}\)-02.*/\1/p' "$R/boot/firmware/cmdline.txt" 2>/dev/null || true)
[ -n "$oldid" ] || die "no root=PARTUUID=xxxxxxxx-02 in $R/boot/firmware/cmdline.txt (not a stock Pi OS layout)"
grep -q "PARTUUID=$oldid-02" "$R/etc/fstab" 2>/dev/null || die "$R/etc/fstab does not mount PARTUUID=$oldid-02"

# ---- target guards --------------------------------------------------------------------------------------
matches="" all=""
while read -r name type; do
  [ "$type" = disk ] || continue
  s=$(lsblk -dno SERIAL "$name" | tr -d ' ')
  all="$all  $name serial=${s:-?} $(lsblk -dno MODEL "$name" | sed 's/ *$//')"$'\n'
  [ "$s" = "$serial" ] && matches="$matches $name"
done < <(lsblk -dnpo NAME,TYPE)
set -- $matches
[ $# -ge 1 ] || die "no disk has serial $serial. Disks:"$'\n'"$all"
[ $# -eq 1 ] || die "several disks have serial $serial:$matches"
dev=$1

disk_of() { local src; src=$(findmnt -no SOURCE "$1" 2>/dev/null | head -1); [ -n "$src" ] || return 0
  lsblk -dnpo PKNAME "$src" 2>/dev/null | head -1; }
rootdisk=$(disk_of /)
bootdisk=$(disk_of /boot/firmware)
[ -n "$rootdisk" ] || die "cannot tell which disk holds / (findmnt/lsblk)"
[ "$dev" != "$rootdisk" ] || die "$dev (serial $serial) holds / : it is the boot drive"
[ "$dev" != "${bootdisk:-}" ] || die "$dev (serial $serial) holds /boot/firmware : it is the boot drive"

stale=""
while read -r name mp; do
  [ -n "${mp:-}" ] || continue
  case $mp in
    "$mnt"|"$mnt"/*) stale="$stale $name" ;;
    /|/boot|/boot/*|/home|/usr|/var|"[SWAP]") die "$name on the target is mounted as a system filesystem ($mp)" ;;
    *) die "$name on the target is mounted at $mp: unmount it first" ;;
  esac
done < <(lsblk -lnpo NAME,MOUNTPOINT "$dev")

model=$(lsblk -dno MODEL "$dev" | sed 's/ *$//')
tran=$(lsblk -dno TRAN "$dev" | tr -d ' ')
tsize=$(lsblk -dnbo SIZE "$dev" | tr -d ' ')
used() { df -B1 --output=used "$1" | tail -n 1 | tr -d ' '; }
used_root=$(used "$ROOT")
[ -d "$R/boot/firmware" ] || die "$R/boot/firmware missing"
used_boot=$(used "$R/boot/firmware")
[ "$used_boot" -lt $(( BOOT_SECTORS * 512 - 16000000 )) ] || die "boot partition contents ($(hb "$used_boot")) do not fit 512 MiB"

excl_lines="" excl_total=0
for p in $excludes; do
  case $p in /proc/*|/sys/*|/dev/*|/run/*|/tmp/*|/mnt/*|/media/*) excl_lines="$excl_lines  $p (pseudo/temp fs, not sized)"$'\n'; continue ;; esac
  path=$R${p%/\*}
  if [ -e "$path" ]; then
    b=$(du -sxB1 "$path" 2>/dev/null | cut -f1 || true); b=${b:-0}  # partial without root
    excl_total=$(( excl_total + b )); excl_lines="$excl_lines  $p  $(hb "$b")"$'\n'
  else
    excl_lines="$excl_lines  $p  (absent)"$'\n'
  fi
done
need=$(( used_root + used_boot - excl_total )); [ "$need" -gt 0 ] || need=$used_boot
want=$(( need + need / 10 + 1000000000 ))
rsync_excl=""; for p in $excludes; do rsync_excl="$rsync_excl --exclude=$p"; done
bootnote=""; if [ -n "${bootdisk:-}" ] && [ "$bootdisk" != "$rootdisk" ]; then bootnote=" (+ $bootdisk for /boot/firmware)"; fi
rsync_cmd="rsync -aHAXx --numeric-ids --info=progress2$rsync_excl $R/ $mnt/"

cat <<EOF
PLAN  live clone of $cur_host (robot_id ${cur_id:-none}) -> $newhost (robot_id $robot_id)
  target     $dev  serial $serial  model '${model:-?}'  transport ${tran:-?}  size $(hb "$tsize")
  boot drive ${rootdisk}${bootnote}  (refused as target)
  source     / uses $(hb "$used_root"), /boot/firmware $(hb "$used_boot"); excluded $(hb "$excl_total"); copy ~$(hb "$need") (need $(hb "$want") with margin)
  erase      wipefs $dev; new dos label-id (old $oldid); $(part "$dev" 1) 512 MiB vfat 'bootfs', $(part "$dev" 2) rest ext4 'rootfs'
  copy       $rsync_cmd
             rsync -rt $R/boot/firmware/ $mnt/boot/firmware/
  reset      PARTUUID $oldid -> new in cmdline.txt + fstab; hostname + /etc/hosts; empty machine-id; new ssh host keys;
             no Tailscale state; robot_id=$robot_id; no identity.json; empty ~/fc-v1/run
  verify     remount read-only: label-id/PARTUUIDs, cmdline/fstab, hostname, hosts, robot_id, machine-id,
             host key differs from $cur_host's, no tailscaled.state, ~/fc-v1/run empty
  stale      ${stale:-none}   mount $mnt   log ${log:-none (dry run)}
EXCLUDES
$excl_lines
EOF
[ "$tran" = usb ] || echo "WARN: $dev transport is '${tran:-?}', not usb"
[ "$tsize" -ge "$want" ] || die "$dev is $(hb "$tsize"); the copy needs about $(hb "$want")"

if [ "$dry" = 1 ]; then echo "DRY RUN: nothing changed."; exit 0; fi

# ---- real run -----------------------------------------------------------------------------------------
[ "$(id -u)" = 0 ] || die "must run as root (sudo) to erase $dev"
if [ "$yes" = 0 ]; then
  [ -t 0 ] || die "stdin is not a terminal: pass --yes to erase $dev without asking"
  printf 'Type %s to ERASE %s (serial %s): ' "${dev#/dev/}" "$dev" "$serial"
  read -r answer; [ "$answer" = "${dev#/dev/}" ] || die "not confirmed"
fi
set -x
p1=$(part "$dev" 1) p2=$(part "$dev" 2)
umount -R "$mnt" 2>/dev/null || true
for n in $stale; do umount "$n" 2>/dev/null || true; done
wipefs -a "$dev" >/dev/null
newid=$(od -An -N4 -tx4 /dev/urandom | tr -d ' \n')
while [ "$newid" = "$oldid" ] || [ "$newid" = 00000000 ]; do newid=$(od -An -N4 -tx4 /dev/urandom | tr -d ' \n'); done
sfdisk -q "$dev" <<EOF
label: dos
label-id: 0x$newid
start=8192, size=$BOOT_SECTORS, type=c
start=$(( 8192 + BOOT_SECTORS )), type=83
EOF
partprobe "$dev"; udevadm settle; sleep 2
mkfs.vfat -F 32 -n bootfs "$p1" >/dev/null
mkfs.ext4 -q -F -L rootfs "$p2"
mkdir -p "$mnt"; mount "$p2" "$mnt"; mounted=1
mkdir -p "$mnt/boot/firmware"; mount "$p1" "$mnt/boot/firmware"
set +x
echo "partitioned $dev, label-id $newid; copying (live)"
t0=$(date +%s)
# shellcheck disable=SC2086
rsync -aHAXx --numeric-ids --info=progress2 $rsync_excl "$R/" "$mnt/" || { rc=$?; [ $rc = 24 ] || exit 1; }  # 24: files vanished (live system)
rsync -rt "$R/boot/firmware/" "$mnt/boot/firmware/"
echo "copied in $(( $(date +%s) - t0 )) s"

M=$mnt
sed -i "s/PARTUUID=$oldid-/PARTUUID=$newid-/g" "$M/boot/firmware/cmdline.txt" "$M/etc/fstab"
echo "$newhost" > "$M/etc/hostname"
if grep -q '^127\.0\.1\.1' "$M/etc/hosts"; then sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$newhost/" "$M/etc/hosts"
else printf '127.0.1.1\t%s\n' "$newhost" >> "$M/etc/hosts"; fi
: > "$M/etc/machine-id"                          # empty: systemd makes a new one (and a new DHCP client id) at first boot
[ -L "$M/var/lib/dbus/machine-id" ] || { rm -f "$M/var/lib/dbus/machine-id"; mkdir -p "$M/var/lib/dbus"; ln -s /etc/machine-id "$M/var/lib/dbus/machine-id"; }
set +f; rm -f "$M"/etc/ssh/ssh_host_*; set -f
mkdir -p "$M/etc/ssh"; ssh-keygen -A -f "$M" >/dev/null
rm -f "$M/var/lib/tailscale/tailscaled.state"   # never `tailscale logout`: that would log out the source too
if [ -d "$M$PI_HOME/fc-v1/robot" ]; then echo "$robot_id" > "$M$PI_HOME/fc-v1/robot/robot_id"; chown 1000:1000 "$M$PI_HOME/fc-v1/robot/robot_id"; fi
rm -f "$M$PI_HOME/fc-v1/identity.json"
mkdir -p "$M$PI_HOME/fc-v1/run"; chown 1000:1000 "$M$PI_HOME/fc-v1/run"
sync
umount "$M/boot/firmware"; umount "$M"; mounted=0
echo "identity reset; verifying read-only"

# ---- verify (read-only remount) ------------------------------------------------------------------------
partprobe "$dev" 2>/dev/null || true; udevadm settle
mount -o ro "$p2" "$M"; mounted=1; mount -o ro "$p1" "$M/boot/firmware"
fails=0
chk() { if eval "$2"; then echo "VERIFY PASS  $1"; else echo "VERIFY FAIL  $1"; fails=$((fails + 1)); fi; }
chk "label-id 0x$newid"            '[ "$(sfdisk --disk-id "$dev")" = "0x$newid" ]'
chk "PARTUUID $p1 = $newid-01"     '[ "$(blkid -s PARTUUID -o value "$p1")" = "$newid-01" ]'
chk "PARTUUID $p2 = $newid-02"     '[ "$(blkid -s PARTUUID -o value "$p2")" = "$newid-02" ]'
chk "cmdline.txt root=$newid-02"   'grep -q "root=PARTUUID=$newid-02" "$M/boot/firmware/cmdline.txt" && ! grep -q "$oldid" "$M/boot/firmware/cmdline.txt"'
chk "fstab -01/-02 = $newid"       'grep -q "PARTUUID=$newid-01" "$M/etc/fstab" && grep -q "PARTUUID=$newid-02" "$M/etc/fstab" && ! grep -q "$oldid" "$M/etc/fstab"'
chk "hostname $newhost"            '[ "$(cat "$M/etc/hostname")" = "$newhost" ]'
chk "hosts 127.0.1.1 $newhost"     'grep -Eq "^127\.0\.1\.1[[:space:]]+$newhost\$" "$M/etc/hosts"'
chk "robot_id $robot_id"           '[ ! -d "$M$PI_HOME/fc-v1/robot" ] || [ "$(cat "$M$PI_HOME/fc-v1/robot/robot_id")" = "$robot_id" ]'
chk "machine-id empty"             '[ ! -s "$M/etc/machine-id" ]'
chk "new ssh host key"             '[ -s "$M/etc/ssh/ssh_host_ed25519_key" ] && [ "$(ssh-keygen -lf "$M/etc/ssh/ssh_host_ed25519_key.pub" | cut -d" " -f2)" != "$(ssh-keygen -lf "$R/etc/ssh/ssh_host_ed25519_key.pub" | cut -d" " -f2)" ]'
chk "no tailscaled.state"          '[ ! -e "$M/var/lib/tailscale/tailscaled.state" ]'
chk "~/fc-v1/run empty"            '[ -z "$(ls -A "$M$PI_HOME/fc-v1/run" 2>/dev/null)" ]'
chk "source still $oldid"          'grep -q "root=PARTUUID=$oldid-02" "$R/boot/firmware/cmdline.txt"'
umount "$M/boot/firmware"; umount "$M"; mounted=0
[ "$fails" = 0 ] || { echo "FAILED: $fails verify check(s). Do not boot this drive."; exit 1; }
echo "DONE: $dev is $newhost (robot_id $robot_id), label-id $newid. Unplug it (this Pi stays up) and boot robot 2 from it."
