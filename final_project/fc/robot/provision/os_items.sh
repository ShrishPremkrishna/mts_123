#!/usr/bin/env bash
# os_items.sh — the Pi OS / driver changes mts has and a fresh class card lacks. Runs ON the Pi:
#
#   ssh <host> bash -s -- check              < os_items.sh   read-only: one line per item, ok|missing
#   ssh <host> bash -s -- plan               < os_items.sh   read-only: check + what apply would change + undo
#   ssh <host> bash -s -- apply <patch_sha>  < os_items.sh   change only the missing items (owner-approved runs)
#
# Source of truth: fc/robot/README.md, sections "Pi OS changes on mts", "IMU driver on mts" and "I2C bus
# change on mts" (each with its undo). Items:
#   hailo   /boot/firmware/config.txt has an active `dtparam=pciex1=off` (Hailo-8 off; fc does not use it)
#   fan     config.txt has mts's earlier fan curve (4 fan_temp lines, full speed from 55-60 C)
#   gui     pupper-gui (face screen) is neither enabled nor running
#   touch   no active `dtoverlay=waveshare-4dpic-*` line: the kernel no longer drives the GT911 touch
#           controller (0x5d) on the IMU's I2C-1 bus (the I2C bus change of 2026-11-25)
#   imu_v4  the stock driver .so links to our v4 build (filter + stall guard + accel bias from imu_cal.json)
# Never touched: ~/fc-v1/run (imu_cal.json is the accel bias of THIS chip; run `fcctl.sh imu-level` here).
# apply: config.txt items need passwordless sudo and a reboot; imu_v4 needs the stack stopped, builds
# ~/fc-v1/build/imu-filter-v4 from this card's own driver source + the staged patch (a few minutes) and
# refuses if the patch does not apply. Idempotent: items already ok are not touched.
# Exit: 0 all ok, 1 something missing (check/plan) or not done (apply), 2 usage.
export LC_ALL=C   # no set -u: the Mac's bash 3.2 (tests) treats an empty array as unbound
mode=${1:-check}
CFG=${FC_BOOT_CONFIG:-/boot/firmware/config.txt}
SUDO=${FC_OS_SUDO-sudo -n}
MONO=${FC_MONO:-$HOME/pupperv3-monorepo}
PKG=control_board_hardware_interface
SRC=$MONO/ros2_ws/src/$PKG
LIBDIR=$MONO/ros2_ws/install/$PKG/lib
LIB=$LIBDIR/libcontrol_board_hardware_interface.so
B=$HOME/fc-v1/build/imu-filter-v4
V4=$B/install/$PKG/lib/libcontrol_board_hardware_interface.so
PATCH=$HOME/fc-v1/robot/imu/stock-imu-filter.patch
FAN=(
  "dtparam=fan_temp0=40000,fan_temp0_hyst=3000,fan_temp0_speed=125"
  "dtparam=fan_temp1=50000,fan_temp1_hyst=3000,fan_temp1_speed=175"
  "dtparam=fan_temp2=55000,fan_temp2_hyst=3000,fan_temp2_speed=250"
  "dtparam=fan_temp3=60000,fan_temp3_hyst=3000,fan_temp3_speed=255"
)
BLOCK_START="# --- Pupper FC heat reduction"
BLOCK_END="# --- end Pupper FC ---"
TOUCH_RE='^[[:space:]]*dtoverlay=waveshare-4dpic'
STACK_RE='[r]os2_control_node|[d]rive_bridge|[p]ersistent_session.py'

UNDO_hailo="sudo sed -i '/^$BLOCK_START/,/^$BLOCK_END\$/{/^dtparam=pciex1=off\$/d}' $CFG && sudo reboot"
UNDO_fan="sudo sed -i '/^$BLOCK_START/,/^$BLOCK_END\$/{/^dtparam=fan_temp[0-3]=/d}' $CFG && sudo reboot"
UNDO_gui="sudo systemctl link $MONO/pupper-rs/pupper-gui.service && sudo systemctl enable --now pupper-gui"
UNDO_touch="sudo sed -i 's/^#fc-off([^)]*i2c touch) //' $CFG && sudo reboot"
UNDO_imu_v4="$HOME/fc-v1/robot/fcctl.sh stop; cd $LIBDIR && rm libcontrol_board_hardware_interface.so && mv libcontrol_board_hardware_interface.so.stock-link libcontrol_board_hardware_interface.so"

has_line() { [ -r "$CFG" ] && grep -qxF -- "$1" "$CFG"; }
sha12() { sha256sum "$1" 2>/dev/null | cut -c1-12; }
gui_state() { systemctl is-enabled pupper-gui 2>/dev/null | head -1; }

# ---- predicates: each sets $detail and returns 0 when the item is ok ----
st_hailo() {
  [ -r "$CFG" ] || { detail="cannot read $CFG"; return 1; }
  if has_line "dtparam=pciex1=off"; then detail="dtparam=pciex1=off"; else detail="no dtparam=pciex1=off (Hailo-8 powered)"; return 1; fi
}
st_fan() {
  [ -r "$CFG" ] || { detail="cannot read $CFG"; return 1; }
  local n=0 l; for l in "${FAN[@]}"; do has_line "$l" && n=$((n + 1)); done
  detail="$n/4 fan_temp lines of mts's curve"; [ $n -eq 4 ]
}
st_gui() {
  local en act; en=$(gui_state); act=$(systemctl is-active pupper-gui 2>/dev/null | head -1)
  detail="pupper-gui is-enabled=${en:-none} is-active=${act:-none}"
  case $en in enabled*|linked*|static|alias|indirect|generated|transient) return 1 ;; esac
  [ "$act" != active ] && [ "$act" != activating ]
}
st_touch() {
  [ -r "$CFG" ] || { detail="cannot read $CFG"; return 1; }
  local n; n=$(grep -cE "$TOUCH_RE" "$CFG")
  if [ "$n" -eq 0 ]; then detail="no active waveshare-4dpic overlay ($(grep -c '^#fc-off(.*i2c touch)' "$CFG") commented)"
  else detail="$n active waveshare-4dpic overlay line(s) (GT911 touch on the IMU's I2C bus)"; return 1; fi
}
v4_built() { [ -f "$V4" ] && grep -qa FC_IMU_CAL "$V4"; }
st_imu_v4() {
  if [ ! -e "$LIB" ] && [ ! -L "$LIB" ]; then detail="no stock driver at $LIB"; return 1; fi
  if [ -L "$LIB" ] && v4_built && [ "$(readlink -f "$LIB")" = "$(readlink -f "$V4")" ]; then
    detail="driver -> imu-filter-v4 (sha256 $(sha12 "$V4"))"; return 0; fi
  detail="driver -> $(readlink -f "$LIB" | sed "s|^$HOME/|~/|") (sha256 $(sha12 "$LIB")); v4 build: $(v4_built && echo present || echo none)"
  return 1
}

ITEMS=(hailo fan gui touch imu_v4)
scan() {   # STATE_<item>, DETAIL_<item>, missing=(...) (plain variables: bash 3.2 has no assoc arrays)
  local i; missing=()
  for i in "${ITEMS[@]}"; do
    if "st_$i"; then printf -v "STATE_$i" ok; else printf -v "STATE_$i" missing; missing+=("$i"); fi
    printf -v "DETAIL_$i" '%s' "$detail"
  done
}
show() {
  local i s d; for i in "${ITEMS[@]}"; do s=STATE_$i d=DETAIL_$i; printf '  %-8s %-7s %s\n' "${!s}" "$i" "${!d}"; done
  if [ -f "$HOME/fc-v1/run/imu_cal.json" ]; then
    printf '  %-8s %-7s %s\n' info imu_cal "~/fc-v1/run/imu_cal.json present (this chip's bias; never copied between robots)"
  else
    printf '  %-8s %-7s %s\n' info imu_cal "none: robot level on the stand, then ~/fc-v1/robot/fcctl.sh imu-level (per chip)"
  fi
}
what() {   # what apply would do for one item
  case $1 in
    hailo) echo "append 'dtparam=pciex1=off' in a marked '$BLOCK_START' block to $CFG (backup first; reboot)" ;;
    fan)   echo "append mts's 4 fan_temp lines in a marked '$BLOCK_START' block to $CFG (backup first; reboot)" ;;
    gui)   echo "sudo systemctl disable --now pupper-gui (screen goes dark now)" ;;
    touch) echo "comment out the waveshare-4dpic dtoverlay lines in $CFG with '#fc-off(<date> i2c touch) ' (backup first; reboot)" ;;
    imu_v4)
      if v4_built; then echo "relink $LIB -> existing $V4 (stack stopped; old target kept as .so.stock-link)"
      else echo "build $B from $SRC + ~/fc-v1/robot/imu/stock-imu-filter.patch (colcon, a few min), run its tests, then relink $LIB (stack stopped; old target kept as .so.stock-link)"; fi ;;
  esac
}

case $mode in
  check) scan; show; [ ${#missing[@]} -eq 0 ]; exit $? ;;
  plan)
    scan; show
    if [ ${#missing[@]} -eq 0 ]; then echo "  nothing to change"; exit 0; fi
    echo "  apply would change:"
    for i in "${missing[@]}"; do v=UNDO_$i; printf '    %-7s %s\n            undo: %s\n' "$i" "$(what "$i")" "${!v}"; done
    exit 1 ;;
  apply) ;;
  *) echo "usage: os_items.sh check | plan | apply <patch_sha256>" >&2; exit 2 ;;
esac

# ---------------------------------------------------------------- apply
want_sha=${2:-}
scan
[ ${#missing[@]} -eq 0 ] && { show; echo "  nothing to change"; exit 0; }
need() { local i; for i in "${missing[@]}"; do [ "$i" = "$1" ] && return 0; done; return 1; }
fail=0 reboot=0
cfg_items=0; for i in hailo fan gui touch; do need $i && cfg_items=1; done
if [ $cfg_items -eq 1 ] && ! $SUDO true 2>/dev/null; then
  echo "  REFUSED hailo/fan/gui/touch: need passwordless sudo on $(hostname)" >&2; exit 1; fi

if need hailo || need fan || need touch; then
  bak="$CFG.bak-pre-fc-os-$(date +%Y%m%d)"
  if [ -e "$bak" ]; then echo "  backup: $bak (kept from an earlier run today)"
  else $SUDO cp -p "$CFG" "$bak" || { echo "  REFUSED: could not back up $CFG" >&2; exit 1; }; echo "  backup: $bak"; fi
fi
if need hailo || need fan; then
  {
    printf '\n%s (%s, provision.sh --apply-os); undo: delete this block, reboot ---\n[all]\n' "$BLOCK_START" "$(date +%F)"
    need hailo && echo "dtparam=pciex1=off"
    if need fan; then for l in "${FAN[@]}"; do has_line "$l" || echo "$l"; done; fi
    echo "$BLOCK_END"
  } | $SUDO tee -a "$CFG" >/dev/null && reboot=1
fi
if need touch; then
  tmp=$(mktemp)   # sed to a temp file + cp back (portable, keeps the file's owner/mode on the FAT boot partition)
  if sed "s/^[[:space:]]*dtoverlay=waveshare-4dpic/#fc-off($(date +%F) i2c touch) dtoverlay=waveshare-4dpic/" "$CFG" > "$tmp" &&
     [ -s "$tmp" ] && $SUDO cp "$tmp" "$CFG"; then reboot=1; else echo "  touch: edit failed; $CFG unchanged" >&2; fail=1; fi
  rm -f "$tmp"
fi
if need gui; then
  echo "  pupper-gui was $(gui_state); unit file $(systemctl show -p FragmentPath --value pupper-gui 2>/dev/null)"
  $SUDO systemctl disable --now pupper-gui || fail=1
fi

if need imu_v4; then (
  set -e
  if pgrep -f "$STACK_RE" >/dev/null; then echo "  imu_v4: a robot stack is running; stop it first (fcctl.sh stop)" >&2; exit 1; fi
  if ! v4_built; then
    [ -d "$SRC" ] || { echo "  imu_v4: no driver source at $SRC" >&2; exit 1; }
    [ -f "$PATCH" ] || { echo "  imu_v4: no $PATCH (stage fc/robot first)" >&2; exit 1; }
    [ -z "$want_sha" ] || [ "$(sha256sum "$PATCH" | cut -d' ' -f1)" = "$want_sha" ] ||
      { echo "  imu_v4: staged patch differs from the laptop's; re-stage" >&2; exit 1; }
    [ ! -e "$B" ] || { echo "  imu_v4: $B exists without a v4 .so; move it aside by hand, then rerun" >&2; exit 1; }
    mkdir -p "$B/src" && cp -a "$SRC" "$B/src/" && cp "$PATCH" "$B/"
    (cd "$B/src" && patch -p1 --dry-run -s < "$PATCH" >/dev/null) ||
      { rm -rf "$B"; echo "  imu_v4: the patch does not apply to this card's $PKG source; nothing linked" >&2; exit 1; }
    (cd "$B/src" && patch -p1 -s < "$PATCH")
    echo "  imu_v4: building $B (log: $B/build.log)"
    cd "$B"
    set +e; source /opt/ros/jazzy/setup.bash; source "$MONO/ros2_ws/install/setup.bash"; set -e
    MAKEFLAGS=-j2 nice -n 19 colcon build --base-paths src --build-base build --install-base install \
      --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo > build.log 2>&1 ||
      { echo "  imu_v4: build failed; tail $B/build.log" >&2; exit 1; }
    MAKEFLAGS=-j2 nice -n 19 colcon test --base-paths src --build-base build --install-base install \
      --ctest-args -R test_fc > tests.log 2>&1 || true
    colcon test-result --test-result-base build > test_result.txt 2>&1 &&
      grep -qE 'Summary: [1-9][0-9]* tests, 0 errors, 0 failures' test_result.txt ||
      { echo "  imu_v4: driver tests failed or did not run ($B/test_result.txt); nothing linked" >&2; exit 1; }
    v4_built || { echo "  imu_v4: build has no FC_IMU_CAL marker; nothing linked" >&2; exit 1; }
    if ldd "$V4" | grep -q "not found"; then echo "  imu_v4: unresolved libraries (ldd); nothing linked" >&2; exit 1; fi
    sha256sum "$V4" > so.sha256
  fi
  [ -e "$LIB" ] || [ -L "$LIB" ] || { echo "  imu_v4: no stock driver at $LIB; nothing linked" >&2; exit 1; }
  if [ ! -e "$LIB.stock-link" ] && [ ! -L "$LIB.stock-link" ]; then mv "$LIB" "$LIB.stock-link"
  elif [ ! -L "$LIB" ]; then   # never overwrite a real driver file: the old .stock-link may not be its copy
    echo "  imu_v4: both $LIB (a file) and $LIB.stock-link exist; resolve by hand; nothing linked" >&2; exit 1; fi
  ln -sfn "$V4" "$LIB"
  echo "  imu_v4: $LIB -> $V4 (sha256 $(sha12 "$V4")); previous target kept as $LIB.stock-link"
) || fail=1; fi

echo "  -- after apply:"
scan; show
[ $reboot -eq 1 ] && echo "  REBOOT needed for the config.txt changes: sudo reboot"
[ $fail -eq 0 ] && [ ${#missing[@]} -eq 0 ]
