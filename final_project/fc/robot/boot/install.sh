#!/usr/bin/env bash
# boot/install.sh — install / remove the Pupper FC power-on service on the Pi (needs sudo).
# Runs ON the Pi from ~/fc-v1/robot/boot/ (staged by stage.sh). NOT run by any other script.
#   install.sh install [--profile P] [--mode home|idle] [--delay S]
#                     write ~/fc-v1/boot.env, install + enable fc-boot.service (takes effect at the NEXT
#                     power-on; nothing starts now)
#   install.sh uninstall   disable + remove the unit (keeps boot.env; does not touch a running stack)
#   install.sh status      what is installed / enabled, boot.env, last boot result
#   install.sh print       print the unit and boot.env it would write (no changes; works anywhere)
# Refuses to install while the course's robot.service is enabled or running (two stacks would fight
# over the motors; fcctl refuses to start next to a foreign stack anyway).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
UNIT=fc-boot.service
DEST=/etc/systemd/system/$UNIT
USER_NAME=${FC_BOOT_USER:-$(id -un)}
HOME_DIR=${FC_BOOT_HOME:-$HOME}
FC_ROOT=$HOME_DIR/fc-v1
ENVF=$FC_ROOT/boot.env

profile=stock mode=home delay=15
render_unit() { sed -e "s|@USER@|$USER_NAME|g" -e "s|@HOME@|$HOME_DIR|g" "$HERE/$UNIT"; }
render_env() {
  printf '# Pupper FC power-on settings (read by fcctl.sh boot; written by boot/install.sh)\n'
  printf '# FC_BOOT_MODE=home: home the legs at power-on (robot ON THE STAND); idle: do nothing\n'
  printf 'FC_BOOT_PROFILE=%s\nFC_BOOT_MODE=%s\nFC_BOOT_DELAY_S=%s\n' "$profile" "$mode" "$delay"
}
parse_opts() {
  while [ $# -gt 0 ]; do
    case "$1" in
      --profile) profile=${2:-}; shift 2 ;;
      --mode) mode=${2:-}; shift 2 ;;
      --delay) delay=${2:-}; shift 2 ;;
      *) echo "unknown option '$1'" >&2; exit 2 ;;
    esac
  done
  [[ "$profile" =~ ^[a-z0-9_]+$ ]] || { echo "bad profile '$profile'" >&2; exit 2; }
  [ -f "$HERE/../profiles/$profile.sh" ] || { echo "no profile '$profile' in $HERE/../profiles" >&2; exit 2; }
  case "$mode" in home|idle) ;; *) echo "mode must be home or idle" >&2; exit 2 ;; esac
  [[ "$delay" =~ ^[0-9]+$ ]] && [ "$delay" -le 300 ] || { echo "delay must be 0-300 s" >&2; exit 2; }
}
robot_service_in_use() {
  systemctl is-enabled --quiet robot.service 2>/dev/null || systemctl is-active --quiet robot.service 2>/dev/null
}

cmd=${1:-}; shift || true
case "$cmd" in
  print)
    parse_opts "$@"
    echo "# ---- $DEST"; render_unit
    echo "# ---- $ENVF"; render_env
    ;;
  install)
    parse_opts "$@"
    command -v systemctl > /dev/null || { echo "no systemctl here: run this on the Pi" >&2; exit 1; }
    [ -f "$FC_ROOT/robot/fcctl.sh" ] || { echo "missing $FC_ROOT/robot/fcctl.sh: stage first" >&2; exit 1; }
    if robot_service_in_use; then
      echo "robot.service (course autostart) is enabled or running: refusing. It would start a second stack." >&2
      echo "Disable it deliberately first (sudo systemctl disable --now robot.service) if that is intended." >&2
      exit 1
    fi
    mkdir -p "$FC_ROOT/logs" "$FC_ROOT/run"
    umask 077; render_env > "$ENVF.tmp" && mv "$ENVF.tmp" "$ENVF"
    render_unit > "$FC_ROOT/run/$UNIT.rendered"
    sudo install -m 644 -o root -g root "$FC_ROOT/run/$UNIT.rendered" "$DEST"
    sudo systemctl daemon-reload
    sudo systemctl enable "$UNIT"
    echo "installed $DEST (enabled; NOT started). At the next power-on: $(sed -n 's/^FC_BOOT_MODE=//p' "$ENVF") mode, profile $profile, delay ${delay} s."
    [ "$mode" = home ] && echo "From now on: power the robot on ONLY while it is on the stand (it homes the legs)."
    echo "Roll back: bash $HERE/install.sh uninstall"
    ;;
  uninstall)
    command -v systemctl > /dev/null || { echo "no systemctl here" >&2; exit 1; }
    sudo systemctl disable "$UNIT" 2>/dev/null || true
    sudo rm -f "$DEST"
    sudo systemctl daemon-reload
    echo "removed $DEST (a stack it already started keeps running: fcctl.sh down stops it). boot.env kept: $ENVF"
    ;;
  status)
    echo "unit file: $([ -f "$DEST" ] && echo "$DEST" || echo 'not installed')"
    echo "enabled:   $(systemctl is-enabled "$UNIT" 2>/dev/null || true)"
    echo "active:    $(systemctl is-active "$UNIT" 2>/dev/null || true)"
    echo "robot.service: enabled=$(systemctl is-enabled robot.service 2>/dev/null || echo none) active=$(systemctl is-active robot.service 2>/dev/null || true)"
    echo "boot.env:"; sed 's/^/  /' "$ENVF" 2>/dev/null || echo "  (none)"
    echo "last boot: $(cat "$FC_ROOT/run/boot.json" 2>/dev/null || echo none)"
    ;;
  *) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
