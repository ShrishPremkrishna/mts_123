#!/usr/bin/env bash
# fcctl.sh — Pupper FC robot control. Runs ON the Pi from ~/fc-v1/robot/.
#   status                      one JSON line (incl. "phase": off|starting|homed|standing|ready|fault, "light")
#   ready [profile] [--confirm [--stand-now]] [--no-bridge] [--skip-imu-check] [STACK OPTIONS]
#                               homed -> playing: wait READY, stand the walker in software (or wait for the
#                               gamepad), start the bridge. Not running: homes (needs --confirm) and stops
#                               at "homed" (move it to the floor, run ready again) unless --stand-now.
#                               E-stopped (run/estop.json): --confirm re-arms (clears the latch, restands).
#                               Progress as JSON lines on stdout; the last one has "done":true.
#   down [--keep-stack] [--force]
#                               park the bridge (zero only; link E-STOP still armed), lie down (stock: lowered
#                               under position control by the lie-down controller, aborted by a link E-STOP /
#                               fallen cutoff / R3; otherwise a damped drop like R3), bridge stop, confirm no
#                               leg controller active, stop the stack; JSON lines. Not confirmed: abort,
#                               stack left running, exit 1 (--force = UNSAFE: stops a possibly standing robot)
#   boot                        run by fc-boot.service at power-on (boot/README.md); never by hand
#   start <profile> --confirm [STACK OPTIONS]
#                               start a robot stack (HOMES THE LEGS: robot on the stand)
#   STACK OPTIONS (SPEC v1.1)   --speed-cap <0.30..1.00> (walker max_vx_forward/speed_cap; default 0.49)
#                               --heading-hold on|off (default on). Built into one verified per-start
#                               overlay (stack_overlay.py) under the run dir; *_hh0 profiles = --heading-hold off.
#                               --lean on|off (stock; default on: foxglove_bridge and the onboard camera node
#                               are not launched, less Pi heat) --camera on|off (with lean: keep the camera
#                               node; default off). The walking path (controllers, joy, estop) is unchanged.
#   stop [--force]              stop what fcctl started (bridge, walker, stack) and only that; refuses
#                               (stack left running) if a leg controller cannot be confirmed inactive
#   bridge start [--profile P] [--robot-id ID] [--allow-from IP[,IP]] [--reset-replay]
#                [--no-imu-stop] [--no-mux-guard] | bridge stop
#                               (robot id defaults to ~/fc-v1/robot/robot_id, written by stage.sh;
#                                key: ~/fc-v1/robot/link.key, 0600; replay state: run/bridge_state.json)
#   cam start [--width W] [--fps F] [--quality Q] | cam stop
#                               optional robot-POV feed: camstream.py, MJPEG on :8090 (robots.json cam_url).
#                               Off unless started; default 160x120 @ 5 fps (~1.3% of one core, ~0.13 Mbit/s,
#                               README). Running stack's camera node if launched (read-only ROS), else
#                               picamera2 at the stream rate. stop (and down when it stops the stack) ends it
#   imu-test <seconds>          delegates to imu_test.py (IMU lane) if present
#   i2c-recover [--check] [--force]
#                               free a hung I2C-1 bus (BNO "SDA stuck at low") without a power cycle:
#                               imu/i2c_recover.sh (9+ SCL pulses, controller unbind/bind; one JSON line).
#                               Refuses while a robot stack runs unless --force; --check only reads levels
#   imu-level [--seconds S] [--dry-run] [--legs-on] | imu-level check|show
#                               BNO accel calibration (fc/notes/rca-20261127-tilt.md): robot sitting LEVEL on the
#                               stand, legs off, stack running. Captures the published IMU (read-only), refuses on
#                               motion/noise, saves run/imu_cal.json (survives stage.sh); the v4 driver applies it
#                               at the next stack start. check = tilt at rest now; show = stored vs applied
#   walk | rest                 activate / deactivate the profile's walker controller (optional;
#                               the gamepad X / R3 do the same)
# Profiles: profiles/<name>.sh (*_hh0 = alias: same stack with --heading-hold off). State: ~/fc-v1/run/ (pidfiles, active,
# estop.json latch), runs: ~/fc-v1/runs/ (stack.json = the start's options/overlay record).
# Never touches systemd units. Never SIGKILLs a robot stack.
set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
FC_ROOT=${FC_ROOT:-$HOME/fc-v1}
FC_RUN=$FC_ROOT/run
FC_RUNS=$FC_ROOT/runs
FC_LOGS=$FC_ROOT/logs
mkdir -p "$FC_RUN" "$FC_RUNS" "$FC_LOGS"
chmod 700 "$FC_ROOT" "$FC_RUN" "$FC_RUNS" 2>/dev/null || true

fc_log() { echo "[fcctl $(date +%H:%M:%S)] $*" >&2; }
die() { fc_log "$*"; printf '{"ok":false,"error":"%s"}\n' "${*//\"/\'}"; exit 1; }

# ---------------------------------------------------------------- process helpers
_starttime() { awk '{print $22}' "/proc/$1/stat" 2>/dev/null; }

# fc_spawn ROLE LOG CMD... : detached session leader; pidfile holds "pid starttime".
# 9>&- 8>&- : the child must not inherit the fcctl lock (fd 9) or the ready lock (fd 8), or every later
# stop/bridge start/ready is refused.
fc_spawn() {
  local role=$1 log=$2; shift 2
  local pidfile=$FC_RUN/$role.pid
  rm -f "$pidfile"
  # bash -c (not a background job) so the child keeps default SIGINT handling.
  setsid -f bash -c 'echo $$ > "$1.tmp"; shift; exec "$@"' fc_spawn "$pidfile" "$@" >> "$log" 2>&1 < /dev/null 9>&- 8>&-
  local i pid
  for i in $(seq 40); do
    if [ -s "$pidfile.tmp" ]; then
      pid=$(cat "$pidfile.tmp"); echo "$pid $(_starttime "$pid")" > "$pidfile"; rm -f "$pidfile.tmp"
      return 0
    fi
    sleep 0.05
  done
  fc_log "spawn $role: no pid"; return 1
}

fc_pid() {  # prints pid if ROLE's recorded process is still the same process
  local pidfile=$FC_RUN/$1.pid pid st
  [ -f "$pidfile" ] || return 1
  read -r pid st < "$pidfile" || return 1
  [ -n "${pid:-}" ] && [ -d "/proc/$pid" ] && [ "$(_starttime "$pid")" = "${st:-x}" ] || return 1
  echo "$pid"
}
fc_alive() { fc_pid "$1" > /dev/null; }

# A starter killed between setsid and writing the pidfile leaves only ROLE.pid.tmp: adopt those.
fc_adopt_orphans() {
  local f pid role
  for f in "$FC_RUN"/*.pid.tmp; do
    [ -f "$f" ] || continue
    role=$(basename "$f" .pid.tmp); pid=$(cat "$f" 2>/dev/null)
    if [[ "$pid" =~ ^[0-9]+$ ]] && [ -d "/proc/$pid" ] && [ ! -f "$FC_RUN/$role.pid" ]; then
      echo "$pid $(_starttime "$pid")" > "$FC_RUN/$role.pid"; fc_log "adopted $role (pid $pid)"
    fi
    rm -f "$f"
  done
}

# fc_signal_group ROLE SIG TIMEOUT : signal ROLE's process group, wait, escalate once to TERM.
fc_signal_group() {
  local role=$1 sig=$2 timeout=$3 pid
  pid=$(fc_pid "$role") || { rm -f "$FC_RUN/$role.pid"; return 0; }
  for s in "$sig" TERM; do
    kill "-$s" -- "-$pid" 2>/dev/null || kill "-$s" "$pid" 2>/dev/null
    local end=$((SECONDS + timeout))
    while [ $SECONDS -lt $end ]; do
      fc_alive "$role" || { rm -f "$FC_RUN/$role.pid"; return 0; }
      sleep 0.2
    done
    timeout=5
  done
  fc_log "$role (pid $pid) did not exit; left running for manual inspection"
  return 1
}

load_profile() {
  local name=$1
  [[ "$name" =~ ^[a-z0-9_]+$ ]] && [ -f "$HERE/profiles/$name.sh" ] || die "unknown profile '$name' (have: $(cd "$HERE/profiles" && ls *.sh | sed 's/\.sh$//' | tr '\n' ' '))"
  PROFILE=$name
  # The profile file decides the default heading hold (an alias sets FC_HEADING_HOLD=0 and sources its base).
  unset FC_HEADING_HOLD PROFILE_HEADING_HOLD PROFILE_LIEDOWN PROFILE_LEAN PROFILE_CAMERA
  # shellcheck disable=SC1090
  source "$HERE/profiles/$name.sh"
  case "${PROFILE_HEADING_HOLD:-}" in on|off|"") ;; *) die "profile '$name': bad heading hold '$PROFILE_HEADING_HOLD' (FC_HEADING_HOLD must be 0 or 1)" ;; esac
  PROFILE_HH_ALIAS=${FC_HEADING_HOLD+set}   # the profile itself fixed the heading hold (stock_hh0, tej_u6_hh0)
}

# ---------------------------------------------------------------- stack options (SPEC v1.1)
# STACK_SPEED_CAP ("" = profile default 0.49) and STACK_HH ("" = profile default) come from --speed-cap /
# --heading-hold; the detached starter receives them as FC_STACK_SPEED_CAP / FC_STACK_HEADING_HOLD.
STACK_SPEED_CAP=${FC_STACK_SPEED_CAP:-}
STACK_HH=${FC_STACK_HEADING_HOLD:-}
STACK_LEAN=${FC_STACK_LEAN:-}       # "" = profile default (stock: on); --lean
STACK_CAMERA=${FC_STACK_CAMERA:-}   # "" = profile default (stock: off); --camera
STACK_ERR=""
# stack_opt OPTION VALUE : 0 = consumed (sets STACK_*), 1 = invalid (STACK_ERR says why)
stack_opt() {
  case "$1" in
    --speed-cap)
      [[ "${2:-}" =~ ^[0-9]*[.]?[0-9]+$ ]] && awk -v v="$2" 'BEGIN { exit !(v >= 0.30 && v <= 1.00) }' ||
        { STACK_ERR="--speed-cap needs a number in 0.30..1.00 (got '${2:-}')"; return 1; }
      STACK_SPEED_CAP=$2 ;;
    --heading-hold)
      [[ "${2:-}" =~ ^(on|off)$ ]] || { STACK_ERR="--heading-hold needs on|off (got '${2:-}')"; return 1; }
      STACK_HH=$2 ;;
    --lean)
      [[ "${2:-}" =~ ^(on|off)$ ]] || { STACK_ERR="--lean needs on|off (got '${2:-}')"; return 1; }
      STACK_LEAN=$2 ;;
    --camera)
      [[ "${2:-}" =~ ^(on|off)$ ]] || { STACK_ERR="--camera needs on|off (got '${2:-}')"; return 1; }
      STACK_CAMERA=$2 ;;
    *) STACK_ERR="unknown stack option '$1'"; return 1 ;;
  esac
}
# apply_stack_opts : after load_profile. Refuses a heading hold that contradicts an alias profile.
apply_stack_opts() {
  if [ -n "$STACK_HH" ]; then
    if [ -n "$PROFILE_HH_ALIAS" ] && [ "$STACK_HH" != "${PROFILE_HEADING_HOLD:-on}" ]; then
      STACK_ERR="profile '$PROFILE' fixes heading hold ${PROFILE_HEADING_HOLD}; drop --heading-hold $STACK_HH or use the base profile"
      return 1
    fi
    PROFILE_HEADING_HOLD=$STACK_HH
  fi
  if [ -n "$STACK_LEAN$STACK_CAMERA" ] && [ -z "${PROFILE_LEAN:-}" ]; then
    STACK_ERR="profile '$PROFILE' has no --lean/--camera option (stock only)"
    return 1
  fi
  [ -z "$STACK_LEAN" ] || PROFILE_LEAN=$STACK_LEAN
  [ -z "$STACK_CAMERA" ] || PROFILE_CAMERA=$STACK_CAMERA
  return 0
}
fc_stack_options_set() { [ -n "$STACK_SPEED_CAP" ] || [ "${PROFILE_HEADING_HOLD:-on}" = off ] || [ "${PROFILE_LEAN:-off}" = on ]; }
# _cam_effective : the onboard camera node is launched (lean off = stock list, or lean on + --camera on)
_cam_effective() { [ "${PROFILE_LEAN:-off}" = off ] || [ "${PROFILE_CAMERA:-off}" = on ]; }

# fc_stack_overlay KIND RUN [on|off liedown] : build + verify a fresh overlay (stack_overlay.py) in RUN/overlay
# with the start's options, record its JSON in RUN/stack.json, set OVERLAY_LAUNCH / OVERLAY_HW. Writes only
# under FC_ROOT. Any failure returns 1, so the profile refuses to start (before any motion).
fc_stack_overlay() {
  local kind=$1 run=$2 liedown=${3:-off} res args=()
  [ ! -e "$run/overlay" ] || { fc_log "stack overlay: $run/overlay already exists; refusing"; return 1; }
  [ -z "$STACK_SPEED_CAP" ] || args+=(--speed-cap "$STACK_SPEED_CAP")
  args+=(--heading-hold "${PROFILE_HEADING_HOLD:-on}" --liedown "$liedown")
  [ -z "${PROFILE_LEAN:-}" ] || args+=(--lean "$PROFILE_LEAN" --camera "${PROFILE_CAMERA:-off}")
  res=$(python3 "$HERE/stack_overlay.py" "$kind" "$run/overlay" --root "$FC_ROOT" "${args[@]}") && [ -n "$res" ] ||
    { fc_log "STACK OVERLAY ($kind ${args[*]}) COULD NOT BE BUILT/VERIFIED: refusing to start (see above)"; return 1; }
  printf '%s\n' "$res" > "$run/stack.json"
  OVERLAY_LAUNCH=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["launch"])' <<< "$res")
  OVERLAY_HW=$(dirname "$OVERLAY_LAUNCH")
  fc_log "stack overlay verified in $run/overlay: $kind ${args[*]}"
}
# fc_stack_record RUN : the start's options when no overlay is built (stock defaults, nothing changed).
fc_stack_record() {
  local lean=null cam=null   # profiles without the option (tej_u6): unknown; stock without overlay = stock list
  if [ -n "${PROFILE_LEAN:-}" ]; then lean="\"$PROFILE_LEAN\""; if _cam_effective; then cam=true; else cam=false; fi; fi
  printf '{"kind":null,"verified":null,"overlay":false,"speed_cap":null,"speed_cap_effective":0.49,"heading_hold":"%s","liedown":null,"lean":%s,"not_launched":[],"camera":%s}\n' \
    "${PROFILE_HEADING_HOLD:-on}" "$lean" "$cam" > "$1/stack.json"
}
_stack_field() { python3 -c 'import json,sys
try: v=json.load(open(sys.argv[1])).get(sys.argv[2])
except Exception: v=None
print("" if v is None else (json.dumps(v) if isinstance(v,(dict,list)) else v))' "$1" "$2"; }

active_profile() { [ -f "$FC_RUN/active" ] && sed -n 1p "$FC_RUN/active"; }
active_run() { [ -f "$FC_RUN/active" ] && sed -n 2p "$FC_RUN/active"; }
boot_id() { cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown; }
active_stale() {  # the active file was written during an earlier boot (line 3 = boot id; absent = unknown)
  local b; b=$(sed -n 3p "$FC_RUN/active" 2>/dev/null)
  [ -n "$b" ] && [ "$b" != "$(boot_id)" ]
}

foreign_stack() {  # robot processes fcctl did not start
  local pids p mine=" "
  for r in launch starter; do p=$(fc_pid "$r") && mine+="$p "; done
  pids=$(pgrep -f 'ros2_control_node|persistent_session.py|ros2 launch' 2>/dev/null) || return 1
  [ -n "$(active_profile)" ] && ! active_stale && return 1   # ours (tej_u6 children are detached by Tej's helper)
  for p in $pids; do [[ "$mine" == *" $p "* ]] || return 0; done
  return 1
}

lock() { exec 9> "$FC_RUN/lock"; flock -n 9 || die "another fcctl command is running"; }
unlock() { exec 9>&-; }

# ---------------------------------------------------------------- ROS queries (rclpy, no daemon)
# rosq probe [CMD_TOPIC] | rosq controllers | rosq switch "<activate...>" "<deactivate...>" -> one compact JSON line.
# probe waits until the DDS graph settles (2-8 s; a fixed 1.5 s spin missed the mux, see rosq.py).
rosq() { timeout 25 python3 "$HERE/rosq.py" "$@"; }

# _legs_down JSON : the controller manager answered and no leg controller is active (robot limp/lying).
_legs_down() { local st; st=$(_legs "$1"); [[ "$st" == *'"reachable":true'* && "$st" == *'"active_legs":[]'* ]]; }

# fc_lie_down : deactivate the leg controllers (the stock R3 action: kp=0, kd=estop_kd, i.e. a damped
# collapse), then CONFIRM from a fresh controller list that none is active. 0 = confirmed; 1 = not confirmed
# (LIE_WHY says why). LIE_JSON = the switch result. Never decides on the switch reply alone.
fc_lie_down() {
  local q i st
  LIE_WHY=""
  LIE_JSON=$(rosq switch "" "$PROFILE_LEG_CONTROLLERS" 2>> "$FC_LOGS/ready.log")
  q=$LIE_JSON
  _legs_down "$q" && return 0
  for i in 1 2 3; do
    sleep 1
    q=$(rosq controllers 2>> "$FC_LOGS/ready.log")
    _legs_down "$q" && return 0
  done
  st=$(_legs "$q")
  if [[ "$st" != *'"reachable":true'* ]]; then LIE_WHY="controller manager not answering, state unknown"
  else LIE_WHY="leg controller still active: $(python3 -c 'import json,sys; print(" ".join(json.loads(sys.argv[1])["active_legs"]))' "$st" 2>/dev/null)"; fi
  return 1
}

# fc_gentle_down RUN : stock gentle lie-down. If this run loaded the lie-down controller (RUN/stack.json) and a
# leg controller is active, hand the legs to it with ONE STRICT switch (walker off + lie-down on in the same
# controller-manager cycle, or nothing changes): its init blend lowers the body from the current pose to the
# lying pose under position control (kp 7.5 / kd 0.25, like standing up), then it holds the pose. The caller
# waits GENTLE_WAIT s (blend + FC_LIEDOWN_HOLD_S), then deactivates (fc_lie_down): the robot rests on its belly.
# 0 = lowering started; 1 = not available or refused (GENTLE_WHY; nothing changed: the caller drops as before);
# 2 = nothing to do (no leg controller active). Sim evidence: sim/liedown_sim.py.
fc_gentle_down() {
  local run=$1 ctl init q sw st
  GENTLE_WHY="" GENTLE_WAIT=0
  ctl=$(_stack_field "$run/stack.json" liedown | python3 -c 'import json,sys
try: d=json.loads(sys.stdin.read() or "null"); print("%s %s" % (d["controller"], d["init_s"]))
except Exception: print("")')
  q=$(rosq controllers 2>> "$FC_LOGS/ready.log")
  if [ -z "$ctl" ] || [ -z "${PROFILE_LIEDOWN:-}" ]; then
    _legs_down "$q" && { GENTLE_WHY="no leg controller active"; return 2; }
    GENTLE_WHY="no lie-down controller in this run (tej_u6, FC_LIEDOWN=0 or an older start)"; return 1
  fi
  read -r ctl init <<< "$ctl"
  st=$(python3 -c 'import json,sys
try: c=json.loads(sys.argv[1]).get("controllers") or {}
except Exception: c={}
legs=[x for x in sys.argv[3].split() if x != sys.argv[2] and c.get(x) == "active"]
print("%s %s" % (c.get(sys.argv[2], "missing"), "active" if legs else "none"))' "$q" "$ctl" "$PROFILE_LEG_CONTROLLERS")
  case "$st" in
    "missing "*) GENTLE_WHY="$ctl not loaded (controller manager: ${q:0:80})"; return 1 ;;
    *" none") GENTLE_WHY="no leg controller active"; return 2 ;;
    "active "*) GENTLE_WHY="$ctl already active with another leg controller"; return 1 ;;
  esac
  sw=$(rosq switch "$ctl" "$PROFILE_LEG_CONTROLLERS" strict 2>> "$FC_LOGS/ready.log")
  st=$(python3 -c 'import json,sys
try: c=json.loads(sys.argv[1]).get("controllers") or {}
except Exception: c={}
others=[x for x in sys.argv[3].split() if x != sys.argv[2] and c.get(x) == "active"]
print("ok" if c.get(sys.argv[2]) == "active" and not others else "no")' "$sw" "$ctl" "$PROFILE_LEG_CONTROLLERS")
  if [ "$st" != ok ]; then GENTLE_WHY="STRICT switch to $ctl not accepted ($(_json_field "$sw" error))"; return 1; fi
  GENTLE_WAIT=$(awk -v a="$init" -v b="${FC_LIEDOWN_HOLD_S:-1.0}" 'BEGIN { print a + b }')
  return 0
}

# fc_bridge_park : ask the running bridge to PARK (SIGUSR1): from then on it publishes only zero, but it keeps
# admitting packets, so a link E-STOP (and its fallen cutoff) can still switch the legs off (the bridge's --legs
# include the lie-down controller) while the robot is lowered. 0 = the bridge confirmed in its log within 1 s;
# 1 = no bridge or no confirmation (the caller stops it instead, as before: no link E-STOP while lowering).
fc_bridge_park() {
  local pid log=$FC_LOGS/bridge.log size i
  pid=$(fc_pid bridge) || return 1
  size=$(stat -c %s "$log" 2>/dev/null || echo 0)
  kill -USR1 "$pid" 2>/dev/null || return 1
  for i in $(seq 20); do
    tail -c +$((size + 1)) "$log" 2>/dev/null | grep -q "PARKED pid=$pid" && return 0
    sleep 0.05
  done
  return 1
}

# fc_bridge_parked : the running bridge was parked (a down/stop that did not finish): ready restarts it.
fc_bridge_parked() { local pid; pid=$(fc_pid bridge) || return 1; grep -q "PARKED pid=$pid" "$FC_LOGS/bridge.log" 2>/dev/null; }

_estopped() { [ -f "$FC_RUN/estop.json" ]; }
_estop_why() { local w; w=$(_stack_field "$FC_RUN/estop.json" reason); echo "${w:-unknown}"; }

# fc_lower_wait SECONDS : wait out the lie-down blend + hold, checking every 0.1 s for the bridge's e-stop latch
# (link E-STOP or fallen cutoff: the bridge has already sent the deactivation, which stops the blend at once).
# 0 = done; 3 = aborted (LOWER_WHY = latch reason). R3 aborts too (the stock estop_controller lists the
# lie-down controller via the overlay) but leaves no latch: the following confirmed deactivation covers it.
fc_lower_wait() {
  local end
  LOWER_WHY=""
  end=$(awk -v a="$(_now)" -v b="$1" 'BEGIN { printf "%.3f", a + b }')
  while :; do
    _estopped && { LOWER_WHY=$(_estop_why); return 3; }
    awk -v a="$(_now)" -v e="$end" 'BEGIN { exit !(a < e) }' || return 0
    sleep 0.1
  done
}

# fc_topology_wait SECONDS : (profile env sourced) wait until rosq reports the velocity topology ok.
fc_topology_wait() {
  local end=$((SECONDS + $1)) verdict last=""
  while :; do
    verdict=$(rosq probe "$PROFILE_CMD_TOPIC" 2>/dev/null | python3 -c '
import json, sys
try:
    m = json.loads(sys.stdin.read())["mux"]
except Exception:
    print("BAD no probe"); sys.exit()
print(("OK /cmd_vel<-%s %s->%s teleop=%s" % (m["cmd_vel_pubs"], m["topic"], m["topic_subs"], m["teleop_pubs"]))
      if m["ok"] else "BAD " + "; ".join(m["problems"]))')
    if [ "${verdict%% *}" = OK ]; then fc_log "topology ${verdict#OK }"; return 0; fi
    [ "$verdict" != "$last" ] && { fc_log "topology not ok: ${verdict#BAD }"; last=$verdict; }
    [ $SECONDS -lt $end ] || return 1
    sleep 1
  done
}

# ---------------------------------------------------------------- commands
cmd_status() {
  local prof run alive=false stack=false foreign=false q='null' bpid=''
  prof=$(active_profile || true); run=$(active_run || true)
  if [ -n "$prof" ] && load_profile "$prof" > /dev/null && profile_alive "$run"; then alive=true; fi
  pgrep -f ros2_control_node > /dev/null && stack=true
  foreign_stack && foreign=true
  if $alive; then profile_env; q=$(rosq probe "$PROFILE_CMD_TOPIC" 2>/dev/null) || q='null'; [ -n "$q" ] || q='null'; fi
  bpid=$(fc_pid bridge || true)
  local cpid; cpid=$(fc_pid cam || true)
  local temp thr batt='null'
  temp=$(vcgencmd measure_temp 2>/dev/null | sed -n "s/temp=\([0-9.]*\).*/\1/p")
  thr=$(vcgencmd get_throttled 2>/dev/null | sed -n 's/throttled=//p')
  if [ "${FC_STATUS_BATTERY:-0}" = 1 ]; then  # opt-in: reads the ADC on the shared I2C bus
    batt=$(timeout 3 python3 /home/pi/pupper-foundation-session-20261029/battery-monitor-v1/pupperv3-monorepo/robot/utils/check_batt_voltage.py 2>/dev/null |
      sed -n 's/^Battery:[[:space:]]*\([0-9]*\)%[[:space:]]*\([0-9.]*\)V.*/{"pct":\1,"volts":\2}/p' | head -1)
    [ -n "$batt" ] || batt='null'
  fi
  local stale=false; active_stale && stale=true
  local heat; heat=$(timeout 5 python3 "$HERE/heat.py" 2>/dev/null | tail -1); [[ "$heat" == \{*\} ]] || heat='null'
  FC_HEAT=$heat FC_CAM_PID=$cpid python3 - "$prof" "$run" "$alive" "$stack" "$foreign" "$bpid" "${temp:-}" "${thr:-}" "$q" "$batt" \
    "$(cat /proc/loadavg)" "$(fc_pid starter || true)" "${PROFILE_HEADING_HOLD:-}" \
    "${PROFILE_WALKER:-}" "${PROFILE_LEG_CONTROLLERS:-}" "$stale" "$HERE" "$FC_RUN/boot.json" "$FC_RUN/estop.json" <<'PY'
import json, os, sys, time
prof, run, alive, stack, foreign, bpid, temp, thr, q, batt, load, starter, hh = sys.argv[1:14]
walker, legs, stale, here, boot_file, latch_file = sys.argv[14:20]
sys.path.insert(0, here)
import readiness
overlay, st_rec = None, None
try:  # the start's options (fc_stack_overlay / fc_stack_record); older runs have none
    with open(os.path.join(run, "stack.json")) as f:
        st_rec = json.load(f)
    if st_rec.get("policy"):    # heading-hold-off policy of the overlay
        overlay = {"kind": st_rec.get("kind"), "verified": st_rec.get("verified"), "policy": st_rec.get("policy"),
                   "sha256": st_rec.get("sha256"), "heading_hold": st_rec.get("hh_policy")}
except (OSError, ValueError, AttributeError):
    st_rec = None
if st_rec and st_rec.get("heading_hold") in ("on", "off"):
    hh = st_rec["heading_hold"]
estop = None
if os.path.exists(latch_file):  # the bridge's e-stop latch (fail closed: unreadable still counts)
    try:
        with open(latch_file) as f:
            estop = json.load(f)
    except (OSError, ValueError):
        estop = {"reason": "unreadable_latch_file"}
ready = bool(run) and os.path.exists(os.path.join(run, "READY"))
imu_cal = None
try:  # accel bias calibration (imu/imu_level.py): stored file vs what the running driver logged at start
    sys.path.insert(0, os.path.join(here, "imu"))
    import imu_level
    try:
        with open(imu_level.DEFAULT_CAL) as f:
            stored = json.load(f)
        stored = {"accel_bias_chip": stored.get("accel_bias_chip"), "measured_utc": stored.get("measured_utc"),
                  "raw_tilt_deg": stored.get("raw_tilt_deg")}
    except (OSError, ValueError):
        stored = None
    log = ""
    if run:
        try:
            with open(os.path.join(run, "robot.log"), errors="replace") as f:
                log = f.read()
        except OSError:
            pass
    b, applied = imu_level.applied_bias_from_log(log)
    imu_cal = {"file": imu_level.DEFAULT_CAL, "stored": stored, "applied": b if applied else None,
               "driver_supports_cal": applied is not None if run else None}
except Exception as error:  # status must never fail on this
    imu_cal = {"error": repr(error)}
ros = json.loads(q) if q and q != "null" else None
mux = (ros or {}).get("mux") or {}
failed = bool(run) and os.path.exists(os.path.join(run, "FAILED"))
ph = readiness.phase({  # noqa
    "profile": prof, "profile_alive": alive == "true", "starting": bool(starter), "ready": ready,
    "start_failed": failed, "stale": stale == "true", "foreign_stack": foreign == "true",
    "ros2_control_node": stack == "true", "ros": ros, "bridge_alive": bool(bpid),
    "walker": walker, "leg_controllers": legs.split()})
ctl = readiness.legs((ros or {}).get("controllers"), walker, legs.split()) if ros else None
try:  # written by 'fcctl.sh boot' (fc-boot.service)
    with open(boot_file) as f:
        boot = json.load(f)
except (OSError, ValueError):
    boot = None
detail = ph["detail"]
if estop is not None:
    detail = "E-STOPPED (%s): ready --confirm re-arms; " % estop.get("reason") + detail
print(json.dumps({
    "ok": True, "t": round(time.time(), 3), "host": os.uname().nodename,
    "phase": ph["phase"], "light": ph["light"], "phase_detail": detail,
    "estopped": estop is not None, "estop": estop,
    "walker": walker or None, "walker_active": ctl["walker_active"] if ctl else None,
    "active_legs": ctl["active_legs"] if ctl else None, "boot": boot,
    "profile": prof or None, "run_dir": run or None,
    "profile_alive": alive == "true", "starting": bool(starter), "ready": ready, "start_failed": failed,
    "heading_hold": (hh or None) if prof else None, "heading_hold_overlay": overlay,   # "on"/"off"; null = no profile
    # walker forward/jump-slot cap of the running stack: the start's --speed-cap, else the stock 0.49
    "speed_cap": ((st_rec or {}).get("speed_cap_effective") or 0.49) if prof else None,
    "speed_cap_source": ("overlay" if (st_rec or {}).get("speed_cap") is not None else "default") if prof else None,
    "liedown": bool((st_rec or {}).get("liedown")) if prof else None,   # gentle lie-down controller loaded
    # --lean: launch nodes not started (foxglove_bridge, camera_node); camera = onboard camera node launched
    "lean": (st_rec or {}).get("lean") if prof else None,
    "not_launched": (st_rec or {}).get("not_launched") if prof else None,
    "camera": (st_rec or {}).get("camera") if prof else None,
    "stack": st_rec,
    "imu_cal": imu_cal,   # accel bias: stored run/imu_cal.json vs applied by the running driver (v4)
    "ros2_control_node": stack == "true", "foreign_stack": foreign == "true",
    "mux_alive": mux.get("alive"), "mux_ok": mux.get("ok"),   # null = not probed (no live profile)
    "dds": (ros or {}).get("dds"), "discovery": (ros or {}).get("discovery"),
    "ros": ros,
    "bridge": {"pid": int(bpid) if bpid else None, "alive": bool(bpid)},
    "cam": {"pid": int(os.environ["FC_CAM_PID"]) if os.environ.get("FC_CAM_PID") else None,   # cam start (POV feed)
            "alive": bool(os.environ.get("FC_CAM_PID"))},
    "temp_c": float(temp) if temp else None, "throttled": thr or None,
    "battery": json.loads(batt) if batt != "null" else None,
    "load": [float(x) for x in load.split()[:3]],
    # heat.py: CPU governor/MHz, fan state/rpm, top processes by CPU over 0.5 s (pct per core)
    "heat": json.loads(os.environ.get("FC_HEAT") or "null"),
}, separators=(",", ":")))
PY
}

cmd_start() {
  local name="" confirm=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --confirm) confirm=$1; shift ;;
      --speed-cap|--heading-hold|--lean|--camera) stack_opt "$1" "${2:-}" || die "$STACK_ERR"; shift 2 ;;
      -*) die "unknown start option '$1' (start <profile> --confirm [--speed-cap X] [--heading-hold on|off] [--lean on|off] [--camera on|off])" ;;
      *) [ -z "$name" ] || die "one profile only"; name=$1; shift ;;
    esac
  done
  [ -n "$name" ] || die "usage: start <profile> --confirm [--speed-cap X] [--heading-hold on|off]"
  [ "$confirm" = "--confirm" ] || die "start homes the legs: put the robot on the stand, then rerun with --confirm"
  [ -z "${FC_ROS_DOMAIN_ID:-}" ] || die "FC_ROS_DOMAIN_ID is for bridge tests only; unset it to start a stack"
  lock
  load_profile "$name"
  apply_stack_opts || die "$STACK_ERR"
  if _cam_effective && fc_alive cam; then   # the camera node needs the sensor 'cam start' (picamera2) holds
    fc_log "stopping the POV feed (cam): this stack launches the camera node"; fc_signal_group cam INT 5
  fi
  local prev; prev=$(active_profile || true)
  if [ -n "$prev" ]; then
    ( load_profile "$prev" && profile_alive "$(active_run)" ) && die "profile '$prev' is running; fcctl.sh stop first"
    fc_alive starter && die "profile '$prev' is still starting"
    fc_log "clearing stale state for '$prev'"; rm -f "$FC_RUN/active"
  fi
  pgrep -f 'ros2_control_node|persistent_session.py|ros2 launch' > /dev/null &&
    die "a robot stack/session not started by fcctl is running; refusing"
  local run; run=$FC_RUNS/$(date -u +%Y%m%dT%H%M%SZ)-$name
  mkdir -m 700 -p "$run"
  printf '%s\n%s\n%s\n' "$name" "$run" "$(boot_id)" > "$FC_RUN/active"
  fc_log "!!! HOMING: starting '$name' moves the legs. Robot ON THE STAND, R3 in reach."
  fc_log "temp $(vcgencmd measure_temp 2>/dev/null) $(vcgencmd get_throttled 2>/dev/null) run $run"
  FC_STACK_SPEED_CAP=$STACK_SPEED_CAP FC_STACK_HEADING_HOLD=$STACK_HH FC_STACK_LEAN=$STACK_LEAN FC_STACK_CAMERA=$STACK_CAMERA \
    fc_spawn starter "$run/starter.log" bash "$HERE/fcctl.sh" _starter "$name" "$run" || die "could not spawn starter"
  unlock   # the starter now owns the start; 'stop' must work during homing
  local end=$((SECONDS + 160)) shown=0
  while [ $SECONDS -lt $end ]; do
    if [ -f "$run/starter.log" ]; then
      local n; n=$(wc -l < "$run/starter.log"); [ "$n" -gt "$shown" ] && { tail -n +$((shown + 1)) "$run/starter.log" >&2; shown=$n; }
    fi
    [ -f "$run/READY" ] && break
    [ -f "$run/FAILED" ] && break
    fc_alive starter || { [ -f "$run/READY" ] || [ -f "$run/FAILED" ] || { touch "$run/FAILED"; break; }; }
    sleep 0.5
  done
  if [ -f "$run/READY" ]; then
    fc_log "ready. Next: on the floor 'fcctl.sh ready' (stands + bridge), or press X (tej_u6: Options) and 'fcctl.sh bridge start'."
    printf '{"ok":true,"profile":"%s","run_dir":"%s","walker":"%s","cmd_topic":"%s","heading_hold":"%s","speed_cap":%s,"liedown":%s,"lean":%s,"camera":%s}\n' \
      "$name" "$run" "$PROFILE_WALKER" "$PROFILE_CMD_TOPIC" "${PROFILE_HEADING_HOLD:-}" \
      "$(_stack_field "$run/stack.json" speed_cap_effective | grep -E '^[0-9.]+$' || echo null)" \
      "$([ -n "$(_stack_field "$run/stack.json" liedown)" ] && echo true || echo false)" \
      "$(_stack_field "$run/stack.json" lean | sed -nE 's/^(on|off)$/"\1"/p' | grep . || echo null)" \
      "$(_stack_field "$run/stack.json" camera | sed -nE 's/^True$/true/p; s/^False$/false/p' | grep . || echo null)"
  else
    fc_log "start failed or timed out; logs in $run. Run 'fcctl.sh stop' to clean up what was started."
    printf '{"ok":false,"profile":"%s","run_dir":"%s","error":"start failed"}\n' "$name" "$run"
    return 1
  fi
}

cmd__starter() {  # internal: runs detached so an ssh drop cannot interrupt homing
  local name=$1 run=$2
  load_profile "$name"
  apply_stack_opts || { fc_log "$STACK_ERR"; touch "$run/FAILED"; return 1; }
  fc_stack_record "$run"     # overwritten by fc_stack_overlay when the profile builds an overlay
  if profile_start "$run"; then touch "$run/READY"; fc_log "profile $name ready"
  else touch "$run/FAILED"; fc_log "profile $name failed"; fi
}

cmd_bridge() {
  local sub=${1:-} prof="" rid="" allow="" extra=()
  shift || true
  while [ $# -gt 0 ]; do
    case "$1" in
      --profile) prof=${2:-}; shift 2 || shift ;;
      --robot-id) rid=${2:-}; shift 2 || shift ;;
      --allow-from) allow=${2:-}; shift 2 || shift ;;
      --reset-replay|--no-imu-stop|--no-mux-guard) extra+=("$1"); shift ;;
      *) die "unknown bridge option '$1'" ;;
    esac
  done
  case "$sub" in
    start)
      lock
      fc_alive bridge && die "bridge already running (pid $(fc_pid bridge))"
      [ -n "$prof" ] || prof=$(active_profile || true)
      [ -n "$prof" ] || die "no running profile; use 'bridge start --profile <name>'"
      [ -n "$rid" ] || rid=$(sed -n 1p "$HERE/robot_id" 2>/dev/null || true)
      [[ "$rid" =~ ^[A-Za-z0-9_.-]{1,32}$ ]] || die "no robot id: re-run stage.sh or pass --robot-id <id>"
      [[ "$allow" =~ ^[0-9.,]*$ ]] || die "bad --allow-from '$allow'"
      load_profile "$prof"
      profile_env
      [ -n "${FC_ROS_DOMAIN_ID:-}" ] && export ROS_DOMAIN_ID=$FC_ROS_DOMAIN_ID
      # vx ceiling = the running stack's walker cap (stack.json speed_cap_effective: --speed-cap, else 0.49);
      # a run without stack.json (older start) keeps the historic 0.5
      local maxvx; maxvx=$(_stack_field "$(active_run)/stack.json" speed_cap_effective 2>/dev/null | grep -E '^[0-9.]+$' || echo 0.5)
      # IMU lie-down (drive_bridge --imu-lie): the run's lie-down controller + blend/hold time, as fc_gentle_down
      local lie=() ld ld_ctl ld_init; ld=$(_stack_field "$(active_run)/stack.json" liedown 2>/dev/null | python3 -c 'import json,sys
try: d=json.loads(sys.stdin.read() or "null"); print("%s %s" % (d["controller"], float(d["init_s"])))
except Exception: print("")')
      if [ -n "$ld" ] && [ -n "${PROFILE_LIEDOWN:-}" ]; then
        read -r ld_ctl ld_init <<< "$ld"
        lie=(--liedown "$ld_ctl" --liedown-wait "$(awk -v a="$ld_init" -v b="${FC_LIEDOWN_HOLD_S:-1.0}" 'BEGIN { print a + b }')")
      fi
      fc_spawn bridge "$FC_LOGS/bridge.log" python3 -u "$HERE/drive_bridge.py" --profile "$prof" \
        --topic "$PROFILE_CMD_TOPIC" --imu-topic "$PROFILE_IMU_TOPIC" --port "${FC_BRIDGE_PORT:-9870}" \
        --robot-id "$rid" --key-file "$HERE/link.key" --state-file "$FC_RUN/bridge_state.json" \
        --max-vx "$maxvx" --walker "$PROFILE_WALKER" --legs "$PROFILE_LEG_CONTROLLERS" \
        --latch-file "$FC_RUN/estop.json" ${allow:+--allow-from "$allow"} "${lie[@]}" "${extra[@]}"
      sleep 1.5
      fc_alive bridge || die "bridge exited (missing/loose link.key? bad state file?); see $FC_LOGS/bridge.log"
      [ -f "$FC_RUN/estop.json" ] && fc_log "bridge started LATCHED (run/estop.json): it publishes only zero until 'ready --confirm'"
      printf '{"ok":true,"bridge_pid":%s,"profile":"%s","robot_id":"%s","topic":"%s","domain":%s,"max_vx":%s,"estopped":%s}\n' \
        "$(fc_pid bridge)" "$prof" "$rid" "$PROFILE_CMD_TOPIC" "${ROS_DOMAIN_ID:-0}" "$maxvx" \
        "$([ -f "$FC_RUN/estop.json" ] && echo true || echo false)"
      ;;
    stop)
      fc_signal_group bridge INT 5 || die "bridge did not stop"
      echo '{"ok":true,"bridge":"stopped"}'
      ;;
    *) die "usage: bridge start [--profile P] [--robot-id ID] [--allow-from IP[,IP]] [--reset-replay] [--no-imu-stop] [--no-mux-guard] | bridge stop" ;;
  esac
}

# cam: the optional robot-POV MJPEG feed (camstream.py). Never in the control path; measured cost in README.
cmd_cam() {
  local sub=${1:-} width=160 fps=5 quality=60 port=${FC_CAM_PORT:-8090}
  shift || true
  while [ $# -gt 0 ]; do
    case "$1" in
      --width) width=${2:-}; shift 2 || shift ;;
      --fps) fps=${2:-}; shift 2 || shift ;;
      --quality) quality=${2:-}; shift 2 || shift ;;
      *) die "unknown cam option '$1' (cam start [--width W] [--fps F] [--quality Q] | cam stop)" ;;
    esac
  done
  case "$sub" in
    start)
      [[ "$width" =~ ^[0-9]+$ ]] && [ "$width" -ge 64 ] && [ "$width" -le 640 ] || die "--width needs 64..640 (got '$width')"
      [[ "$fps" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk -v v="$fps" 'BEGIN { exit !(v >= 1 && v <= 15) }' || die "--fps needs 1..15 (got '$fps')"
      [[ "$quality" =~ ^[0-9]+$ ]] && [ "$quality" -ge 20 ] && [ "$quality" -le 90 ] || die "--quality needs 20..90 (got '$quality')"
      lock
      fc_alive cam && die "cam already running (pid $(fc_pid cam)); 'cam stop' first"
      # A running stack that launched the camera node owns the sensor: subscribe to its topic (read-only).
      # Otherwise (lean stock = no camera node, or no stack) open the sensor here.
      local src=picamera2 prof run cam_node
      prof=$(active_profile || true); run=$(active_run || true)
      if [ -n "$prof" ] && _profile_ok "$prof" && load_profile "$prof" && profile_alive "$run"; then
        cam_node=$(_stack_field "$run/stack.json" camera 2>/dev/null)
        if [ "$cam_node" != False ]; then src=ros; profile_env; fi
      fi
      fc_spawn cam "$FC_LOGS/cam.log" python3 -u "$HERE/camstream.py" --source "$src" --width "$width" --fps "$fps" \
        --quality "$quality" --port "$port" || die "could not spawn camstream"
      local i frames=0
      for i in $(seq "$(( ${FC_CAM_WAIT_S:-10} * 2 ))"); do
        sleep 0.5
        fc_alive cam || break
        frames=$(python3 -c 'import json,sys,urllib.request
try: print(int(json.load(urllib.request.urlopen(sys.argv[1], timeout=1))["frames"]))
except Exception: print(0)' "http://127.0.0.1:$port/health")
        [ "$frames" -gt 0 ] && break
      done
      if [ "$frames" -le 0 ]; then
        fc_signal_group cam INT 3
        die "camstream ($src) gave no frame (sensor busy or no camera topic?); see $FC_LOGS/cam.log"
      fi
      local ip; ip=$(hostname -I 2>/dev/null | awk '{print $1}')
      printf '{"ok":true,"cam_pid":%s,"source":"%s","url":"http://%s:%s/stream.mjpg","width":%s,"fps":%s,"quality":%s}\n' \
        "$(fc_pid cam)" "$src" "${ip:-127.0.0.1}" "$port" "$width" "$fps" "$quality"
      ;;
    stop)
      fc_signal_group cam INT 5 || die "cam did not stop"
      echo '{"ok":true,"cam":"stopped"}'
      ;;
    *) die "usage: cam start [--width W] [--fps F] [--quality Q] | cam stop" ;;
  esac
}

cmd_walk() {  # walk | rest
  local prof; prof=$(active_profile || true); [ -n "$prof" ] || die "no running profile"
  load_profile "$prof"; profile_env
  if [ "$1" = walk ]; then rosq switch "$PROFILE_WALKER" "$PROFILE_LEG_CONTROLLERS"
  else rosq switch "" "$PROFILE_LEG_CONTROLLERS"; fi
}

# stop [--force] : never cuts motor power under a standing robot. If the stack has a leg controller that
# cannot be confirmed inactive (and ros2_control_node runs and the profile reached READY), it refuses and
# leaves the stack running. --force overrides: UNSAFE, a standing robot then drops with the motors off.
cmd_stop() {
  local force=0
  case "${1:-}" in --force) force=1 ;; "") ;; *) die "usage: stop [--force]" ;; esac
  lock
  local rc=0 prof run deact='null' parked=0
  prof=$(active_profile || true); run=$(active_run || true)
  fc_signal_group cam INT 5 || rc=1   # the optional POV feed (cam start) never outlives a stop
  # A READY stack with a bridge: park it (zero only, link E-STOP still armed) for the lie-down; else stop it now.
  if [ -n "$prof" ] && [ -f "$run/READY" ] && fc_bridge_park; then parked=1; fc_log "bridge parked (zero only; link E-STOP armed)"
  else fc_signal_group bridge INT 5 || rc=1; fi   # bridge publishes one zero Twist on exit
  if [ -z "$prof" ]; then echo "{\"ok\":$([ $rc = 0 ] && echo true || echo false),\"stopped\":null}"; return $rc; fi
  load_profile "$prof"
  fc_signal_group starter TERM 3 || rc=1
  fc_adopt_orphans
  if profile_alive "$run"; then
    profile_env
    local g=0
    if _estopped; then g=4; GENTLE_WHY="e-stopped ($(_estop_why))"
    elif [ -f "$run/READY" ]; then fc_gentle_down "$run"; g=$?; else g=1; GENTLE_WHY="stack never reached READY"; fi
    case $g in
      0) fc_log "lowering under position control (lie-down controller, ${GENTLE_WAIT} s; link E-STOP / R3 abort)"
         if fc_lower_wait "$GENTLE_WAIT"; then fc_log "lowered; deactivating leg controllers (resting on its belly)"
         else fc_log "LOWERING ABORTED by E-STOP ($LOWER_WHY): legs switched off part-way (damped drop); confirming"; fi ;;
      2) fc_log "no leg controller active; confirming before stopping" ;;
      4) fc_log "$GENTLE_WHY: the bridge already switched the legs off; confirming before stopping" ;;
      *) fc_log "gentle lie-down unavailable ($GENTLE_WHY): deactivating leg controllers, a standing robot WILL DROP (damped, same as R3)" ;;
    esac
    [ $parked = 0 ] || { fc_signal_group bridge INT 5 || rc=1; parked=0; }
    if fc_lie_down; then
      sleep 1
    elif ! pgrep -f ros2_control_node > /dev/null; then
      fc_log "ros2_control_node not running: motors are not driven; stopping"
    elif [ ! -f "$run/READY" ]; then
      fc_log "lie-down not confirmed ($LIE_WHY) but the stack never reached READY (homing/loading); stopping"
    elif [ $force = 1 ]; then
      fc_log "--force: stopping although not confirmed lying ($LIE_WHY). UNSAFE if it is standing."
    else
      fc_log "STILL STANDING? $LIE_WHY: stack left running. Use R3 or support the robot, then stop again (--force = unsafe)"
      printf '{"ok":false,"stopped":null,"run_dir":"%s","error":"%s","left_running":"stack","deactivate":%s}\n' \
        "$run" "$(jesc "not confirmed lying ($LIE_WHY): use R3 or support the robot; stack left running (stop --force is unsafe)")" \
        "$(_raw_json "$LIE_JSON")"
      return 1
    fi
    deact=$(_raw_json "$LIE_JSON")
  fi
  [ $parked = 0 ] || fc_signal_group bridge INT 5 || rc=1
  profile_stop "$run" || rc=1
  local left=""
  for r in launch teleop mux starter bridge; do fc_alive "$r" && left+="$r "; done
  if [ -z "$left" ] && ! profile_alive "$run"; then rm -f "$FC_RUN/active"; else rc=1; fi
  printf '{"ok":%s,"stopped":"%s","run_dir":"%s","left_running":"%s","deactivate":%s}\n' \
    "$([ $rc = 0 ] && echo true || echo false)" "$prof" "$run" "$left" "$deact"
  return $rc
}

# ---------------------------------------------------------------- ready / down / boot (JSON-lines progress)
# Each progress line: {"phase":P,"ok":bool,"elapsed":s,"msg":text,...}; the last line has "done":true.
_now() { if [ -n "${EPOCHREALTIME:-}" ]; then echo "${EPOCHREALTIME/,/.}"; else date +%s.%N; fi; }
_elapsed() { awk -v a="$(_now)" -v b="${READY_T0:-0}" 'BEGIN { printf "%.1f", a - b }'; }
jesc() { local s=${1//\\/\\\\}; s=${s//\"/\\\"}; s=${s//$'\n'/ }; s=${s//$'\t'/ }; s=${s//$'\r'/ }; printf '%s' "$s"; }
emit() {  # emit PHASE true|false MSG [RAW_JSON_TAIL starting with ,]
  printf '{"phase":"%s","ok":%s,"elapsed":%s,"msg":"%s"%s}\n' "$1" "$2" "$(_elapsed)" "$(jesc "$3")" "${4:-}"
  fc_log "$1: $3"
}
ready_fail() {  # ready_fail AT MSG EXIT
  emit error false "$2" ",\"at\":\"$1\",\"exit\":$3,\"done\":true"; exit "$3"
}
_json_field() { python3 -c 'import json,sys
try: print(json.loads(sys.argv[1]).get(sys.argv[2]) or "")
except Exception: print("")' "$1" "$2"; }
_legs() { python3 "$HERE/readiness.py" legs "$1" "$PROFILE_WALKER" "$PROFILE_LEG_CONTROLLERS"; }
_profile_ok() { [[ "$1" =~ ^[a-z0-9_]+$ ]] && [ -f "$HERE/profiles/$1.sh" ]; }
_raw_json() { if [[ "$1" == \{*\} ]]; then printf '%s' "$1"; else printf 'null'; fi; }

# _wait_start RUN TIMEOUT : wait for the (already running) starter to finish; heartbeat every 5 s.
_wait_start() {
  local run=$1 end=$((SECONDS + $2)) beat=$SECONDS
  while [ $SECONDS -lt $end ]; do
    [ -f "$run/READY" ] && return 0
    [ -f "$run/FAILED" ] && return 1
    fc_alive starter || { [ -f "$run/READY" ]; return; }
    [ $((SECONDS - beat)) -ge 5 ] && { emit starting true "homing / loading controllers"; beat=$SECONDS; }
    sleep 0.5
  done
  return 1
}

# imu_rest_check : one JSON line, tilt from the published orientation over 2 s (imu/imu_level.py check).
# "ok": true/false against FC_READY_IMU_MAX_DEG (default 6 = ImuHealth warn); "ok": null = no samples.
# Profiles (and the tests' fake profile) may override it.
imu_rest_check() {
  FC_IMU_TOPIC=$PROFILE_IMU_TOPIC timeout 20 python3 "$HERE/imu/imu_level.py" check --seconds 2 \
    --max-deg "${FC_READY_IMU_MAX_DEG:-6}"
}

cmd_ready() {
  local name="" confirm="" stand_now="" bridge=1 a rearmed=0 skip_imu=""
  READY_T0=$(_now)
  while [ $# -gt 0 ]; do
    a=$1; shift
    case "$a" in
      --confirm) confirm=1 ;;
      --stand-now) stand_now=1 ;;
      --no-bridge) bridge=0 ;;
      --skip-imu-check) skip_imu=1 ;;
      --speed-cap|--heading-hold|--lean|--camera) stack_opt "$a" "${1:-}" || ready_fail args "$STACK_ERR" 2; shift ;;
      -*) ready_fail args "unknown option '$a' (ready [profile] [--confirm [--stand-now]] [--no-bridge] [--skip-imu-check] [--speed-cap X] [--heading-hold on|off] [--lean on|off] [--camera on|off])" 2 ;;
      *) [ -z "$name" ] || ready_fail args "one profile only" 2; name=$a ;;
    esac
  done
  [ -z "${FC_ROS_DOMAIN_ID:-}" ] || ready_fail args "FC_ROS_DOMAIN_ID is for bridge tests only; unset it" 2
  exec 8> "$FC_RUN/ready.lock"
  flock -n 8 || ready_fail args "another 'ready' or 'down' is running" 1
  local cur="" prev; prev=$(active_profile || true)
  if [ -n "$prev" ] && ! active_stale && _profile_ok "$prev"; then
    if fc_alive starter || ( load_profile "$prev" > /dev/null && profile_alive "$(active_run)" ); then cur=$prev; fi
  fi
  [ -n "$name" ] || name=$cur
  [ -n "$name" ] || ready_fail args "no profile running: ready <profile> --confirm" 2
  _profile_ok "$name" || ready_fail args "unknown profile '$name' (have: $(cd "$HERE/profiles" && ls *.sh | sed 's/\.sh$//' | tr '\n' ' '))" 2
  [ -z "$cur" ] || [ "$cur" = "$name" ] || ready_fail profile "profile '$cur' is running, not '$name': fcctl.sh down first" 1
  load_profile "$name"
  apply_stack_opts || ready_fail args "$STACK_ERR" 2
  emit begin true "profile $name" ",\"profile\":\"$name\",\"walker\":\"$PROFILE_WALKER\""

  # 0. e-stop latch (SPEC v1.1): only a confirmed Ready re-arms. Stop the latched bridge, clear the latch;
  # step 2 then cycles the walker (an e-stopped controller can still read "active" while limp).
  if [ -f "$FC_RUN/estop.json" ]; then
    local why; why=$(_stack_field "$FC_RUN/estop.json" reason)
    if [ -z "$confirm" ]; then
      emit needs_confirm false "E-STOPPED (${why:-unknown}): the legs were switched off and the bridge is latched. Check the robot, then rerun with --confirm to re-arm" \
        ",\"profile\":\"$name\",\"estop\":\"$(jesc "${why:-unknown}")\",\"exit\":3,\"done\":true"
      exit 3
    fi
    if fc_alive bridge; then
      bash "$HERE/fcctl.sh" bridge stop > /dev/null 2>> "$FC_LOGS/ready.log" 8>&- || ready_fail rearm "latched bridge did not stop; see ~/fc-v1/logs/bridge.log" 1
    fi
    mv -f "$FC_RUN/estop.json" "$FC_LOGS/estop-$(date -u +%Y%m%dT%H%M%SZ).json" 2>/dev/null || rm -f "$FC_RUN/estop.json"
    rearmed=1
    emit rearmed true "e-stop (${why:-unknown}) cleared by the operator; standing up again" ",\"estop\":\"$(jesc "${why:-unknown}")\""
  fi

  # 1. stack up and homed
  local run
  if [ -n "$cur" ]; then
    run=$(active_run)
    if fc_alive starter; then
      emit starting true "a start is already in progress (boot service or another operator); waiting for READY"
      _wait_start "$run" 200 || ready_fail starting "start failed or timed out; see $run/starter.log, then fcctl.sh down" 1
    fi
    [ -f "$run/READY" ] || ready_fail starting "profile running but never READY; fcctl.sh down, then ready $name --confirm" 1
    # Requested stack options must match the running stack (changing them means a new start = homing).
    local have_cap have_hh
    have_cap=$(_stack_field "$run/stack.json" speed_cap_effective); have_cap=${have_cap:-0.49}
    have_hh=$(_stack_field "$run/stack.json" heading_hold); have_hh=${have_hh:-${PROFILE_HEADING_HOLD:-on}}
    if { [ -n "$STACK_SPEED_CAP" ] && ! awk -v a="$STACK_SPEED_CAP" -v b="$have_cap" 'BEGIN { exit !((a - b) ^ 2 < 1e-6) }'; } ||
       { [ -n "$STACK_HH" ] && [ "$STACK_HH" != "$have_hh" ]; }; then
      ready_fail profile "running stack has speed cap $have_cap / heading hold $have_hh, requested ${STACK_SPEED_CAP:-$have_cap} / ${STACK_HH:-$have_hh}: fcctl.sh down first (the next start homes the legs)" 1
    fi
    local have_lean have_cam
    have_lean=$(_stack_field "$run/stack.json" lean); have_cam=$(_stack_field "$run/stack.json" camera)
    if { [ -n "$STACK_LEAN" ] && [ "$STACK_LEAN" != "$have_lean" ]; } ||
       { [ "$STACK_CAMERA" = on ] && [ "$have_cam" != True ]; } ||
       { [ "$STACK_CAMERA" = off ] && [ "$have_cam" = True ] && [ "$have_lean" = on ]; }; then
      ready_fail profile "running stack has lean ${have_lean:-?} / camera ${have_cam:-?}, requested --lean ${STACK_LEAN:-same} --camera ${STACK_CAMERA:-same}: fcctl.sh down first (the next start homes the legs)" 1
    fi
    emit homed true "homed (already running)" ",\"run_dir\":\"$run\",\"speed_cap\":$have_cap,\"heading_hold\":\"$have_hh\""
  else
    if [ -z "$confirm" ]; then
      emit needs_confirm false "stack not running: starting it HOMES THE LEGS. Put the robot on the stand, then rerun with --confirm" \
        ",\"profile\":\"$name\",\"exit\":3,\"done\":true"
      exit 3
    fi
    emit starting true "starting $name: HOMING THE LEGS (robot on the stand)"
    local out=$FC_RUN/ready_start.out spid beat=$SECONDS rc res
    local sopts=()
    [ -z "$STACK_SPEED_CAP" ] || sopts+=(--speed-cap "$STACK_SPEED_CAP")
    [ -z "$STACK_HH" ] || sopts+=(--heading-hold "$STACK_HH")
    [ -z "$STACK_LEAN" ] || sopts+=(--lean "$STACK_LEAN")
    [ -z "$STACK_CAMERA" ] || sopts+=(--camera "$STACK_CAMERA")
    bash "$HERE/fcctl.sh" start "$name" --confirm "${sopts[@]}" > "$out" 2>> "$FC_LOGS/ready.log" 8>&- &
    spid=$!
    while kill -0 "$spid" 2> /dev/null; do
      sleep 0.5
      [ $((SECONDS - beat)) -ge 5 ] && { emit starting true "homing / loading controllers"; beat=$SECONDS; }
    done
    wait "$spid"; rc=$?
    res=$(grep '^{' "$out" | tail -1)
    run=$(active_run || true)
    [ "$rc" = 0 ] && [[ "$res" == *'"ok":true'* ]] ||
      ready_fail starting "start failed: $(_json_field "$res" error) (logs: ${run:-~/fc-v1/runs}; then fcctl.sh down)" 1
    if [ -z "$stand_now" ]; then  # it just homed on the stand: standing up there is not the goal
      emit homed true "homed. Move the robot from the stand to the floor, then run ready again" \
        ",\"run_dir\":\"$run\",\"next\":\"ready\",\"done\":true"
      exit 0
    fi
    emit homed true "homed, controllers loaded, mux ok (--stand-now: standing on the stand)" ",\"run_dir\":\"$run\""
  fi

  # 2. stand: the same SwitchController request the gamepad X sends (activate the walker, deactivate the
  # other leg controllers, BEST_EFFORT). A latched Stop gateway (tej_u6) refuses it: then wait for the gamepad.
  profile_env
  local st sw
  st=$(_legs "$(rosq probe "$PROFILE_CMD_TOPIC" 2>> "$FC_LOGS/ready.log")")
  [[ "$st" == *'"reachable":true'* ]] || ready_fail stand "controller manager not answering" 1
  if [ "$rearmed" = 1 ] && [[ "$st" != *'"active_legs":[]'* ]]; then
    # after an e-stop a leg controller may still read active while limp (its estop flag clears only on activation)
    st=$(_legs "$(rosq switch "" "$PROFILE_LEG_CONTROLLERS" 2>> "$FC_LOGS/ready.log")")
    [[ "$st" == *'"active_legs":[]'* ]] || ready_fail rearm "could not deactivate the e-stopped leg controllers: R3, then fcctl.sh down" 1
  fi
  if [[ "$st" == *'"walker_active":true'* ]]; then
    emit standing true "already standing on $PROFILE_WALKER"
  else
    if [ -z "$confirm" ]; then  # standing takes load: always an explicit operator action
      emit needs_confirm false "homed but not standing: standing up MOVES THE ROBOT. Clear the area, then rerun with --confirm" \
        ",\"profile\":\"$name\",\"exit\":3,\"done\":true"
      exit 3
    fi
    emit standing_up true "activating $PROFILE_WALKER (software stand: the request the gamepad X sends)"
    sw=$(rosq switch "$PROFILE_WALKER" "$PROFILE_LEG_CONTROLLERS" 2>> "$FC_LOGS/ready.log")
    st=$(_legs "$sw")
    if [[ "$st" != *'"walker_active":true'* ]]; then
      local why hint=${PROFILE_STAND_HINT:-press X on the gamepad}
      why=$(_json_field "$sw" error); [ -n "$why" ] || why="switch refused"
      emit waiting_operator true "software stand not accepted ($why): $hint" ",\"action\":\"$(jesc "$hint")\""
      local end=$((SECONDS + ${FC_READY_OPERATOR_S:-180})) beat=$SECONDS
      while :; do
        sleep 1
        profile_alive "$run" || ready_fail waiting_operator "stack exited while waiting; fcctl.sh down" 1
        st=$(_legs "$(rosq probe "$PROFILE_CMD_TOPIC" 2>> "$FC_LOGS/ready.log")")
        [[ "$st" == *'"walker_active":true'* ]] && break
        [ $SECONDS -lt $end ] || ready_fail waiting_operator "walker not active after ${FC_READY_OPERATOR_S:-180} s: $hint, then run ready again" 4
        [ $((SECONDS - beat)) -ge 10 ] && { emit waiting_operator true "$hint" ",\"action\":\"$(jesc "$hint")\""; beat=$SECONDS; }
      done
    fi
    emit standing true "$PROFILE_WALKER active; settling ${FC_READY_SETTLE_S:-3} s (blend to stance, then policy fade-in)"
    sleep "${FC_READY_SETTLE_S:-3}"
  fi
  # 2b. IMU sanity at rest (rca-20261127-tilt): standing still, the IMU should read near level. A bad accel
  # calibration otherwise surfaces seconds later as a cryptic imu_tilt lie-down. Fails BEFORE the bridge,
  # robot left standing (gamepad R3 / fcctl.sh down work as usual).
  if [ -z "$skip_imu" ] && [ -n "${PROFILE_IMU_TOPIC:-}" ]; then
    local ic; ic=$(imu_rest_check 2>> "$FC_LOGS/ready.log" | grep '^{' | tail -1)
    if [[ "$ic" == *'"ok": false'* ]]; then
      ready_fail imu "$(_json_field "$ic" error) [robot left STANDING without the bridge: fcctl.sh down; --skip-imu-check overrides]" 5
    elif [[ "$ic" == *'"ok": true'* ]]; then
      fc_log "IMU at rest OK: $ic"
    else
      fc_log "IMU at-rest check gave no verdict (continuing; the bridge guard still applies): ${ic:-no output}"
    fi
  fi
  if [ "$bridge" = 0 ]; then emit standing true "standing; bridge not started (--no-bridge)" ',"done":true'; exit 0; fi

  # 3. bridge
  if fc_bridge_parked; then   # left parked (zero only) by an interrupted down/stop: replace it
    bash "$HERE/fcctl.sh" bridge stop > /dev/null 2>> "$FC_LOGS/ready.log" 8>&- || ready_fail bridge "parked bridge did not stop; see ~/fc-v1/logs/bridge.log" 1
  fi
  if fc_alive bridge; then
    emit bridge true "bridge already running (pid $(fc_pid bridge))"
  else
    local b; b=$(bash "$HERE/fcctl.sh" bridge start 2>> "$FC_LOGS/ready.log" 8>&- | grep '^{' | tail -1)
    [[ "$b" == *'"ok":true'* ]] || ready_fail bridge "bridge start failed: $(_json_field "$b" error)" 1
    emit bridge true "bridge up" ",\"bridge\":$(_raw_json "$b")"
  fi

  # 4. verify the whole chain once more
  st=$(_legs "$(rosq probe "$PROFILE_CMD_TOPIC" 2>> "$FC_LOGS/ready.log")")
  [[ "$st" == *'"walker_active":true'* ]] || ready_fail verify "walker no longer active (R3 / gamepad?)" 1
  if [[ "$st" != *'"mux_ok":true'* ]]; then  # one probe can be a partial DDS view: recheck for up to 15 s
    fc_topology_wait 15 2>> "$FC_LOGS/ready.log" ||
      ready_fail verify "velocity topology not ok: $(rosq probe "$PROFILE_CMD_TOPIC" 2> /dev/null | python3 -c 'import json,sys
try: print("; ".join(json.load(sys.stdin)["mux"]["problems"]))
except Exception: print("probe failed")') (fcctl.sh status)" 1
  fi
  fc_alive bridge || ready_fail verify "bridge exited; see ~/fc-v1/logs/bridge.log" 1
  emit ready true "ready to play" ",\"profile\":\"$name\",\"run_dir\":\"$run\",\"done\":true"
}

# down [--keep-stack] [--force] : park the bridge (zero only, link E-STOP still armed), lie down (stock: the
# lie-down controller lowers it, with an e-stop check every 0.1 s; else leg controllers off = damped collapse,
# same as R3), stop the bridge, CONFIRM no leg controller is active, then stop the stack. Not confirmed ->
# abort, stack left running, exit 1 (--force stops anyway: UNSAFE, a standing robot drops with the motors off).
# A link E-STOP / fallen cutoff during the lowering aborts it (the bridge switches the legs off: damped drop from
# part-way down); down then carries on (confirm, stop) and its last line carries "aborted":"<reason>".
_down_bridge_stop() {
  local b; b=$(bash "$HERE/fcctl.sh" bridge stop 2>> "$FC_LOGS/ready.log" 8>&- | grep '^{' | tail -1)
  if [[ "$b" == *'"ok":true'* ]]; then emit bridge_stopped true "bridge stopped (sent one zero)"
  else emit bridge_stopped false "bridge stop: $(_json_field "$b" error)"; fi
}
cmd_down() {
  local keep=0 force=0 a
  READY_T0=$(_now)
  for a in "$@"; do
    case "$a" in --keep-stack) keep=1 ;; --force) force=1 ;; *) ready_fail args "usage: down [--keep-stack] [--force]" 2 ;; esac
  done
  exec 8> "$FC_RUN/ready.lock"
  flock -n 8 || ready_fail args "another 'ready' or 'down' is running" 1
  emit begin true "down$([ $keep = 1 ] && echo ' --keep-stack')$([ $force = 1 ] && echo ' --force')"
  local prof run alive=0 parked=0 aborted=""; prof=$(active_profile || true); run=$(active_run || true)
  if [ -z "$prof" ] || active_stale || ! _profile_ok "$prof"; then
    _down_bridge_stop
    emit down true "no fcctl profile running" ',"done":true'; exit 0
  fi
  load_profile "$prof"
  profile_alive "$run" && alive=1
  # keep the bridge up (parked: zero only) while the robot is lowered, so the link E-STOP can still act
  if [ $alive = 1 ] && [ -f "$run/READY" ] && fc_bridge_park; then
    parked=1; emit bridge_parked true "bridge parked: publishes only zero; the link E-STOP stays armed while it lies down"
  else
    _down_bridge_stop
  fi
  if [ $alive = 1 ]; then
    profile_env
    local g=1 gentle=false
    if _estopped; then g=4; GENTLE_WHY="e-stopped ($(_estop_why)): the legs were already switched off"
    elif [ -f "$run/READY" ]; then fc_gentle_down "$run"; g=$?; else GENTLE_WHY="stack never reached READY"; fi
    case $g in
      0) gentle=true
         emit lowering true "lowering the body under position control (lie-down controller, ${GENTLE_WAIT} s). Abort: E-STOP (Esc) or R3 = legs off, damped drop" ',"gentle":true'
         if ! fc_lower_wait "$GENTLE_WAIT"; then
           gentle=false aborted=$LOWER_WHY
           emit aborted false "LOWERING ABORTED by E-STOP ($aborted): the bridge switched the legs off part-way (damped drop); confirming" \
             ",\"gentle\":false,\"estop\":\"$(jesc "$aborted")\""
         fi ;;
      2) ;;
      4) emit lowering true "$GENTLE_WHY; confirming" ',"gentle":false' ;;
      *) emit lowering true "gentle lie-down unavailable ($GENTLE_WHY): switching the legs off, a standing robot WILL DROP (damped, like R3). Support it if needed" ',"gentle":false' ;;
    esac
    [ $parked = 0 ] || { _down_bridge_stop; parked=0; }
    if fc_lie_down; then
      if [ "$gentle" = true ]; then
        emit lying_down true "lowered, then leg controllers off (confirmed): resting on its belly" ",\"gentle\":true,\"switch\":$(_raw_json "$LIE_JSON")"
      else
        emit lying_down true "leg controllers off (confirmed): damped collapse, the same as R3" ",\"gentle\":false,\"switch\":$(_raw_json "$LIE_JSON")${aborted:+,\"aborted\":\"$(jesc "$aborted")\"}"
      fi
      sleep "${FC_DOWN_SETTLE_S:-1.5}"
    elif [ $force = 1 ]; then
      emit lying_down false "NOT confirmed lying ($LIE_WHY); --force: continuing (UNSAFE)" ",\"switch\":$(_raw_json "$LIE_JSON")"
    else
      emit error false "still standing? lie-down not confirmed ($LIE_WHY): use R3 or support the robot. Stack left running; down --force is unsafe" \
        ",\"at\":\"lying_down\",\"switch\":$(_raw_json "$LIE_JSON"),\"stack\":\"running\",\"exit\":1,\"done\":true"
      exit 1
    fi
  fi
  [ $parked = 0 ] || _down_bridge_stop
  local ab=${aborted:+,\"aborted\":\"$(jesc "$aborted")\"}
  if [ $keep = 1 ]; then
    emit down true "lying down, stack kept homed: ready stands it again without homing" ",\"stack\":\"kept\"$ab,\"done\":true"; exit 0
  fi
  local s; s=$(bash "$HERE/fcctl.sh" stop $([ $force = 1 ] && echo --force) 2>> "$FC_LOGS/ready.log" 8>&- | grep '^{' | tail -1)
  if [[ "$s" == *'"ok":true'* ]]; then emit down true "stack stopped" ",\"stop\":$(_raw_json "$s")$ab,\"done\":true"
  else emit down false "stop incomplete: $(_json_field "$s" error) $(_json_field "$s" left_running) still running (fcctl.sh status)" ",\"stop\":$(_raw_json "$s"),\"exit\":1,\"done\":true"; exit 1; fi
}

# boot: run by fc-boot.service at power-on (boot/README.md). Settings: ~/fc-v1/boot.env (outside robot/, so
# stage.sh --delete keeps it): FC_BOOT_PROFILE=<profile> FC_BOOT_MODE=home|idle FC_BOOT_DELAY_S=<0-300>.
#   home: wait the delay, then 'start <profile> --confirm' = HOME THE LEGS. Precondition: robot on the stand
#         when powered on. Ends at phase "homed": never stands, never starts the bridge (that is 'ready').
#   idle: record the boot and do nothing (Ready from the UI then needs --confirm to home).
# One attempt, no retry. State: run/boot.json (shown by 'status' as "boot"), log: logs/boot.log.
_boot_state() {  # _boot_state RESULT MSG
  local j
  j=$(printf '{"t":%s,"boot_id":"%s","mode":"%s","profile":"%s","result":"%s","msg":"%s"}' \
    "$(date +%s)" "$(boot_id)" "$BOOT_MODE" "$BOOT_PROFILE" "$1" "$(jesc "$2")")
  printf '%s\n' "$j" > "$FC_RUN/boot.json.tmp" && mv "$FC_RUN/boot.json.tmp" "$FC_RUN/boot.json"
  printf '%s\n' "$j"
}
cmd_boot() {
  BOOT_PROFILE=stock BOOT_MODE=home
  local delay=15 envf=$FC_ROOT/boot.env line
  if [ -f "$envf" ]; then
    while IFS= read -r line || [ -n "$line" ]; do
      [[ "$line" =~ ^[[:space:]]*(#.*)?$ ]] && continue
      if [[ "$line" =~ ^(FC_BOOT_PROFILE|FC_BOOT_MODE|FC_BOOT_DELAY_S)=([A-Za-z0-9_]*)$ ]]; then
        case ${BASH_REMATCH[1]} in
          FC_BOOT_PROFILE) BOOT_PROFILE=${BASH_REMATCH[2]} ;;
          FC_BOOT_MODE) BOOT_MODE=${BASH_REMATCH[2]} ;;
          FC_BOOT_DELAY_S) delay=${BASH_REMATCH[2]} ;;
        esac
      else fc_log "boot.env: ignored line: $line"; fi
    done < "$envf"
  fi
  case "$BOOT_MODE" in home|idle) ;; *) _boot_state failed "bad FC_BOOT_MODE '$BOOT_MODE' (home|idle)"; return 1 ;; esac
  _profile_ok "$BOOT_PROFILE" || { _boot_state failed "unknown profile '$BOOT_PROFILE'"; return 1; }
  [[ "$delay" =~ ^[0-9]+$ ]] && [ "$delay" -le 300 ] || { _boot_state failed "bad FC_BOOT_DELAY_S '$delay' (0-300)"; return 1; }
  if [ "$BOOT_MODE" = idle ]; then _boot_state idle "boot mode idle: nothing started; Ready (with --confirm) homes"; return 0; fi
  local prev; prev=$(active_profile || true)
  if [ -n "$prev" ] && ! active_stale && _profile_ok "$prev" && ( load_profile "$prev" > /dev/null && profile_alive "$(active_run)" ); then
    _boot_state skipped "profile '$prev' already running"; return 0
  fi
  _boot_state waiting "homing $BOOT_PROFILE in $delay s (robot must be on the stand)"
  sleep "$delay"
  local temp; temp=$(vcgencmd measure_temp 2>/dev/null | sed -n "s/temp=\([0-9]*\).*/\1/p")
  if [ -n "$temp" ] && [ "$temp" -ge 80 ]; then _boot_state failed "Pi at ${temp} C: not homing"; return 1; fi
  _boot_state starting "homing $BOOT_PROFILE"
  local res; res=$(bash "$HERE/fcctl.sh" start "$BOOT_PROFILE" --confirm | grep '^{' | tail -1)
  if [[ "$res" == *'"ok":true'* ]]; then _boot_state homed "homed; press Ready in the UI (or: fcctl.sh ready) to stand and start the bridge"
  else _boot_state failed "start failed: $(_json_field "$res" error) ($(_json_field "$res" run_dir))"; return 1; fi
}

cmd_imu_test() {
  local tool=$HERE/imu_test.py
  # Require an explicit duration: with no stack running the IMU tool reads the BNO085 directly.
  [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "usage: imu-test <seconds> [imu_test.py options]"
  if [ ! -f "$tool" ]; then echo '{"ok":false,"available":false,"error":"imu_test.py not staged"}'; return 1; fi
  local prof; prof=$(active_profile || true)
  if [ -n "$prof" ]; then load_profile "$prof"; profile_env; export FC_PROFILE=$prof FC_IMU_TOPIC=$PROFILE_IMU_TOPIC; fi
  set -- --seconds "$@"  # imu-test 60 -> imu_test.py --seconds 60
  python3 -u "$tool" "$@"
}

cmd_i2c_recover() {
  local a check=false force=false
  for a in "$@"; do
    case "$a" in --check) check=true ;; --force) force=true ;; *) die "usage: i2c-recover [--check] [--force]" ;; esac
  done
  if ! $check && ! $force && pgrep -f 'ros2_control_node|persistent_session.py|ros2 launch' > /dev/null 2>&1; then
    echo '{"ok":false,"action":"refused","msg":"a robot stack is running (it holds the IMU): fcctl.sh stop first, or --force"}'
    return 1
  fi
  $check || lock
  if $check; then bash "$HERE/imu/i2c_recover.sh" --check --force; else bash "$HERE/imu/i2c_recover.sh" --force; fi
}

cmd_imu_level() {  # imu-level [check|show] [--seconds S] [--dry-run] [--legs-on]
  local mode=level legs_on="" args=() prof q
  [ "${1:-}" = check ] || [ "${1:-}" = show ] && { mode=$1; shift; }
  while [ $# -gt 0 ]; do
    case "$1" in
      --legs-on) legs_on=1; shift ;;
      --dry-run) args+=(--dry-run); shift ;;
      --seconds|--max-deg) [ $# -ge 2 ] || die "usage: imu-level [check|show] [--seconds S] [--max-deg D] [--dry-run] [--legs-on]"
        args+=("$1" "$2"); shift 2 ;;
      *) die "usage: imu-level [check|show] [--seconds S] [--max-deg D] [--dry-run] [--legs-on]" ;;
    esac
  done
  if [ "$mode" = show ]; then python3 "$HERE/imu/imu_level.py" show; return; fi
  prof=$(active_profile || true)
  [ -n "$prof" ] && profile_alive "$(active_run)" ||
    { echo '{"ok":false,"action":"refused","error":"no robot stack running: fcctl.sh start <profile> --confirm (robot on the stand)"}'; return 1; }
  load_profile "$prof" > /dev/null; profile_env
  if [ "$mode" = level ] && [ -z "$legs_on" ]; then
    q=$(rosq controllers 2>> "$FC_LOGS/ready.log")
    _legs_down "$q" || { echo '{"ok":false,"action":"refused","error":"a leg controller is active (or the controller manager did not answer): sit the robot level on the stand with the legs off (R3 or fcctl.sh down --keep-stack), or pass --legs-on"}'; return 1; }
  fi
  FC_IMU_TOPIC=${PROFILE_IMU_TOPIC:-/imu_sensor_broadcaster/imu} python3 "$HERE/imu/imu_level.py" "$mode" ${args[@]+"${args[@]}"}
}

case "${1:-}" in
  status) cmd_status ;;
  start) shift; cmd_start "$@" ;;
  _starter) shift; cmd__starter "$@" ;;
  stop) shift; cmd_stop "$@" ;;
  ready) shift; cmd_ready "$@" ;;
  down) shift; cmd_down "$@" ;;
  boot) cmd_boot ;;
  bridge) shift; cmd_bridge "$@" ;;
  cam) shift; cmd_cam "$@" ;;
  walk|rest) cmd_walk "$1" ;;
  imu-test) shift; cmd_imu_test "$@" ;;
  i2c-recover) shift; cmd_i2c_recover "$@" ;;
  imu-level) shift; cmd_imu_level "$@" ;;
  *) sed -n '2,47p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
