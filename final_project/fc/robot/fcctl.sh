#!/usr/bin/env bash
# fcctl.sh — Pupper FC robot control. Runs ON the Pi from ~/fc-v1/robot/.
#   status                      one JSON line (incl. "phase": off|starting|homed|standing|ready|fault, "light")
#   ready [profile] [--confirm [--stand-now]] [--no-bridge]
#                               homed -> playing: wait READY, stand the walker in software (or wait for the
#                               gamepad), start the bridge. Not running: homes (needs --confirm) and stops
#                               at "homed" (move it to the floor, run ready again) unless --stand-now.
#                               Progress as JSON lines on stdout; the last one has "done":true.
#   down [--keep-stack] [--force]
#                               bridge stop, lie down (leg controllers off = damped collapse, same as R3),
#                               confirm none active, stop the stack; JSON lines. Not confirmed: abort, stack
#                               left running, exit 1 (--force = UNSAFE: stops a possibly standing robot)
#   boot                        run by fc-boot.service at power-on (boot/README.md); never by hand
#   start <profile> --confirm   start a robot stack (HOMES THE LEGS: robot on the stand)
#   stop [--force]              stop what fcctl started (bridge, walker, stack) and only that; refuses
#                               (stack left running) if a leg controller cannot be confirmed inactive
#   bridge start [--profile P] [--robot-id ID] [--allow-from IP[,IP]] [--reset-replay]
#                [--no-imu-stop] [--no-mux-guard] | bridge stop
#                               (robot id defaults to ~/fc-v1/robot/robot_id, written by stage.sh;
#                                key: ~/fc-v1/robot/link.key, 0600; replay state: run/bridge_state.json)
#   imu-test <seconds>          delegates to imu_test.py (IMU lane) if present
#   walk | rest                 activate / deactivate the profile's walker controller (optional;
#                               the gamepad X / R3 do the same)
# Profiles: profiles/<name>.sh (*_hh0 = same stack, heading hold off via a verified kp=0 policy overlay). State: ~/fc-v1/run/ (pidfiles, active), runs: ~/fc-v1/runs/.
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
  # Only the profile file decides heading hold (a variant sets FC_HEADING_HOLD=0 and sources its base).
  unset FC_HEADING_HOLD PROFILE_HEADING_HOLD
  # shellcheck disable=SC1090
  source "$HERE/profiles/$name.sh"
  case "${PROFILE_HEADING_HOLD:-}" in on|off|"") ;; *) die "profile '$name': bad heading hold '$PROFILE_HEADING_HOLD' (FC_HEADING_HOLD must be 0 or 1)" ;; esac
}

# fc_hh_overlay KIND RUN : build + verify a fresh heading-hold-off overlay (imu/heading_hold.py KIND, lane D)
# in RUN/hh0, record its JSON in RUN/heading_hold.json. Writes only under FC_ROOT. Any failure returns 1,
# so the profile refuses to start (before any motion).
fc_hh_overlay() {
  local kind=$1 run=$2 res
  [ ! -e "$run/hh0" ] || { fc_log "heading-hold overlay: $run/hh0 already exists; refusing"; return 1; }
  res=$(python3 "$HERE/imu/heading_hold.py" "$kind" "$run/hh0" --root "$FC_ROOT") && [ -n "$res" ] ||
    { fc_log "HEADING-HOLD-OFF OVERLAY ($kind) COULD NOT BE BUILT/VERIFIED: refusing to start (see above)"; return 1; }
  printf '%s\n' "$res" > "$run/heading_hold.json"
  fc_log "heading hold OFF: $kind overlay verified in $run/hh0 (policy sha256 $(python3 -c 'import json,sys; print(json.load(sys.stdin)["sha256"][:12])' <<< "$res"))"
}

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
  local temp thr batt='null'
  temp=$(vcgencmd measure_temp 2>/dev/null | sed -n "s/temp=\([0-9.]*\).*/\1/p")
  thr=$(vcgencmd get_throttled 2>/dev/null | sed -n 's/throttled=//p')
  if [ "${FC_STATUS_BATTERY:-0}" = 1 ]; then  # opt-in: reads the ADC on the shared I2C bus
    batt=$(timeout 3 python3 /home/pi/pupper-foundation-session-20261029/battery-monitor-v1/pupperv3-monorepo/robot/utils/check_batt_voltage.py 2>/dev/null |
      sed -n 's/^Battery:[[:space:]]*\([0-9]*\)%[[:space:]]*\([0-9.]*\)V.*/{"pct":\1,"volts":\2}/p' | head -1)
    [ -n "$batt" ] || batt='null'
  fi
  local stale=false; active_stale && stale=true
  python3 - "$prof" "$run" "$alive" "$stack" "$foreign" "$bpid" "${temp:-}" "${thr:-}" "$q" "$batt" \
    "$(cat /proc/loadavg)" "$(fc_pid starter || true)" "${PROFILE_HEADING_HOLD:-}" \
    "${PROFILE_WALKER:-}" "${PROFILE_LEG_CONTROLLERS:-}" "$stale" "$HERE" "$FC_RUN/boot.json" <<'PY'
import json, os, sys, time
prof, run, alive, stack, foreign, bpid, temp, thr, q, batt, load, starter, hh = sys.argv[1:14]
walker, legs, stale, here, boot_file = sys.argv[14:19]
sys.path.insert(0, here)
import readiness
overlay = None
try:  # written by fc_hh_overlay when the running profile turned heading hold off
    with open(os.path.join(run, "heading_hold.json")) as f:
        o = json.load(f)
    overlay = {k: o.get(k) for k in ("kind", "verified", "policy", "sha256", "heading_hold")}
except (OSError, ValueError, AttributeError):
    pass
ready = bool(run) and os.path.exists(os.path.join(run, "READY"))
ros = json.loads(q) if q and q != "null" else None
mux = (ros or {}).get("mux") or {}
failed = bool(run) and os.path.exists(os.path.join(run, "FAILED"))
ph = readiness.phase({
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
print(json.dumps({
    "ok": True, "t": round(time.time(), 3), "host": os.uname().nodename,
    "phase": ph["phase"], "light": ph["light"], "phase_detail": ph["detail"],
    "walker": walker or None, "walker_active": ctl["walker_active"] if ctl else None,
    "active_legs": ctl["active_legs"] if ctl else None, "boot": boot,
    "profile": prof or None, "run_dir": run or None,
    "profile_alive": alive == "true", "starting": bool(starter), "ready": ready, "start_failed": failed,
    "heading_hold": hh or None, "heading_hold_overlay": overlay,   # "on"/"off" per profile; null = no profile
    "ros2_control_node": stack == "true", "foreign_stack": foreign == "true",
    "mux_alive": mux.get("alive"), "mux_ok": mux.get("ok"),   # null = not probed (no live profile)
    "dds": (ros or {}).get("dds"), "discovery": (ros or {}).get("discovery"),
    "ros": ros,
    "bridge": {"pid": int(bpid) if bpid else None, "alive": bool(bpid)},
    "temp_c": float(temp) if temp else None, "throttled": thr or None,
    "battery": json.loads(batt) if batt != "null" else None,
    "load": [float(x) for x in load.split()[:3]],
}, separators=(",", ":")))
PY
}

cmd_start() {
  local name=${1:-} confirm=${2:-}
  [ -n "$name" ] || die "usage: start <profile> --confirm"
  [ "$confirm" = "--confirm" ] || die "start homes the legs: put the robot on the stand, then rerun with --confirm"
  [ -z "${FC_ROS_DOMAIN_ID:-}" ] || die "FC_ROS_DOMAIN_ID is for bridge tests only; unset it to start a stack"
  lock
  load_profile "$name"
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
    printf '{"ok":true,"profile":"%s","run_dir":"%s","walker":"%s","cmd_topic":"%s","heading_hold":"%s"}\n' "$name" "$run" "$PROFILE_WALKER" "$PROFILE_CMD_TOPIC" "${PROFILE_HEADING_HOLD:-}"
  else
    fc_log "start failed or timed out; logs in $run. Run 'fcctl.sh stop' to clean up what was started."
    printf '{"ok":false,"profile":"%s","run_dir":"%s","error":"start failed"}\n' "$name" "$run"
    return 1
  fi
}

cmd__starter() {  # internal: runs detached so an ssh drop cannot interrupt homing
  local name=$1 run=$2
  load_profile "$name"
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
      fc_spawn bridge "$FC_LOGS/bridge.log" python3 -u "$HERE/drive_bridge.py" --profile "$prof" \
        --topic "$PROFILE_CMD_TOPIC" --imu-topic "$PROFILE_IMU_TOPIC" --port "${FC_BRIDGE_PORT:-9870}" \
        --robot-id "$rid" --key-file "$HERE/link.key" --state-file "$FC_RUN/bridge_state.json" \
        ${allow:+--allow-from "$allow"} "${extra[@]}"
      sleep 1.5
      fc_alive bridge || die "bridge exited (missing/loose link.key? bad state file?); see $FC_LOGS/bridge.log"
      printf '{"ok":true,"bridge_pid":%s,"profile":"%s","robot_id":"%s","topic":"%s","domain":%s}\n' "$(fc_pid bridge)" "$prof" "$rid" "$PROFILE_CMD_TOPIC" "${ROS_DOMAIN_ID:-0}"
      ;;
    stop)
      fc_signal_group bridge INT 5 || die "bridge did not stop"
      echo '{"ok":true,"bridge":"stopped"}'
      ;;
    *) die "usage: bridge start [--profile P] [--robot-id ID] [--allow-from IP[,IP]] [--reset-replay] [--no-imu-stop] [--no-mux-guard] | bridge stop" ;;
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
  local rc=0 prof run deact='null'
  fc_signal_group bridge INT 5 || rc=1   # bridge publishes one zero Twist on exit
  prof=$(active_profile || true); run=$(active_run || true)
  if [ -z "$prof" ]; then echo "{\"ok\":$([ $rc = 0 ] && echo true || echo false),\"stopped\":null}"; return $rc; fi
  load_profile "$prof"
  fc_signal_group starter TERM 3 || rc=1
  fc_adopt_orphans
  if profile_alive "$run"; then
    fc_log "deactivating leg controllers (damped collapse, same as R3) and confirming before stopping"
    profile_env
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

cmd_ready() {
  local name="" confirm="" stand_now="" bridge=1 a
  READY_T0=$(_now)
  for a in "$@"; do
    case "$a" in
      --confirm) confirm=1 ;;
      --stand-now) stand_now=1 ;;
      --no-bridge) bridge=0 ;;
      -*) ready_fail args "unknown option '$a' (ready [profile] [--confirm [--stand-now]] [--no-bridge])" 2 ;;
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
  emit begin true "profile $name" ",\"profile\":\"$name\",\"walker\":\"$PROFILE_WALKER\""

  # 1. stack up and homed
  local run
  if [ -n "$cur" ]; then
    run=$(active_run)
    if fc_alive starter; then
      emit starting true "a start is already in progress (boot service or another operator); waiting for READY"
      _wait_start "$run" 200 || ready_fail starting "start failed or timed out; see $run/starter.log, then fcctl.sh down" 1
    fi
    [ -f "$run/READY" ] || ready_fail starting "profile running but never READY; fcctl.sh down, then ready $name --confirm" 1
    emit homed true "homed (already running)" ",\"run_dir\":\"$run\""
  else
    if [ -z "$confirm" ]; then
      emit needs_confirm false "stack not running: starting it HOMES THE LEGS. Put the robot on the stand, then rerun with --confirm" \
        ",\"profile\":\"$name\",\"exit\":3,\"done\":true"
      exit 3
    fi
    emit starting true "starting $name: HOMING THE LEGS (robot on the stand)"
    local out=$FC_RUN/ready_start.out spid beat=$SECONDS rc res
    bash "$HERE/fcctl.sh" start "$name" --confirm > "$out" 2>> "$FC_LOGS/ready.log" 8>&- &
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
  if [ "$bridge" = 0 ]; then emit standing true "standing; bridge not started (--no-bridge)" ',"done":true'; exit 0; fi

  # 3. bridge
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

# down [--keep-stack] [--force] : bridge zero, lie down (leg controllers off = damped collapse, same as R3),
# CONFIRM no leg controller is active, then stop the stack. Not confirmed -> abort, stack left running, exit 1
# (--force stops anyway: UNSAFE, a standing robot drops with the motors off).
cmd_down() {
  local keep=0 force=0 a
  READY_T0=$(_now)
  for a in "$@"; do
    case "$a" in --keep-stack) keep=1 ;; --force) force=1 ;; *) ready_fail args "usage: down [--keep-stack] [--force]" 2 ;; esac
  done
  exec 8> "$FC_RUN/ready.lock"
  flock -n 8 || ready_fail args "another 'ready' or 'down' is running" 1
  emit begin true "down$([ $keep = 1 ] && echo ' --keep-stack')$([ $force = 1 ] && echo ' --force')"
  local b; b=$(bash "$HERE/fcctl.sh" bridge stop 2>> "$FC_LOGS/ready.log" 8>&- | grep '^{' | tail -1)
  if [[ "$b" == *'"ok":true'* ]]; then emit bridge_stopped true "bridge stopped (sent one zero)"
  else emit bridge_stopped false "bridge stop: $(_json_field "$b" error)"; fi
  local prof run; prof=$(active_profile || true); run=$(active_run || true)
  if [ -z "$prof" ] || active_stale || ! _profile_ok "$prof"; then
    emit down true "no fcctl profile running" ',"done":true'; exit 0
  fi
  load_profile "$prof"
  if profile_alive "$run"; then
    profile_env
    if fc_lie_down; then
      emit lying_down true "leg controllers off (confirmed): damped collapse, the same as R3" ",\"switch\":$(_raw_json "$LIE_JSON")"
      sleep "${FC_DOWN_SETTLE_S:-1.5}"
    elif [ $force = 1 ]; then
      emit lying_down false "NOT confirmed lying ($LIE_WHY); --force: continuing (UNSAFE)" ",\"switch\":$(_raw_json "$LIE_JSON")"
    else
      emit error false "still standing? lie-down not confirmed ($LIE_WHY): use R3 or support the robot. Stack left running; down --force is unsafe" \
        ",\"at\":\"lying_down\",\"switch\":$(_raw_json "$LIE_JSON"),\"stack\":\"running\",\"exit\":1,\"done\":true"
      exit 1
    fi
  fi
  if [ $keep = 1 ]; then
    emit down true "lying down, stack kept homed: ready stands it again without homing" ',"stack":"kept","done":true'; exit 0
  fi
  local s; s=$(bash "$HERE/fcctl.sh" stop $([ $force = 1 ] && echo --force) 2>> "$FC_LOGS/ready.log" 8>&- | grep '^{' | tail -1)
  if [[ "$s" == *'"ok":true'* ]]; then emit down true "stack stopped" ",\"stop\":$(_raw_json "$s"),\"done\":true"
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

case "${1:-}" in
  status) cmd_status ;;
  start) shift; cmd_start "$@" ;;
  _starter) shift; cmd__starter "$@" ;;
  stop) shift; cmd_stop "$@" ;;
  ready) shift; cmd_ready "$@" ;;
  down) shift; cmd_down "$@" ;;
  boot) cmd_boot ;;
  bridge) shift; cmd_bridge "$@" ;;
  walk|rest) cmd_walk "$1" ;;
  imu-test) shift; cmd_imu_test "$@" ;;
  *) sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
