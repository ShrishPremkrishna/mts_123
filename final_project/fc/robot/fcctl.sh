#!/usr/bin/env bash
# fcctl.sh — Pupper FC robot control. Runs ON the Pi from ~/fc-v1/robot/.
#   status                      one JSON line
#   start <profile> --confirm   start a robot stack (HOMES THE LEGS: robot on the stand)
#   stop                        stop what fcctl started (bridge, walker, stack) and only that
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
# 9>&- : the child must not inherit the fcctl lock (fd 9), or every later stop/bridge start is refused.
fc_spawn() {
  local role=$1 log=$2; shift 2
  local pidfile=$FC_RUN/$role.pid
  rm -f "$pidfile"
  # bash -c (not a background job) so the child keeps default SIGINT handling.
  setsid -f bash -c 'echo $$ > "$1.tmp"; shift; exec "$@"' fc_spawn "$pidfile" "$@" >> "$log" 2>&1 < /dev/null 9>&-
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

foreign_stack() {  # robot processes fcctl did not start
  local pids p mine=" "
  for r in launch starter; do p=$(fc_pid "$r") && mine+="$p "; done
  pids=$(pgrep -f 'ros2_control_node|persistent_session.py|ros2 launch' 2>/dev/null) || return 1
  [ -n "$(active_profile)" ] && return 1   # ours (tej_u6 children are detached by Tej's helper)
  for p in $pids; do [[ "$mine" == *" $p "* ]] || return 0; done
  return 1
}

lock() { exec 9> "$FC_RUN/lock"; flock -n 9 || die "another fcctl command is running"; }
unlock() { exec 9>&-; }

# ---------------------------------------------------------------- ROS queries (rclpy, no daemon)
rosq() {  # rosq probe [CMD_TOPIC] | rosq switch "<activate...>" "<deactivate...>"
  timeout 15 python3 - "$@" <<'PY'
import json, sys, time
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
rclpy.init()
node = rclpy.create_node("fcctl_query")
def call(srv, name, req, wait=3.0):
    cli = node.create_client(srv, name)
    if not cli.wait_for_service(timeout_sec=wait):
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=wait)
    return fut.result()
def controllers():
    res = call(ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
    return None if res is None else {c.name: c.state for c in res.controller}
out = {}
if sys.argv[1] == "probe":
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out["nodes"] = sorted({n for n, _ in node.get_node_names_and_namespaces()} - {"fcctl_query"})
    # Velocity topology: exactly one /cmd_vel publisher (the mux), the mux listening on our topic,
    # nothing but our bridge publishing our topic, at most one gamepad teleop.
    topic = sys.argv[2] if len(sys.argv) > 2 else "/llm_cmd_vel"
    names = lambda infos: sorted(i.node_name for i in infos)
    cv = names(node.get_publishers_info_by_topic("/cmd_vel"))
    cs = names(node.get_subscriptions_info_by_topic(topic))
    cp = names(node.get_publishers_info_by_topic(topic))
    tp = names(node.get_publishers_info_by_topic("/teleop_cmd_vel"))
    problems = []
    if cv != ["cmd_vel_mux"]:
        problems.append("/cmd_vel publishers %s (want exactly cmd_vel_mux)" % cv)
    if "cmd_vel_mux" not in cs:
        problems.append("cmd_vel_mux not subscribed to " + topic)
    if [n for n in cp if n != "fc_drive_bridge"]:
        problems.append("%s publishers %s (want only fc_drive_bridge)" % (topic, cp))
    if len(tp) > 1:
        problems.append("/teleop_cmd_vel publishers %s (want at most one)" % tp)
    out["mux"] = {"ok": not problems, "alive": "cmd_vel_mux" in cv, "problems": problems,
                  "cmd_vel_pubs": cv, "topic": topic, "topic_subs": cs, "topic_pubs": cp, "teleop_pubs": tp}
    out["controllers"] = controllers()
else:
    want_on = sys.argv[2].split() if len(sys.argv) > 2 else []
    want_off = sys.argv[3].split() if len(sys.argv) > 3 else []
    states = controllers()
    if states is None:
        out = {"ok": False, "error": "controller_manager not reachable"}
    else:
        req = SwitchController.Request()
        req.activate_controllers = [c for c in want_on if states.get(c) != "active"]
        req.deactivate_controllers = [c for c in want_off if states.get(c) == "active" and c not in want_on]
        req.strictness = SwitchController.Request.BEST_EFFORT
        if not req.activate_controllers and not req.deactivate_controllers:
            out = {"ok": True, "changed": False}
        else:
            res = call(SwitchController, "/controller_manager/switch_controller", req, wait=5.0)
            out = {"ok": bool(res and res.ok), "activate": list(req.activate_controllers),
                   "deactivate": list(req.deactivate_controllers),
                   "error": None if res else "no answer (gateway latched or manager busy)"}
        out["controllers"] = controllers()
print(json.dumps(out))
node.destroy_node(); rclpy.try_shutdown()
PY
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
  python3 - "$prof" "$run" "$alive" "$stack" "$foreign" "$bpid" "${temp:-}" "${thr:-}" "$q" "$batt" \
    "$(cat /proc/loadavg)" "$(fc_pid starter || true)" "${PROFILE_HEADING_HOLD:-}" <<'PY'
import json, os, sys, time
prof, run, alive, stack, foreign, bpid, temp, thr, q, batt, load, starter, hh = sys.argv[1:14]
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
print(json.dumps({
    "ok": True, "t": round(time.time(), 3), "host": os.uname().nodename,
    "profile": prof or None, "run_dir": run or None,
    "profile_alive": alive == "true", "starting": bool(starter), "ready": ready, "start_failed": failed,
    "heading_hold": hh or None, "heading_hold_overlay": overlay,   # "on"/"off" per profile; null = no profile
    "ros2_control_node": stack == "true", "foreign_stack": foreign == "true",
    "mux_alive": mux.get("alive"), "mux_ok": mux.get("ok"),   # null = not probed (no live profile)
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
  printf '%s\n%s\n' "$name" "$run" > "$FC_RUN/active"
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
    fc_log "ready. Next: on the floor press X (tej_u6: Options first, then X) to stand; then 'fcctl.sh bridge start'."
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

cmd_stop() {
  lock
  local rc=0 prof run deact='null'
  fc_signal_group bridge INT 5 || rc=1   # bridge publishes one zero Twist on exit
  prof=$(active_profile || true); run=$(active_run || true)
  if [ -z "$prof" ]; then echo "{\"ok\":$([ $rc = 0 ] && echo true || echo false),\"stopped\":null}"; return $rc; fi
  load_profile "$prof"
  fc_signal_group starter TERM 3 || rc=1
  fc_adopt_orphans
  if profile_alive "$run"; then
    fc_log "deactivating leg controllers (robot will go limp / sit)"
    profile_env
    deact=$(rosq switch "" "$PROFILE_LEG_CONTROLLERS" 2>/dev/null) || deact='null'
    [ -n "$deact" ] || deact='null'
    sleep 1
  fi
  profile_stop "$run" || rc=1
  local left=""
  for r in launch teleop mux starter bridge; do fc_alive "$r" && left+="$r "; done
  if [ -z "$left" ] && ! profile_alive "$run"; then rm -f "$FC_RUN/active"; else rc=1; fi
  printf '{"ok":%s,"stopped":"%s","run_dir":"%s","left_running":"%s","deactivate":%s}\n' \
    "$([ $rc = 0 ] && echo true || echo false)" "$prof" "$run" "$left" "$deact"
  return $rc
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
  stop) cmd_stop ;;
  bridge) shift; cmd_bridge "$@" ;;
  walk|rest) cmd_walk "$1" ;;
  imu-test) shift; cmd_imu_test "$@" ;;
  *) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
