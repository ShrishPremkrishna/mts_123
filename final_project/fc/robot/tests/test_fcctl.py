"""fcctl.sh process/lock behaviour with a fake profile (no ROS, no robot).

Needs Linux (flock, setsid, /proc); skipped elsewhere. On the Mac run it in a throwaway container:
  docker run --rm --init --network none -v "$PWD/fc/robot:/src:ro" ros:jazzy-ros-base \
    bash -c 'apt-get -qq install -y python3-pytest >/dev/null 2>&1; cd /src && python3 -m pytest -q -p no:cacheprovider tests/test_fcctl.py'
"""
import json
import os
import shutil
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pytestmark = pytest.mark.skipif(
    not (sys.platform.startswith("linux") and shutil.which("flock") and shutil.which("setsid")),
    reason="fcctl.sh needs Linux flock/setsid//proc")

FAKE_PROFILE = r'''
PROFILE_DESC="fake"; PROFILE_CMD_TOPIC=/llm_cmd_vel; PROFILE_IMU_TOPIC=/imu
PROFILE_WALKER=w; PROFILE_LEG_CONTROLLERS="w"
profile_env() { export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; }
rosq() {  # fake graph: FAKE_TOPO=good|bad; walker state in $FC_ROOT/fake_walker; FAKE_SWITCH=refuse = latched gateway
  # FAKE_DEACT=ignore: deactivation answered ok but the walker stays active; FAKE_CTL=down: manager unreachable
  local f=$FC_ROOT/fake_walker w=inactive
  [ -f "$f" ] && w=$(cat "$f")
  if [ "${FAKE_CTL:-up}" = down ]; then
    if [ "$1" = switch ]; then echo '{"ok":false,"error":"controller_manager not reachable"}'; else echo '{"mux":{"ok":true,"alive":true,"problems":[]},"controllers":null}'; fi
    return
  fi
  if [ "$1" = switch ] && [ -z "$2" ] && [ "${FAKE_DEACT:-ok}" = ignore ]; then
    echo '{"ok":true,"deactivate":["w"],"controllers":{"w":"'$w'"}}'; return
  fi
  if [ "$1" = switch ]; then
    if [ -n "$2" ] && [ "${FAKE_SWITCH:-ok}" = refuse ]; then
      echo '{"ok":false,"error":"no answer (gateway latched or manager busy)","controllers":{"w":"'$w'"}}'; return
    fi
    if [ -n "$2" ]; then echo active > "$f"; else echo inactive > "$f"; fi
    echo '{"ok":true,"controllers":{"w":"'$(cat "$f")'"}}'; return
  fi
  if [ "${FAKE_TOPO:-good}" = good ]; then
    echo '{"mux":{"ok":true,"alive":true,"problems":[],"cmd_vel_pubs":["cmd_vel_mux"],"topic":"/llm_cmd_vel","topic_subs":["cmd_vel_mux"],"topic_pubs":[],"teleop_pubs":["teleop_twist_joy_node"]},"controllers":{"w":"'$w'","joint_state_broadcaster":"active"}}'
  else
    echo '{"mux":{"ok":false,"alive":true,"problems":["/cmd_vel publishers [cmd_vel_mux, cmd_vel_mux] (want exactly cmd_vel_mux)"],"cmd_vel_pubs":["cmd_vel_mux","cmd_vel_mux"],"topic":"/llm_cmd_vel","topic_subs":[],"topic_pubs":[],"teleop_pubs":[]},"controllers":{"w":"'$w'"}}'
  fi
}
profile_start() {
  # the fake launch carries "ros2_control_node" in its command line (stop treats that as motors driven)
  fc_spawn launch "$1/robot.log" bash -c 'sleep 300; true' ros2_control_node
  fc_spawn mux "$1/mux.log" sleep 300
  sleep "${FAKE_HOME_S:-0}"
  fc_topology_wait 2
}
profile_alive() { fc_alive launch; }
profile_stop() { fc_signal_group mux INT 5; fc_signal_group launch INT 5; }
upright_check() {  # FAKE_UPRIGHT=tilted: lying on its side (a recorded fall rested at 76 deg); none: no IMU verdict
  case "${FAKE_UPRIGHT:-good}" in
    tilted) echo '{"ok": false, "tilt_deg": 76.04, "max_deg": 25.0, "n": 50}' ;;
    none) echo '{"ok": null, "error": "no IMU samples: is the stack running?", "n": 0}' ;;
    *) echo '{"ok": true, "tilt_deg": 3.1, "max_deg": 25.0, "n": 50}' ;;
  esac
}
imu_rest_check() {  # FAKE_IMU=bad: the IMU reads 10.7 deg at rest (rca-20261127-tilt); none: no verdict
  case "${FAKE_IMU:-good}" in
    bad) echo '{"ok": false, "tilt_deg": 10.68, "error": "IMU reads 10.7 deg at rest (limit 6.0): IMU calibration off? Put the robot level on the stand, legs off, run fcctl.sh imu-level, then restart the stack"}' ;;
    none) echo '{"ok": null, "error": "no IMU samples", "n": 0}' ;;
    *) echo '{"ok": true, "tilt_deg": 1.9}' ;;
  esac
}
'''

FAKE_BRIDGE = r'''
import json, os, signal, sys, time
root = os.environ["FC_ROOT"]
open(os.path.join(root, "bridge_argv.json"), "w").write(json.dumps(sys.argv[1:]))
def park(*_):  # like drive_bridge.py; FAKE_PARK=ignore: an older bridge that never confirms
    if os.environ.get("FAKE_PARK") == "ignore":
        return
    print("[bridge] PARKED pid=%d: zero only" % os.getpid(), flush=True)
    d = os.environ.get("FAKE_ESTOP_AFTER_PARK_S")   # a link E-STOP arrives while the robot is lowered
    if d:
        time.sleep(float(d))
        for c in ("fake_walker", "fake_ld"):        # the R3 action: every listed leg controller off
            open(os.path.join(root, c), "w").write("inactive\n")
        latch = sys.argv[sys.argv.index("--latch-file") + 1]
        open(latch, "w").write(json.dumps({"reason": "link", "legs": sys.argv[sys.argv.index("--legs") + 1]}))
signal.signal(signal.SIGUSR1, park)
for _ in range(3000):
    time.sleep(0.1)
'''


@pytest.fixture
def fc(tmp_path):
    robot = tmp_path / "robot"
    shutil.copytree(HERE, robot, ignore=shutil.ignore_patterns("vendor", "tests", "__pycache__", ".pytest_cache"))
    (robot / "profiles" / "fake.sh").write_text(FAKE_PROFILE)
    (robot / "drive_bridge.py").write_text(FAKE_BRIDGE)
    (robot / "robot_id").write_text("blue\n")
    root = tmp_path / "fc-v1"
    env = {**os.environ, "FC_ROOT": str(root)}

    def run(*args, extra_env=None, timeout=60, background=False):
        cmd = ["bash", str(robot / "fcctl.sh"), *args]
        e = {**env, **(extra_env or {})}
        if background:
            return subprocess.Popen(cmd, env=e, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        out = subprocess.run(cmd, env=e, capture_output=True, text=True, timeout=timeout)
        lines = [l for l in out.stdout.splitlines() if l.startswith("{")]
        return json.loads(lines[-1]) if lines else {"raw": out.stdout, "err": out.stderr}

    def lines(*args, extra_env=None, timeout=120):
        out = subprocess.run(["bash", str(robot / "fcctl.sh"), *args], env={**env, "FC_READY_SETTLE_S": "0",
                             "FC_DOWN_SETTLE_S": "0", **(extra_env or {})}, capture_output=True, text=True, timeout=timeout)
        return out.returncode, [json.loads(l) for l in out.stdout.splitlines() if l.startswith("{")]

    run.root = root
    run.robot = robot
    run.lines = lines
    run.env = env
    yield run
    run("stop")
    for pidfile in (root / "run").glob("*.pid"):   # belt and braces: never leak sleepers
        try:
            os.kill(int(pidfile.read_text().split()[0]), 9)
        except (OSError, ValueError):
            pass


def lock_holders(root, name="lock"):
    lock = str(root / "run" / name)
    holders = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
            if any(os.readlink(f"/proc/{pid}/fd/{fd}") == lock for fd in fds):
                holders.append(pid)
        except OSError:
            continue
    return holders


def test_start_then_bridge_start_then_stop(fc):
    start = fc("start", "fake", "--confirm")
    assert start["ok"], start
    assert lock_holders(fc.root) == []                      # no spawned child inherited the lock
    bridge = fc("bridge", "start", "--reset-replay", "--no-mux-guard")
    assert bridge["ok"], bridge
    argv = json.loads((fc.root / "bridge_argv.json").read_text())
    assert argv[argv.index("--state-file") + 1] == str(fc.root / "run" / "bridge_state.json")
    assert "--reset-replay" in argv and "--no-mux-guard" in argv and argv[argv.index("--robot-id") + 1] == "blue"
    assert lock_holders(fc.root) == []
    assert fc("status")["heading_hold"] == "on"                # recorded default (stack.json)
    stop = fc("stop")
    assert stop["ok"] and stop["left_running"] == "", stop
    assert not (fc.root / "run" / "active").exists()


def test_stop_works_during_homing(fc):
    starter = fc("start", "fake", "--confirm", extra_env={"FAKE_HOME_S": "30"}, background=True)
    deadline = time.time() + 10
    while not (fc.root / "run" / "launch.pid").exists() and time.time() < deadline:
        time.sleep(0.1)
    t0 = time.time()
    stop = fc("stop")
    assert stop["ok"] and stop["left_running"] == "", stop
    assert time.time() - t0 < 15
    out = starter.communicate(timeout=30)[0]
    assert json.loads(out.strip().splitlines()[-1])["ok"] is False


def test_lock_still_serialises_commands(fc):
    run_dir = fc.root / "run"
    run_dir.mkdir(parents=True, exist_ok=True)
    holder = subprocess.Popen(["flock", str(run_dir / "lock"), "sleep", "5"])
    try:
        time.sleep(0.3)
        assert fc("stop") == {"ok": False, "error": "another fcctl command is running"}
    finally:
        holder.kill()
        holder.wait()


def test_wrong_topology_refuses_ready(fc):
    start = fc("start", "fake", "--confirm", extra_env={"FAKE_TOPO": "bad"})
    assert start["ok"] is False
    run_dir = sorted((fc.root / "runs").iterdir())[-1]
    assert (run_dir / "FAILED").exists() and "topology not ok" in (run_dir / "starter.log").read_text()
    assert fc("stop")["ok"]


def test_adopts_orphan_pid_tmp(fc):
    assert fc("start", "fake", "--confirm")["ok"]
    sleeper = subprocess.Popen(["setsid", "sleep", "300"])
    (fc.root / "run" / "teleop.pid.tmp").write_text(str(sleeper.pid))   # starter died before finalising
    profile = (fc.root / "run" / "active").read_text()
    assert "fake" in profile
    # fake profile_stop does not know teleop; adoption makes it visible to stop's left_running check
    stop = fc("stop")
    assert "teleop" in stop["left_running"]
    sleeper.kill()
    sleeper.wait()
    os.remove(fc.root / "run" / "teleop.pid")


# Profiles with stack options (stock/tej_u6 + --speed-cap/--heading-hold, *_hh0 aliases) call fc_stack_overlay
# first in profile_start, like the real ones (profiles/stock.sh: `fc_stack_options_set`).
FAKE_HH0_PROFILE = FAKE_PROFILE.replace("profile_start() {\n", 'profile_start() {\n  fc_stack_overlay u6 "$1" || return 1\n') + \
    "\nFC_HEADING_HOLD=0; PROFILE_HEADING_HOLD=off\n"
FAKE_OPT_PROFILE = FAKE_PROFILE.replace("profile_start() {\n", 'profile_start() {\n  if fc_stack_options_set; then fc_stack_overlay u6 "$1" || return 1; fi\n') + \
    "\nPROFILE_HEADING_HOLD=on\n"
FAKE_HH_TOOL = r"""
import json, os, sys
kind, out = sys.argv[1], sys.argv[2]
a = sys.argv[3:]
root = a[a.index("--root") + 1]
cap = float(a[a.index("--speed-cap") + 1]) if "--speed-cap" in a else None
hold = a[a.index("--heading-hold") + 1]
assert os.path.realpath(out).startswith(os.path.realpath(root) + "/")
os.makedirs(out)
open(os.path.join(os.path.dirname(out), "overlay_argv.json"), "w").write(json.dumps(a))
print(json.dumps({"kind": kind, "verified": True, "speed_cap": cap, "speed_cap_effective": cap or 0.49,
                  "heading_hold": hold, "liedown": None, "launch": out + "/launch_fc.py", "config": out + "/c.yaml",
                  "policy": out + "/p.json" if hold == "off" else None,
                  "sha256": "9f5a3e98" + "0" * 56 if hold == "off" else None, "hh_policy": {"kp": 0.0}}))
"""


def test_overlay_failure_refuses_start(fc):
    # The real stack_overlay.py: no Tej tree here, so it must fail loudly and nothing may be spawned.
    (fc.robot / "profiles" / "fakehh.sh").write_text(FAKE_HH0_PROFILE)
    start = fc("start", "fakehh", "--confirm")
    assert start["ok"] is False
    run_dir = sorted((fc.root / "runs").iterdir())[-1]
    log = (run_dir / "starter.log").read_text()
    assert "COULD NOT BE BUILT" in log and "stack_overlay: " in log, log
    assert not (fc.root / "run" / "launch.pid").exists() and not (run_dir / "overlay" / "launch_fc.py").exists()
    assert fc("status")["heading_hold"] == "off"
    assert fc("stop")["ok"]
    assert lock_holders(fc.root) == []


def test_status_reports_heading_hold_off(fc):
    (fc.robot / "profiles" / "fakehh.sh").write_text(FAKE_HH0_PROFILE)
    (fc.robot / "stack_overlay.py").write_text(FAKE_HH_TOOL)
    start = fc("start", "fakehh", "--confirm")
    assert start["ok"] and start["heading_hold"] == "off", start
    st = fc("status")
    assert st["heading_hold"] == "off" and st["heading_hold_overlay"]["verified"] is True
    assert st["heading_hold_overlay"]["sha256"].startswith("9f5a3e98") and st["heading_hold_overlay"]["kind"] == "u6"
    assert lock_holders(fc.root) == []
    assert fc("stop")["ok"]


def test_lean_option_recorded_and_refused_without_profile_support(fc):
    # a profile without PROFILE_LEAN (like tej_u6) refuses --lean/--camera before anything starts
    r = fc("start", "fake", "--confirm", "--lean", "on")
    assert r["ok"] is False and "no --lean/--camera option" in r["error"], r
    assert fc("start", "fake", "--confirm", "--lean", "maybe")["error"].startswith("--lean needs on|off")
    # a stock-like profile: lean on by default -> overlay args carry --lean on --camera off; status reports it
    (fc.robot / "profiles" / "fakelean.sh").write_text(FAKE_OPT_PROFILE + "\nPROFILE_LEAN=on\nPROFILE_CAMERA=off\n")
    (fc.robot / "stack_overlay.py").write_text(FAKE_HH_TOOL.replace(
        '"liedown": None,', '"liedown": None, "lean": a[a.index("--lean") + 1], "camera": a[a.index("--camera") + 1] == "on",'))
    start = fc("start", "fakelean", "--confirm", "--camera", "on")
    assert start["ok"] and start["lean"] == "on" and start["camera"] is True, start
    run_dir = sorted((fc.root / "runs").iterdir())[-1]
    argv = json.loads((run_dir / "overlay_argv.json").read_text())
    assert argv[argv.index("--lean") + 1] == "on" and argv[argv.index("--camera") + 1] == "on"
    st = fc("status")
    assert st["lean"] == "on" and st["camera"] is True
    assert isinstance(st["heat"], dict) and isinstance(st["heat"]["top"], list) and "governor" in st["heat"]["cpu"]
    # ready may not silently change it: a mismatch refuses (changing it means a new start)
    rc, out = fc.lines("ready", "--lean", "off")
    assert rc == 1 and "lean on" in out[-1]["msg"], out
    assert fc("stop")["ok"]
    # --lean off with no other option: no overlay, record says the stock list (camera launched)
    start = fc("start", "fakelean", "--confirm", "--lean", "off")
    assert start["ok"] and start["lean"] == "off" and start["camera"] is True, start
    assert fc("stop")["ok"]
    assert lock_holders(fc.root) == []


# ---------------------------------------------------------------- ready / down / status phase / boot
def phases(lines):
    return [l["phase"] for l in lines]


def test_ready_needs_confirm_before_homing(fc):
    rc, out = fc.lines("ready", "fake")
    assert rc == 3 and out[-1]["phase"] == "needs_confirm" and out[-1]["done"] is True and out[-1]["ok"] is False
    assert not (fc.root / "run" / "launch.pid").exists()                # nothing started, nothing homed
    assert fc("status")["phase"] == "off"


def test_ready_homes_then_stops_for_the_floor(fc):
    rc, out = fc.lines("ready", "fake", "--confirm")
    assert rc == 0 and phases(out) == ["begin", "starting", "homed"], out
    assert out[-1]["done"] and out[-1]["next"] == "ready"
    assert fc("status")["phase"] == "homed" and (fc.root / "fake_walker").exists() is False
    rc, out = fc.lines("ready", "--confirm")
    assert rc == 0 and phases(out) == ["begin", "homed", "standing_up", "standing", "bridge", "ready"], out


def test_ready_full_path_then_down(fc):
    rc, out = fc.lines("ready", "fake", "--confirm", "--stand-now")
    assert rc == 0, out
    assert phases(out) == ["begin", "starting", "homed", "standing_up", "standing", "bridge", "ready"], out
    assert out[-1]["ok"] and out[-1]["done"] and all(isinstance(l["elapsed"], (int, float)) for l in out)
    assert out[-2]["bridge"]["ok"] is True
    assert lock_holders(fc.root) == [] and lock_holders(fc.root, "ready.lock") == []
    st = fc("status")
    assert st["phase"] == "ready" and st["light"] == "green" and st["walker_active"] is True, st
    rc, out = fc.lines("down")
    assert rc == 0 and phases(out) == ["begin", "bridge_parked", "lowering", "bridge_stopped", "lying_down", "down"], out
    assert out[2]["gentle"] is False and "WILL DROP" in out[2]["msg"]          # no lie-down controller here
    assert "aborted" not in out[-1]
    assert out[-1]["done"] and out[-1]["stop"]["ok"]
    assert (fc.root / "fake_walker").read_text().strip() == "inactive"
    assert fc("status")["phase"] == "off"


def test_down_never_stops_a_robot_not_confirmed_lying(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    launch_pid = (fc.root / "run" / "launch.pid").read_text()
    rc, out = fc.lines("down", extra_env={"FAKE_DEACT": "ignore"})
    assert rc == 1 and out[-1]["phase"] == "error" and out[-1]["at"] == "lying_down" and out[-1]["done"], out
    assert "R3" in out[-1]["msg"] and out[-1]["stack"] == "running"
    assert "lying_down" not in phases(out)[:-1] and "down" not in phases(out)
    assert (fc.root / "run" / "launch.pid").read_text() == launch_pid      # stack untouched
    st = fc("status")
    assert st["phase"] == "standing" and st["profile_alive"] and not st["bridge"]["alive"], st
    stop = fc("stop", extra_env={"FAKE_DEACT": "ignore"})                     # stop refuses the same way
    assert stop["ok"] is False and stop["left_running"] == "stack" and "R3" in stop["error"], stop
    assert fc("status")["profile_alive"] is True
    rc, out = fc.lines("down", "--force", extra_env={"FAKE_DEACT": "ignore"})  # explicit unsafe override
    assert rc == 0 and phases(out) == ["begin", "bridge_stopped", "lowering", "lying_down", "down"], out
    assert out[3]["ok"] is False and "UNSAFE" in out[3]["msg"]
    assert fc("status")["phase"] == "off"


def test_stop_refuses_when_controller_state_unknown(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    stop = fc("stop", extra_env={"FAKE_CTL": "down"})
    assert stop["ok"] is False and "not answering" in stop["error"], stop
    assert fc("status")["profile_alive"] is True
    assert fc("stop")["ok"]                                                    # manager back: lies down, stops


def test_ready_is_idempotent(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    rc, out = fc.lines("ready")                                           # profile defaults to the running one
    assert rc == 0 and phases(out) == ["begin", "homed", "standing", "bridge", "ready"], out
    assert "already" in out[2]["msg"] and "already" in out[3]["msg"]


def test_down_keep_stack_then_ready_without_homing(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    launch_pid = (fc.root / "run" / "launch.pid").read_text()
    rc, out = fc.lines("down", "--keep-stack")
    assert rc == 0 and out[-1]["stack"] == "kept"
    st = fc("status")
    assert st["phase"] == "homed" and st["bridge"]["alive"] is False, st
    rc, out = fc.lines("ready", "--confirm")                                           # no --confirm needed: already homed
    assert rc == 0 and phases(out) == ["begin", "homed", "standing_up", "standing", "bridge", "ready"], out
    assert (fc.root / "run" / "launch.pid").read_text() == launch_pid   # same stack, no re-homing


def test_ready_waits_for_gamepad_when_gateway_refuses(fc):
    assert fc("start", "fake", "--confirm")["ok"]
    assert fc("status")["phase"] == "homed"
    p = subprocess.Popen(["bash", str(fc.robot / "fcctl.sh"), "ready", "--confirm"], env={**fc.env, "FAKE_SWITCH": "refuse",
                         "FC_READY_SETTLE_S": "0"}, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    seen = []
    for line in p.stdout:
        seen.append(json.loads(line))
        if seen[-1]["phase"] == "waiting_operator":
            break
    assert "latched" in seen[-1]["msg"] and seen[-1]["action"]
    (fc.root / "fake_walker").write_text("active\n")                     # the operator presses the button
    rest = [json.loads(l) for l in p.stdout]
    assert p.wait(timeout=30) == 0
    assert phases(rest) == ["standing", "bridge", "ready"], rest


def test_ready_operator_timeout(fc):
    assert fc("start", "fake", "--confirm")["ok"]
    rc, out = fc.lines("ready", "--confirm", extra_env={"FAKE_SWITCH": "refuse", "FC_READY_OPERATOR_S": "2"})
    assert rc == 4 and out[-1]["phase"] == "error" and out[-1]["at"] == "waiting_operator" and out[-1]["done"]
    assert fc("status")["phase"] == "homed" and not (fc.root / "run" / "bridge.pid").exists()


def test_ready_refuses_other_profile_and_bad_topology(fc):
    (fc.robot / "profiles" / "fake2.sh").write_text(FAKE_PROFILE)
    assert fc("start", "fake", "--confirm")["ok"]
    rc, out = fc.lines("ready", "fake2")
    assert rc == 1 and out[-1]["at"] == "profile"
    assert fc("stop")["ok"]
    rc, out = fc.lines("ready", "fake", "--confirm", extra_env={"FAKE_TOPO": "bad"})
    assert rc == 1 and out[-1]["at"] == "starting", out
    assert fc("status")["phase"] == "fault"


def test_status_phases_walk_and_stale_state(fc):
    assert fc("status")["phase"] == "off"
    assert fc("start", "fake", "--confirm")["ok"]
    assert fc("status")["phase"] == "homed"
    fc("walk")
    st = fc("status")
    assert st["phase"] == "standing" and st["active_legs"] == ["w"], st
    assert fc("stop")["ok"]
    run = sorted((fc.root / "runs").iterdir())[-1]
    (fc.root / "run" / "active").write_text(f"fake\n{run}\nnot-this-boot\n")   # left by an earlier boot
    st = fc("status")
    assert st["phase"] == "off" and "earlier boot" in st["phase_detail"], st
    boot = open("/proc/sys/kernel/random/boot_id").read().strip()
    (fc.root / "run" / "active").write_text(f"fake\n{run}\n{boot}\n")           # this boot, stack gone
    assert fc("status")["phase"] == "fault"
    (fc.root / "run" / "active").unlink()


def test_boot_modes(fc):
    fc.root.mkdir(parents=True, exist_ok=True)
    (fc.root / "boot.env").write_text("FC_BOOT_PROFILE=fake\nFC_BOOT_MODE=idle\n")
    assert fc("boot")["result"] == "idle" and fc("status")["phase"] == "off"
    (fc.root / "boot.env").write_text("# comment\nFC_BOOT_PROFILE=fake\nFC_BOOT_MODE=home\nFC_BOOT_DELAY_S=0\nrm -rf /=x\n")
    b = fc("boot")
    assert b["result"] == "homed", b
    st = fc("status")
    assert st["phase"] == "homed" and st["boot"]["result"] == "homed" and st["boot"]["profile"] == "fake"
    assert fc("boot")["result"] == "skipped"                              # never homes twice
    assert lock_holders(fc.root) == [] and lock_holders(fc.root, "ready.lock") == []
    rc, out = fc.lines("ready", "--confirm")                                           # Ready after boot: stand + bridge only
    assert rc == 0 and phases(out)[1:3] == ["homed", "standing_up"], out
    (fc.root / "boot.env").write_text("FC_BOOT_MODE=stand\n")
    assert fc("boot")["result"] == "failed"


# ---------------------------------------------------------------- SPEC v1.1: stack options, e-stop latch, gentle down
def test_stack_options_after_confirm_reach_overlay_status_and_bridge(fc):
    (fc.robot / "profiles" / "fakeopt.sh").write_text(FAKE_OPT_PROFILE)
    (fc.robot / "stack_overlay.py").write_text(FAKE_HH_TOOL)
    rc, out = fc.lines("ready", "fakeopt", "--confirm", "--speed-cap", "0.80", "--heading-hold", "off", "--stand-now")
    assert rc == 0 and out[-1]["phase"] == "ready", out
    run = sorted((fc.root / "runs").iterdir())[-1]
    argv = json.loads((run / "overlay_argv.json").read_text())
    assert argv[argv.index("--speed-cap") + 1] == "0.80" and argv[argv.index("--heading-hold") + 1] == "off"
    st = fc("status")
    assert st["speed_cap"] == 0.8 and st["speed_cap_source"] == "overlay" and st["heading_hold"] == "off", st
    assert st["heading_hold_overlay"]["sha256"].startswith("9f5a3e98") and st["estopped"] is False
    bargv = json.loads((fc.root / "bridge_argv.json").read_text())
    assert bargv[bargv.index("--max-vx") + 1] == "0.8" and bargv[bargv.index("--legs") + 1] == "w"
    assert "--liedown" not in bargv                                   # no lie-down controller: R3 at once
    assert bargv[bargv.index("--latch-file") + 1] == str(fc.root / "run" / "estop.json")
    # same options again: no re-home; different options: refused (changing them needs a new start)
    rc, out = fc.lines("ready", "fakeopt", "--confirm", "--heading-hold", "off", "--speed-cap", "0.8")
    assert rc == 0 and out[1]["phase"] == "homed" and out[1]["speed_cap"] == 0.8, out
    rc, out = fc.lines("ready", "fakeopt", "--confirm", "--speed-cap", "0.6")
    assert rc == 1 and out[-1]["at"] == "profile" and "down first" in out[-1]["msg"], out
    assert lock_holders(fc.root) == [] and lock_holders(fc.root, "ready.lock") == []


def test_default_stack_is_recorded_without_overlay(fc):
    (fc.robot / "profiles" / "fakeopt.sh").write_text(FAKE_OPT_PROFILE)
    assert fc("start", "fakeopt", "--confirm")["ok"]
    run = sorted((fc.root / "runs").iterdir())[-1]
    assert not (run / "overlay").exists()
    st = fc("status")
    assert st["speed_cap"] == 0.49 and st["speed_cap_source"] == "default" and st["heading_hold"] == "on", st
    assert fc("bridge", "start", "--reset-replay")["max_vx"] == 0.49


def test_top_speed_cap_reaches_bridge(fc):
    # 1.50 = the policy's trained vx range: the highest cap fcctl accepts, and the bridge follows it
    (fc.robot / "profiles" / "fakeopt.sh").write_text(FAKE_OPT_PROFILE)
    (fc.robot / "stack_overlay.py").write_text(FAKE_HH_TOOL)
    assert fc("start", "fakeopt", "--confirm", "--speed-cap", "1.50")["ok"]
    assert fc("status")["speed_cap"] == 1.5
    assert fc("bridge", "start", "--reset-replay")["max_vx"] == 1.5


@pytest.mark.parametrize("args", [["--speed-cap", "0.29"], ["--speed-cap", "1.51"], ["--speed-cap", "fast"],
                                  ["--heading-hold", "maybe"], ["--speed-cap"]])
def test_bad_stack_options_refused_before_anything_starts(fc, args):
    (fc.robot / "profiles" / "fakeopt.sh").write_text(FAKE_OPT_PROFILE)
    assert fc("start", "fakeopt", "--confirm", *args)["ok"] is False
    rc, out = fc.lines("ready", "fakeopt", "--confirm", *args)
    assert rc == 2 and out[-1]["at"] == "args", out
    assert not (fc.root / "runs").exists() or not any((fc.root / "runs").iterdir())


def test_alias_refuses_contradicting_heading_hold(fc):
    (fc.robot / "profiles" / "fakehh.sh").write_text(FAKE_HH0_PROFILE)
    (fc.robot / "stack_overlay.py").write_text(FAKE_HH_TOOL)
    r = fc("start", "fakehh", "--confirm", "--heading-hold", "on")
    assert r["ok"] is False and "fixes heading hold off" in r["error"], r
    assert fc("start", "fakehh", "--confirm", "--heading-hold", "off", "--speed-cap", "0.7")["ok"]
    assert fc("status")["speed_cap"] == 0.7


def test_estop_latch_needs_confirmed_ready_to_rearm(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    # what the bridge leaves after a link e-stop: latch file; here the walker still reads active (limp)
    (fc.root / "run" / "estop.json").write_text('{"reason": "link", "r3": {"confirmed": false}}')
    st = fc("status")
    assert st["estopped"] is True and st["estop"]["reason"] == "link" and "E-STOPPED" in st["phase_detail"], st
    rc, out = fc.lines("ready")
    assert rc == 3 and out[-1]["phase"] == "needs_confirm" and "E-STOPPED (link)" in out[-1]["msg"], out
    assert (fc.root / "run" / "estop.json").exists() and (fc.root / "run" / "bridge.pid").exists()
    rc, out = fc.lines("ready", "--confirm")
    assert rc == 0 and phases(out) == ["begin", "rearmed", "homed", "standing_up", "standing", "bridge", "ready"], out
    assert not (fc.root / "run" / "estop.json").exists() and list((fc.root / "logs").glob("estop-*.json"))
    assert fc("status")["estopped"] is False


def test_down_and_stop_ignore_the_latch(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    (fc.root / "run" / "estop.json").write_text('{"reason": "fallen"}')
    rc, out = fc.lines("down", "--keep-stack")
    assert rc == 0 and (fc.root / "run" / "estop.json").exists()          # only ready --confirm clears it


FAKE_LIE_PROFILE = r'''
PROFILE_DESC="fake+liedown"; PROFILE_CMD_TOPIC=/llm_cmd_vel; PROFILE_IMU_TOPIC=/imu
PROFILE_WALKER=w; PROFILE_LEG_CONTROLLERS="w ld"; PROFILE_LIEDOWN=ld
profile_env() { export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; }
rosq() {  # walker w + lie-down ld; FAKE_STRICT=refuse: the STRICT handover is refused (nothing changes)
  local fw=$FC_ROOT/fake_walker fl=$FC_ROOT/fake_ld
  [ -f "$fw" ] || echo inactive > "$fw"; [ -f "$fl" ] || echo inactive > "$fl"
  echo "$*" >> "$FC_ROOT/fake_rosq.log"
  if [ "$1" = switch ]; then
    if [ "$2" = ld ]; then
      if [ "${FAKE_STRICT:-ok}" = refuse ] || [ "${4:-}" != strict ]; then
        echo '{"ok":false,"error":"refused","controllers":{"w":"'$(cat $fw)'","ld":"'$(cat $fl)'"}}'; return
      fi
      echo active > "$fl"; echo inactive > "$fw"
    elif [ -n "$2" ]; then echo active > "$fw"; echo inactive > "$fl"
    else echo inactive > "$fw"; echo inactive > "$fl"; fi
  fi
  echo '{"ok":true,"mux":{"ok":true,"alive":true,"problems":[]},"controllers":{"w":"'$(cat $fw)'","ld":"'$(cat $fl)'"}}'
}
profile_start() {
  fc_spawn launch "$1/robot.log" bash -c 'sleep 300; true' ros2_control_node
  printf '{"kind":"stock","verified":true,"speed_cap":null,"speed_cap_effective":0.49,"heading_hold":"on","liedown":{"controller":"ld","init_s":0.2}}\n' > "$1/stack.json"
}
profile_alive() { fc_alive launch; }
profile_stop() { fc_signal_group launch INT 5; }
upright_check() {  # FAKE_UPRIGHT=tilted: lying on its side (a recorded fall rested at 76 deg); none: no IMU verdict
  case "${FAKE_UPRIGHT:-good}" in
    tilted) echo '{"ok": false, "tilt_deg": 76.04, "max_deg": 25.0, "n": 50}' ;;
    none) echo '{"ok": null, "error": "no IMU samples: is the stack running?", "n": 0}' ;;
    *) echo '{"ok": true, "tilt_deg": 3.1, "max_deg": 25.0, "n": 50}' ;;
  esac
}
'''


def test_down_lowers_with_liedown_controller_then_deactivates(fc):
    (fc.robot / "profiles" / "fakelie.sh").write_text(FAKE_LIE_PROFILE)
    assert fc.lines("ready", "fakelie", "--confirm", "--stand-now")[0] == 0
    assert fc("status")["liedown"] is True
    bargv = json.loads((fc.root / "bridge_argv.json").read_text())     # the bridge's IMU lie-down target
    assert bargv[bargv.index("--liedown") + 1] == "ld" and float(bargv[bargv.index("--liedown-wait") + 1]) == 1.2
    (fc.root / "fake_rosq.log").write_text("")
    rc, out = fc.lines("down", "--keep-stack", extra_env={"FC_LIEDOWN_HOLD_S": "0"})
    assert rc == 0 and phases(out) == ["begin", "bridge_parked", "lowering", "bridge_stopped", "lying_down", "down"], out
    assert out[2]["gentle"] is True and out[4]["gentle"] is True and "belly" in out[4]["msg"]
    assert "E-STOP" in out[2]["msg"] and not fc("status")["bridge"]["alive"]
    calls = [l for l in (fc.root / "fake_rosq.log").read_text().splitlines() if l.startswith("switch")]
    assert calls[0] == "switch ld w ld strict" and calls[1].startswith("switch  w ld"), calls   # handover, then off
    assert (fc.root / "fake_walker").read_text().strip() == (fc.root / "fake_ld").read_text().strip() == "inactive"
    assert fc("status")["phase"] == "homed"
    rc, out = fc.lines("ready", "--confirm")                       # stands again on the walker
    assert rc == 0 and (fc.root / "fake_walker").read_text().strip() == "active"


def test_down_falls_back_to_drop_with_clear_message(fc):
    (fc.robot / "profiles" / "fakelie.sh").write_text(FAKE_LIE_PROFILE)
    assert fc.lines("ready", "fakelie", "--confirm", "--stand-now")[0] == 0
    rc, out = fc.lines("down", extra_env={"FAKE_STRICT": "refuse"})
    assert rc == 0 and phases(out) == ["begin", "bridge_parked", "lowering", "bridge_stopped", "lying_down", "down"], out
    assert out[2]["gentle"] is False and "WILL DROP" in out[2]["msg"] and out[4]["gentle"] is False
    assert fc("status")["phase"] == "off"


def test_stop_uses_gentle_lie_down_too(fc):
    (fc.robot / "profiles" / "fakelie.sh").write_text(FAKE_LIE_PROFILE)
    assert fc.lines("ready", "fakelie", "--confirm", "--stand-now")[0] == 0
    (fc.root / "fake_rosq.log").write_text("")
    stop = fc("stop", extra_env={"FC_LIEDOWN_HOLD_S": "0"})
    assert stop["ok"], stop
    assert "switch ld w ld strict" in (fc.root / "fake_rosq.log").read_text()
    assert not fc("status")["bridge"]["alive"]                              # parked for the lowering, then stopped


def test_link_estop_during_lowering_aborts_it_then_down_carries_on(fc):
    """The bridge stays up (parked) while the lie-down controller lowers the robot; a link E-STOP then switches
    every leg controller off (lie-down included), down notices the latch within 0.1 s, confirms, stops."""
    (fc.robot / "profiles" / "fakelie.sh").write_text(FAKE_LIE_PROFILE)
    assert fc.lines("ready", "fakelie", "--confirm", "--stand-now", extra_env={"FAKE_ESTOP_AFTER_PARK_S": "1.5"})[0] == 0
    (fc.root / "fake_rosq.log").write_text("")
    rc, out = fc.lines("down", extra_env={"FC_LIEDOWN_HOLD_S": "4"})
    assert rc == 0, out
    assert phases(out) == ["begin", "bridge_parked", "lowering", "aborted", "bridge_stopped", "lying_down", "down"], out
    assert out[2]["gentle"] is True and out[3]["ok"] is False and out[3]["estop"] == "link"
    assert out[3]["elapsed"] - out[2]["elapsed"] < 2.5                     # not the full 4.2 s blend + hold
    assert out[5]["gentle"] is False and out[5]["aborted"] == "link" and out[-1]["aborted"] == "link"
    calls = [l for l in (fc.root / "fake_rosq.log").read_text().splitlines() if l.startswith("switch")]
    assert calls[0] == "switch ld w ld strict", calls
    latch = json.loads((fc.root / "run" / "estop.json").read_text())
    assert latch["reason"] == "link" and "ld" in latch["legs"].split()      # the bridge's --legs list the lie-down
    st = fc("status")
    assert st["phase"] == "off" and st["estopped"] is True                  # latched until ready --confirm


def test_down_after_an_estop_never_hands_over_to_the_liedown(fc):
    (fc.robot / "profiles" / "fakelie.sh").write_text(FAKE_LIE_PROFILE)
    assert fc.lines("ready", "fakelie", "--confirm", "--stand-now")[0] == 0
    (fc.root / "run" / "estop.json").write_text('{"reason": "link"}')
    (fc.root / "fake_rosq.log").write_text("")
    rc, out = fc.lines("down", "--keep-stack")
    assert rc == 0 and out[phases(out).index("lowering")]["gentle"] is False and "e-stopped (link)" in out[2]["msg"], out
    assert "switch ld" not in (fc.root / "fake_rosq.log").read_text()


def test_down_falls_back_to_bridge_stop_when_park_unconfirmed(fc):
    (fc.robot / "profiles" / "fakelie.sh").write_text(FAKE_LIE_PROFILE)
    assert fc.lines("ready", "fakelie", "--confirm", "--stand-now", extra_env={"FAKE_PARK": "ignore"})[0] == 0
    rc, out = fc.lines("down", "--keep-stack", extra_env={"FC_LIEDOWN_HOLD_S": "0"})
    assert rc == 0 and phases(out) == ["begin", "bridge_stopped", "lowering", "lying_down", "down"], out


def test_ready_replaces_a_bridge_left_parked(fc):
    import signal
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    pid = int((fc.root / "run" / "bridge.pid").read_text().split()[0])
    os.kill(pid, signal.SIGUSR1)                                            # an interrupted down
    deadline = time.time() + 5
    while "PARKED pid=%d" % pid not in (fc.root / "logs" / "bridge.log").read_text() and time.time() < deadline:
        time.sleep(0.05)
    rc, out = fc.lines("ready")
    assert rc == 0 and out[phases(out).index("bridge")]["msg"] == "bridge up", out
    assert int((fc.root / "run" / "bridge.pid").read_text().split()[0]) != pid


FAKE_CAMSTREAM = r'''
import json, os, sys
from http.server import BaseHTTPRequestHandler, HTTPServer
open(os.path.join(os.environ["FC_ROOT"], "cam_argv.json"), "w").write(json.dumps(sys.argv[1:]))
if os.environ.get("FAKE_CAM") == "busy":
    sys.exit("Camera __init__ sequence did not complete")   # like picamera2 when the sensor is held
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        body = b'{"frames":3,"age":0.1}'
        self.send_response(200); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
HTTPServer(("127.0.0.1", int(sys.argv[sys.argv.index("--port") + 1])), H).serve_forever()
'''


def _free_port():
    import socket
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return str(s.getsockname()[1])


def test_cam_start_status_stop_and_stop_ends_it(fc):
    (fc.robot / "camstream.py").write_text(FAKE_CAMSTREAM)
    env = {"FC_CAM_PORT": _free_port()}
    r = fc("cam", "start", extra_env=env)
    assert r["ok"] and r["source"] == "picamera2" and (r["width"], r["fps"]) == (160, 5), r
    argv = json.loads((fc.root / "cam_argv.json").read_text())
    assert argv[argv.index("--source") + 1] == "picamera2" and argv[argv.index("--width") + 1] == "160"
    assert fc("cam", "start", extra_env=env)["error"].startswith("cam already running")
    assert fc("status")["cam"] == {"pid": r["cam_pid"], "alive": True}
    assert fc("cam", "stop") == {"ok": True, "cam": "stopped"}
    assert fc("status")["cam"]["alive"] is False
    # 'stop' (and so 'down') never leaves the feed running
    assert fc("cam", "start", "--width", "320", "--fps", "10", extra_env=env)["ok"]
    fc("stop")
    assert not (fc.root / "run" / "cam.pid").exists()


def test_cam_refuses_bad_options_and_reports_a_busy_sensor(fc):
    (fc.robot / "camstream.py").write_text(FAKE_CAMSTREAM)
    for args in (["--width", "4000"], ["--fps", "60"], ["--quality", "5"], ["--bogus"]):
        assert fc("cam", "start", *args)["ok"] is False
    r = fc("cam", "start", extra_env={"FC_CAM_PORT": _free_port(), "FAKE_CAM": "busy", "FC_CAM_WAIT_S": "2"})
    assert r["ok"] is False and "no frame" in r["error"]
    assert fc("status")["cam"]["alive"] is False


def test_cam_uses_the_stack_camera_topic_when_the_node_runs(fc):
    (fc.robot / "camstream.py").write_text(FAKE_CAMSTREAM)
    assert fc("start", "fake", "--confirm")["ok"]   # fake profile: no lean option -> camera node unknown
    r = fc("cam", "start", extra_env={"FC_CAM_PORT": _free_port()})
    assert r["ok"] and r["source"] == "ros", r


def test_ready_stops_before_the_bridge_when_the_imu_is_not_level_at_rest(fc):
    """rca-20261127-tilt: a bad accel calibration is reported as such, not as a seconds-later imu_tilt lie-down."""
    rc, out = fc.lines("ready", "fake", "--confirm", "--stand-now", extra_env={"FAKE_IMU": "bad"})
    assert rc == 5 and out[-1]["phase"] == "error" and out[-1]["at"] == "imu", out
    assert "imu-level" in out[-1]["msg"] and "STANDING" in out[-1]["msg"]
    assert "bridge" not in phases(out) and not (fc.root / "run" / "bridge.pid").exists()
    rc, out = fc.lines("ready", "--confirm", "--skip-imu-check", extra_env={"FAKE_IMU": "bad"})
    assert rc == 0 and phases(out)[-2:] == ["bridge", "ready"], out


def test_ready_continues_without_an_imu_verdict(fc):
    rc, out = fc.lines("ready", "fake", "--confirm", "--stand-now", extra_env={"FAKE_IMU": "none"})
    assert rc == 0 and phases(out)[-2:] == ["bridge", "ready"], out


def test_imu_level_refuses_without_a_stack_and_status_reports_imu_cal(fc):
    out = fc("imu-level")
    assert out["ok"] is False and out["action"] == "refused" and "no robot stack" in out["error"]
    assert fc("imu-level", "--bogus")["ok"] is False
    rc, _ = fc.lines("ready", "fake", "--confirm", "--stand-now")
    assert rc == 0
    out = fc("imu-level")                      # walker active: legs not off
    assert out["ok"] is False and "leg controller is active" in out["error"]
    st = fc("status")
    assert st["imu_cal"]["stored"] is None and st["imu_cal"]["applied"] is None
    assert st["imu_cal"]["driver_supports_cal"] is False   # the fake robot.log has no FC_IMU_CAL line


# ---- fall safety: tilt gate before any stand, getup (no homing) ------------------------------------------------
def _fallen(fc, reason="fallen"):
    """What a fall leaves: walker switched off by the bridge (R3 action), latch file, latched bridge running."""
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    (fc.root / "fake_walker").write_text("inactive\n")
    (fc.root / "run" / "estop.json").write_text(json.dumps({"reason": reason, "tilt_deg": 76.0}))
    return (fc.root / "run" / "launch.pid").read_text()


def test_ready_never_stands_a_robot_lying_on_its_side(fc):
    assert fc("start", "fake", "--confirm")["ok"]
    rc, out = fc.lines("ready", "--confirm", extra_env={"FAKE_UPRIGHT": "tilted"})
    assert rc == 6 and out[-1]["at"] == "upright" and out[-1]["upright"] is False and out[-1]["tilt_deg"] == 76.04, out
    assert "belly" in out[-1]["msg"] and out[-1]["done"]
    assert "standing_up" not in phases(out) and not (fc.root / "fake_walker").exists()      # no stand request at all
    assert not (fc.root / "run" / "bridge.pid").exists()
    # the no-stack path (start, then --stand-now) checks too, after homing and before the stand
    assert fc("stop")["ok"]
    rc, out = fc.lines("ready", "fake", "--confirm", "--stand-now", extra_env={"FAKE_UPRIGHT": "tilted"})
    assert rc == 6 and phases(out) == ["begin", "starting", "homed", "error"], out
    assert (fc.root / "fake_walker").read_text().strip() == "inactive"      # (stop above wrote it) never activated


def test_ready_tilt_gate_runs_before_the_rearm_and_keeps_the_latch(fc):
    _fallen(fc)
    rc, out = fc.lines("ready", "--confirm", extra_env={"FAKE_UPRIGHT": "tilted"})
    assert rc == 6 and out[-1]["at"] == "upright" and "rearmed" not in phases(out), out
    assert (fc.root / "run" / "estop.json").exists()                          # still latched
    assert (fc.root / "fake_walker").read_text().strip() == "inactive"


def test_ready_without_an_imu_verdict_refuses_unless_skipped(fc):
    assert fc("start", "fake", "--confirm")["ok"]
    rc, out = fc.lines("ready", "--confirm", extra_env={"FAKE_UPRIGHT": "none"})
    assert rc == 6 and out[-1]["at"] == "upright" and out[-1]["upright"] is None and "no IMU reading" in out[-1]["msg"], out
    assert not (fc.root / "fake_walker").exists()
    rc, out = fc.lines("ready", "--confirm", "--skip-imu-check", extra_env={"FAKE_UPRIGHT": "none"})
    assert rc == 0 and phases(out)[-2:] == ["bridge", "ready"], out


def test_getup_rearms_and_stands_without_homing(fc):
    launch = _fallen(fc)
    rc, out = fc.lines("getup")
    assert rc == 0 and phases(out) == ["begin", "rearmed", "homed", "standing_up", "standing", "bridge", "ready"], out
    assert out[0]["getup"] is True and "no homing" in out[0]["msg"]
    assert (fc.root / "run" / "launch.pid").read_text() == launch             # same stack: no homing
    assert not (fc.root / "run" / "estop.json").exists() and list((fc.root / "logs").glob("estop-*.json"))
    assert fc("status")["phase"] == "ready"
    rc, out = fc.lines("getup")                                                 # idempotent on a playing robot
    assert rc == 0 and phases(out) == ["begin", "homed", "standing", "bridge", "ready"], out


def test_getup_after_an_operator_estop_with_the_walker_reading_active(fc):
    assert fc.lines("ready", "fake", "--confirm", "--stand-now")[0] == 0
    (fc.root / "run" / "estop.json").write_text('{"reason": "link"}')         # walker still reads active (limp)
    rc, out = fc.lines("getup")
    assert rc == 0 and phases(out) == ["begin", "rearmed", "homed", "standing_up", "standing", "bridge", "ready"], out


def test_getup_needs_a_running_stack(fc):
    rc, out = fc.lines("getup")
    assert rc == 7 and out[-1]["at"] == "stack" and "needs Ready with homing (on the stand)" in out[-1]["msg"], out
    assert not (fc.root / "run" / "launch.pid").exists() and not (fc.root / "fake_walker").exists()
    rc, out = fc.lines("getup", "fake")
    assert rc == 2 and out[-1]["at"] == "args"


def test_getup_stays_down_on_an_imu_fault_latch(fc):
    _fallen(fc, reason="imu_frozen")
    rc, out = fc.lines("getup")
    assert rc == 8 and out[-1]["at"] == "imu_fault" and "imu_frozen" in out[-1]["msg"] and "Ready" in out[-1]["msg"], out
    assert (fc.root / "run" / "estop.json").exists() and (fc.root / "fake_walker").read_text().strip() == "inactive"
    assert (fc.root / "run" / "bridge.pid").exists()                          # the latched bridge is left as it was


def test_getup_stays_down_when_not_upright_or_without_imu(fc):
    _fallen(fc)
    for up in ("tilted", "none"):
        rc, out = fc.lines("getup", extra_env={"FAKE_UPRIGHT": up})
        assert rc == 6 and out[-1]["at"] == "upright" and "rearmed" not in phases(out), (up, out)
        assert (fc.root / "run" / "estop.json").exists()
        assert (fc.root / "fake_walker").read_text().strip() == "inactive"
