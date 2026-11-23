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
rosq() {  # fake graph: FAKE_TOPO=good|bad
  if [ "${FAKE_TOPO:-good}" = good ]; then
    echo '{"mux":{"ok":true,"alive":true,"problems":[],"cmd_vel_pubs":["cmd_vel_mux"],"topic":"/llm_cmd_vel","topic_subs":["cmd_vel_mux"],"topic_pubs":[],"teleop_pubs":["teleop_twist_joy_node"]}}'
  else
    echo '{"mux":{"ok":false,"alive":true,"problems":["/cmd_vel publishers [cmd_vel_mux, cmd_vel_mux] (want exactly cmd_vel_mux)"],"cmd_vel_pubs":["cmd_vel_mux","cmd_vel_mux"],"topic":"/llm_cmd_vel","topic_subs":[],"topic_pubs":[],"teleop_pubs":[]}}'
  fi
}
profile_start() {
  fc_spawn launch "$1/robot.log" sleep 300
  fc_spawn mux "$1/mux.log" sleep 300
  sleep "${FAKE_HOME_S:-0}"
  fc_topology_wait 2
}
profile_alive() { fc_alive launch; }
profile_stop() { fc_signal_group mux INT 5; fc_signal_group launch INT 5; }
'''

FAKE_BRIDGE = r'''
import json, os, sys, time
open(os.path.join(os.environ["FC_ROOT"], "bridge_argv.json"), "w").write(json.dumps(sys.argv[1:]))
time.sleep(300)
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

    run.root = root
    run.robot = robot
    yield run
    run("stop")
    for pidfile in (root / "run").glob("*.pid"):   # belt and braces: never leak sleepers
        try:
            os.kill(int(pidfile.read_text().split()[0]), 9)
        except (OSError, ValueError):
            pass


def lock_holders(root):
    lock = str(root / "run" / "lock")
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
    assert fc("status")["heading_hold"] is None                # fake profile declares nothing
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


# Heading-hold-off variants (stock_hh0 / tej_u6_hh0) call fc_hh_overlay first in profile_start.
FAKE_HH0_PROFILE = FAKE_PROFILE.replace("profile_start() {\n", 'profile_start() {\n  fc_hh_overlay u6 "$1" || return 1\n') + \
    "\nPROFILE_HEADING_HOLD=off\n"
FAKE_HH_TOOL = r"""
import json, os, sys
kind, out, root = sys.argv[1], sys.argv[2], sys.argv[4]
assert os.path.realpath(out).startswith(os.path.realpath(root) + "/")
os.makedirs(out)
print(json.dumps({"kind": kind, "verified": True, "policy": out + "/p.json", "sha256": "9f5a3e98" + "0" * 56,
                  "heading_hold": {"kp": 0.0}}))
"""


def test_overlay_failure_refuses_start(fc):
    # The real imu/heading_hold.py: no Tej tree here, so it must fail loudly and nothing may be spawned.
    (fc.robot / "profiles" / "fakehh.sh").write_text(FAKE_HH0_PROFILE)
    start = fc("start", "fakehh", "--confirm")
    assert start["ok"] is False
    run_dir = sorted((fc.root / "runs").iterdir())[-1]
    log = (run_dir / "starter.log").read_text()
    assert "COULD NOT BE BUILT" in log and "heading_hold: " in log, log
    assert not (fc.root / "run" / "launch.pid").exists() and not (run_dir / "heading_hold.json").exists()
    assert fc("status")["heading_hold"] == "off"
    assert fc("stop")["ok"]
    assert lock_holders(fc.root) == []


def test_status_reports_heading_hold_off(fc):
    (fc.robot / "profiles" / "fakehh.sh").write_text(FAKE_HH0_PROFILE)
    (fc.robot / "imu" / "heading_hold.py").write_text(FAKE_HH_TOOL)
    start = fc("start", "fakehh", "--confirm")
    assert start["ok"] and start["heading_hold"] == "off", start
    st = fc("status")
    assert st["heading_hold"] == "off" and st["heading_hold_overlay"]["verified"] is True
    assert st["heading_hold_overlay"]["sha256"].startswith("9f5a3e98") and st["heading_hold_overlay"]["kind"] == "u6"
    assert lock_holders(fc.root) == []
    assert fc("stop")["ok"]
