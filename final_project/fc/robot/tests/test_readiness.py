"""readiness.phase / legs (pure) and boot/install.sh print (no systemd needed)."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import readiness  # noqa: E402

LEGS = ["w", "w2"]
GOOD_MUX = {"ok": True, "problems": []}


def st(**kw):
    base = {"profile": "stock", "profile_alive": True, "starting": False, "ready": True, "start_failed": False,
            "stale": False, "foreign_stack": False, "ros2_control_node": True, "bridge_alive": False,
            "walker": "w", "leg_controllers": LEGS,
            "ros": {"mux": GOOD_MUX, "controllers": {"w": "inactive", "w2": "inactive", "jsb": "active"}}}
    base.update(kw)
    return readiness.phase(base)


def test_power_on_sequence():
    assert readiness.phase({})["phase"] == "off"
    assert st(starting=True, ready=False)["phase"] == "starting"
    assert st()["phase"] == "homed"
    standing = {"mux": GOOD_MUX, "controllers": {"w": "active", "w2": "inactive"}}
    assert st(ros=standing)["phase"] == "standing"
    r = st(ros=standing, bridge_alive=True)
    assert r["phase"] == "ready" and r["light"] == "green"


def test_other_leg_controller_counts_as_standing():
    assert st(ros={"mux": GOOD_MUX, "controllers": {"w": "inactive", "w2": "active"}})["phase"] == "standing"


def test_faults():
    assert st(foreign_stack=True)["phase"] == "fault"
    assert st(start_failed=True)["phase"] == "fault"
    assert st(profile_alive=False)["phase"] == "fault"
    assert st(ready=False)["phase"] == "fault"                          # alive, no starter, never READY
    assert st(ros=None)["phase"] == "fault"                             # manager did not answer
    bad = {"mux": {"ok": False, "problems": ["/cmd_vel publishers [a, b]"]}, "controllers": {"w": "active"}}
    r = st(ros=bad, bridge_alive=True)
    assert r["phase"] == "fault" and "/cmd_vel publishers" in r["detail"] and r["light"] == "red"


def test_stale_state_from_earlier_boot_is_off():
    assert st(stale=True, profile_alive=False, ros2_control_node=False)["phase"] == "off"
    # ...unless something is actually running
    assert st(stale=True, profile_alive=False, ros2_control_node=True)["phase"] == "fault"
    assert st(stale=True)["phase"] == "homed"                          # alive wins over a stale marker


def test_legs_cli():
    q = json.dumps({"ok": False, "error": "latched", "mux": {"ok": True}, "controllers": {"w": "inactive"}})
    out = subprocess.run([sys.executable, os.path.join(HERE, "readiness.py"), "legs", q, "w", "w w2"],
                         capture_output=True, text=True, check=True).stdout
    r = json.loads(out)
    assert r == {"reachable": True, "walker": "inactive", "walker_active": False, "active_legs": [],
                 "mux_ok": True, "switch_ok": False}
    bad = subprocess.run([sys.executable, os.path.join(HERE, "readiness.py"), "legs", "not json", "w", "w"],
                         capture_output=True, text=True, check=True).stdout
    assert json.loads(bad)["reachable"] is False


def test_install_print_renders_unit_without_changes(tmp_path):
    env = {**os.environ, "FC_BOOT_USER": "pi", "FC_BOOT_HOME": "/home/pi"}
    out = subprocess.run(["bash", os.path.join(HERE, "boot", "install.sh"), "print", "--profile", "tej_u6",
                          "--mode", "home", "--delay", "20"], capture_output=True, text=True, env=env, check=True).stdout
    assert "@USER@" not in out and "@HOME@" not in out
    assert "ExecStart=/bin/bash /home/pi/fc-v1/robot/fcctl.sh boot" in out
    assert "ExecStop=/bin/bash /home/pi/fc-v1/robot/fcctl.sh stop" in out
    assert "User=pi" in out and "KillMode=process" in out and "RemainAfterExit=yes" in out
    assert "Conflicts=" not in out                                     # must never stop someone's stack
    assert "FC_BOOT_PROFILE=tej_u6\nFC_BOOT_MODE=home\nFC_BOOT_DELAY_S=20" in out
    for bad in (["--mode", "stand"], ["--profile", "nope"], ["--delay", "999"]):
        r = subprocess.run(["bash", os.path.join(HERE, "boot", "install.sh"), "print", *bad],
                           capture_output=True, text=True, env=env)
        assert r.returncode == 2, bad
