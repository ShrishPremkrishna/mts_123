"""imu/i2c_recover.sh against a fake pinctrl/sudo/sysfs (no hardware). Run anywhere with bash.

The fake bus: a device holds SDA low until it has seen `release_after` SCL falling edges (the classic
stuck-mid-byte slave), or holds SCL low forever (`scl_held`, needs a power cycle)."""
import json
import os
import stat
import subprocess
import textwrap

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "imu", "i2c_recover.sh")

FAKE_PINCTRL = textwrap.dedent(r"""
    #!/usr/bin/env python3
    import json, os, sys
    st = os.environ["FAKE_BUS"]
    s = json.load(open(st))
    def level(pin):
        p = s["pins"][str(pin)]
        if pin == 3:
            if s["scl_held"]: return 0
            return p["out"] if p["func"] == "op" else 1
        if s["sda_stuck"]: return 0
        return p["out"] if p["func"] == "op" else 1
    args = sys.argv[1:]
    if args[0] == "lev":
        print(level(int(args[1])))
    elif args[0] == "set":
        for pin in args[1].split(","):
            p = s["pins"][pin]
            for o in args[2:]:
                if o in ("ip", "op", "a3"): p["func"] = o
                elif o in ("dh", "dl"):
                    new = 1 if o == "dh" else 0
                    if pin == "3" and p["out"] == 1 and new == 0 and p["func"] == "op":
                        s["edges"] += 1
                        if s["sda_stuck"] and s["edges"] >= s["release_after"]:
                            s["sda_stuck"] = False
                    p["out"] = new
        s["log"].append(" ".join(args))
    json.dump(s, open(st, "w"))
""").lstrip()


def make_env(tmp_path, sda_stuck=True, scl_held=False, release_after=5, ack=True, bound=True):
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    for name, body in (("pinctrl", FAKE_PINCTRL),
                       ("i2ctransfer", "#!/bin/sh\nexit %d\n" % (0 if ack else 1)),
                       ("fakesudo", '#!/bin/sh\n"$@"\n')):
        p = bin_ / name
        p.write_text(body)
        p.chmod(p.stat().st_mode | stat.S_IEXEC)
    drv = tmp_path / "drv"
    drv.mkdir()
    (drv / "unbind").write_text("")
    (drv / "bind").write_text("")
    if bound:
        (drv / "1f00074000.i2c").mkdir()
    dev = tmp_path / "i2c-1"
    dev.write_text("")
    bus = tmp_path / "bus.json"
    a3 = {"func": "a3", "out": 1}
    bus.write_text(json.dumps({"pins": {"2": dict(a3), "3": dict(a3)}, "sda_stuck": sda_stuck,
                               "scl_held": scl_held, "release_after": release_after, "edges": 0, "log": []}))
    env = dict(os.environ, PATH=str(bin_) + os.pathsep + os.environ["PATH"], FAKE_BUS=str(bus),
               FC_PINCTRL="pinctrl", FC_SUDO=str(bin_ / "fakesudo"), FC_I2C_DRV=str(drv),
               FC_I2C_DEVNODE=str(dev), FC_I2CTRANSFER="i2ctransfer", FC_I2C_SLEEP="0")
    return env, bus, drv


def run(env, *args):
    r = subprocess.run(["bash", SCRIPT, "--force", *args], env=env, capture_output=True, text=True, timeout=60)
    return r.returncode, json.loads(r.stdout.strip().splitlines()[-1]), r.stderr


def test_idle_bus_does_nothing(tmp_path):
    env, bus, drv = make_env(tmp_path, sda_stuck=False)
    rc, out, _ = run(env)
    assert rc == 0 and out["action"] == "none" and out["bno_ack"] is True
    assert json.loads(bus.read_text())["log"] == [] and (drv / "unbind").read_text() == ""


def test_check_is_read_only(tmp_path):
    env, bus, drv = make_env(tmp_path)
    rc, out, _ = run(env, "--check")
    assert rc == 1 and out["action"] == "check" and out["before"] == {"sda": 0, "scl": 1}
    assert json.loads(bus.read_text())["log"] == [] and (drv / "unbind").read_text() == ""


def test_stuck_sda_freed_by_pulses_then_rebound(tmp_path):
    env, bus, drv = make_env(tmp_path, release_after=5)
    rc, out, err = run(env)
    s = json.loads(bus.read_text())
    assert rc == 0 and out["action"] == "recovered", err
    assert out["pulses"] == 5 and out["after"] == {"sda": 1, "scl": 1} and out["rebound"] is True
    assert (drv / "unbind").read_text().strip() == "1f00074000.i2c"
    assert (drv / "bind").read_text().strip() == "1f00074000.i2c"
    assert s["pins"]["2"]["func"] == "a3" and s["pins"]["3"]["func"] == "a3"     # pins handed back
    assert s["log"][-1] == "set 2,3 a3 pu"


def test_scl_held_by_device_needs_power_cycle(tmp_path):
    env, bus, _ = make_env(tmp_path, scl_held=True)
    rc, out, _ = run(env)
    s = json.loads(bus.read_text())
    assert rc == 1 and out["action"] == "scl_held" and out["scl_held"] is True and out["pulses"] == 0
    assert "power cycle" in out["msg"] and s["pins"]["3"]["func"] == "a3"


def test_device_never_releases_reports_failure(tmp_path):
    env, _, _ = make_env(tmp_path, release_after=1000)
    rc, out, _ = run(env)
    assert rc == 1 and out["action"] == "failed" and out["pulses"] == 16


def test_refuses_while_a_stack_runs(tmp_path):
    env, bus, _ = make_env(tmp_path)
    fake = tmp_path / "bin" / "pgrep"
    fake.write_text("#!/bin/sh\nexit 0\n")
    fake.chmod(0o755)
    r = subprocess.run(["bash", SCRIPT], env=env, capture_output=True, text=True, timeout=30)
    out = json.loads(r.stdout.strip())
    assert r.returncode == 1 and out["action"] == "refused" and json.loads(bus.read_text())["log"] == []
