"""provision/: robots.json guards, probe evaluation, the read-only probe, and provision.sh end to end
against a fake `ssh` that runs "remote" commands locally with HOME = a temp "Pi home"."""
import json
import os
import platform
import shutil
import stat
import subprocess
import textwrap
from pathlib import Path

import pytest

from fc.robot.provision import report as rp

ROBOT = Path(__file__).resolve().parents[1]
PROV = ROBOT / "provision"
ROBOTS = [
    {"id": "blue", "team": "blue", "tag": 10, "host": "mts", "profile": "stock", "udp_host": "10.0.0.5"},
    {"id": "red", "team": "red", "tag": 11, "host": "", "profile": "stock", "udp_host": ""},
]


# ---- robots.json guards -----------------------------------------------------------------------
def test_config_fill_host_then_ok(tmp_path):
    assert rp.config_action(ROBOTS, "red", "red", 11, "pupper-red") == "fill-host"
    path = tmp_path / "robots.json"
    path.write_text(json.dumps(ROBOTS))
    rp.set_host(path, "red", "pupper-red")
    rp.set_host(path, "red", "pupper-red")            # idempotent
    robots = json.loads(path.read_text())
    assert robots[1]["host"] == "pupper-red" and robots[0] == ROBOTS[0]
    assert rp.config_action(robots, "red", "red", 11, "pupper-red") == "ok"
    with pytest.raises(rp.ConfigError, match="already has host"):
        rp.set_host(path, "red", "elsewhere")


@pytest.mark.parametrize("args, msg", [
    (("red", "red", 11, "mts"), "belongs to robot 'blue'"),        # never put red's identity on mts
    (("red", "red", 10, "pupper-red"), "robots.json says red is team red tag 11"),
    (("red", "blue", 11, "pupper-red"), "robots.json says red is team red tag 11"),
    (("green", "red", 12, "x"), "no robots.json entry"),
    (("red", "red", 2, "x"), "field corners"),
    (("red", "pink", 11, "x"), "team must be"),
    (("../x", "red", 11, "x"), "bad robot id"),
    (("red", "red", 11, "-oProxyCommand=x"), "bad ssh host"),
    (("blue", "blue", 10, "pupper-red"), "not 'pupper-red'"),
])
def test_config_refusals(args, msg):
    with pytest.raises(rp.ConfigError, match=msg):
        rp.config_action(ROBOTS, *args)


def test_config_refuses_shared_tag_or_team():
    robots = [dict(ROBOTS[0]), dict(ROBOTS[1], tag=10)]
    with pytest.raises(rp.ConfigError, match="tag 10 is already robot 'blue'"):
        rp.config_action(robots, "red", "red", 10, "pupper-red")
    robots = [dict(ROBOTS[0]), dict(ROBOTS[1], team="blue")]
    with pytest.raises(rp.ConfigError, match="team blue is already"):
        rp.config_action(robots, "red", "blue", 11, "pupper-red")


# ---- probe evaluation -------------------------------------------------------------------------
GOOD = {
    "probe_done": "yes", "user": "pi", "hostname": "pupper-red", "model": "Raspberry Pi 5", "os": "Debian 12",
    "arch": "aarch64", "ros_jazzy": "yes", "ros_distros": "jazzy", "mono": "yes", "mono_rev": "0e50afb",
    **{f"pkg_{p}": "install" for p in rp.STOCK_PKGS}, "stock_config": "yes", "launch_starts_mux": "no",
    "launch_teleop_arg": "yes", "policy_sha": "ecf3a4ab0000", "python": "3.12.3", "rclpy": "yes",
    "disk_free_gb": "40.0", "spidev": "2", "i2c": "2", "gamepad_paired": "1", "temp_c": "55.0",
    "running_control": "0", "running_launch": "0", "fc_robot_id": "red", "fc_link_key_mode": "600",
    "fc_identity": json.dumps({"robot_id": "red", "team": "red", "tag": 11}), "ssh_hostkey_fp": "SHA256:red",
}


def levels(rows):
    return {item: lvl for lvl, item, _ in rows}


def test_good_probe_is_ready():
    rows = rp.evaluate(GOOD, "red", "red", 11)
    assert not [r for r in rows if r[0] != "PASS"], rows
    assert rp.render(rows)[1] is True


@pytest.mark.parametrize("change, item, level", [
    ({"launch_starts_mux": "yes"}, "launch.py mux", "FAIL"),
    ({"pkg_cmd_vel_mux": "missing"}, "packages", "FAIL"),
    ({"ros_jazzy": "no"}, "ros", "FAIL"),
    ({"rclpy": "no"}, "rclpy", "FAIL"),
    ({"python": "3.8.10"}, "python", "FAIL"),
    ({"disk_free_gb": "0.5"}, "disk", "FAIL"),
    ({"spidev": "0"}, "motor SPI", "FAIL"),
    ({"fc_robot_id": "blue"}, "robot_id", "FAIL"),
    ({"fc_link_key_mode": "644"}, "link.key", "FAIL"),
    ({"gamepad_paired": "0"}, "gamepad", "FAIL"),     # no stand-up (X) and no R3 e-stop
    ({"policy_sha": "123456789abc"}, "policy", "WARN"),
    ({"running_control": "1"}, "idle", "WARN"),
    ({"hostname": "mts-pi"}, "identity", "WARN"),
    ({"appliance_mode": "yes"}, "appliance", "WARN"),
])
def test_probe_findings(change, item, level):
    assert levels(rp.evaluate({**GOOD, **change}, "red", "red", 11))[item] == level


def test_clone_of_another_robot_is_detected_by_host_key():
    rows = rp.evaluate({**GOOD, "ssh_hostkey_fp": "SHA256:mts"}, "red", "red", 11, other_fps={"blue": "SHA256:mts"})
    assert levels(rows)["identity"] == "FAIL" and rp.render(rows)[1] is False


def test_truncated_probe_fails():
    rows = rp.evaluate({"hostname": "x"}, "red", "red", 11)
    assert [(lvl, item) for lvl, item, _ in rows] == [("FAIL", "probe")]


def test_staged_files_compared_with_laptop():
    local = rp.local_shas(ROBOT)
    assert set(local) == set(rp.STAGED)
    p = {**GOOD, **{f"fc_sha_{f}": s for f, s in local.items()}}
    assert levels(rp.evaluate(p, "red", "red", 11, local=local))["fc files"] == "PASS"
    p["fc_sha_fcctl.sh"] = "000000000000"
    assert levels(rp.evaluate(p, "red", "red", 11, local=local))["fc files"] == "FAIL"


# ---- the probe script and provision.sh against a fake Pi --------------------------------------
def _exe(path: Path, body: str) -> None:
    path.write_text(body)
    path.chmod(0o755)


@pytest.fixture
def fake(tmp_path):
    """A copied fc/robot + config tree (so robots.json edits stay in tmp), a fake Pi home, and a PATH
    whose `ssh` runs remote commands locally there and whose system tools are inert stubs."""
    tree = tmp_path / "fc"
    shutil.copytree(ROBOT, tree / "robot", ignore=shutil.ignore_patterns("vendor", "__pycache__", ".pytest_cache"))
    (tree / "config").mkdir()
    (tree / "config" / "robots.json").write_text(json.dumps(ROBOTS, indent=2) + "\n")
    key = tree / "config" / "link.key"
    key.write_text("ab" * 32 + "\n")
    key.chmod(0o600)
    home = tmp_path / "pihome"
    home.mkdir()
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    log = tmp_path / "ssh.log"
    _exe(bin_ / "ssh", textwrap.dedent(f"""\
        #!/bin/bash
        echo "$*" >> {log}
        while [[ "${{1:-}}" == -* ]]; do
          case $1 in -G) echo "hostname 10.9.9.9"; exit 0;; -o|-p|-l|-i|-e) shift 2;; *) shift;; esac
        done
        shift   # host
        cd {home}
        if [ $# -eq 0 ]; then exec env HOME={home} bash; fi
        exec env HOME={home} bash -c "$*"
        """))
    for tool in ("sudo", "systemctl", "tailscale", "bluetoothctl", "nmcli", "iwgetid", "hostnamectl"):
        _exe(bin_ / tool, "#!/bin/sh\nexit 1\n")
    if platform.system() == "Darwin":   # GNU spellings used by stage.sh / probe.sh
        _exe(bin_ / "stat", '#!/bin/sh\nif [ "$1" = -c ]; then shift 2; exec /usr/bin/stat -f %Lp "$@"; fi\n'
                            'exec /usr/bin/stat "$@"\n')
        _exe(bin_ / "sha256sum", '#!/bin/sh\nexec shasum -a 256 "$@"\n')
    env = {**os.environ, "PATH": f"{bin_}:{os.environ['PATH']}"}
    return {"tree": tree, "home": home, "env": env, "log": log}


def _files(root: Path) -> set[str]:
    return {str(p.relative_to(root)) for p in root.rglob("*")}


def test_probe_script_writes_nothing(fake):
    out = subprocess.run(["bash", "-s"], stdin=open(PROV / "probe.sh"), capture_output=True, text=True,
                         env={**fake["env"], "HOME": str(fake["home"])}, cwd=fake["home"], timeout=60)
    p = rp.parse_probe(out.stdout)
    assert p["probe_done"] == "yes", out.stderr
    assert p["fc_dir"] == "no" and p["sudo_nopass"] == "no"
    assert _files(fake["home"]) == set()


def run_prov(fake, *args):
    return subprocess.run(["bash", str(fake["tree"] / "robot" / "provision" / "provision.sh"), *args],
                          capture_output=True, text=True, env=fake["env"], timeout=120)


def test_check_mode_writes_nothing(fake):
    robots_before = (fake["tree"] / "config" / "robots.json").read_text()
    r = run_prov(fake, "--check", "pupper-red", "red", "red", "11")
    assert r.returncode == 1, r.stdout + r.stderr        # fake Pi has no ROS: not ready
    assert "READY for `fcctl.sh start`: NO" in r.stdout
    assert "robot_id" in r.stdout and "FAIL" in r.stdout
    assert _files(fake["home"]) == set()
    assert (fake["tree"] / "config" / "robots.json").read_text() == robots_before


def test_fix_clone_identity_needs_passwordless_sudo_and_changes_nothing(fake):
    r = run_prov(fake, "--fix-clone-identity", "pupper-red", "--yes", "pupper-red", "red", "red", "11")
    assert r.returncode == 2 and "passwordless sudo" in r.stderr
    assert _files(fake["home"]) == set()
    assert run_prov(fake, "--fix-clone-identity", "Bad_Name", "pupper-red", "red", "red", "11").returncode == 2
    assert run_prov(fake, "--check", "--fix-clone-identity", "x", "pupper-red", "red", "red", "11").returncode == 2


def test_refuses_to_put_red_on_blues_host_before_any_ssh(fake):
    r = run_prov(fake, "mts", "red", "red", "11")
    assert r.returncode == 2 and "belongs to robot 'blue'" in r.stderr
    assert not fake["log"].exists()


def test_no_default_host(fake):
    r = run_prov(fake)
    assert r.returncode == 2 and not fake["log"].exists()


@pytest.mark.skipif(shutil.which("rsync") is None, reason="needs rsync")
def test_provision_stages_only_into_fc_v1(fake):
    r = run_prov(fake, "pupper-red", "red", "red", "11")
    out = r.stdout + r.stderr
    assert r.returncode in (0, 1), out
    fc = fake["home"] / "fc-v1"
    assert all(f.startswith("fc-v1") for f in _files(fake["home"])), _files(fake["home"])
    assert (fc / "robot" / "robot_id").read_text().strip() == "red"
    assert stat.S_IMODE((fc / "robot" / "link.key").stat().st_mode) == 0o600
    assert not (fc / "robot" / "provision").exists()       # laptop-only
    ident = json.loads((fc / "identity.json").read_text())
    assert ident["robot_id"] == "red" and ident["team"] == "red" and ident["tag"] == 11
    robots = json.loads((fake["tree"] / "config" / "robots.json").read_text())
    assert robots[1]["host"] == "pupper-red" and robots[0]["host"] == "mts"
    assert "PASS  robot_id" in r.stdout and "PASS  fc files" in r.stdout
    # idempotent: a second run changes nothing it already did
    r2 = run_prov(fake, "pupper-red", "red", "red", "11")
    assert r2.returncode == r.returncode, r2.stdout + r2.stderr
    assert not [ln for ln in r2.stdout.splitlines() if ln.startswith(("<f", "*deleting"))], r2.stdout
