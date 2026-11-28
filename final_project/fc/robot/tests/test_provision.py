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


# ---- Pi OS items (os_items.sh): what mts has and a fresh class card lacks --------------------------
FRESH_CFG = ("dtparam=spi=on\ndtparam=i2c_arm=on,i2c_arm_baudrate=100000\n"      # infra/pupper_image_builder
             "dtoverlay=waveshare-4dpic-3b\ndtoverlay=waveshare-4dpic-4b\ndtoverlay=waveshare-4dpic-5b\n"
             "dtoverlay=hifiberry-dac\n")
FAN = [f"dtparam=fan_temp{i}={t},fan_temp{i}_hyst=3000,fan_temp{i}_speed={s}"
       for i, t, s in ((0, 40000, 125), (1, 50000, 175), (2, 55000, 250), (3, 60000, 255))]
MTS_CFG = ("dtparam=spi=on\ndtparam=i2c_arm=on,i2c_arm_baudrate=100000\n"         # as changed Nov 24 + 25
           + "".join(f"#fc-off(2026-11-25 i2c touch) dtoverlay=waveshare-4dpic-{n}b\n" for n in (3, 4, 5))
           + "dtoverlay=hifiberry-dac\n\n# --- Pupper FC heat reduction (2026-11-24); undo: delete this block, reboot ---\n"
           + "[all]\ndtparam=pciex1=off\n" + "".join(f + "\n" for f in FAN) + "# --- end Pupper FC ---\n")
LIBREL = "pupperv3-monorepo/ros2_ws/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so"
V4REL = "fc-v1/build/imu-filter-v4/install/control_board_hardware_interface/lib/libcontrol_board_hardware_interface.so"


@pytest.fixture
def card(fake, tmp_path):
    """The fake Pi as a fresh class card: stock config.txt, a stock driver .so, a per-chip imu_cal.json."""
    cfg = tmp_path / "config.txt"
    cfg.write_text(FRESH_CFG)
    home = fake["home"]
    lib = home / LIBREL
    lib.parent.mkdir(parents=True)
    lib.write_text("stock driver\n")
    cal = home / "fc-v1" / "run" / "imu_cal.json"
    cal.parent.mkdir(parents=True)
    cal.write_text('{"accel_bias_chip": [3.05, -0.34, -0.09]}\n')
    env = {**fake["env"], "HOME": str(home), "FC_BOOT_CONFIG": str(cfg)}
    return {**fake, "cfg": cfg, "lib": lib, "cal": cal, "osenv": env}


def run_os(card, *args, **env):
    return subprocess.run(["bash", "-s", "--", *args], stdin=open(PROV / "os_items.sh"), capture_output=True,
                          text=True, env={**card["osenv"], **env}, cwd=card["home"], timeout=60)


def os_states(out):
    return {ln.split()[1]: ln.split()[0] for ln in out.splitlines()
            if ln.split()[:1] and ln.split()[0] in ("ok", "missing")}


def _snapshot(root):
    return {p: (p.read_bytes() if p.is_file() and not p.is_symlink() else os.readlink(p) if p.is_symlink() else None)
            for p in root.rglob("*")}


def _fake_v4(card):
    v4 = card["home"] / V4REL
    v4.parent.mkdir(parents=True)
    v4.write_bytes(b"\x7fELF ... FC_IMU_CAL ...\n")
    return v4


def test_os_check_fresh_card_is_read_only_and_names_every_item(card):
    before, cfg_before = _snapshot(card["home"]), card["cfg"].read_text()
    for mode in ("check", "plan"):
        r = run_os(card, mode)
        assert r.returncode == 1, r.stdout + r.stderr
        assert os_states(r.stdout) == {"hailo": "missing", "fan": "missing", "gui": "ok", "touch": "missing",
                                       "imu_v4": "missing"}
    assert "undo:" in r.stdout and r.stdout.count("undo:") == 4
    assert _snapshot(card["home"]) == before and card["cfg"].read_text() == cfg_before


def test_os_check_mts_state_is_all_ok(card):
    card["cfg"].write_text(MTS_CFG)
    v4 = _fake_v4(card)
    card["lib"].rename(str(card["lib"]) + ".stock-link")
    card["lib"].symlink_to(v4)
    r = run_os(card, "check")
    assert r.returncode == 0, r.stdout + r.stderr
    assert set(os_states(r.stdout).values()) == {"ok"}


def test_os_apply_refuses_without_sudo_and_changes_nothing(card):
    before, cfg_before = _snapshot(card["home"]), card["cfg"].read_text()
    r = run_os(card, "apply")                   # fake sudo exits 1
    assert r.returncode == 1 and "passwordless sudo" in r.stderr
    assert _snapshot(card["home"]) == before and card["cfg"].read_text() == cfg_before


def test_os_apply_matches_mts_and_is_idempotent(card):
    v4 = _fake_v4(card)                          # build already present: apply only relinks (no colcon here)
    r = run_os(card, "apply", FC_OS_SUDO="")
    assert r.returncode == 0, r.stdout + r.stderr
    assert "REBOOT needed" in r.stdout
    text = card["cfg"].read_text()
    assert text.startswith("dtparam=spi=on\ndtparam=i2c_arm=on,i2c_arm_baudrate=100000\n#fc-off(")   # rest kept
    assert "dtoverlay=hifiberry-dac\n" in text
    assert all(f in text.splitlines() for f in FAN + ["dtparam=pciex1=off"])
    assert not [ln for ln in text.splitlines() if ln.startswith("dtoverlay=waveshare-4dpic")]
    assert len([ln for ln in text.splitlines() if ln.startswith("#fc-off(") and "i2c touch) dtoverlay=waveshare" in ln]) == 3
    assert list(card["cfg"].parent.glob("config.txt.bak-pre-fc-os-*"))[0].read_text() == FRESH_CFG
    assert card["lib"].is_symlink() and card["lib"].resolve() == v4.resolve()
    assert Path(str(card["lib"]) + ".stock-link").read_text() == "stock driver\n"
    assert card["cal"].read_text() == '{"accel_bias_chip": [3.05, -0.34, -0.09]}\n'   # per chip: never touched
    snap, cfg_after = _snapshot(card["home"]), card["cfg"].read_text()
    r2 = run_os(card, "apply", FC_OS_SUDO="")
    assert r2.returncode == 0 and "nothing to change" in r2.stdout, r2.stdout + r2.stderr
    assert _snapshot(card["home"]) == snap and card["cfg"].read_text() == cfg_after


def test_os_apply_driver_refusals_leave_the_stock_driver(card, fake, tmp_path):
    card["cfg"].write_text(MTS_CFG)              # only imu_v4 missing: needs no sudo
    r = run_os(card, "apply")                    # no v4 build, no driver source on this fake card
    assert r.returncode == 1 and "no driver source" in r.stderr, r.stdout + r.stderr
    assert not card["lib"].is_symlink() and card["lib"].read_text() == "stock driver\n"
    _fake_v4(card)
    stale = Path(str(card["lib"]) + ".stock-link")
    stale.write_text("older driver\n")            # a stale backup next to a real driver file: never overwrite
    r = run_os(card, "apply")
    assert r.returncode == 1 and "resolve by hand" in r.stderr, r.stdout + r.stderr
    assert card["lib"].read_text() == "stock driver\n" and stale.read_text() == "older driver\n"
    stale.unlink()
    _exe(tmp_path / "bin" / "pgrep", "#!/bin/sh\necho 1234\n")      # a robot stack is running
    r = run_os(card, "apply")
    assert r.returncode == 1 and "stack is running" in r.stderr
    assert not card["lib"].is_symlink()


def test_provision_apply_os_end_to_end(card):
    assert run_prov(card, "--check", "--apply-os", "pupper-red", "red", "red", "11").returncode == 2
    r = run_prov(card, "--check", "pupper-red", "red", "red", "11")
    assert "== Pi OS items vs mts" in r.stdout and "missing  hailo" in r.stdout, r.stdout + r.stderr
    if shutil.which("rsync") is None:
        pytest.skip("needs rsync")
    _fake_v4(card)
    env = {**card["osenv"], "FC_OS_SUDO": ""}
    r = subprocess.run(["bash", str(card["tree"] / "robot" / "provision" / "provision.sh"), "--apply-os", "--yes",
                        "pupper-red", "red", "red", "11"], capture_output=True, text=True, env=env, timeout=120)
    out = r.stdout + r.stderr
    assert "apply would change:" in out and "REBOOT needed" in out, out
    assert "dtparam=pciex1=off" in card["cfg"].read_text()
    assert card["lib"].is_symlink()
    final = r.stdout.split("== Pi OS items vs mts")[-1]
    assert set(os_states(final).values()) == {"ok"}, final


def test_os_gui_linked_and_running_is_disabled(card, tmp_path):
    card["cfg"].write_text(MTS_CFG)
    v4 = _fake_v4(card)
    card["lib"].unlink()
    card["lib"].symlink_to(v4)
    calls = tmp_path / "systemctl.log"
    state = tmp_path / "gui_off"
    _exe(tmp_path / "bin" / "systemctl", textwrap.dedent(f"""\
        #!/bin/sh
        echo "$*" >> {calls}
        case "$1" in
          is-enabled) [ -e {state} ] && exit 1; echo linked ;;
          is-active) [ -e {state} ] && {{ echo inactive; exit 3; }}; echo active ;;
          disable) touch {state} ;;
          show) echo /home/pi/pupperv3-monorepo/pupper-rs/pupper-gui.service ;;
        esac
        """))
    r = run_os(card, "check")
    assert r.returncode == 1 and os_states(r.stdout)["gui"] == "missing"
    assert "disable" not in calls.read_text()
    r = run_os(card, "apply", FC_OS_SUDO="")
    assert r.returncode == 0, r.stdout + r.stderr
    assert "disable --now pupper-gui" in calls.read_text()
    assert "REBOOT" not in r.stdout                  # a service change needs no reboot
    assert card["cfg"].read_text() == MTS_CFG and not list(card["cfg"].parent.glob("*.bak-*"))
