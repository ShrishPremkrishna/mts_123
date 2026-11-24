"""Heading-hold-off profiles (stock_hh0, tej_u6_hh0) and the overlay builder imu/heading_hold.py.

Pure: fake source trees (the u6 launch dir is the vendored real one), bash with stubbed fcctl helpers.
No ROS, no robot, nothing written outside tmp_path. fcctl.sh's fc_stack_overlay/status are covered in
test_fcctl.py (Linux only).
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

import pytest

ROBOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROBOT, "imu"))
import heading_hold as hh  # noqa: E402

PROFILES = os.path.join(ROBOT, "profiles")
VENDOR = os.path.join(ROBOT, "vendor", "tej_u6")
U6_HW_VENDOR = os.path.join(VENDOR, "software-ballkick-20261120-u6", "foundation", "hardware")
KIT = os.path.join(VENDOR, "kickseen-kit-20261120")
PRESETS = os.path.join(ROBOT, "..", "config", "presets")

POLICY = ('{"gait_reference": {"period": 0.5}, "obs": [1, 2, 3],\n'
          ' "heading_hold": {"kp": 1.5, "clip": 1.0, "yaw_threshold": 0.1, "walk_threshold": 0.05},\n'
          ' "weights": [0.25, -1.0]}\n')
STOCK_LAUNCH = '''from launch_ros.parameter_descriptions import ParameterFile
def generate_launch_description():
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("neural_controller"), "launch", "config.yaml"]
        ),
        allow_substs=True,
    )
'''
STOCK_CONFIG = '''neural_controller:
  ros__parameters:
    model_path: "$(find-pkg-share neural_controller)/launch/legacy_walk_policy.json"
neural_controller_default:
  ros__parameters:
    model_path: "$(find-pkg-share neural_controller)/launch/default_policy.json"
neural_controller_default_alt:
  ros__parameters:
    model_path: "$(find-pkg-share neural_controller)/launch/default_policy.json"
'''


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


@pytest.fixture
def u6(tmp_path, monkeypatch):
    hw = tmp_path / "release" / "foundation" / "hardware"
    (hw / "floor_stock").mkdir(parents=True)
    for rel in ("lab6_session.launch.py", "lab6-session.yaml", "floor_stock/baseline-expanded.urdf",
                "floor_stock/shutdown_coordinator.py"):
        shutil.copy2(os.path.join(U6_HW_VENDOR, rel), hw / rel)
    pol = tmp_path / "default_policy.json"
    pol.write_text(POLICY)
    monkeypatch.setattr(hh, "U6_HW", str(hw))
    monkeypatch.setattr(hh, "U6_POLICY", str(pol))
    monkeypatch.setattr(hh, "U6_POLICY_SHA", sha(pol))
    root = tmp_path / "fc-v1"
    root.mkdir(exist_ok=True)
    return root


@pytest.fixture
def stock(tmp_path, monkeypatch):
    src, inst = tmp_path / "src", tmp_path / "install"
    for d in (src, inst):
        d.mkdir()
        (d / "launch.py").write_text(STOCK_LAUNCH)
        (d / "config.yaml").write_text(STOCK_CONFIG)
        (d / "default_policy.json").write_text(POLICY)
    monkeypatch.setattr(hh, "STOCK_NC", str(src))
    monkeypatch.setattr(hh, "STOCK_INSTALLED", str(inst))
    root = tmp_path / "fc-v1"
    root.mkdir(exist_ok=True)
    return root


# ---------------------------------------------------------------- overlay builder
def test_u6_overlay_points_both_walkers_at_pinned_kp0_policy(u6):
    r = hh.build("u6", str(u6 / "runs" / "r1" / "hh0"), str(u6))
    assert r["verified"] and r["heading_hold"]["kp"] == 0 and r["heading_hold"]["clip"] == 1.0
    launch = open(r["launch"]).read()
    orig = open(os.path.join(U6_HW_VENDOR, "lab6_session.launch.py")).read()
    assert launch.count(r["sha256"]) == 1 and repr(r["policy"]) in launch
    assert "models = [fc_policy.resolve() for _ in models]" in launch
    # Only the inserted block differs; it sits right before the controllers line (after the stock-sha check).
    added = [l for l in launch.splitlines() if l not in orig.splitlines()]
    assert len(added) == 6 and launch.index("fc_policy = Path") < launch.index(hh.U6_LAUNCH_ANCHOR)
    assert launch.index("STOCK_POLICY_SHA:") < launch.index("fc_policy = Path")
    hw = os.path.dirname(r["launch"])
    for rel in ("lab6-session.yaml", "floor_stock/baseline-expanded.urdf"):
        assert sha(os.path.join(hw, rel)) == sha(os.path.join(U6_HW_VENDOR, rel))
    assert r["config"] == os.path.join(hw, "lab6-session.yaml")
    assert r["urdf"] == os.path.join(hw, "floor_stock", "baseline-expanded.urdf")
    # Policy: byte-identical except the kp value.
    assert open(r["policy"]).read().replace('"kp": 0.0', '"kp": 1.5') == POLICY


def test_stock_overlay_replaces_only_default_policy_paths(stock):
    r = hh.build("stock", str(stock / "runs" / "r1" / "hh0"), str(stock))
    assert r["verified"] and r["model_paths_replaced"] == 2
    cfg = open(r["config"]).read()
    assert cfg.count(json.dumps(r["policy"])) == 2 and "legacy_walk_policy.json" in cfg
    assert cfg.replace(json.dumps(r["policy"]), hh.STOCK_NEEDLE) == STOCK_CONFIG
    launch = open(r["launch"]).read()
    assert json.dumps(r["config"]) in launch and "FindPackageShare" not in launch


def test_stock_refuses_when_src_and_installed_differ(stock):
    with open(os.path.join(hh.STOCK_INSTALLED, "config.yaml"), "a") as f:
        f.write("# local edit\n")
    with pytest.raises(hh.OverlayError, match="src and installed"):
        hh.build("stock", str(stock / "hh0"), str(stock))


@pytest.mark.parametrize("kind", ["u6", "stock"])
def test_refuses_to_write_outside_root(kind, u6, stock, tmp_path):
    root = u6 if kind == "u6" else stock
    with pytest.raises(hh.OverlayError, match="not inside"):
        hh.build(kind, str(tmp_path / "elsewhere"), str(root))
    os.symlink(tmp_path, root / "escape")                       # symlink out of the root is resolved
    with pytest.raises(hh.OverlayError, match="not inside"):
        hh.build(kind, str(root / "escape" / "hh0"), str(root))
    with pytest.raises(hh.OverlayError, match="not inside"):
        hh.build(kind, str(root / "hh0"), str(root), policy=str(tmp_path / "p.json"))
    assert not (tmp_path / "elsewhere").exists() and not (tmp_path / "hh0").exists()


def test_u6_refuses_unpinned_policy(u6):
    with open(hh.U6_POLICY, "a") as f:
        f.write(" ")
    with pytest.raises(hh.OverlayError, match="not the demonstrated one"):
        hh.build("u6", str(u6 / "hh0"), str(u6))


def test_u6_refuses_changed_launch_layout(u6):
    p = os.path.join(hh.U6_HW, "lab6_session.launch.py")
    open(p, "w").write(open(p).read().replace(hh.U6_LAUNCH_ANCHOR, "    controllers = {}\n"))
    with pytest.raises(hh.OverlayError, match="layout changed"):
        hh.build("u6", str(u6 / "hh0"), str(u6))


def test_verify_catches_tampering(u6, stock):
    r = hh.build("u6", str(u6 / "hh0"), str(u6))
    with open(os.path.join(os.path.dirname(r["launch"]), "lab6-session.yaml"), "a") as f:
        f.write("# x\n")
    with pytest.raises(hh.OverlayError, match="differs from the release"):
        hh.verify_u6(r)
    s = hh.build("stock", str(stock / "hh0"), str(stock))
    open(s["config"], "w").write(STOCK_CONFIG)
    with pytest.raises(hh.OverlayError, match="stock overlay config"):
        hh.verify_stock(s)
    open(s["policy"], "w").write(POLICY)
    with pytest.raises(hh.OverlayError, match="changed after writing"):
        hh.verify_stock(s)


def test_policy_edit_changes_only_kp(tmp_path):
    src = tmp_path / "p.json"
    src.write_text('{"gait_reference": {}, "a": 1}')            # no block: one is added with kp 0
    r = hh.policy_hh0(str(src), str(tmp_path / "o.json"))
    assert r["heading_hold"]["kp"] == 0 and json.load(open(r["policy"]))["a"] == 1
    src.write_text('{"a": 1}')                                    # no gait reference: hold already off
    with pytest.raises(hh.OverlayError, match="already off"):
        hh.policy_hh0(str(src), str(tmp_path / "o2.json"))


def test_cli_fails_loudly(tmp_path):
    root = tmp_path / "fc-v1"
    root.mkdir()
    env = {**os.environ, "HOME": str(tmp_path)}                 # no Tej/monorepo trees under this HOME
    for kind in ("u6", "stock"):
        out = subprocess.run([sys.executable, os.path.join(ROBOT, "imu", "heading_hold.py"), kind,
                              str(root / kind), "--root", str(root)], capture_output=True, text=True, env=env)
        assert out.returncode == 1 and out.stdout == "" and out.stderr.startswith("heading_hold: "), out


# ---------------------------------------------------------------- profiles (bash, stubbed fcctl helpers)
STUBS = r'''
set -uo pipefail
fc_log() { echo "LOG $*" >&2; }
fc_alive() { return 1; }
fc_spawn() { echo "SPAWN $*"; }
fc_topology_wait() { return 0; }
STACK_SPEED_CAP=${STACK_SPEED_CAP:-}
fc_stack_options_set() { [ -n "$STACK_SPEED_CAP" ] || [ "${PROFILE_HEADING_HOLD:-on}" = off ]; }
'''


def sh(script, tmp_path, extra_path=True):
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    if not shutil.which("sha256sum"):                            # macOS: shim with shasum
        shim = bindir / "sha256sum"
        shim.write_text('#!/bin/sh\nexec shasum -a 256 "$@"\n')
        shim.chmod(0o755)
    env = {k: v for k, v in os.environ.items() if k not in ("FC_HEADING_HOLD", "PROFILE_HEADING_HOLD")}
    env["PATH"] = str(bindir) + os.pathsep + env["PATH"]
    return subprocess.run(["bash", "-c", STUBS + script], capture_output=True, text=True, env=env,
                          cwd=str(tmp_path), timeout=60)


@pytest.mark.parametrize("base", ["stock", "tej_u6"])
def test_variant_is_base_with_hold_off(base, tmp_path):
    out = sh(f'''
for p in {base} {base}_hh0; do
  unset FC_HEADING_HOLD PROFILE_HEADING_HOLD
  source "{PROFILES}/$p.sh"
  echo "$p|$PROFILE_HEADING_HOLD|$PROFILE_CMD_TOPIC|$PROFILE_WALKER|$(declare -f profile_start profile_stop profile_alive profile_env _tej_derive | cksum)"
done''', tmp_path)
    assert out.returncode == 0, out.stderr
    (b, hb, tb, wb, fb), (v, hv, tv, wv, fv) = [l.split("|") for l in out.stdout.split("\n") if l]
    assert (b, hb) == (base, "on") and (v, hv) == (base + "_hh0", "off")
    assert (tb, wb, fb) == (tv, wv, fv)                          # same logic: no copy-paste fork


def test_bad_heading_hold_value_is_marked_invalid(tmp_path):
    out = sh(f'FC_HEADING_HOLD=yes; source "{PROFILES}/stock.sh"; echo "$PROFILE_HEADING_HOLD"', tmp_path)
    assert out.stdout.strip() == "invalid:yes"                   # fcctl load_profile dies on it


@pytest.mark.skipif(not os.path.isdir(PRESETS), reason="fc/config not present (robot dir mounted alone)")
def test_presets_name_existing_profiles():
    known = {f[:-3] for f in os.listdir(PROFILES) if f.endswith(".sh")}
    assert {"stock_hh0", "tej_u6_hh0"} <= known
    for name in os.listdir(PRESETS):
        prof = json.load(open(os.path.join(PRESETS, name))).get("_profile")
        if prof is not None:
            assert prof in known and re.fullmatch(r"[a-z0-9_]+", prof), (name, prof)


def test_u6_derive_respects_pins_and_changes_only_launch_inputs(tmp_path):
    hw = "/home/pi/fc-v1/runs/20261123T120000Z-tej_u6_hh0/overlay/foundation/hardware"
    out = sh(f'''
source "{PROFILES}/tej_u6_hh0.sh"
_tej_derive "{KIT}/kickseen6-prepare.sh" "$TEJ_PREPARE_SHA" prep.sh /x/session "{hw}" || exit 3
_tej_derive "{KIT}/kickseen6-prepare.sh" "$TEJ_PREPARE_SHA" plain.sh /x/session || exit 4
_tej_derive "{KIT}/kickseen6-session.sh" "$TEJ_SESSION_SHA" bad.sh /x/session "{hw}" && exit 5
cp "{KIT}/kickseen6-prepare.sh" changed.sh; echo "# edit" >> changed.sh
_tej_derive changed.sh "$TEJ_PREPARE_SHA" never.sh /x/session "{hw}" && exit 6
exit 0''', tmp_path)
    assert out.returncode == 0, (out.returncode, out.stderr)
    assert "kit script changed" in out.stderr and not (tmp_path / "never.sh").exists()
    orig = open(os.path.join(KIT, "kickseen6-prepare.sh")).read().splitlines()
    prep = (tmp_path / "prep.sh").read_text().splitlines()
    changed = [(a, b) for a, b in zip(orig, prep) if a != b]
    assert len(prep) == len(orig) and len(changed) == 4
    assert changed[0][1] == 'RUN_DIR="/x/session"'
    assert [b.split()[0] for _, b in changed[1:]] == ["--launch", "--config", "--urdf"]
    for a, b in changed[1:]:
        assert b == a.replace('"$APP_RELEASE/foundation/hardware/', f'"{hw}/')
    # Tej's persistent_session.py, native/imu pins and manifest are untouched.
    text = "\n".join(prep)
    assert text.count('"$APP_RELEASE/foundation/hardware/persistent_session.py"') == 2
    assert "--native-sha256 37938659" in text and "--imu-sha256 1dd92b65" in text
    plain = (tmp_path / "plain.sh").read_text().splitlines()
    assert [b for a, b in zip(orig, plain) if a != b] == ['RUN_DIR="/x/session"']


@pytest.mark.parametrize("prof", ["stock_hh0", "tej_u6_hh0"])
def test_overlay_failure_refuses_before_anything_starts(prof, tmp_path):
    out = sh(f'''
source "{PROFILES}/{prof}.sh"
STOCK_CONFIG=$PWD/teleop.yaml; touch "$STOCK_CONFIG"
profile_env() {{ :; }}
fc_stack_overlay() {{ echo "OVERLAY $*"; return 1; }}
bash() {{ echo "RAN $*"; return 0; }}
mkdir run; profile_start "$PWD/run"; echo "rc=$?"''', tmp_path)
    assert "OVERLAY" in out.stdout and "rc=1" in out.stdout, out
    assert "SPAWN" not in out.stdout and "RAN" not in out.stdout and not (tmp_path / "run" / "prepare.sh").exists()


def test_stock_hh0_launches_overlay(tmp_path):
    out = sh(f'''
source "{PROFILES}/stock_hh0.sh"
STOCK_CONFIG=$PWD/teleop.yaml; touch "$STOCK_CONFIG"
profile_env() {{ :; }}
fc_stack_overlay() {{ echo "OVERLAY $1 $3"; mkdir -p "$2/overlay"; OVERLAY_LAUNCH=$2/overlay/launch_fc.py; }}
mkdir run; profile_start "$PWD/run"; echo "rc=$?"''', tmp_path)
    spawns = [l for l in out.stdout.splitlines() if l.startswith("SPAWN")]
    assert "OVERLAY stock on" in out.stdout and "rc=1" in out.stdout          # fc_alive stub: "exited"
    assert spawns[0] == f"SPAWN launch {tmp_path}/run/robot.log ros2 launch {tmp_path}/run/overlay/launch_fc.py teleop:=False"


def test_stock_default_loads_liedown_overlay(tmp_path):
    out = sh(f'''
source "{PROFILES}/stock.sh"
STOCK_CONFIG=$PWD/teleop.yaml; touch "$STOCK_CONFIG"
profile_env() {{ :; }}
fc_stack_overlay() {{ echo "OVERLAY $1 $3 hh=$PROFILE_HEADING_HOLD"; OVERLAY_LAUNCH=$2/overlay/launch_fc.py; }}
mkdir run; profile_start "$PWD/run"''', tmp_path)
    assert "OVERLAY stock on hh=on" in out.stdout
    assert "neural_controller_liedown" in sh(f'source "{PROFILES}/stock.sh"; echo $PROFILE_LEG_CONTROLLERS $PROFILE_LIEDOWN',
                                             tmp_path).stdout


def test_stock_unchanged_launch(tmp_path):
    out = sh(f'''
source "{PROFILES}/stock.sh"
FC_LIEDOWN=0
STOCK_CONFIG=$PWD/teleop.yaml; touch "$STOCK_CONFIG"
profile_env() {{ :; }}
fc_stack_overlay() {{ echo "OVERLAY"; }}
mkdir run; profile_start "$PWD/run"''', tmp_path)
    spawns = [l for l in out.stdout.splitlines() if l.startswith("SPAWN")]
    assert "OVERLAY" not in out.stdout
    assert spawns[0] == f"SPAWN launch {tmp_path}/run/robot.log ros2 launch neural_controller launch.py teleop:=False"


def test_tej_u6_hh0_prepares_with_overlay(tmp_path):
    out = sh(f'''
source "{PROFILES}/tej_u6_hh0.sh"
TEJ_KIT="{KIT}"
profile_env() {{ :; }}
fc_stack_overlay() {{ echo "OVERLAY $1"; mkdir -p "$2/overlay/foundation/hardware"; OVERLAY_HW=$2/overlay/foundation/hardware; }}
bash() {{ echo "RAN $*"; return 1; }}                  # stop at prepare: never runs Tej's scripts
mkdir run; profile_start "$PWD/run"; echo "rc=$?"''', tmp_path)
    assert "OVERLAY u6" in out.stdout and "rc=1" in out.stdout and "prepare failed" in out.stderr, out
    assert (tmp_path / "run" / "prepare.log").read_text() == f"RAN {tmp_path}/run/prepare.sh\n"
    prep = (tmp_path / "run" / "prepare.sh").read_text()
    assert prep.count(f'"{tmp_path}/run/overlay/foundation/hardware/') == 3
    assert f'RUN_DIR="{tmp_path}/run/session"' in prep
