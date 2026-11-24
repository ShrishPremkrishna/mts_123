"""stack_overlay.py: the per-start overlay (speed cap, heading hold, gentle lie-down controller).

Pure: fake stock trees shaped like the Pi's installed files, the vendored real u6 launch dir. No ROS.
"""
import json
import os
import shutil
import subprocess
import sys

import pytest

yaml = pytest.importorskip("yaml")
ROBOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROBOT)
import stack_overlay as so  # noqa: E402

hh = so.hh
U6_HW_VENDOR = os.path.join(ROBOT, "vendor", "tej_u6", "software-ballkick-20261120-u6", "foundation", "hardware")
LIM_LO = [-1.22, -0.42, -2.79, -2.51, -3.14, -0.71] * 2
LIM_HI = [2.51, 3.14, 0.71, 1.22, 0.42, 2.79] * 2
STANCE = [0.26, 0.0, -0.52, -0.26, 0.0, 0.52] * 2
DEFAULT_POLICY = json.dumps({"kp": 5.5, "kd": 0.25, "default_joint_pos": STANCE, "joint_lower_limits": LIM_LO,
                             "joint_upper_limits": LIM_HI, "gait_reference": {"period": 0.5}, "jump_slot": {},
                             "heading_hold": {"kp": 1.5, "clip": 1.0, "yaw_threshold": 0.1, "walk_threshold": 0.05},
                             "layers": [[0.5, -0.25]]}, indent=1)
LEGACY_POLICY = json.dumps({"use_imu": True, "kp": 5.0, "kd": 0.25, "default_joint_pos": STANCE,
                            "joint_lower_limits": LIM_LO, "joint_upper_limits": LIM_HI, "layers": [[1.0, 2.0]]})
WALKER_PARAMS = '''    repeat_action: 10
    gain_multiplier: 1.0
    estop_kd: 0.1
    joint_names: [a, b]
    init_kps: [7.5, 7.5]
    init_kds: [0.25, 0.25]
    init_duration: 2.0
    fade_in_duration: 2.0
    max_body_angle: 1.0
'''
STOCK_CONFIG = '''controller_manager:
  ros__parameters:
    update_rate: 520


    neural_controller_default:
      type: neural_controller/NeuralController


neural_controller:
  ros__parameters:
    model_path: "$(find-pkg-share neural_controller)/launch/legacy_walk_policy.json"


neural_controller_default:
  ros__parameters:
''' + WALKER_PARAMS + '''    game_buttons: false
    max_vx_forward: 0.49
    max_vx_backward: 0.3


    model_path: "$(find-pkg-share neural_controller)/launch/default_policy.json"


    chord_partner: "neural_controller_default_alt"


neural_controller_default_alt:
  ros__parameters:
''' + WALKER_PARAMS + '''    model_path: "$(find-pkg-share neural_controller)/launch/default_policy.json"


estop_controller:
  ros__parameters:
    switch_button_indices: [0, 1]
    controller_names:
      [
        "neural_controller_default",
        "neural_controller",
      ]
'''
STOCK_LAUNCH = '''from launch_ros.parameter_descriptions import ParameterFile
def generate_launch_description():
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("neural_controller"), "launch", "config.yaml"]
        ),
        allow_substs=True,
    )
    default_alt_robot_controller_spawner = Node(package="controller_manager")
    nodes = [
        control_node,
        default_alt_robot_controller_spawner,
        joint_state_broadcaster_spawner,
        foxglove_bridge,
        joy_util_node,
        joy_node,
        camera_node,
    ]
    return LaunchDescription(nodes)
'''


@pytest.fixture
def stock(tmp_path, monkeypatch):
    src, inst = tmp_path / "src", tmp_path / "install"
    for d in (src, inst):
        d.mkdir()
        (d / "launch.py").write_text(STOCK_LAUNCH)
        (d / "config.yaml").write_text(STOCK_CONFIG)
        (d / "default_policy.json").write_text(DEFAULT_POLICY)
        (d / "legacy_walk_policy.json").write_text(LEGACY_POLICY)
    monkeypatch.setattr(hh, "STOCK_NC", str(src))
    monkeypatch.setattr(hh, "STOCK_INSTALLED", str(inst))
    root = tmp_path / "fc-v1"
    root.mkdir()
    return root


@pytest.fixture
def u6(tmp_path, monkeypatch):
    hw = tmp_path / "release" / "foundation" / "hardware"
    (hw / "floor_stock").mkdir(parents=True)
    for rel in ("lab6_session.launch.py", "lab6-session.yaml", "floor_stock/baseline-expanded.urdf",
                "floor_stock/shutdown_coordinator.py"):
        shutil.copy2(os.path.join(U6_HW_VENDOR, rel), hw / rel)
    pol = tmp_path / "default_policy.json"
    pol.write_text(DEFAULT_POLICY)
    monkeypatch.setattr(hh, "U6_HW", str(hw))
    monkeypatch.setattr(hh, "U6_POLICY", str(pol))
    monkeypatch.setattr(hh, "U6_POLICY_SHA", hh.sha256(str(pol)))
    root = tmp_path / "fc-v1"
    root.mkdir()
    return root


def out(root, name="r1"):
    return str(root / "runs" / name / "overlay")


def test_stock_all_options(stock):
    r = so.build("stock", out(stock), str(stock), speed_cap="0.8", heading_hold="off", liedown=True)
    assert r["verified"] and r["speed_cap"] == 0.8 == r["speed_cap_effective"] and r["heading_hold"] == "off"
    cfg = yaml.safe_load(open(r["config"]))
    src = yaml.safe_load(STOCK_CONFIG)
    for w in so.STOCK_WALKERS:
        p = cfg[w]["ros__parameters"]
        assert p["max_vx_forward"] == 0.8 and p["speed_cap"] == 0.8 and p["model_path"] == r["policy"]
    assert cfg["neural_controller_default"]["ros__parameters"]["max_vx_backward"] == 0.3     # unchanged
    assert cfg["neural_controller"] == src["neural_controller"]
    # R3 (stock estop_controller) now also deactivates the lie-down controller; buttons keep their names
    ep, sp = cfg["estop_controller"]["ros__parameters"], src["estop_controller"]["ros__parameters"]
    assert ep["controller_names"] == sp["controller_names"] + [so.LIEDOWN] and r["liedown"]["r3_deactivates"]
    assert ep["switch_button_indices"] == sp["switch_button_indices"]
    # lie-down instance: walker params + pose/policy/timing, never other gains
    assert cfg["controller_manager"]["ros__parameters"][so.LIEDOWN] == {"type": so.NEURAL_TYPE}
    lp = cfg[so.LIEDOWN]["ros__parameters"]
    assert lp["default_joint_pos"] == so.LIE_POSE and lp["init_duration"] == so.LIEDOWN_INIT_S
    assert lp["init_kps"] == [7.5, 7.5] and lp["init_kds"] == [0.25, 0.25] and "chord_partner" not in lp
    assert lp["model_path"] == r["liedown"]["policy"]
    pol, legacy = json.load(open(lp["model_path"])), json.loads(LEGACY_POLICY)
    assert pol.pop("default_joint_pos") == so.LIE_POSE and legacy.pop("default_joint_pos") and pol == legacy
    launch = open(r["launch"]).read()
    assert so.LIEDOWN in launch and "        fc_liedown_spawner,\n" in launch and json.dumps(r["config"]) in launch
    assert "FindPackageShare(\"neural_controller\"), \"launch\", \"config.yaml\"" not in launch
    compile(launch, "launch", "exec")


def test_stock_lean_drops_viewer_and_camera_only(stock):
    r = so.build("stock", out(stock), str(stock), liedown=True, lean="on")
    assert r["lean"] == "on" and r["not_launched"] == ["foxglove_bridge", "camera_node"] and r["camera"] is False
    launch = open(r["launch"]).read()
    listed = so._node_list(launch)
    assert {"foxglove_bridge", "camera_node"}.isdisjoint(listed)
    assert {"control_node", "joint_state_broadcaster_spawner", "joy_node", "joy_util_node",
            "default_alt_robot_controller_spawner", "fc_liedown_spawner"} <= listed
    # only the two list lines differ from the liedown-only overlay
    base = so.build("stock", out(stock, "r2"), str(stock), liedown=True)
    a, b = open(base["launch"]).read().splitlines(), launch.splitlines()
    diff = [(x, y) for x, y in zip(a, b) if x.replace(base["config"], r["config"]) != y]
    assert len(a) == len(b) and [y.split("#")[1].split(",")[0].strip() for _, y in diff] == ["foxglove_bridge", "camera_node"]
    assert base["lean"] == "off" and base["not_launched"] == [] and base["camera"] is True
    # the config is untouched by lean
    od = lambda rec: os.path.dirname(rec["config"])
    assert open(r["config"]).read().replace(od(r), "D") == open(base["config"]).read().replace(od(base), "D")


def test_stock_lean_camera_on_keeps_camera(stock):
    r = so.build("stock", out(stock), str(stock), lean="on", camera="on")
    assert r["not_launched"] == ["foxglove_bridge"] and r["camera"] is True
    listed = so._node_list(open(r["launch"]).read())
    assert "camera_node" in listed and "foxglove_bridge" not in listed


def test_lean_refused_on_changed_layout(stock, tmp_path):
    for d in ("src", "install"):
        p = tmp_path / d / "launch.py"
        p.write_text(STOCK_LAUNCH.replace("        camera_node,\n", ""))
    with pytest.raises(so.OverlayError, match="camera_node"):
        so.build("stock", out(stock), str(stock), lean="on")
    so.build("stock", out(stock, "r2"), str(stock), lean="on", camera="on")     # camera kept: no edit needed


def test_lean_refused_on_u6(u6):
    for kw in ({"lean": "on"}, {"camera": "on"}):
        with pytest.raises(so.OverlayError, match="lean"):
            so.build("u6", out(u6), str(u6), **kw)


def test_lean_verify_catches_relisted_node(stock):
    r = so.build("stock", out(stock), str(stock), lean="on")
    rec = so.build_stock(out(stock, "r3"), removed=["foxglove_bridge", "camera_node"])
    text = open(rec["launch"]).read()
    open(rec["launch"], "w").write(text.replace("        # camera_node," + so.LEAN_MARK, "        camera_node,"))
    with pytest.raises(so.OverlayError):
        so.verify_stock(rec)
    assert so.verify_stock({**rec, "launch": r["launch"], "config": r["config"]})


def test_lie_pose_is_mirrored_and_inside_driver_limits():
    lim = json.load(open(os.path.join(ROBOT, "..", "..", "..", "foundation", "configs", "class-driver-joint-limits.json"))) \
        if os.path.exists(os.path.join(ROBOT, "..", "..", "..", "foundation", "configs", "class-driver-joint-limits.json")) else None
    assert so.LIE_POSE[:3] == [-v for v in so.LIE_POSE[3:6]] and so.LIE_POSE[:6] == so.LIE_POSE[6:]
    if lim:
        for name, q in zip(so.JOINT_NAMES, so.LIE_POSE):
            lo, hi = lim["joint_limits"][name]
            assert lo < q < hi, (name, q)


def test_r3_list_untouched_without_liedown_and_refused_when_odd(stock, tmp_path):
    r = so.build("stock", out(stock), str(stock), speed_cap=0.6)
    assert yaml.safe_load(open(r["config"]))["estop_controller"] == yaml.safe_load(STOCK_CONFIG)["estop_controller"]
    assert so.add_estop_name('estop_controller:\n  ros__parameters:\n    controller_names: ["a"]\n', "b").endswith(
        'controller_names: ["a", "b"]\n')
    for bad in ('estop_controller:\n  ros__parameters:\n    controller_names: ["a", "b"]\n',      # already listed
                'estop_controller:\n  ros__parameters:\n    controller_names:\n      - a\n',         # block list
                'other:\n  ros__parameters:\n    x: 1\n'):                                        # no section
        with pytest.raises(so.OverlayError):
            so.add_estop_name(bad, "b")
    # a config with more switch buttons than names before ours would let a button select the lie-down: refused
    for d in ("src", "install"):
        p = tmp_path / d / "config.yaml"
        p.write_text(STOCK_CONFIG.replace("switch_button_indices: [0, 1]", "switch_button_indices: [0, 1, 2]"))
    with pytest.raises(so.OverlayError, match="R3 list"):
        so.build("stock", out(stock, "r9"), str(stock), liedown=True)


def test_stock_speed_cap_only_touches_caps(stock):
    r = so.build("stock", out(stock), str(stock), speed_cap=0.3)
    cfg, src = yaml.safe_load(open(r["config"])), yaml.safe_load(STOCK_CONFIG)
    for w in so.STOCK_WALKERS:
        src[w]["ros__parameters"].update(max_vx_forward=0.3, speed_cap=0.3)
    assert cfg == src and r["liedown"] is None and r["policy"] is None


@pytest.mark.parametrize("cap", ["0.29", "1.01", "abc", "-0.5", "nan"])
def test_speed_cap_range_refused(stock, cap):
    with pytest.raises(so.OverlayError):
        so.build("stock", out(stock), str(stock), speed_cap=cap)
    assert not os.path.exists(out(stock))


def test_liedown_refuses_wider_gains_or_pose_outside_limits(stock, tmp_path):
    for d in ("src", "install"):
        p = json.loads(LEGACY_POLICY)
        p["kp"] = 6.0
        (tmp_path / d / "legacy_walk_policy.json").write_text(json.dumps(p))
    with pytest.raises(so.OverlayError, match="gains"):
        so.build("stock", out(stock, "a"), str(stock), liedown=True)
    for d in ("src", "install"):
        p = json.loads(LEGACY_POLICY)
        p["joint_upper_limits"] = [0.1] * 12
        (tmp_path / d / "legacy_walk_policy.json").write_text(json.dumps(p))
    with pytest.raises(so.OverlayError, match="outside the policy limits"):
        so.build("stock", out(stock, "b"), str(stock), liedown=True)
    for d in ("src", "install"):
        p = json.loads(LEGACY_POLICY)
        p["gait_reference"] = {}
        (tmp_path / d / "legacy_walk_policy.json").write_text(json.dumps(p))
    with pytest.raises(so.OverlayError, match="gait"):
        so.build("stock", out(stock, "c"), str(stock), liedown=True)


def test_liedown_refuses_changed_launch_layout(stock, tmp_path):
    for d in ("src", "install"):
        (tmp_path / d / "launch.py").write_text(STOCK_LAUNCH.replace("    nodes = [\n", "    nodes = [  # edited\n"))
    with pytest.raises(so.OverlayError, match="node list"):
        so.build("stock", out(stock), str(stock), liedown=True)


def test_verify_catches_tampering(stock):
    r = so.build("stock", out(stock), str(stock), speed_cap=0.8, liedown=True)
    raw = {"speed_cap": 0.8, "heading_hold": "on", "hh_policy": None, "launch": r["launch"], "config": r["config"],
           "liedown": r["liedown"]}
    assert so.verify_stock(raw)
    text = open(r["config"]).read()
    open(r["config"], "w").write(text.replace("init_duration: 2.5", "init_duration: 0.5"))
    with pytest.raises(so.OverlayError):
        so.verify_stock(raw)
    open(r["config"], "w").write(text.replace("init_kps: [7.5, 7.5]\n    kp_scale", "init_kps: [9.5, 9.5]\n    kp_scale")
                                 if "kp_scale" in text else text.replace("init_kps: [7.5, 7.5]", "init_kps: [9.5, 9.5]"))
    with pytest.raises(so.OverlayError):
        so.verify_stock(raw)


def test_stock_refuses_src_install_mismatch(stock, tmp_path):
    (tmp_path / "install" / "legacy_walk_policy.json").write_text(LEGACY_POLICY + " ")
    with pytest.raises(so.OverlayError, match="differ"):
        so.build("stock", out(stock), str(stock), liedown=True)
    assert so.build("stock", out(stock, "r2"), str(stock), speed_cap=0.5)["verified"]   # not needed without lie-down


def test_u6_speed_cap_and_hold_off(u6):
    r = so.build("u6", out(u6), str(u6), speed_cap="0.75", heading_hold="off")
    assert r["verified"] and r["speed_cap"] == 0.75 and r["policy"] and r["liedown"] is None
    hw = os.path.dirname(r["launch"])
    assert hh.sha256(os.path.join(hw, "lab6-session.yaml")) == hh.sha256(os.path.join(hh.U6_HW, "lab6-session.yaml"))
    launch = open(r["launch"]).read()
    src = open(os.path.join(hh.U6_HW, "lab6_session.launch.py")).read()
    added = [l for l in launch.splitlines() if l not in src.splitlines()]
    assert any("fc_params[\"max_vx_forward\"] = 0.75" in l for l in added)
    assert any("fc_params[\"speed_cap\"] = 0.75" in l for l in added) and any(r["sha256"] in l for l in added)
    # the inserted code runs against the real baseline yaml and sets exactly the caps
    snippet = so._u6_patch(None, 0.75)
    base = yaml.safe_load(open(os.path.join(hh.U6_HW, "lab6-session.yaml")))
    before = json.loads(json.dumps(base))
    exec(compile("def f(baseline):\n" + snippet, "patch", "exec"), env := {})
    env["f"](base)
    for w in so.U6_WALKERS:
        before[w]["ros__parameters"].update(max_vx_forward=0.75, speed_cap=0.75)
    assert base == before


def test_u6_refuses_liedown_and_tampering(u6):
    with pytest.raises(so.OverlayError, match="lie-down"):
        so.build("u6", out(u6, "a"), str(u6), liedown=True)
    r = so.build("u6", out(u6, "b"), str(u6), speed_cap=0.8)
    raw = {"speed_cap": 0.8, "heading_hold": "on", "hh_policy": None, "launch": r["launch"], "config": r["config"],
           "urdf": r["urdf"]}
    assert so.verify_u6(raw)
    open(r["launch"], "a").write("# edit\n")
    with pytest.raises(so.OverlayError):
        so.verify_u6(raw)


def test_refuses_existing_or_outside_dir(stock, tmp_path):
    so.build("stock", out(stock), str(stock), speed_cap=0.8)
    with pytest.raises(so.OverlayError, match="exists"):
        so.build("stock", out(stock), str(stock), speed_cap=0.8)
    with pytest.raises(hh.OverlayError):
        so.build("stock", str(tmp_path / "elsewhere"), str(stock), speed_cap=0.8)


def test_cli_fails_loudly(tmp_path):
    root = tmp_path / "fc-v1"
    root.mkdir()
    env = {**os.environ, "HOME": str(tmp_path)}
    for args in (["stock", str(root / "a"), "--speed-cap", "0.8"], ["u6", str(root / "b"), "--speed-cap", "2"],
                 ["stock", str(root / "c"), "--liedown", "on"]):
        p = subprocess.run([sys.executable, os.path.join(ROBOT, "stack_overlay.py"), *args, "--root", str(root)],
                           capture_output=True, text=True, env=env)
        assert p.returncode == 1 and p.stdout == "" and p.stderr.startswith("stack_overlay: "), p
