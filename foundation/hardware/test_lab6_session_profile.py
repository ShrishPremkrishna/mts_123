"""Construct the selected persistent profile with ROS launch stubs; never run nodes."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "gemini_pupper/robot"))
import runtime_config
from tricks import load_tricks


class Item:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


@pytest.fixture
def profile(monkeypatch, tmp_path):
    for name, symbols in {
        "launch": ["LaunchDescription"],
        "launch.actions": ["EmitEvent", "RegisterEventHandler"],
        "launch.event_handlers": ["OnProcessExit"],
        "launch.events": ["Shutdown"],
        "launch_ros.actions": ["Node"],
        "launch_ros.parameter_descriptions": ["ParameterFile"],
    }.items():
        module = types.ModuleType(name)
        for symbol in symbols:
            setattr(module, symbol, type(symbol, (Item,), {}))
        monkeypatch.setitem(sys.modules, name, module)
    share = tmp_path / "neural_controller"
    (share / "launch").mkdir(parents=True)
    (share / "launch/default_policy.json").symlink_to(ROOT / "pupper_gait_deploy/policies/default_policy.json")
    package_module = types.ModuleType("ament_index_python.packages")
    package_module.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", package_module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "168")
    monkeypatch.setenv("PUPPER_GEMINI_DIR", str(ROOT / "gemini_pupper"))
    monkeypatch.setenv("PUPPER_GAIT_DEPLOY_DIR", str(ROOT / "pupper_gait_deploy"))
    monkeypatch.setenv("PUPPER_SESSION_MANIFEST", str(tmp_path / "runtime_manifest.json"))
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", "")
    spec = importlib.util.spec_from_file_location("persistent_profile_test", ROOT / "foundation/hardware/lab6_session.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("selection", ["", "sit_stay_shake", "sit_stay_shake,secret_dance"])
def test_selected_profile_preserves_baseline_and_binds_catalog(profile, monkeypatch, selection):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", selection)
    baseline = yaml.safe_load((profile.ROOT / "lab6-session.yaml").read_text())
    description = profile.generate_launch_description()
    manifest = runtime_config.validate_session_catalog(load_tricks())
    assert manifest["skills"] == sorted(filter(None, selection.split(",")))
    generated = yaml.safe_load(Path(manifest["configuration"]).read_text())
    for name in ("neural_controller_default", "gemini_walk"):
        old = deepcopy(baseline[name]); new = deepcopy(generated[name])
        old["ros__parameters"].pop("model_path"); new["ros__parameters"].pop("model_path")
        assert old == new
    assert generated["floor_physical_joy"] == baseline["floor_physical_joy"]
    assert generated["cmd_vel_mux"] == baseline["cmd_vel_mux"]
    stop = generated["estop_controller"]["ros__parameters"]
    original_stop = baseline["estop_controller"]["ros__parameters"]
    assert stop["controller_names"][:len(original_stop["controller_names"])] == original_stop["controller_names"]
    for key in ("switch_button_indices", "estop_index", "estop_release_index"):
        assert stop[key] == original_stop[key]
    entities = description.args[0]
    nodes = [e for e in entities if type(e).__name__ == "Node"]
    assert len(nodes) == 7
    motion = nodes[-1].kwargs["arguments"]
    assert motion[:len(manifest["motion_controllers"])] == manifest["motion_controllers"]
    assert "--inactive" in motion and "--param-file" in motion
    assert not any(n.kwargs["package"] in ("teleop_twist_joy", "camera_ros", "person_follower") for n in nodes)
    guards = [e for e in entities if type(e).__name__ == "RegisterEventHandler"]
    assert {id(e.args[0].kwargs["target_action"]) for e in guards} == {id(n) for n in nodes[:5]}
    assert all(entities.index(g) < entities.index(g.args[0].kwargs["target_action"]) for g in guards)
    assert not any(type(e).__name__ == "TimerAction" for e in entities)
    assert hashlib.sha256((profile.ROOT / "lab6-session.yaml").read_bytes()).hexdigest() == profile.CONFIG_SHA
    if "secret_dance" in selection:
        assert load_tricks()["secret_dance"].secret  # selection never unlocks it


@pytest.mark.parametrize("selection", ["missing_skill", "sit", "sit_stay_shake,sit_stay_shake", "sit_stay_shake,", "../sit", "jump"])
def test_invalid_or_unsupported_selection_refused_before_manifest(profile, monkeypatch, selection):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", selection)
    with pytest.raises(ValueError):
        profile.generate_launch_description()
    assert not runtime_config.session_manifest_path().exists()


@pytest.mark.parametrize("change", ["unset", "different_selection", "catalog", "input_bytes"])
def test_server_refuses_profile_mismatch(profile, monkeypatch, change):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", "sit_stay_shake")
    profile.generate_launch_description()
    if change == "unset":
        monkeypatch.delenv("PUPPER_SESSION_SKILLS")
    elif change == "different_selection":
        monkeypatch.setenv("PUPPER_SESSION_SKILLS", "")
    else:
        path = runtime_config.session_manifest_path()
        manifest = json.loads(path.read_text())
        if change == "catalog":
            manifest["catalog"] = {}
            path.write_text(json.dumps(manifest))
        else:
            Path(manifest["configuration"]).write_text("changed\n")
    with pytest.raises(ValueError):
        runtime_config.validate_session_catalog(load_tricks())


def test_explicit_selection_requires_manifest_but_ordinary_full_mode_is_compatible(profile, monkeypatch):
    with pytest.raises(ValueError, match="missing"):
        runtime_config.validate_session_catalog({})
    monkeypatch.delenv("PUPPER_SESSION_SKILLS")
    assert runtime_config.validate_session_catalog(load_tricks()) is None
    runtime_config.session_manifest_path().write_text('{"schema":3}')
    assert runtime_config.validate_session_catalog(load_tricks()) is None


def test_baseline_selection_does_not_parse_unselected_assets(profile, tmp_path):
    broken = tmp_path / "tricks/broken"
    broken.mkdir(parents=True)
    (broken / "trick.yaml").write_text("not: [valid")
    assert load_tricks(broken.parent) == {}


def test_persistent_launch_requires_explicit_scope_before_hardware(profile, monkeypatch):
    monkeypatch.delenv("PUPPER_SESSION_SKILLS")
    with pytest.raises(ValueError, match="explicitly"):
        profile.generate_launch_description()
    assert not runtime_config.session_manifest_path().exists()


def test_motion_bounds_match_selected_profile_and_fake_baseline(profile):
    profile.generate_launch_description()
    manifest = runtime_config.validate_session_catalog(load_tricks())
    native_bounds = runtime_config.session_motion_bounds(manifest)
    assert native_bounds == runtime_config.session_motion_bounds()
    assert native_bounds == pytest.approx({"forward": 0.49, "backward": 0.3,
                                           "sideways": 0.49, "yaw_deg_s": 114.59155902616465})


def test_motion_bounds_intersect_actual_model_and_configuration(profile, tmp_path):
    profile.generate_launch_description()
    manifest = runtime_config.validate_session_catalog(load_tricks())
    config_path = Path(manifest["configuration"])
    config = yaml.safe_load(config_path.read_text())
    params = config["gemini_walk"]["ros__parameters"]
    model_path = tmp_path / "narrowed-model.json"
    model_path.write_text(json.dumps({
        "command_clip": {"vx": [-0.2, 0.4], "vy": [-0.1, 0.3], "wz": [-1.0, 0.5]},
        "jump_slot": {"walk_speed_cap": 0.49}}))
    params["model_path"] = str(model_path)
    params["speed_cap"] = 0.35
    config_path.write_text(yaml.safe_dump(config))
    # Deliberate standalone derivation test; a real start rejects the modified
    # config against the manifest before calling this helper.
    assert runtime_config.session_motion_bounds(manifest) == pytest.approx({
        "forward": 0.35, "backward": 0.2, "sideways": 0.1, "yaw_deg_s": 28.64788975654116})
    with pytest.raises(ValueError, match="input changed"):
        runtime_config.validate_session_catalog(load_tricks())


def test_motion_bounds_legacy_unscoped_needs_no_profile(monkeypatch):
    monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
    assert runtime_config.session_motion_bounds() is None


SUPPLIED_RECIPE = ROOT / "foundation/hardware/lab6-supplied-skills.json"


def test_reviewed_supplied_roster_identity_and_inactive_stop_coverage(profile, monkeypatch):
    recipe = json.loads(SUPPLIED_RECIPE.read_text())
    for relative, expected in recipe["assets_sha256"].items():
        assert hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() == expected, relative
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", recipe["environment"]["PUPPER_SESSION_SKILLS"])
    # Reuse baseline-preservation and required-process lifecycle assertions.
    test_selected_profile_preserves_baseline_and_binds_catalog(
        profile, monkeypatch, recipe["environment"]["PUPPER_SESSION_SKILLS"])
    manifest = runtime_config.validate_session_catalog(load_tricks())
    assert manifest["motion_controllers"] == recipe["motion_controllers"]
    assert set(load_tricks()) == set(recipe["selection"])
    generated = yaml.safe_load(Path(manifest["configuration"]).read_text())
    baseline = yaml.safe_load((profile.ROOT / "lab6-session.yaml").read_text())
    assert generated["estop_controller"]["ros__parameters"]["controller_names"] == (
        baseline["estop_controller"]["ros__parameters"]["controller_names"]
        + recipe["motion_controllers"][2:])
    assert "trick_secret_dance" not in generated["controller_manager"]["ros__parameters"]
    assert runtime_config.session_motion_bounds(manifest) == runtime_config.session_motion_bounds()
    for name, trick in load_tricks().items():
        expected = recipe["skills"][name]
        params = generated[trick.controller]["ros__parameters"]
        assert trick.kind == "policy" and not trick.walks and not trick.secret
        assert params["init_duration"] == expected["init_s"] == .5
        assert params["fade_in_duration"] == expected["fade_s"] == 1.
        assert params["repeat_action"] == 10 and params["gain_multiplier"] == 1.
        assert params["init_kps"] == [7.5] * 12 and params["init_kds"] == [.25] * 12
        assert params["max_vx_forward"] == params["max_vx_backward"] == 0.
        assert not params["game_buttons"] and "chord_partner" not in params
        assert params["max_body_angle"] == trick.max_body_angle
        policy = json.loads(trick.path.read_text())
        assert policy["kp"] == expected["policy_kp"]
        assert policy["kd"] == expected["policy_kd"]
        assert hashlib.sha256(trick.path.read_bytes()).hexdigest() == expected["policy_sha256"]


@pytest.mark.parametrize("name", ["pee", "push_up", "stand_downward_dog", "stand_sit_stand"])
def test_selected_supplied_trick_returns_before_a_following_walk(monkeypatch, name):
    """Actual selected policies through queue/endpoint checks; fake ROS, not physics."""
    import asyncio
    import test_tools as harness
    import tool_registry
    from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER
    recipe = json.loads(SUPPLIED_RECIPE.read_text())
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", recipe["environment"]["PUPPER_SESSION_SKILLS"])
    async def case():
        rig = harness.Rig(tricks=load_tricks)
        try:
            assert await rig.settle()
            assert set(rig.ts.tricks) == set(recipe["selection"])
            rejected = await tool_registry.call(rig.ts, "play_trick", {"name": "secret_dance", "count": 1, "count_unit": "routine_playback"})
            assert not rejected.ok and rig.ts.commands.empty()
            accepted = await tool_registry.call(rig.ts, "play_trick", {"name": name, "count": 1, "count_unit": "routine_playback"})
            assert accepted.ok
            assert await rig.settle()
            assert rig.ts.last_outcome["success"], rig.ts.last_outcome
            assert rig.ts.state_now == LOCOMOTION
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert rig.robot.activations(rig.ts.tricks[name].controller)
            assert (await tool_registry.call(rig.ts, "move", {
                "direction": "forward", "speed": .2, "duration": .5})).ok
            assert await rig.settle() and not rig.failures(), rig.outcomes()
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert harness.walk_twists(rig)
            final = rig.robot.twists()[-1][1]
            assert (final.linear.x, final.linear.y, final.angular.z) == (0., 0., 0.)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())
