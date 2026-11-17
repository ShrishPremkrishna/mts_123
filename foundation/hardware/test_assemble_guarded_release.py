"""Assembly refuses drift and creates a separately verifiable walk candidate."""
import json
from pathlib import Path
import sys

import pytest

from assemble_guarded_release import assemble, sha, SOURCE_ROLES, NATIVE_ROLES
from runtime_identity import verify_files, guard_command, guard_mode


@pytest.fixture
def selected(tmp_path):
    base = tmp_path / "stock"
    base.mkdir()
    layout = {
        "driver": "prefix/driver/libcontrol_board_hardware_interface.so",
        "controller": "prefix/controller/libneural_controller.so",
        "gateway": "prefix/gateway/estop_controller", "mux": "prefix/mux/cmd_vel_mux_node",
        "policy": "policies/default_policy.json", "config": "walk.yaml", "urdf": "robot.urdf",
        "shot_node": "final_project/behaviors/run_through_kick_node.py",
        "shot_core": "final_project/behaviors/run_through_kick.py",
        **{role: SOURCE_ROLES[role] for role in ("launch", "runtime_checker", "trial_tool")},
    }
    for role, relative in layout.items():
        p = base / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(role + "\n")
    (base / "walk.yaml").write_text("neural_controller_default:\n  ros__parameters:\n    model_path: " + str(base / layout["policy"]) + "\n")
    (base / "environment.sh").write_text("source " + str(base / "prefix/setup.bash") + "\n")
    for name in ("sprint.yaml", "sprint-manifest.json", "STAGING.json", "FILE_SHA256.json"):
        (base / name).write_text("old claims")
    manifest = {"schema": "pupper_native_release_v1", "trial_profile": "walk",
                "physical_acceptance": True,
                "files": {r: {"path": str(base / p), "sha256": sha(base / p)} for r, p in layout.items()}}
    path = base / "walk-manifest.json"
    path.write_text(json.dumps(manifest))
    sources = tmp_path / "sources"
    for relative in SOURCE_ROLES.values():
        p = sources / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("# reviewed " + relative + "\n")
    index = tmp_path / "sources.json"
    index.write_text(json.dumps({"schema": "pupper_guard_sources_v1",
                                "files": {r: sha(sources / r) for r in SOURCE_ROLES.values()}}))
    return dict(baseline_manifest=path, baseline_sha256=sha(path), source_root=sources,
                source_index=index, source_sha256=sha(index), destination=tmp_path / "guarded",
                guard_python=Path(sys.executable))


def test_new_candidate_verifies_and_preserves_native_baseline(selected):
    original = json.loads(selected["baseline_manifest"].read_text())
    before = {p: sha(p) for p in selected["baseline_manifest"].parent.rglob("*") if p.is_file()}
    receipt = assemble(**selected)
    root = selected["destination"]
    manifest = json.loads((root / "walk-manifest.json").read_text())
    verify_files(manifest)
    assert receipt["native_unchanged"] and not receipt["physical_acceptance"]
    assert receipt["target_preflight_required"] and not receipt["software_start_service_added"]
    assert all(manifest["files"][role]["sha256"] == original["files"][role]["sha256"] for role in NATIVE_ROLES)
    assert all(sha(p) == digest for p, digest in before.items())
    assert all(str(root) in str(item["path"]) for r, item in manifest["files"].items() if r != "guard_python")
    assert not (root / "sprint-manifest.json").exists()
    assert not (root / "sprint.yaml").exists()
    assert str(root) in (root / "environment.sh").read_text()
    payload = json.loads((root / "FILE_SHA256.json").read_text())
    assert all(sha(root / relative) == digest for relative, digest in payload.items())
    with pytest.raises(ValueError, match="Destination exists"):
        assemble(**selected)


@pytest.mark.parametrize("arena", [False, True])
def test_guard_mode_is_explicit_pinned_and_bound_to_launch_arguments(selected, arena):
    receipt = assemble(**selected, arena_ball_contact=arena)
    manifest = json.loads((selected["destination"] / "walk-manifest.json").read_text())
    pins = verify_files(manifest)
    mode = "arena_ball_contact" if arena else "all_obstacles"
    assert receipt["guard_mode"] == guard_mode(pins) == mode
    assert ("--arena-ball-contact" in guard_command(pins)) is arena
    policy = Path(pins["guard_policy"]["path"])
    policy.write_text(json.dumps({"schema": "pupper_guard_policy_v1", "mode": "all_obstacles" if arena else "arena_ball_contact"}))
    with pytest.raises(ValueError, match="mismatch.*guard_policy"):
        verify_files(manifest)


def test_guard_mode_refuses_ambiguous_truthy_selection(selected):
    with pytest.raises(ValueError, match="must be boolean"):
        assemble(**selected, arena_ball_contact="false")
    assert not selected["destination"].exists()


@pytest.mark.parametrize("drift", ["baseline_manifest", "native", "source_index", "source", "symlink", "profile"])
def test_refuses_drift_before_creating_candidate(selected, drift):
    base = selected["baseline_manifest"].parent
    if drift == "baseline_manifest":
        selected["baseline_manifest"].write_text("{}")
    elif drift == "native":
        (base / "prefix/controller/libneural_controller.so").write_text("changed")
    elif drift == "source_index":
        selected["source_index"].write_text("{}")
    elif drift == "source":
        (selected["source_root"] / SOURCE_ROLES["trial_tool"]).write_text("changed")
    elif drift == "symlink":
        (base / "unselected-link").symlink_to(selected["source_root"])
    else:
        data = json.loads(selected["baseline_manifest"].read_text())
        data["trial_profile"] = "sprint"
        selected["baseline_manifest"].write_text(json.dumps(data))
        selected["baseline_sha256"] = sha(selected["baseline_manifest"])
    with pytest.raises(ValueError):
        assemble(**selected)
    assert not selected["destination"].exists()


def test_refuses_destination_inside_baseline(selected):
    selected["destination"] = selected["baseline_manifest"].parent / "nested"
    with pytest.raises(ValueError, match="separate"):
        assemble(**selected)
    assert not selected["destination"].exists()


def test_does_not_repin_concurrent_change_to_preserved_source(selected, monkeypatch):
    import assemble_guarded_release as module
    original = module.shutil.copytree
    def changed_during_copy(src, dst, **kwargs):
        (src / "final_project/behaviors/run_through_kick.py").write_text("unreviewed change")
        # Restore before invoking copytree's recursive calls through shutil.
        monkeypatch.setattr(module.shutil, "copytree", original)
        return original(src, dst, **kwargs)
    monkeypatch.setattr(module.shutil, "copytree", changed_during_copy)
    with pytest.raises(ValueError, match="Preserved baseline changed.*shot_core"):
        assemble(**selected)
    assert selected["destination"].exists()  # retained partial output, never selected
    assert not (selected["destination"] / "STAGING.json").exists()
