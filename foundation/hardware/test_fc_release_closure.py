"""Static source closure of the selected FC release. Reads source only; imports nothing it inspects.

Every local module reached from a process the Pi session starts (by import, spawn,
launch or spec_from_file_location) must be a selected release file. Otherwise the
staged tree raises ModuleNotFoundError only when that path first runs on the robot.
"""
import ast
from pathlib import Path, PurePosixPath
import sys

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import fc_walk_kick_release as release

# Directories the staged processes place on sys.path, besides each script's own directory.
SEARCH = ("", "gemini_pupper/robot", "gemini_pupper/lab", "foundation/hardware", "follow_me_pupper/robot")
LOCAL_PACKAGES = ("final_project", "foundation", "follow_me_pupper", "gemini_pupper", "pupper_gait_deploy")
# Started by path: fc_session/persistent_session spawn or exec these, the launch file
# runs the guard, and the lifecycle/launch load the last two with spec_from_file_location.
ENTRIES = (
    release.LAUNCH, "foundation/hardware/fc_session.py", "foundation/hardware/persistent_session.py",
    "foundation/hardware/fc_walk_kick_release.py", "gemini_pupper/robot/session_processes.py",
    "final_project/behaviors/combined_fc_runner.py", "final_project/behaviors/body_command_guard.py",
    "final_project/behaviors/live_field_attack.py", "follow_me_pupper/robot/viser_camera.py",
    "foundation/hardware/floor_stock/shutdown_coordinator.py", "pupper_gait_deploy/download_policy.py",
)
# Function-scope imports on paths the release never executes. Each must stay function-scoped.
UNSTAGED_LAZY = {
    ("final_project/behaviors/fc_executor_bridge.py", "run_through_kick_node"):
        "standalone bridge node and main; the runner imports BridgeCore only",
    ("final_project/behaviors/fc_executor_bridge.py", "foundation.hardware.runtime_identity"):
        "standalone bridge main only",
    ("final_project/pupper_fc/integration.py", "final_project.pupper_fc.simulation"):
        "simulated foot replay only",
}
# Files that are read, not imported, by the staged processes.
DATA = ("follow_me_pupper/robot/camera_params.yaml", "follow_me_pupper/robot/coco.txt",
        "final_project/pupper_fc/arena.json", release.TELEMETRY, release.EVENT_TELEMETRY)


def imports(relative):
    """(module, function_scoped, maybe_symbol) with relative imports made absolute."""
    tree = ast.parse((ROOT / relative).read_text(), relative)
    scoped = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            scoped.update(id(child) for child in ast.walk(node) if child is not node)
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [(alias.name, False) for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            base = node.module or ""
            if node.level:
                package = PurePosixPath(relative).parents[node.level - 1]
                base = ".".join([*package.parts, *([base] if base else [])])
            names = [(base, False), *((f"{base}.{alias.name}", True) for alias in node.names)]
        for name, maybe_symbol in names:
            if name:
                yield name, id(node) in scoped, maybe_symbol


def resolve(module, importer):
    """Return the local file Python would load for module, or None for stdlib/third party."""
    parts = module.split(".")
    for base in (PurePosixPath(importer).parent.as_posix(), *SEARCH):
        prefix = [base] if base not in ("", ".") else []
        stem = "/".join([*prefix, *parts])
        for candidate in (stem + ".py", stem + "/__init__.py"):
            if (ROOT / candidate).is_file():
                return candidate
        if base == "" and (ROOT / stem).is_dir():
            return NAMESPACE
    return None


NAMESPACE = "<namespace package>"


def closure():
    seen, missing, lazy, queue = set(), [], {}, list(ENTRIES)
    while queue:
        path = queue.pop()
        if path in seen:
            continue
        seen.add(path)
        if not (ROOT / path).is_file():
            missing.append(path)
            continue
        for module, function_scoped, maybe_symbol in imports(path):
            found = resolve(module, path)
            if (path, module) in UNSTAGED_LAZY:
                lazy[(path, module)] = lazy.get((path, module), True) and function_scoped
                continue
            if found is None:
                if not maybe_symbol and module.split(".")[0] in LOCAL_PACKAGES:
                    missing.append(f"{module} (imported by {path})")
                continue
            if found != NAMESPACE and found not in seen:
                queue.append(found)
    return seen, missing, lazy


def test_every_selected_file_exists():
    absent = [path for path in release.selected_paths(ROOT) if not (ROOT / path).is_file()]
    assert not absent, "selected release files are missing from this source tree: " + ", ".join(absent)


def test_runtime_import_closure_is_staged():
    seen, missing, _ = closure()
    assert not missing, "runtime entry points reach absent local modules: " + ", ".join(sorted(set(missing)))
    unstaged = sorted(path for path in seen if path not in set(release.selected_paths(ROOT)))
    assert not unstaged, "reached but not staged: " + ", ".join(unstaged)


def test_unstaged_lazy_imports_remain_off_the_release_path():
    _, _, lazy = closure()
    for key, reason in UNSTAGED_LAZY.items():
        if key in lazy:
            assert lazy[key], f"{key[1]} became a module-level import of {key[0]}; stage it ({reason})"


def test_read_only_data_files_are_staged():
    assert set(DATA) <= set(release.selected_paths(ROOT))


@pytest.mark.parametrize("retired", ["foundation/hardware/fc_enrollment.py", "foundation/hardware/fc_advertise.py"])
def test_retired_enrollment_paths_are_not_staged(retired):
    assert retired not in release.selected_paths(ROOT)


def test_owned_runtime_selects_only_the_strict_board_profile():
    retired = ("supported_rehome", "supported_standby", "supported_damping", "field_redundant_imu",
               "recover_imu", "recover-supported", "test-imu-delivery", "SupportedDampingLease")
    owned = ("foundation/hardware/fc_session.py", "foundation/hardware/fc_operator.py",
             "foundation/hardware/persistent_session.py", release.LAUNCH,
             "foundation/hardware/fc_walk_kick_release.py", "gemini_pupper/robot/fc_release_profile.py",
             "gemini_pupper/robot/session_processes.py")
    found = [(path, word) for path in owned for word in retired if word in (ROOT / path).read_text()]
    assert not found
