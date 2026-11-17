"""Inspect launch actions without importing ROS or starting any process."""
import importlib.util
from pathlib import Path
import sys
import types

import pytest


@pytest.fixture
def launch_module(monkeypatch, tmp_path):
    class Action:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class Node(Action):
        pass

    class ExecuteProcess(Action):
        pass

    class LogInfo(Action):
        pass

    class RegisterEventHandler:
        def __init__(self, handler):
            self.handler = handler

    class LaunchDescription:
        def __init__(self, entities):
            self.entities = entities

    modules = {
        "ament_index_python": {},
        "ament_index_python.packages": {"get_package_prefix": lambda name: "/unused"},
        "launch": {"LaunchDescription": LaunchDescription},
        "launch.actions": {"EmitEvent": Action, "ExecuteProcess": ExecuteProcess,
                           "LogInfo": LogInfo, "RegisterEventHandler": RegisterEventHandler},
        "launch.event_handlers": {"OnProcessExit": Action}, "launch.events": {"Shutdown": Action},
        "launch_ros": {}, "launch_ros.actions": {"Node": Node},
    }
    for name, attrs in modules.items():
        module = types.ModuleType(name)
        vars(module).update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("stock_session_launch_test", Path(__file__).with_name("stock_session.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='offline-test'/>")
    files = {"config": {"path": str(tmp_path / "selected.yaml")}, "urdf": {"path": str(urdf)}}
    monkeypatch.setattr(module, "selected_inputs", lambda: files)
    return module, files, Node, ExecuteProcess, RegisterEventHandler, LogInfo


def test_stock_default_launch_actions_and_mux_routing_remain_unchanged(launch_module):
    module, _, Node, ExecuteProcess, Register, _ = launch_module
    actions = module.generate_launch_description().entities
    nodes = [item for item in actions if isinstance(item, Node)]
    assert len(nodes) == 7
    assert not any(isinstance(item, ExecuteProcess) for item in actions)
    mux = next(item for item in nodes if item.kwargs["package"] == "cmd_vel_mux")
    assert "remappings" not in mux.kwargs
    assert len([item for item in actions if isinstance(item, Register)]) == 7


def test_guarded_launch_remaps_only_mux_and_guard_exit_does_not_shutdown_native(launch_module, tmp_path):
    module, files, Node, ExecuteProcess, Register, LogInfo = launch_module
    files.update({"guard_python": {"invocation_path": "/selected/venv/bin/python", "path": "/usr/bin/python3.12"},
                  "guard_script": {"path": str(tmp_path / "final_project/behaviors/body_command_guard.py")}})
    actions = module.generate_launch_description().entities
    nodes = [item for item in actions if isinstance(item, Node)]
    mux = next(item for item in nodes if item.kwargs["package"] == "cmd_vel_mux")
    assert mux.kwargs["remappings"] == [("/cmd_vel", "/fc/requested_cmd_vel")]
    assert all("remappings" not in item.kwargs for item in nodes
               if item.kwargs["package"] not in {"cmd_vel_mux", "controller_manager"})
    guards = [item for item in actions if isinstance(item, ExecuteProcess)]
    assert len(guards) == 1
    guard = guards[0]
    assert guard.kwargs["cmd"] == ["/selected/venv/bin/python", "-B", "-s", files["guard_script"]["path"]]
    assert guard.kwargs["cwd"] == str(tmp_path)
    assert guard.kwargs["respawn"] is False
    exits = [item.handler.kwargs for item in actions if isinstance(item, Register)]
    guard_exit = next(item for item in exits if item["target_action"] is guard)
    assert len(guard_exit["on_exit"]) == 1 and isinstance(guard_exit["on_exit"][0], LogInfo)
    assert "native manager retained" in guard_exit["on_exit"][0].kwargs["msg"]
    # The original five required processes still own the global-shutdown handlers.
    shutdowns = [item for item in exits if isinstance(item["on_exit"], list)
                 and item["on_exit"] and not isinstance(item["on_exit"][0], LogInfo)]
    assert len(shutdowns) == 5
    assert all(item["target_action"] is not guard for item in shutdowns)


def test_selected_input_failure_precedes_creation_of_any_launch_actions(launch_module, monkeypatch):
    module, _, _, _, _, _ = launch_module
    def refused():
        raise ValueError("selected files changed")
    monkeypatch.setattr(module, "selected_inputs", refused)
    with pytest.raises(ValueError, match="selected files changed"):
        module.generate_launch_description()
