"""Controller ownership changes and unavailable startup inventory; fake ROS only."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER


def test_unavailable_startup_inventory_refuses_admission():
    async def case():
        rig = harness.Rig()
        original = rig.robot.handle
        def unavailable(service, request):
            if service.endswith("list_controllers"):
                return None
            return original(service, request)
        rig.robot.handle = unavailable
        try:
            result = await tool_registry.call(rig.ts, "move", {"direction": "forward"})
            assert not result.ok, "missing startup inventory was accepted as idle"
            assert "startup" in result.message
            assert rig.ts.commands.empty()
            assert not rig.robot.switches and not harness.walk_twists(rig)
            assert not rig.ts.worker.done()
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())


@pytest.mark.parametrize("exit_succeeds", [True, False])
def test_manually_selected_interactive_pose_uses_its_exit_before_locomotion(exit_succeeds):
    async def case():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            assert rig.ts.state_now == LOCOMOTION
            trick = rig.ts.tricks["hold_pose"]
            # A manual controller selection occurred after the server adopted
            # locomotion. Its cached state is deliberately stale.
            rig.robot.controllers[LOCOMOTION_CONTROLLER] = "inactive"
            rig.robot.controllers[trick.controller] = "active"
            if not exit_succeeds:
                rig.ts.clip_pubs[trick.controller].publish = lambda message: None
            result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
            if exit_succeeds:
                assert result.ok, result.message
                assert rig.robot.clips(trick.controller) == ["leave"]
                assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
                receipt_time = rig.ts.clip_receipts[trick.controller][3]
                assert rig.robot.activations(LOCOMOTION_CONTROLLER)[0] >= receipt_time
            else:
                assert not result.ok
                assert "cannot leave" in result.message
                assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())


@pytest.mark.parametrize("at_startup", [True, False])
def test_multiple_active_leg_controllers_do_not_silently_select_a_pose(at_startup):
    async def case():
        rig = harness.Rig()
        try:
            if not at_startup:
                assert await rig.settle()
            rig.robot.controllers["neural_controller_default"] = "active"
            result = (await tool_registry.call(rig.ts, "stand_up", {}) if at_startup
                      else await rig.ts.switch_to(LOCOMOTION_CONTROLLER))
            assert not result.ok
            assert not rig.robot.switches and not harness.walk_twists(rig)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())


@pytest.mark.parametrize("scope", ["", "sit_stay_shake"])
@pytest.mark.parametrize("state", [None, "unconfigured"])
def test_explicit_session_never_auto_spawns_missing_controller(monkeypatch, scope, state):
    async def case():
        monkeypatch.setenv("PUPPER_SESSION_SKILLS", scope)
        rig = harness.Rig()
        try:
            assert await rig.settle()
            if state is None:
                del rig.robot.controllers[LOCOMOTION_CONTROLLER]
            else:
                rig.robot.controllers[LOCOMOTION_CONTROLLER] = state
            # Scope is fixed when the API starts; later env edits cannot weaken it.
            monkeypatch.delenv("PUPPER_SESSION_SKILLS")
            def forbidden(*args):
                raise AssertionError("selected session tried to spawn an unconfigured controller")
            monkeypatch.setattr(harness.robot_api, "spawn", forbidden)
            result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
            assert not result.ok and "selected session" in result.message
            assert not rig.robot.switches
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())


def test_unscoped_full_runtime_preserves_existing_auto_load(monkeypatch):
    async def case():
        monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
        rig = harness.Rig()
        try:
            assert await rig.settle()
            del rig.robot.controllers[LOCOMOTION_CONTROLLER]
            requested = []
            def load(names):
                requested.extend(names)
                for name in names:
                    rig.robot.controllers[name] = "inactive"
            monkeypatch.setattr(harness.robot_api, "spawn", load)
            result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
            assert result.ok, result.message
            assert requested == [LOCOMOTION_CONTROLLER]
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())


@pytest.mark.parametrize("scope", [None, "", "sit_stay_shake"])
def test_selected_manifest_failure_precedes_ros_initialization(monkeypatch, scope):
    if scope is None:
        monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
    else:
        monkeypatch.setenv("PUPPER_SESSION_SKILLS", scope)
    monkeypatch.setattr(harness.robot_api, "load_tricks", lambda: {})
    checked = []
    def reject(catalog):
        checked.append(catalog)
        raise ValueError("selected session manifest mismatch")
    def forbidden(*args, **kwargs):
        raise AssertionError("manifest rejection happened after ROS resource creation")
    monkeypatch.setattr(harness.robot_api.runtime_config, "validate_session_catalog", reject, raising=False)
    monkeypatch.setattr(harness.robot_api.rclpy, "ok", forbidden)
    monkeypatch.setattr(harness.robot_api, "Node", forbidden)
    with pytest.raises(ValueError, match="manifest mismatch"):
        harness.robot_api.Robot()
    assert checked == [{}]
