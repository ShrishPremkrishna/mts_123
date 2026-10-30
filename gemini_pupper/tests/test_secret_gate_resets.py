"""Optional Lab 3 unlock lifetime through actual Robot callbacks, fake ROS only."""
import asyncio
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import ESTOP_LOG, LOCOMOTION_CONTROLLER


def test_fresh_robot_instance_does_not_inherit_unlock(tmp_path, monkeypatch):
    password = tmp_path / "test_password"
    password.write_text("offline-test-only")
    monkeypatch.setenv("PUPPER_SECRET_FILE", str(password))

    async def case():
        for first in (True, False):
            rig = harness.Rig()
            try:
                assert await rig.settle()
                rig.ts.tricks["hold_pose"].secret = True
                assert not rig.ts.secret_gate.unlocked
                assert "hold_pose" not in json.dumps(tool_registry.declarations(rig.ts))
                if first:
                    assert (await tool_registry.call(rig.ts, "unlock", {"password": "offline-test-only"})).ok
                    assert rig.ts.secret_gate.unlocked
                else:
                    # Same password file remains, but unlock is session-local.
                    denied = await tool_registry.call(rig.ts, "play_trick", {"name": "hold_pose"})
                    assert not denied.ok and "unlock" in denied.message
                    assert rig.ts.commands.empty()
            finally:
                rig.close()
                await asyncio.sleep(0)
    asyncio.run(case())


@pytest.mark.parametrize("event", ["fall_log", "gamepad_stop", "shared_stop_latched"])
def test_safety_event_revokes_unlock_and_hides_secret_actions(tmp_path, monkeypatch, event):
    password = tmp_path / "test_password"
    password.write_text("offline-test-only")
    monkeypatch.setenv("PUPPER_SECRET_FILE", str(password))

    async def case():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            trick = rig.ts.tricks["hold_pose"]
            trick.secret = True
            assert (await tool_registry.call(rig.ts, "unlock", {"password": "offline-test-only"})).ok
            assert "hold_pose" in json.dumps(tool_registry.declarations(rig.ts))
            if event == "fall_log":
                rig.ts._on_rosout(harness._Msg(name=LOCOMOTION_CONTROLLER, msg=ESTOP_LOG))
            elif event == "gamepad_stop":
                rig.robot.send("/emergency_stop", harness._Msg())
            else:
                rig.robot.stop_status.update(epoch=2, latched=True, stop_required=True)
                rig.robot.emit_stop_status()
            # ROS receipt schedules asyncio cleanup: yield bounded turns rather
            # than substituting the unlock field or invoking lock directly.
            for _ in range(30):
                await asyncio.sleep(0)
                if not rig.ts.secret_gate.unlocked:
                    break
            assert not rig.ts.secret_gate.unlocked
            declarations = json.dumps(tool_registry.declarations(rig.ts))
            assert "hold_pose" not in declarations and "wave_left" not in declarations
            assert not (await tool_registry.call(rig.ts, "play_trick", {"name": "hold_pose"})).ok
            assert not rig.robot.activations(trick.controller)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())
