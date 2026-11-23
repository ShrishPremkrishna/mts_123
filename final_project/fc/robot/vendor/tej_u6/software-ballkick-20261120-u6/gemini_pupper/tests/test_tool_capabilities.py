"""Declared tools match configured capabilities without relying on model refusal."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry


@pytest.mark.parametrize("available", [False, True])
def test_follow_declaration_matches_capability_and_disabled_call_is_refused(available):
    async def check():
        rig = harness.Rig()
        try:
            await rig.settle()
            rig.ts.follow_available = available
            declarations = {d["name"] for d in tool_registry.declarations(rig.ts)}
            assert ("follow" in declarations) is available
            assert {"stop", "robot_status", "move"} <= declarations
            # Removing a declaration must not be the runtime protection: a stale
            # client can still submit the registered function directly.
            assert "follow" in tool_registry.TOOLS
            if not available:
                follow_calls = list(rig.robot.follow_calls)
                switches = list(rig.robot.switches)
                result = await tool_registry.call(rig.ts, "follow", {"enabled": True})
                assert not result.ok and "disabled" in result.message
                assert rig.robot.follow_calls == follow_calls
                assert rig.robot.switches == switches
        finally:
            rig.close()
            for _ in range(5):
                await asyncio.sleep(0)
    asyncio.run(check())
