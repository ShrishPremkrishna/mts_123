"""Turn angle and rate are different API quantities; fake ROS only."""
import asyncio, math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import LLM_CMD_RATE
from gemini_chat import prompt_text


def test_small_positive_turn_angle_is_valid_after_a_speed_refusal():
    async def case():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            refused = await tool_registry.call(rig.ts, "move", {"direction": "forward", "speed": .15, "duration": 1})
            assert not refused.ok
            assert refused.details["refusal"]["code"] == "minimum_speed"
            assert rig.ts.commands.empty()
            turn_speed = rig.ts.speeds().turn
            accepted = await tool_registry.call(rig.ts, "turn", {"direction": "left", "angle": 15})
            assert accepted.ok and accepted.details["submission"]["state"] == "queued"
            assert await rig.settle()
            commands = [c for c in harness.walk_twists(rig) if c.angular.z != 0]
            assert commands and all(abs(c.angular.z - math.radians(turn_speed)) < 1e-6 for c in commands)
            assert all(c.linear.x == 0 and c.linear.y == 0 for c in commands)
            assert abs(sum(math.degrees(c.angular.z) for c in commands) / LLM_CMD_RATE - 15) < .01
            declarations = {t["name"]: t for t in tool_registry.declarations(rig.ts)}
            wording = declarations["turn"]["description"] + declarations["turn"]["parameters"]["properties"]["angle"]["description"]
            assert "deg/s" in wording and "not a minimum angle" in wording
            assert "15" in wording and "physical" in wording
            prompt = prompt_text()
            assert "deg/s" in prompt and "not a minimum turn angle" in prompt
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(case())
