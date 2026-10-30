"""Real installed skill metadata through the course fake ROS/queue; no physics/network.

Run with the repository's tested Python:
    python -m pytest gemini_pupper/lab/test_skill_sequences.py -q
"""
import asyncio
from contextlib import asynccontextmanager
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER
from secret_gate import SecretGate
from tricks import load_tricks


@asynccontextmanager
async def installed_rig():
    rig = harness.Rig(tricks=load_tricks)
    try:
        assert await rig.settle()
        yield rig
    finally:
        rig.close()
        await asyncio.sleep(0)


async def call_and_settle(rig, tool, **args):
    result = await tool_registry.call(rig.ts, tool, args)
    assert await rig.settle()
    return result


@pytest.mark.parametrize("interruption", ["fall", "stop"])
def test_last_walk_interval_does_not_report_success_after_interruption(interruption):
    async def case():
        async with installed_rig() as rig:
            original_sleep = rig.ts.sleep
            injected = False

            async def last_interval(seconds):
                nonlocal injected
                await original_sleep(seconds)
                if not injected and harness.walk_twists(rig):
                    injected = True
                    # The synchronous event arrives after the final command's
                    # sleep, before asynchronous stop cleanup has run.
                    if interruption == "fall":
                        rig.ts.fallen = LOCOMOTION_CONTROLLER
                    else:
                        rig.ts.stops += 1

            rig.ts.sleep = last_interval
            accepted = await tool_registry.call(rig.ts, "move", {
                "direction": "forward", "speed": 0.2, "duration": 0.05})
            assert accepted.ok
            assert await rig.settle()
            assert injected
            assert rig.ts.last_outcome["success"] is False
            assert "moved" not in rig.ts.last_outcome["message"]
            final = rig.robot.twists()[-1][1]
            assert (final.linear.x, final.linear.y, final.angular.z) == (0, 0, 0)
    asyncio.run(case())


def test_installed_sit_stay_both_paws_exit_and_walk_queued_together():
    async def case():
        async with installed_rig() as rig:
            assert "sit" not in rig.ts.tricks
            assert "stand_sit_shake_sit_stand" not in rig.ts.tricks
            sit = rig.ts.tricks["sit_stay_shake"]
            assert sit.interactive and sit.exit_clip == "stand_up"
            sequence = [
                ("play_trick", {"name": sit.name}),
                ("wait", {"duration": 10.0}),
                ("interactive_action", {"action": "shake_left"}),
                ("interactive_action", {"action": "shake_right"}),
                ("interactive_action", {"action": "stand_up"}),
                ("move", {"direction": "forward", "speed": 0.2, "duration": 0.5}),
            ]
            for name, args in sequence:
                result = await tool_registry.call(rig.ts, name, args)
                assert result.ok, result.message
            assert await rig.settle()
            assert not rig.failures(), rig.outcomes()
            assert rig.robot.clips(sit.controller) == ["shake_left", "shake_right", "stand_up"]
            left_at = next(t for t, topic, msg in rig.robot.published
                           if topic == f"/{sit.controller}/clip" and msg.data.endswith(" shake_left"))
            assert left_at - rig.robot.activations(sit.controller)[0] >= sit.play_time + 10.0
            assert rig.ts.state_now == LOCOMOTION
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert harness.walk_twists(rig)
            assert rig.ts.last_outcome["success"] is True
            refused = await tool_registry.call(rig.ts, "interactive_action", {"action": "shake_left"})
            assert not refused.ok and "play it first" in refused.message
    asyncio.run(case())


def test_move_from_seated_idle_uses_exit_clip_before_walking():
    async def case():
        async with installed_rig() as rig:
            sit = rig.ts.tricks["sit_stay_shake"]
            assert (await call_and_settle(rig, "play_trick", name=sit.name)).ok
            assert rig.ts.state_now is sit
            assert (await call_and_settle(rig, "move", direction="forward", speed=0.2, duration=0.5)).ok
            assert not rig.failures(), rig.outcomes()
            assert rig.robot.clips(sit.controller) == ["stand_up"]
            assert rig.ts.state_now == LOCOMOTION
            assert rig.ts.last_outcome["success"] is True
    asyncio.run(case())


FINITE = [name for name, trick in load_tricks().items()
          if trick.kind == "policy" and not trick.interactive and not trick.secret]


@pytest.mark.parametrize("name", FINITE)
def test_installed_finite_skill_holds_declared_endpoint_until_later_request(name):
    async def case():
        async with installed_rig() as rig:
            trick = rig.ts.tricks[name]
            assert (await call_and_settle(rig, "play_trick", name=name)).ok
            assert rig.ts.last_outcome["success"] is True, rig.ts.last_outcome
            if trick.end_pose == "standing":
                assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
                assert rig.ts.state_now == LOCOMOTION
            else:
                assert rig.robot.active() == {trick.controller}
                assert rig.ts.state_now is trick
                assert "holds its final pose" in rig.ts.last_outcome["message"]
                assert (await call_and_settle(rig, "stand_up")).ok
                assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert not rig.failures(), rig.outcomes()
    asyncio.run(case())


def test_installed_generated_secret_unlock_play_return_and_relock(tmp_path):
    password = "offline-test-only"
    secret = tmp_path / "password"
    secret.write_text(password)

    async def case():
        async with installed_rig() as rig:
            rig.ts.secret_gate = SecretGate(secret)
            dance = rig.ts.tricks["secret_dance"]
            assert dance.secret and dance.end_pose == "standing"
            assert dance.name not in json.dumps(tool_registry.declarations(rig.ts))
            denied = await tool_registry.call(rig.ts, "play_trick", {"name": dance.name})
            assert not denied.ok and not rig.robot.activations(dance.controller)
            before = list(rig.robot.switches)
            wrong = await tool_registry.call(rig.ts, "unlock", {"password": "wrong-test-value"})
            assert not wrong.ok and "wrong-test-value" not in wrong.message
            unlocked = await tool_registry.call(rig.ts, "unlock", {"password": password})
            assert unlocked.ok and password not in unlocked.message
            assert rig.robot.switches == before
            assert (await call_and_settle(rig, "play_trick", name=dance.name)).ok
            assert rig.ts.last_outcome["success"] is True
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
            assert await rig.settle()
            assert not rig.ts.secret_gate.unlocked
            assert dance.name not in json.dumps(tool_registry.declarations(rig.ts))
            assert password not in json.dumps(rig.outcomes())
    asyncio.run(case())


@pytest.mark.parametrize("direction,speed", [("forward", 0.491), ("backward", 0.301), ("left", 0.491), ("right", 0.491)])
def test_scoped_locomotion_caps_refuse_before_queue(monkeypatch, direction, speed):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", "")
    async def case():
        async with installed_rig() as rig:
            result = await tool_registry.call(rig.ts, "move", {
                "direction": direction, "speed": speed, "duration": 0.5})
            assert not result.ok and "too fast" in result.message
            assert rig.ts.commands.empty() and not rig.robot.switches
            assert not harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize("direction,axis,expected", [
    ("forward", "x", 0.49), ("backward", "x", -0.30),
    ("left", "y", 0.49), ("right", "y", -0.49),
])
def test_scoped_locomotion_accepts_exact_directional_boundary(monkeypatch, direction, axis, expected):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", "")
    async def case():
        async with installed_rig() as rig:
            result = await call_and_settle(rig, "move", direction=direction,
                                          speed=abs(expected), duration=0.5)
            assert result.ok and rig.ts.last_outcome["success"]
            commands = harness.walk_twists(rig)
            assert commands and all(getattr(m.linear, axis) == expected for m in commands)
    asyncio.run(case())


def test_scoped_defaults_declarations_and_run_refusal(monkeypatch):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", "")
    async def case():
        async with installed_rig() as rig:
            speeds = rig.ts.speeds()
            assert speeds.walk == 0.3 and speeds.turn == 60.0
            assert speeds.limits.vx_forward_max == 0.49
            assert speeds.limits.vx_backward_max == 0.3
            assert speeds.limits.vy_max == 0.49 and speeds.limits.wz_max == 114.0
            declarations = {d["name"]: d for d in tool_registry.declarations(rig.ts)}
            description = declarations["move"]["description"]
            for text in ("0.3 m/s", "Forward maximum 0.49", "backward maximum 0.3",
                         "sideways maximum 0.49", "Running is unavailable"):
                assert text in description
            assert "turn cap 114.0 deg/s" in declarations["arc"]["description"]
            # Declarations consult the instance's real envelope, not the global
            # class limits, including a smaller supported yaw range.
            speeds.limits.wz_max = 30.0
            arc = next(d["description"] for d in tool_registry.declarations(rig.ts) if d["name"] == "arc")
            assert "turn cap 30.0 deg/s" in arc and "v/0.524" in arc
            speeds.limits.wz_max = 114.0
            for direction in ("forward", "backward"):
                refused = await tool_registry.call(rig.ts, "move", {"direction": direction, "run": True})
                assert not refused.ok and "too fast" in refused.message
            assert not rig.robot.switches and not harness.walk_twists(rig)
            result = await call_and_settle(rig, "move", direction="backward", duration=0.5)
            assert result.ok and rig.ts.last_outcome["success"]
            assert all(m.linear.x == -0.3 for m in harness.walk_twists(rig))
    asyncio.run(case())


def test_ordinary_unscoped_defaults_remain_unchanged(monkeypatch):
    monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
    async def case():
        async with installed_rig() as rig:
            speeds = rig.ts.speeds()
            assert speeds.walk == 0.4 and speeds.limits.vx_max == 1.5
            assert speeds.limits.vx_forward_max is None and speeds.limits.vx_backward_max is None
            description = next(d["description"] for d in tool_registry.declarations(rig.ts) if d["name"] == "move")
            assert "0.4 m/s" in description and "Forward maximum 1.5" in description
            result = await call_and_settle(rig, "move", direction="backward", duration=0.5)
            assert result.ok and all(m.linear.x == -0.4 for m in harness.walk_twists(rig))
    asyncio.run(case())
