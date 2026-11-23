"""Count confirmed Sit entry playback, never queued requests or physical poses."""
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
import re
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
import tool_registry
from tricks import load_tricks


@asynccontextmanager
async def installed_rig(*, adopt_sit=False):
    rig = harness.Rig(tricks=lambda: {"sit_stay_shake": load_tricks()["sit_stay_shake"]})
    if adopt_sit:
        rig.robot.controllers = {rig.ts.tricks["sit_stay_shake"].controller: "active"}
    try:
        assert await rig.settle()
        yield rig
    finally:
        rig.close()
        await asyncio.sleep(0)


def context(rig):
    return rig.ts.status()["session_context"]


async def call(rig, tool_name, **args):
    result = await tool_registry.call(rig.ts, tool_name, args)
    assert result.ok, result.message
    assert await rig.settle()
    return result


def test_counter_is_server_scoped_and_not_physical_or_pending():
    async def case():
        async with installed_rig() as rig:
            initial = context(rig)
            assert initial == {"schema": "pupper_session_context_v1",
                               "server_instance": rig.ts.command_instance,
                               "scope": "this_server_session", "completed_sit_entries": 0}
            assert re.fullmatch(r"[0-9a-f]{32}", initial["server_instance"])
            accepted = await tool_registry.call(rig.ts, "play_trick", {"name": "sit_stay_shake"})
            assert accepted.ok and context(rig)["completed_sit_entries"] == 0
            assert await rig.settle()
            assert rig.ts.last_outcome["success"]
            status = await rig.ts.mode()
            assert status.details["session_context"]["completed_sit_entries"] == 1
            assert "Confirmed Sit-entry playbacks in this server session: 1" in status.message
            assert status.details["physical_observation"] == {"pose": None, "speed": None, "stopped": None}
            assert context(rig)["server_instance"] == initial["server_instance"]
    asyncio.run(case())


def test_duplicate_sit_paws_wait_and_exit_do_not_count_another_entry():
    async def case():
        async with installed_rig() as rig:
            await call(rig, "play_trick", name="sit_stay_shake")
            for tool, args in [("play_trick", {"name": "sit_stay_shake"}),
                               ("wait", {"duration": 1}),
                               ("interactive_action", {"action": "shake_left"}),
                               ("interactive_action", {"action": "shake_right"}),
                               ("stand_up", {})]:
                await call(rig, tool, **args)
                assert context(rig)["completed_sit_entries"] == 1
            await call(rig, "play_trick", name="sit_stay_shake")
            assert context(rig)["completed_sit_entries"] == 2
    asyncio.run(case())


def test_repeated_confirmation_and_status_reads_cannot_double_count():
    async def case():
        async with installed_rig() as rig:
            await call(rig, "play_trick", name="sit_stay_shake")
            expected = context(rig)
            for _ in range(3):
                assert (await rig.ts.confirm_clip_endpoint(rig.ts.tricks["sit_stay_shake"])).ok
                # New API clients/reconnections read this same Robot instance.
                assert (await rig.ts.mode()).details["session_context"] == expected
            assert expected["completed_sit_entries"] == 1
            await call(rig, "stop")
            assert context(rig) == expected  # Stop changes epoch, not server lifetime.
    asyncio.run(case())


@pytest.mark.parametrize("failure", ["missing_feedback", "fall", "stop"])
def test_unconfirmed_or_interrupted_entry_does_not_count(failure):
    async def case():
        async with installed_rig() as rig:
            if failure == "missing_feedback":
                rig.ts._matches_clip_endpoint = lambda *args: False
            else:
                sleep = rig.ts.sleep
                async def interrupted(seconds):
                    await sleep(seconds)
                    if failure == "fall":
                        rig.ts.fallen = rig.ts.tricks["sit_stay_shake"].controller
                    else:
                        rig.ts.stops += 1
                rig.ts.sleep = interrupted
            accepted = await tool_registry.call(rig.ts, "play_trick", {"name": "sit_stay_shake"})
            assert accepted.ok and await rig.settle()
            assert rig.ts.last_outcome["success"] is False
            assert context(rig)["completed_sit_entries"] == 0
    asyncio.run(case())


def test_cancel_before_start_and_rejected_name_do_not_count():
    async def case():
        async with installed_rig() as rig:
            assert not (await tool_registry.call(rig.ts, "play_trick", {"name": "not-installed"})).ok
            assert (await tool_registry.call(rig.ts, "play_trick", {"name": "sit_stay_shake"})).ok
            assert (await rig.ts.immediate_stop()).ok
            assert await rig.settle()
            assert context(rig)["completed_sit_entries"] == 0
    asyncio.run(case())


def test_api_restart_has_new_identity_and_does_not_count_adopted_sit():
    async def case():
        async with installed_rig() as first:
            await call(first, "play_trick", name="sit_stay_shake")
            previous = context(first)
        async with installed_rig(adopt_sit=True) as restarted:
            assert restarted.ts.state_now is restarted.ts.tricks["sit_stay_shake"]
            assert context(restarted)["server_instance"] != previous["server_instance"]
            assert context(restarted)["completed_sit_entries"] == 0
            await call(restarted, "play_trick", name="sit_stay_shake")
            assert context(restarted)["completed_sit_entries"] == 0
    asyncio.run(case())
