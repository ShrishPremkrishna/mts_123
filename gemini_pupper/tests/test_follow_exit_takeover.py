"""Follower takeover while a real installed Sit exit is pending; fake ROS only."""
import asyncio
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import tool_registry
from tricks import load_tricks


async def seated_rig():
    rig = harness.Rig(tricks=load_tricks)
    assert await rig.settle()
    result = await tool_registry.call(rig.ts, 'play_trick', {
        'name': 'sit_stay_shake', 'count': 1, 'count_unit': 'routine_playback'})
    assert result.ok and await rig.settle()
    assert rig.ts.last_outcome['success']
    return rig


async def pending_exit(rig):
    entered = asyncio.Event()
    release = asyncio.Event()
    original = rig.ts._matches_clip_endpoint
    sit = rig.ts.tricks['sit_stay_shake']
    def endpoint(controller, endpoint, request, earliest):
        if controller == sit.controller and endpoint['name'] == sit.exit_clip and not release.is_set():
            entered.set()
            return False
        return original(controller, endpoint, request, earliest)
    rig.ts._matches_clip_endpoint = endpoint
    accepted = await tool_registry.call(rig.ts, 'stand_up', {})
    assert accepted.ok
    await entered.wait()  # first exit request actually published; job still pending
    return sit, accepted.details['submission']['job_id'], release


def test_follow_during_published_exit_preserves_one_request_and_prior_jobs():
    async def case():
        rig = await seated_rig()
        try:
            sit, stand_id, release = await pending_exit(rig)
            wait = await tool_registry.call(rig.ts, 'wait', {'duration': .1})
            assert wait.ok and rig.ts.commands.qsize() == 1
            result = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
            assert rig.robot.clips(sit.controller) == [sit.exit_clip], 'takeover replayed native exit'
            assert not result.ok and 'exit' in result.message and 'in progress' in result.message
            assert not rig.robot.follower_on and not rig.ts.following
            assert rig.ts.current_job_id == stand_id and not rig.ts.current.cancelled()
            assert rig.ts.commands.qsize() == 1  # refusal did not clear dependent work
            release.set()
            assert await rig.settle()
            outcomes = {row['job_id']: row for row in rig.ts.recent_outcomes()['outcomes']}
            assert outcomes[stand_id]['terminal_state'] == 'succeeded'
            assert outcomes[wait.details['submission']['job_id']]['terminal_state'] == 'succeeded'
            assert not rig.ts._exit_clip_requests
            # A later explicit request works after the original transition finishes.
            assert (await tool_registry.call(rig.ts, 'follow', {'enabled': True})).ok
            assert rig.robot.follower_on and rig.robot.clips(sit.controller) == [sit.exit_clip]
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(case())


def test_immediate_stop_cancels_pending_exit_and_clears_guard():
    async def case():
        rig = await seated_rig()
        try:
            sit, stand_id, _release = await pending_exit(rig)
            result = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
            assert not result.ok and 'in progress' in result.message
            assert (await tool_registry.call(rig.ts, 'stop', {})).ok
            assert await rig.settle()
            assert not rig.ts._exit_clip_requests  # CancelledError must not leave a permanent block.
            assert not rig.robot.follower_on and not rig.ts.following
            outcomes = {row['job_id']: row for row in rig.ts.recent_outcomes()['outcomes']}
            assert outcomes[stand_id]['terminal_state'] == 'cancelled'
            assert rig.robot.clips(sit.controller) == [sit.exit_clip]
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(case())


def test_isolated_follow_from_sit_still_exits_once_then_activates():
    async def case():
        rig = await seated_rig()
        try:
            sit = rig.ts.tricks['sit_stay_shake']
            result = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
            assert result.ok and rig.robot.follower_on
            assert rig.robot.clips(sit.controller) == [sit.exit_clip]
            assert not rig.ts._exit_clip_requests
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(case())


def test_follow_after_exit_endpoint_waits_for_original_locomotion_handoff():
    async def case():
        rig = await seated_rig()
        try:
            sit = rig.ts.tricks['sit_stay_shake']
            reached_handoff, release = asyncio.Event(), asyncio.Event()
            original = rig.ts._call
            async def blocked_handoff(client, request, timeout=3.0):
                if client is rig.ts.switch_client and sit.controller in request.deactivate_controllers:
                    reached_handoff.set()
                    await release.wait()
                return await original(client, request, timeout)
            rig.ts._call = blocked_handoff
            accepted = await tool_registry.call(rig.ts, 'stand_up', {})
            assert accepted.ok
            await reached_handoff.wait()  # exit endpoint already confirmed, Sit still owns legs
            result = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
            assert not result.ok and 'in progress' in result.message
            assert rig.robot.clips(sit.controller) == [sit.exit_clip]
            assert rig.ts._exit_clip_requests and not rig.robot.follower_on
            release.set()
            assert await rig.settle()
            assert rig.ts.last_outcome['job_id'] == accepted.details['submission']['job_id']
            assert rig.ts.last_outcome['success'] and not rig.ts._exit_clip_requests
            assert (await tool_registry.call(rig.ts, 'follow', {'enabled': True})).ok
            assert rig.robot.clips(sit.controller) == [sit.exit_clip]
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(case())
