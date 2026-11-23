"""Positive and malformed-frame cases for causal interactive playback receipts."""
import asyncio
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot'))
import test_tools as harness
from robot_api import LOCOMOTION_CONTROLLER


def run_case(case):
    async def run():
        rig=harness.Rig()
        try:
            await rig.settle()
            await case(rig)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


def test_action_waits_for_queued_clip_and_auto_next_with_current_request_identity():
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        started=rig.clock.now
        result=await rig.ts.play_clip(trick,'wave_left')
        assert result.ok,result.message
        assert rig.clock.now-started>=trick.clip_length('wave_left')
        name,holding,frame,received,activation,request,queued=rig.ts.clip_receipts[trick.controller]
        assert (name,holding,queued)==('idle',False,0)
        assert 149<=frame<200
        assert request==rig.ts.clip_requests[trick.controller]!='none'
        assert activation==rig.robot.active_execution_tokens[trick.controller]
        published=[m.data for _,topic,m in rig.robot.published if topic==f'/{trick.controller}/clip']
        assert published==[f'pupper_clip_v1 {request} wave_left']
    run_case(case)


@pytest.mark.parametrize('activation',['none','d'*32])
def test_restarted_server_binds_unknown_activation_only_from_its_fresh_explicit_request(activation):
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        rig.robot.controllers[LOCOMOTION_CONTROLLER]='inactive'
        rig.robot.controllers[trick.controller]='active'
        rig.robot.active_execution_tokens[trick.controller]=activation
        rig.ts.state_now=trick
        # An unsolicited status from a previous run must not become our token.
        rig.robot.send(f'/{trick.controller}/clip_status',harness._DataMsg(
            f'leave holding frame 549 queued 0 execution {activation} request none'))
        assert trick.controller not in rig.ts.policy_execution_requests
        assert trick.controller not in rig.ts.clip_receipts
        result=await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert result.ok,result.message
        assert rig.ts.policy_execution_requests[trick.controller]==activation
        assert rig.ts.clip_receipts[trick.controller][5]==rig.ts.clip_requests[trick.controller]!='none'
        assert rig.robot.active()=={LOCOMOTION_CONTROLLER}
    run_case(case)


@pytest.mark.parametrize('fault',['wrong_frame','queued_restart','legacy','activation_only'])
def test_matching_request_does_not_override_bad_terminal_frame_or_queue(fault):
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        original=rig.robot.send
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status' and msg.data.startswith('leave holding'):
                fields=msg.data.split()
                if fault=='wrong_frame':fields[3]='548'
                elif fault=='queued_restart':fields[5]='1'
                elif fault=='activation_only':fields=fields[:8]
                else:fields=fields[:6]
                msg=harness._DataMsg(' '.join(fields))
            original(topic,msg)
        rig.robot.send=send
        result=await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok and rig.ts.controller_fault
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
    run_case(case)


def test_current_request_terminal_receipt_before_minimum_playback_is_not_success():
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        def premature(message):
            request=message.data.split()[1]
            activation=rig.robot.active_execution_tokens[trick.controller]
            rig.robot.send(f'/{trick.controller}/clip_status',harness._DataMsg(
                f'leave holding frame 549 queued 0 execution {activation} request {request}'))
        rig.ts.clip_pubs[trick.controller].publish=premature
        result=await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok and rig.ts.controller_fault
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
    run_case(case)


def test_terminal_receipt_is_sufficient_when_selected_clip_playing_packets_are_lost():
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        original=rig.robot.send
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status' and msg.data.startswith('leave playing'):return
            original(topic,msg)
        rig.robot.send=send
        result=await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert result.ok,result.message
        assert rig.robot.active()=={LOCOMOTION_CONTROLLER}
    run_case(case)


@pytest.mark.parametrize('outcome',['refused','unknown','stop'])
def test_interactive_activation_also_requires_successful_token_setup(outcome):
    async def case(rig):
        trick=rig.ts.tricks['hold_pose'];original=rig.ts._call
        async def call(client,request,timeout=3.):
            if any(p.name=='execution_request_id' for p in getattr(request,'parameters',[])):
                if outcome=='unknown':return None
                if outcome=='refused':return harness._Msg(results=[harness._Msg(successful=False)])
                result=await original(client,request,timeout)
                rig.ts.stops+=1
                return result
            return await original(client,request,timeout)
        rig.ts._call=call
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert not result.ok
        assert rig.robot.activations(trick.controller)==[]
        assert trick.controller not in rig.ts.policy_execution_requests
        assert rig.robot.active()=={LOCOMOTION_CONTROLLER}
    run_case(case)


@pytest.mark.parametrize('phase',['playing','holding'])
def test_stop_during_explicit_clip_receipts_prevents_success_without_tool_context(phase):
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        original=rig.robot.send
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status' and msg.data.startswith('leave '+phase):
                rig.ts.stops+=1
            original(topic,msg)
        rig.robot.send=send
        result=await rig.ts.play_clip(trick,'leave')
        assert not result.ok and 'stop' in result.message
    run_case(case)
