"""Finite playback success requires current controller feedback, not elapsed time."""
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


@pytest.mark.parametrize('name,returns_default',[('bow',True),('roll_over',False)])
def test_finite_success_confirms_endpoint_and_keeps_original_end_state(name,returns_default):
    async def case(rig):
        trick=rig.ts.tricks[name]
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert result.ok,result.message
        expected=LOCOMOTION_CONTROLLER if returns_default else trick.controller
        assert rig.robot.active()=={expected}
        assert not rig.ts.controller_fault
    run_case(case)


@pytest.mark.parametrize('fault',['missing','never_holding','wrong_clip','wrong_frame','stale_early_hold','no_token','wrong_token','queued_restart'])
def test_finite_bad_or_missing_completion_never_hands_back_as_success(fault):
    async def case(rig):
        trick=rig.ts.tricks['bow'];original=rig.robot.send
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status':
                suffix=' execution '+rig.robot.active_execution_tokens[trick.controller]+' request none'
                if fault=='missing':return
                if fault=='stale_early_hold':
                    if 'playing' not in msg.data:return
                    original(topic,msg)
                    original(topic,harness._DataMsg('all holding frame 199 queued 0'+suffix))
                    return
                if 'holding' in msg.data:
                    if fault=='never_holding':msg=harness._DataMsg('all playing frame 199 queued 0'+suffix)
                    elif fault=='wrong_clip':msg=harness._DataMsg('other holding frame 199 queued 0'+suffix)
                    elif fault=='wrong_frame':msg=harness._DataMsg('all holding frame 198 queued 0'+suffix)
                    elif fault=='no_token':msg=harness._DataMsg('all holding frame 199 queued 0')
                    elif fault=='wrong_token':msg=harness._DataMsg('all holding frame 199 queued 0 execution '+'0'*32+' request none')
                    elif fault=='queued_restart':msg=harness._DataMsg('all holding frame 199 queued 1'+suffix)
            original(topic,msg)
        rig.robot.send=send
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert not result.ok
        assert rig.ts.controller_fault
        assert LOCOMOTION_CONTROLLER not in rig.robot.active()
    run_case(case)


def test_previous_activation_hold_is_not_completion_of_a_replayed_finite_trick():
    async def case(rig):
        trick=rig.ts.tricks['roll_over']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        original=rig.robot.send
        rig.robot.send=lambda topic,msg:None if topic==f'/{trick.controller}/clip_status' else original(topic,msg)
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert not result.ok and rig.ts.controller_fault
        assert len(rig.robot.activations(trick.controller))==2
    run_case(case)


@pytest.mark.parametrize('interrupt',['fall','stop'])
def test_finite_completion_does_not_override_fall_or_stop(interrupt):
    async def case(rig):
        trick=rig.ts.tricks['bow'];original=rig.robot.send
        def send(topic,msg):
            original(topic,msg)
            if topic==f'/{trick.controller}/clip_status' and 'holding' in msg.data:
                if interrupt=='fall':rig.ts.fallen=trick.controller
                else:rig.ts.stops+=1
        rig.robot.send=send
        # Establish the same per-call generation guard used by the tool wrapper.
        token=harness.robot_api.CALL_STOPS.set(rig.ts.stops)
        try:result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        finally:harness.robot_api.CALL_STOPS.reset(token)
        assert not result.ok
        assert LOCOMOTION_CONTROLLER not in rig.robot.active()
    run_case(case)


@pytest.mark.parametrize('valid_current_playing',[False,True])
def test_delayed_previous_activation_receipts_arriving_after_new_endpoint_are_ignored(valid_current_playing):
    async def case(rig):
        trick=rig.ts.tricks['roll_over']
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        old=rig.ts.policy_execution_requests[trick.controller]
        original=rig.robot.send;delivered=[]
        def send(topic,msg):
            if topic!=f'/{trick.controller}/clip_status':
                return original(topic,msg)
            if 'playing' in msg.data:
                if valid_current_playing:original(topic,msg)
                return
            delivered.append(rig.clock.now)
            # These valid-shaped old receipts arrive freshly, beyond the new
            # activation's final-frame deadline. Arrival time alone is insufficient.
            if not valid_current_playing:
                original(topic,harness._DataMsg('all playing frame 0 queued 0 execution '+old+' request none'))
            original(topic,harness._DataMsg('all holding frame 299 queued 0 execution '+old+' request none'))
        rig.robot.send=send
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert not result.ok and rig.ts.controller_fault
        assert rig.ts.policy_execution_requests[trick.controller]!=old
        earliest=rig.ts.policy_activation_since[trick.controller]+1.5+299/50
        assert delivered and min(delivered)>=earliest
        assert (trick.controller in rig.ts.clip_receipts)==valid_current_playing
        assert not rig.robot.active()
    run_case(case)


@pytest.mark.parametrize('outcome',['refused','unknown','empty','stop'])
def test_execution_token_setup_failure_or_stop_prevents_activation(outcome):
    async def case(rig):
        trick=rig.ts.tricks['bow'];original=rig.ts._call
        async def call(client,request,timeout=3.):
            if any(p.name=='execution_request_id' for p in getattr(request,'parameters',[])):
                if outcome=='unknown':return None
                if outcome=='empty':return harness._Msg(results=[])
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
        assert bool(rig.ts.controller_fault)==(outcome=='unknown')
    run_case(case)


def test_each_replay_gets_a_new_token_and_active_parameter_change_does_not_rebind_receipt():
    async def case(rig):
        trick=rig.ts.tricks['roll_over'];original=rig.robot.send;tokens=[]
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status' and 'playing' in msg.data:
                token=msg.data.split()[7]
                tokens.append(token)
                # Native on_activate snapshots this value; later parameter writes
                # cannot relabel commands from the in-flight activation.
                rig.robot.execution_parameters[trick.controller]='f'*32
            original(topic,msg)
        rig.robot.send=send
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        first=rig.ts.policy_execution_requests[trick.controller]
        assert (await harness.behaviors.play_policy_trick(rig.ts,trick)).ok
        second=rig.ts.policy_execution_requests[trick.controller]
        assert len(first)==len(second)==32 and first!=second
        assert all(c in '0123456789abcdef' for c in first+second)
        assert tokens==[first,second]
        assert rig.robot.active_execution_tokens[trick.controller]==second
        assert rig.robot.execution_parameters[trick.controller]=='f'*32
    run_case(case)


def test_legacy_interactive_status_is_not_adopted_as_current_execution():
    async def case(rig):
        trick=rig.ts.tricks['hold_pose']
        rig.ts._on_clip_status(trick.controller,harness._DataMsg('idle playing frame 149 queued 0'))
        assert trick.controller not in rig.ts.clip_status
        assert trick.controller not in rig.ts.clip_receipts
    run_case(case)


def test_current_terminal_only_feedback_succeeds_if_early_playing_packets_were_lost():
    async def case(rig):
        trick=rig.ts.tricks['bow'];original=rig.robot.send
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status' and 'playing' in msg.data:return
            original(topic,msg)
        rig.robot.send=send
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert result.ok,result.message
        assert rig.robot.active()=={LOCOMOTION_CONTROLLER}
    run_case(case)


def test_original_policy_v1_eight_field_receipt_remains_valid_for_finite_motion():
    async def case(rig):
        trick=rig.ts.tricks['bow'];original=rig.robot.send
        def send(topic,msg):
            if topic==f'/{trick.controller}/clip_status':
                msg=harness._DataMsg(' '.join(msg.data.split()[:8]))
            original(topic,msg)
        rig.robot.send=send
        result=await harness.behaviors.play_policy_trick(rig.ts,trick)
        assert result.ok,result.message
        assert rig.robot.active()=={LOCOMOTION_CONTROLLER}
    run_case(case)
