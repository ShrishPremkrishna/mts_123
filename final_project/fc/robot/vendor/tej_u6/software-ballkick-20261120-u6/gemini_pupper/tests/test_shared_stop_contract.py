"""Shared authority gates actual Robot code through fake ROS; no robot/network."""
import asyncio
import json
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import robot_api
from robot_api import LOCOMOTION_CONTROLLER, LOCOMOTION, MOTORS_IDLE, Result, MotionBlockedError
from stop_status import StopStatus


def state(**changes):
    data=dict(protocol='pupper_stop_v1',instance_id='1'*32,sequence=1,epoch=1,
              latched=False,fault=False,stop_required=False,in_flight=False,
              unknown_remote_submission=False,shutting_down=False)
    data.update(changes)
    return data


def accept(tracker, now=10., **changes):
    return tracker.accept(json.dumps(state(**changes)),now)


def run_case(case):
    async def run():
        rig=harness.Rig()
        try:
            await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5):await asyncio.sleep(0)
    asyncio.run(run())


def latch(rig, **flags):
    rig.robot.stop_status.update(epoch=rig.robot.stop_status['epoch']+1,latched=True,stop_required=True,**flags)
    rig.robot.emit_stop_status()


async def pump():
    for _ in range(15):await asyncio.sleep(0)


@pytest.mark.parametrize('patch',[
    {'latched':1},{'sequence':True},{'epoch':-1},{'sequence':2**64},
    {'instance_id':'BAD'},{'protocol':'old'},{'extra':False},
])
def test_malformed_status_never_authorizes_motion(patch):
    tracker=StopStatus();assert not accept(tracker,**patch);assert tracker.reason(10.)


def test_duplicate_keys_and_replayed_sequence_do_not_refresh_authority():
    tracker=StopStatus();assert accept(tracker)
    assert not tracker.accept(json.dumps(state())[:-1]+', "latched": false}',10.1)
    assert tracker.reason(10.1)
    assert accept(tracker,10.2,sequence=2)
    assert not accept(tracker,10.6,sequence=2)
    assert tracker.reason(10.71)  # replay cannot refresh the last 10.2s receipt


def test_normal_inflight_is_allowed_but_missing_and_stale_status_are_not():
    tracker=StopStatus();assert tracker.reason(10.)
    assert accept(tracker,in_flight=True);assert tracker.reason(10.5) is None
    assert tracker.reason(10.50001)


def test_empty_barrier_requires_a_release_not_a_delayed_same_epoch_clear():
    tracker=StopStatus();accept(tracker);tracker.inhibit()
    assert accept(tracker,10.1,sequence=2)
    assert tracker.reason(10.1)
    assert accept(tracker,10.2,sequence=3,epoch=2,latched=True,stop_required=True)
    tracker.inhibit();tracker.inhibit()
    assert accept(tracker,10.3,sequence=4,epoch=2)
    assert tracker.reason(10.3) is None


def test_new_epoch_release_can_recover_when_intermediate_blocked_sample_was_lost():
    tracker=StopStatus();accept(tracker);tracker.inhibit()
    assert accept(tracker,10.1,sequence=8,epoch=2)
    assert tracker.reason(10.1) is None


@pytest.mark.parametrize('fault',[{'instance_id':'2'*32},{'epoch':0},{'fault':True},
                                  {'unknown_remote_submission':True},{'shutting_down':True}])
def test_restart_regression_and_faults_never_clear_without_server_restart(fault):
    tracker=StopStatus();accept(tracker)
    accept(tracker,10.1,sequence=2,**fault)
    assert tracker.reason(10.1)
    accept(tracker,10.2,sequence=3)
    assert tracker.reason(10.2)


@pytest.mark.parametrize('flags',[{}, {'fault':True},{'unknown_remote_submission':True}])
def test_fresh_tools_and_existing_active_controller_cannot_bypass_latch(flags):
    async def case(rig):
        before=len(rig.robot.switches)
        latch(rig,**flags)
        assert rig.ts.motion_block_reason()  # synchronous, before asyncio cleanup
        result=await rig.call('stand_up')
        assert not result.ok
        assert not (await rig.ts.switch_to(LOCOMOTION_CONTROLLER)).ok
        assert len(rig.robot.switches)==before
        with pytest.raises(MotionBlockedError):rig.ts.send_speed(.2,0,0)
        assert all(m.linear.x==m.linear.y==m.angular.z==0 for _,m in rig.robot.twists())
        rig.ts.send_speed(0,0,0)  # stop command always permitted
        assert (await rig.ts.switch_to(None)).ok  # explicit deactivation still permitted
    run_case(case)


def test_missing_or_stale_status_blocks_without_using_a_two_second_cooldown():
    async def case(rig):
        rig.robot.stop_status_enabled=False
        rig.clock.now+=.501
        assert not (await rig.call('stand_up')).ok
        with pytest.raises(MotionBlockedError):rig.ts.send_speed(.1,0,0)
        rig.ts._stop_status=StopStatus()
        assert not (await rig.ts.switch_to(LOCOMOTION_CONTROLLER)).ok
    run_case(case)


def test_stop_between_queue_acceptance_and_execution_never_runs_job():
    async def case(rig):
        ran=[]
        async def job():ran.append(True);return Result(True,'unexpected')
        assert (await rig.ts.queue('candidate',job,1.,then=LOCOMOTION)).ok
        latch(rig)
        await rig.settle()
        assert not ran
    run_case(case)


def test_latched_state_refuses_reference_clip_and_follower_commands():
    async def case(rig):
        latch(rig)
        before=len(rig.robot.published)
        assert not (await rig.ts.play_reference_trick(rig.ts.tricks['hop'])).ok
        assert not (await rig.ts.play_clip(rig.ts.tricks['hold_pose'],'wave_left')).ok
        assert not (await rig.ts.set_following(True)).ok
        assert rig.robot.follow_calls.count('on')==0
        assert not any(topic.endswith(('/reference_trick','/clip')) for _,topic,_ in rig.robot.published[before:])
        assert (await rig.ts.set_following(False)).ok
    run_case(case)


def test_repeated_empty_and_status_messages_coalesce_cleanup_without_switch_rpcs(monkeypatch):
    async def case(rig):
        gate=asyncio.Event();entered=[]
        async def follower_off():entered.append(True);await gate.wait();return Result(True,'off')
        monkeypatch.setattr(rig.ts,'stop_following',follower_off)
        before=len(rig.robot.switches);stops=rig.ts.stops
        for _ in range(10):rig.robot.send('/emergency_stop',harness._Msg())
        await pump()
        latch(rig)
        for _ in range(20):rig.robot.emit_stop_status()
        await pump()
        assert len(entered)==1 and rig.ts.stops==stops+1
        assert len(rig.robot.switches)==before
        assert not (await rig.call('stand_up')).ok
        gate.set();await pump()
        for _ in range(10):rig.robot.send('/emergency_stop',harness._Msg())
        await pump();assert len(entered)==1
        rig.robot.stop_status.update(latched=False,stop_required=False)
        rig.robot.emit_stop_status();await pump()
        assert rig.ts.motion_block_reason() is None
        latch(rig);await pump();assert len(entered)==2
        assert len(rig.robot.switches)==before
    run_case(case)


def test_stale_watchdog_cancels_commands_but_does_not_compete_with_gateway():
    async def case(rig):
        rig.robot.stop_status_enabled=False;rig.clock.now+=.6
        before=len(rig.robot.switches)
        rig.ts._stop_watchdog();await pump()
        assert rig.ts.commands.empty() and len(rig.robot.switches)==before
        assert rig.ts.motion_block_reason()
    run_case(case)


def test_arbiter_restart_cannot_be_adopted_by_fresh_clear_status():
    async def case(rig):
        rig.robot.stop_status.update(instance_id='2'*32)
        rig.robot.emit_stop_status();await pump()
        assert not (await rig.call('stand_up')).ok
        rig.robot.stop_status.update(instance_id='1'*32)
        rig.robot.emit_stop_status();await pump()
        assert rig.ts.motion_block_reason()
    run_case(case)


def test_empty_before_first_status_requires_observed_stop_before_clear():
    tracker=StopStatus();tracker.inhibit()
    assert accept(tracker,epoch=500)
    assert tracker.reason(10.)
    assert accept(tracker,10.1,sequence=2,epoch=501,latched=True)
    assert accept(tracker,10.2,sequence=3,epoch=501)
    assert tracker.reason(10.2) is None


def test_stop_while_waiting_for_activation_service_refuses_before_call(monkeypatch):
    async def case(rig):
        await rig.ts.switch_to(None)
        client=rig.ts.switch_client;called=[]
        def availability(timeout_sec=0.):
            latch(rig)
            return False
        monkeypatch.setattr(client,'wait_for_service',availability)
        monkeypatch.setattr(client,'call_async',lambda req:called.append(req))
        result=await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok and not called
        assert rig.ts.controller_fault is None  # no remote request was submitted
    run_case(case)


def test_follower_start_receipt_after_latch_is_cleaned_up_not_reported_success(monkeypatch):
    async def case(rig):
        original=rig.robot.handle
        def handle(service,req):
            result=original(service,req)
            if service=='activate_person_following':latch(rig)
            return result
        monkeypatch.setattr(rig.robot,'handle',handle)
        result=await rig.ts.set_following(True)
        assert not result.ok and not rig.robot.follower_on
        assert rig.robot.follow_calls[-1]=='off'
    run_case(case)


def test_new_latch_during_fall_recovery_cannot_erase_fall_evidence(monkeypatch):
    async def case(rig):
        rig.ts.fallen=LOCOMOTION_CONTROLLER
        rig.imu(0.,seconds=1.5)
        async def stop_while_waiting():latch(rig)
        monkeypatch.setattr(rig.ts,'_wait_walk_ready',stop_while_waiting)
        result=await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok
        assert rig.ts.fallen==LOCOMOTION_CONTROLLER
    run_case(case)


@pytest.mark.parametrize('field,value',[('closing',True),('startup_error','startup failed'),('follow_state_uncertain',True)])
def test_direct_motion_cannot_bypass_existing_global_fault_flags(field,value):
    async def case(rig):
        setattr(rig.ts,field,value)
        with pytest.raises(MotionBlockedError):rig.ts.send_speed(.1,0,0)
        assert not (await rig.ts.switch_to(LOCOMOTION_CONTROLLER)).ok
        rig.ts.send_speed(0,0,0)
        assert (await rig.ts.switch_to(None)).ok
    run_case(case)
