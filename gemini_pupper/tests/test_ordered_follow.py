"""Ordered follow intent and real queue/ROS-client cancellation; no hardware."""
import asyncio
import pytest
from test_follow_exit_takeover import seated_rig
import test_tools as harness
import tool_registry
from robot_api import LOCOMOTION, Result


def run(case, *, seated=False):
    async def scenario():
        rig = await seated_rig() if seated else harness.Rig()
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(6): await asyncio.sleep(0)
    asyncio.run(scenario())


def job_id(result):
    assert result.ok and result.details['submission']['schema'] == 'pupper_submission_v1'
    return result.details['submission']['job_id']


def outcome(rig, result):
    return next(row for row in rig.ts.recent_outcomes()['outcomes'] if row['job_id'] == job_id(result))


async def hold_move(rig):
    entered, release = asyncio.Event(), asyncio.Event()
    original = rig.ts.sleep
    async def blocked(seconds):
        entered.set(); await release.wait()
        return await original(seconds)
    rig.ts.sleep = blocked
    move = await tool_registry.call(rig.ts, 'move', {'direction':'forward', 'speed':.2, 'duration':1})
    assert move.ok
    await entered.wait()
    return move, release


def test_walk_wait_follow_is_fifo_and_admission_does_not_wait_for_queue():
    async def case(rig):
        move, release = await hold_move(rig)
        wait = await tool_registry.call(rig.ts, 'wait', {'duration':1})
        follow = await tool_registry.call(rig.ts, 'follow', {'enabled':True})
        assert job_id(follow) and not rig.robot.follower_on
        assert rig.ts.current_job_id == job_id(move) and rig.ts.commands.qsize() == 2
        release.set(); assert await rig.settle()
        rows = [outcome(rig,r) for r in (move,wait,follow)]
        assert all(r['terminal_state'] == 'succeeded' for r in rows)
        assert rows[0]['event_seq'] < rows[1]['event_seq'] < rows[2]['event_seq']
        assert rig.robot.follower_on
    run(case)


@pytest.mark.parametrize('motion,args', [('move',{'direction':'forward','duration':.2}), ('stand_up',{}), ('lie_down',{})])
def test_new_motion_removes_only_pending_follow_and_preserves_prior_work(motion,args):
    async def case(rig):
        move, release = await hold_move(rig)
        follow = await tool_registry.call(rig.ts, 'follow', {'enabled':True})
        wait = await tool_registry.call(rig.ts, 'wait', {'duration':.1})
        later = await tool_registry.call(rig.ts, motion, args)
        assert later.ok and 'Stopped following' not in later.message
        cancelled = outcome(rig,follow)
        assert cancelled['terminal_state']=='cancelled' and not cancelled['execution_started']
        assert rig.ts.current_job_id == job_id(move)
        release.set(); assert await rig.settle()
        assert all(outcome(rig,r)['terminal_state']=='succeeded' for r in (move,wait,later))
        assert 'on' not in rig.robot.follow_calls
    run(case)


def test_pending_follow_cancellation_keeps_unchanged_wait_projection_in_sit():
    async def case(rig):
        entered, release = asyncio.Event(), asyncio.Event()
        async def blocker():
            entered.set();await release.wait();return Result(True,'held')
        sit=rig.ts.state_now
        accepted=await rig.ts.queue('hold seated',blocker,1,then=sit)
        await entered.wait()
        follow=await rig.ts.set_following(True)
        wait=await rig.ts.wait(.1)
        handoff=await rig.ts.prepare_follow_handoff()
        assert handoff.ok and handoff.details['following']['changed'] is False
        assert rig.ts.state_after_queue is sit
        release.set(); assert await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='cancelled'
        assert outcome(rig,accepted)['success'] and outcome(rig,wait)['success']
        assert rig.ts.state_now is sit and 'on' not in rig.robot.follow_calls
    run(case,seated=True)


def test_wait_status_and_conversation_preserve_pending_and_active_follow():
    async def case(rig):
        move,release=await hold_move(rig)
        follow=await rig.ts.set_following(True)
        wait=await tool_registry.call(rig.ts,'wait',{'duration':.1})
        assert (await tool_registry.call(rig.ts,'robot_status',{})).ok
        assert job_id(follow) in rig.ts._follow_jobs
        release.set();assert await rig.settle()
        assert all(outcome(rig,r)['success'] for r in (move,follow,wait))
        assert (await tool_registry.call(rig.ts,'robot_status',{})).ok
        assert (await rig.ts.wait(.1)).ok and await rig.settle()
        assert rig.robot.follower_on
    run(case)


@pytest.mark.parametrize('boundary',['exit','handoff'])
def test_superseded_current_follow_finishes_one_native_transition(boundary):
    async def case(rig):
        sit=rig.ts.tricks['sit_stay_shake']
        entered,release=asyncio.Event(),asyncio.Event()
        if boundary=='exit':
            original=rig.ts._matches_clip_endpoint
            def endpoint(controller,clip,request,earliest):
                if controller==sit.controller and clip['name']==sit.exit_clip and not release.is_set():
                    entered.set();return False
                return original(controller,clip,request,earliest)
            rig.ts._matches_clip_endpoint=endpoint
        else:
            original=rig.ts._call
            async def handoff(client,request,timeout=3):
                if client is rig.ts.switch_client and sit.controller in request.deactivate_controllers:
                    entered.set();await release.wait()
                return await original(client,request,timeout)
            rig.ts._call=handoff
        follow=await rig.ts.set_following(True)
        await entered.wait()
        current=rig.ts.current
        prepare=await rig.ts.prepare_follow_handoff()
        assert prepare.ok and prepare.details['following']['changed'] is False
        later=await tool_registry.call(rig.ts,'move',{'direction':'forward','duration':.2})
        assert later.ok and not current.done()
        assert rig.robot.clips(sit.controller)==[sit.exit_clip]
        release.set();assert await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='cancelled'
        assert outcome(rig,follow)['execution_started']
        assert outcome(rig,later)['success']
        assert rig.robot.clips(sit.controller)==[sit.exit_clip]
        assert 'on' not in rig.robot.follow_calls and not rig.ts._exit_clip_requests
    run(case,seated=True)


class Deferred:
    finished=False
    cancelled=False
    def done(self):return self.finished
    def result(self):return harness._Msg(success=True)
    def cancel(self):self.cancelled=True


async def pending_activation(rig):
    future=Deferred();entered=asyncio.Event();requests=[]
    client=rig.ts.follow_on
    def delayed(request):
        requests.append(request);entered.set();return future
    client.call_async=delayed
    follow=await rig.ts.set_following(True)
    await entered.wait()
    def complete():
        rig.robot.handle(client.name,requests[0]);future.finished=True
    return follow,future,complete


def test_inflight_activation_is_confirmed_off_before_q7_cue_and_motion():
    async def case(rig):
        follow,future,complete=await pending_activation(rig)
        handoff=asyncio.create_task(rig.ts.prepare_follow_handoff())
        await asyncio.sleep(0)
        assert not handoff.done()
        complete()
        result=await handoff
        assert result.ok and result.details['following']['changed'] is True
        assert rig.robot.follow_calls[-2:]==['on','off'] and not rig.robot.follower_on
        assert not future.cancelled and not rig.ts.controller_fault
        later=await tool_registry.call(rig.ts,'move',{'direction':'forward','duration':.1})
        assert later.ok and await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='cancelled'
        assert outcome(rig,later)['success']
    run(case)


def test_inflight_activation_failure_blocks_new_motion_without_false_off_receipt():
    async def case(rig):
        follow,future,complete=await pending_activation(rig)
        original=rig.robot.handle
        def refuse(name,request):
            return harness._Msg(success=False) if name=='deactivate_person_following' else original(name,request)
        rig.robot.handle=refuse
        handoff=asyncio.create_task(rig.ts.prepare_follow_handoff())
        await asyncio.sleep(0);complete()
        result=await handoff
        assert not result.ok and not result.details
        assert rig.ts.follow_state_uncertain and not rig.robot.active()
        later=await tool_registry.call(rig.ts,'move',{'direction':'forward','duration':.1})
        assert not later.ok and await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='failed'
    run(case)


def test_stop_interrupts_activation_then_late_remote_on_is_cleaned_up():
    async def case(rig):
        follow,future,complete=await pending_activation(rig)
        stopped=await tool_registry.call(rig.ts,'stop',{})
        assert await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='cancelled'
        assert rig.ts.controller_fault and not future.cancelled
        complete()
        for _ in range(30):
            await asyncio.sleep(0)
            if not rig.ts.activation_cleanup_tasks:break
        assert not rig.ts.activation_cleanup_tasks and not rig.robot.follower_on
        assert not (await tool_registry.call(rig.ts,'move',{'direction':'forward'})).ok
    run(case)


def test_follow_off_cancels_only_pending_entry_and_keeps_ordinary_queue():
    async def case(rig):
        move,release=await hold_move(rig)
        follow=await rig.ts.set_following(True)
        off=await rig.ts.set_following(False)
        assert off.ok and off.details['following']['changed'] is False
        assert rig.ts.current_job_id==job_id(move)
        release.set();assert await rig.settle()
        assert outcome(rig,move)['success'] and outcome(rig,follow)['terminal_state']=='cancelled'
    run(case)


def test_true_follow_failure_cancels_dependent_wait():
    async def case(rig):
        original=rig.ts.switch_to
        async def failure(*args,**kwargs):return Result(False,'test transition failure')
        rig.ts.switch_to=failure
        follow=await rig.ts.set_following(True)
        wait=await rig.ts.wait(.1)
        assert await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='failed'
        assert outcome(rig,wait)['terminal_state']=='cancelled' and not outcome(rig,wait)['execution_started']
    run(case)


def test_public_admission_lock_orders_follow_after_motion_whose_off_call_is_pending():
    async def case(rig):
        assert (await rig.ts.set_following(True)).ok and await rig.settle()
        entered,release=asyncio.Event(),asyncio.Event()
        original=rig.ts._call
        async def blocked(client,request,timeout=3):
            if client is rig.ts.follow_off:
                entered.set();await release.wait()
            return await original(client,request,timeout)
        rig.ts._call=blocked
        motion=asyncio.create_task(tool_registry.call(rig.ts,'move',{'direction':'forward','duration':.1}))
        await entered.wait()
        following=asyncio.create_task(tool_registry.call(rig.ts,'follow',{'enabled':True}))
        await asyncio.sleep(0)
        assert not following.done()
        release.set()
        move,follow=await asyncio.gather(motion,following)
        assert await rig.settle()
        assert outcome(rig,move)['success'] and outcome(rig,follow)['success']
        assert outcome(rig,move)['event_seq'] < outcome(rig,follow)['event_seq']
        assert rig.robot.follower_on
    run(case)


def test_explicit_follow_off_retries_uncertain_shutdown_even_with_motion_quarantine():
    async def case(rig):
        assert (await rig.ts.set_following(True)).ok and await rig.settle()
        rig.ts.follow_state_uncertain=True
        rig.ts.controller_fault='test prior activation uncertainty'
        result=await rig.ts.set_following(False)
        assert result.ok and result.details['following']['changed']
        assert not rig.ts.following and not rig.robot.follower_on and not rig.ts.follow_state_uncertain
        assert rig.ts.controller_fault  # confirmed off does not clear independent quarantine
    run(case)


def test_stop_during_current_follow_native_exit_cancels_and_does_not_replay():
    async def case(rig):
        sit=rig.ts.tricks['sit_stay_shake'];entered=asyncio.Event()
        original=rig.ts._matches_clip_endpoint
        def endpoint(controller,clip,request,earliest):
            if controller==sit.controller and clip['name']==sit.exit_clip:
                entered.set();return False
            return original(controller,clip,request,earliest)
        rig.ts._matches_clip_endpoint=endpoint
        follow=await rig.ts.set_following(True);await entered.wait()
        wait=await rig.ts.wait(.1)
        assert (await tool_registry.call(rig.ts,'stop',{})).ok and await rig.settle()
        assert outcome(rig,follow)['terminal_state']=='cancelled'
        assert outcome(rig,wait)['terminal_state']=='cancelled'
        assert rig.robot.clips(sit.controller)==[sit.exit_clip] and not rig.ts._exit_clip_requests
        assert not rig.robot.follower_on and not rig.ts._follow_jobs
    run(case,seated=True)


def test_superseded_follow_during_service_discovery_never_submits_activation():
    async def case(rig):
        available, discovery = asyncio.Event(), asyncio.Event()
        client = rig.ts.follow_on
        def ready(**kwargs):
            discovery.set()
            return available.is_set()
        client.wait_for_service = ready
        follow = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
        await discovery.wait()
        handoff = asyncio.create_task(rig.ts.prepare_follow_handoff())
        await asyncio.sleep(0)
        available.set()
        result = await handoff
        assert result.ok and result.details['following']['changed'] is False
        assert await rig.settle()
        assert outcome(rig, follow)['terminal_state'] == 'cancelled'
        assert 'on' not in rig.robot.follow_calls
        assert not rig.ts.controller_fault and not rig.ts.activation_cleanup_tasks
        later = await tool_registry.call(rig.ts, 'move', {'direction': 'forward', 'duration': .1})
        assert later.ok and await rig.settle() and outcome(rig, later)['success']
    run(case)


def test_superseding_dequeued_unstarted_follow_keeps_actual_seated_projection():
    async def case(rig):
        sit = rig.ts.state_now
        follow = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
        await asyncio.sleep(0)  # worker dequeues and schedules its child, not yet entered
        assert rig.ts.current_job_id == job_id(follow)
        assert not rig.ts.current_execution_started
        handoff = await rig.ts.prepare_follow_handoff()
        assert handoff.ok and handoff.details['following']['changed'] is False
        assert rig.ts.state_after_queue is sit
        paw = await tool_registry.call(rig.ts, 'interactive_action', {'action': 'shake_left'})
        assert paw.ok and await rig.settle()
        assert outcome(rig, follow)['terminal_state'] == 'cancelled'
        assert outcome(rig, paw)['success'] and rig.ts.state_now is sit
        assert rig.robot.clips(sit.controller) == ['shake_left']
        assert 'on' not in rig.robot.follow_calls
    run(case, seated=True)


def test_refused_low_level_paw_does_not_take_over_pending_follow():
    async def case(rig):
        entered, release = asyncio.Event(), asyncio.Event()
        async def held():
            entered.set(); await release.wait(); return Result(True, 'held')
        sit = rig.ts.state_now
        await rig.ts.queue('hold seated', held, 1, then=sit)
        await entered.wait()
        follow = await tool_registry.call(rig.ts, 'follow', {'enabled': True})
        paw = await tool_registry.call(rig.ts, 'interactive_action', {'action': 'shake_left'})
        assert not paw.ok and 'play it first' in paw.message
        assert job_id(follow) in rig.ts._follow_jobs
        release.set(); assert await rig.settle()
        assert outcome(rig, follow)['success'] and rig.robot.follower_on
        assert 'shake_left' not in rig.robot.clips(sit.controller)
    run(case, seated=True)
