"""Offline fault injection against the real queue/controller code and course fake ROS."""
import asyncio
import hashlib
import json
import sys
from pathlib import Path

import pytest

ROBOT_DIR = Path(__file__).resolve().parents[1] / "robot"
sys.path.insert(0, str(ROBOT_DIR))
import test_tools as harness
import robot_api
import tool_registry
from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER, MOTORS_IDLE, Result
from secret_gate import SecretGate
from tricks import load_tricks


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            await rig.settle()
            await case(rig)
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


@pytest.mark.parametrize("tool,args", [
    ("move", {"direction": "forward", "speed": float("nan")}),
    ("arc", {"direction": "left", "angle": float("inf")}),
    ("turn", {"direction": "left", "angle": -90}),
    ("move", {"direction": "forward", "speed": -0.4}),
    ("move", {"direction": "forward", "speed": True}),
    ("arc", {"direction": "left", "angle": 90, "radius": float("nan")}),
])
def test_invalid_motion_never_queues_or_moves(tool, args):
    async def case(rig):
        before = list(rig.robot.switches)
        result = await tool_registry.call(rig.ts, tool, args)
        assert not result.ok
        await rig.settle()
        assert rig.ts.commands.empty()
        assert rig.robot.switches == before
        assert not harness.walk_twists(rig)
    run_case(case)


def test_bad_argument_container_is_a_refusal():
    async def case(rig):
        assert not (await tool_registry.call(rig.ts, "move", ["forward"])).ok
        assert not (await rig.ts.wait(float("nan"))).ok
        assert not (await rig.ts.wait(-1)).ok
        assert rig.ts.commands.empty()
    run_case(case)


@pytest.mark.parametrize('source_args', [
    [],
    ['entity/project/run', '--policy-file', 'p.json'],
    ['entity/project/run', '--walk-policy-file', 'w.json'],
    ['--policy-file', 'p.json', '--walk', 'entity/project/walk'],
    ['--policy-file', 'p.json', '--walk-policy-file', 'w.json', '--walk', 'entity/project/walk'],
    ['--policy-file', 'p.json', '--walk-limits', 'limits.json'],
])
def test_trick_installer_rejects_ambiguous_or_mixed_sources_before_access(monkeypatch, source_args):
    import get_trick
    monkeypatch.setattr(get_trick, 'resolve_run_path', lambda *args: pytest.fail('source validation must precede remote access'))
    monkeypatch.setattr(sys, 'argv', ['get_trick.py', '--name', 'new_trick', *source_args])
    with pytest.raises(SystemExit) as exc:
        get_trick.main()
    assert exc.value.code == 2


@pytest.mark.parametrize('extra', [[], ['--walk-policy-file', 'walk.json']])
def test_local_spider_install_requires_both_policies_and_evaluated_limits(monkeypatch, extra):
    import get_trick
    monkeypatch.setattr(sys, 'argv', ['get_trick.py', '--name', 'spider_walk', '--policy-file', 'clip.json', *extra])
    with pytest.raises(SystemExit) as exc:
        get_trick.main()
    assert exc.value.code == 2


def test_local_policy_install_validates_and_records_provenance_without_wandb(tmp_path, monkeypatch):
    import get_trick
    source = ROBOT_DIR.parent / 'tricks/sneeze/policy.json'
    destination = tmp_path / 'tricks'
    monkeypatch.setattr(get_trick, 'TRICK_DIR', destination)
    monkeypatch.setattr(get_trick, 'resolve_run_path', lambda *args: pytest.fail('local import must not use W&B'))
    monkeypatch.setattr(sys, 'argv', ['get_trick.py', '--name', 'local_sneeze', '--policy-file', str(source)])
    get_trick.main()
    installed = destination / 'local_sneeze'
    assert (installed / 'policy.json').read_bytes() == source.read_bytes()
    manifest = json.loads((installed / 'artifact_manifest.json').read_text())
    assert manifest['policy_source']['type'] == 'local'
    assert manifest['policy_source']['path'] == str(source.resolve())
    assert manifest['policy_source']['sha256'] == manifest['files']['policy.json']
    assert manifest['policy_run'] is None and not manifest['hardware_validated']
    get_trick.main()
    backups = list((destination / '.backups').iterdir())
    assert len(backups) == 1 and (backups[0] / 'policy.json').read_bytes() == source.read_bytes()


def test_local_second_policy_failure_keeps_previous_installation(tmp_path, monkeypatch):
    import get_trick
    source = ROBOT_DIR.parent / 'tricks/sneeze/policy.json'
    destination = tmp_path / 'tricks'
    installed = destination / 'existing'
    installed.mkdir(parents=True)
    (installed / 'marker').write_text('preserved baseline')
    bad_walk = tmp_path / 'not_a_walking_policy.json'
    bad_walk.write_text('{"trick": {}}')
    limits = tmp_path / 'limits.json'
    limits.write_text(json.dumps({'evaluation_source': 'test fixture; never hardware',
        'walk_policy_sha256': hashlib.sha256(bad_walk.read_bytes()).hexdigest(),
        'speed': .04, 'max_forward': .08, 'max_sideways': .04, 'turn': 15.}))
    monkeypatch.setattr(get_trick, 'TRICK_DIR', destination)
    monkeypatch.setattr(sys, 'argv', ['get_trick.py', '--name', 'existing', '--policy-file', str(source), '--walk-policy-file', str(bad_walk), '--walk-limits', str(limits)])
    with pytest.raises(ValueError, match='velocity policy'):
        get_trick.main()
    assert sorted(p.name for p in installed.iterdir()) == ['marker']
    assert not (destination / '.backups').exists()


def test_new_evaluated_walk_limits_replace_old_limits_without_losing_actions(tmp_path):
    import get_trick
    import yaml
    spec = tmp_path / 'trick.yaml'
    spec.write_text(yaml.safe_dump({'description': 'existing spider', 'secret': True,
        'actions': {'spider_up': 'stand'}, 'walk': {'policy': 'walk_policy.json', 'speed': .1}}))
    limits = {'evaluation_source': 'new-run/evaluation.json', 'speed': .04,
              'max_forward': .08, 'max_sideways': .04, 'turn': 15.}
    get_trick.write_template(tmp_path, 'spider_walk', {}, walks=True, walk_limits=limits)
    result = yaml.safe_load(spec.read_text())
    assert result['walk']['speed'] == .04
    assert result['evaluation_source'] == 'new-run/evaluation.json'
    assert result['secret'] and result['actions'] == {'spider_up': 'stand'}
    before = spec.read_bytes()
    with pytest.raises(ValueError, match='finite positive'):
        get_trick.write_template(tmp_path, 'spider_walk', {}, walks=True,
                                 walk_limits={**limits, 'speed': float('nan')})
    assert spec.read_bytes() == before


@pytest.mark.parametrize("crashes", [False, True])
def test_failed_job_cancels_dependent_sequence_and_reports_outcome(crashes):
    async def case(rig):
        ran = []
        async def failing():
            if crashes:
                raise RuntimeError("injected controller failure")
            return Result(False, "controller refused")
        async def dependent():
            ran.append("must not run")
            return Result(True, "bad")
        await rig.ts.queue("first", failing, 1, then=rig.ts.tricks["hold_pose"])
        await rig.ts.queue("dependent", dependent, 1, then=LOCOMOTION)
        await rig.settle()
        assert ran == []
        assert rig.ts.state_after_queue == rig.ts.state_now
        assert rig.ts.status()["last_outcome"]["success"] is False
        assert "failed" in (await rig.ts.mode()).message
    run_case(case)


def test_controller_refusal_does_not_claim_a_new_state():
    async def case(rig):
        original = rig.robot.handle
        def refuse(service, request):
            if service.endswith("switch_controller"):
                return harness._Msg(ok=False)
            return original(service, request)
        rig.robot.handle = refuse
        old_state = rig.ts.state_now
        assert not (await rig.ts.switch_to(rig.ts.tricks["bow"].controller)).ok
        assert rig.ts.state_now == old_state
        assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
    run_case(case)


def test_exit_failure_does_not_jump_to_another_pose():
    async def case(rig):
        trick = rig.ts.tricks["hold_pose"]
        await harness.behaviors.play_policy_trick(rig.ts, trick)
        async def fail(*args):
            return Result(False, "clip feedback missing")
        rig.ts.play_clip = fail
        before = list(rig.robot.switches)
        result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok and "cannot leave" in result.message
        assert rig.robot.switches == before
        assert rig.ts.state_now is trick
    run_case(case)


@pytest.mark.parametrize('feedback', ['missing', 'wrong_endpoint'])
def test_interactive_entry_requires_new_endpoint_feedback(feedback):
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        rig.ts.clip_status[trick.controller] = ('idle', False)  # old activation
        original = rig.robot.send
        def feedback_fault(topic, message):
            if topic == f'/{trick.controller}/clip_status':
                if feedback == 'wrong_endpoint':
                    original(topic, harness._DataMsg('enter playing frame 0 queued 0'))
                return
            original(topic, message)
        rig.robot.send = feedback_fault
        result = await harness.behaviors.play_policy_trick(rig.ts, trick)
        assert not result.ok and 'endpoint' in result.message
        assert rig.ts.controller_fault
        assert not rig.robot.active()
    run_case(case)


def test_walking_stance_exit_requires_terminal_feedback_before_locomotion():
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        trick.walk_path = Path('test_walking_policy.json')
        rig.ts.leg_controllers.add(trick.walk_controller)
        rig.robot.controllers = {name: 'inactive' for name in rig.robot.controllers}
        rig.robot.controllers[trick.walk_controller] = 'active'
        rig.ts.state_now = rig.ts.state_after_queue = trick
        # A previous exit must not satisfy the confirmation for this activation.
        rig.ts.clip_status[trick.controller] = (trick.exit_clip, True)
        original = rig.robot.send
        rig.robot.send = lambda topic, msg: None if topic.endswith('/clip_status') else original(topic, msg)
        result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok and 'endpoint' in result.message
        assert not rig.robot.activations(LOCOMOTION_CONTROLLER)
        assert rig.robot.start_clips[trick.controller][-1] == trick.exit_clip
        assert rig.ts.controller_fault and not rig.robot.active()
    run_case(case)


def test_rejected_start_clip_prevents_controller_activation():
    async def case(rig):
        original = rig.robot.handle
        def refuse(service, request):
            if service.endswith("/set_parameters"):
                return harness._Msg(results=[harness._Msg(successful=False)])
            return original(service, request)
        rig.robot.handle = refuse
        trick = rig.ts.tricks["hold_pose"]
        result = await rig.ts.switch_to(trick.controller, start_clip="leave")
        assert not result.ok
        assert trick.controller not in rig.robot.active()
    run_case(case)


def test_unknown_start_clip_update_quarantines_later_activation():
    async def case(rig):
        original = rig.ts._call
        async def lost_reply(client, request, timeout=3.0):
            if client.name.endswith('/set_parameters'):
                return None
            return await original(client, request, timeout)
        rig.ts._call = lost_reply
        trick = rig.ts.tricks['hold_pose']
        assert not (await rig.ts.switch_to(trick.controller, start_clip='leave')).ok
        assert rig.ts.controller_fault
        assert trick.controller not in rig.robot.active()
        assert not (await rig.ts.switch_to(trick.controller)).ok
    run_case(case)


@pytest.mark.parametrize('reported_success', [False, True])
def test_failed_fall_reset_never_claims_recovery(reported_success):
    async def case(rig):
        rig.ts.fallen = LOCOMOTION_CONTROLLER
        rig.imu(0., seconds=2.)
        original = rig.robot.handle
        def no_change(service, request):
            if service.endswith('switch_controller'):
                return harness._Msg(ok=reported_success)  # legs are still active
            return original(service, request)
        rig.robot.handle = no_change
        result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok
        assert rig.ts.fallen == LOCOMOTION_CONTROLLER
    run_case(case)


def test_cleanup_checks_observed_state_after_positive_deactivation_reply():
    async def case(rig):
        original = rig.robot.handle
        def no_change(service, request):
            if service.endswith('switch_controller'):
                return harness._Msg(ok=True)
            return original(service, request)
        rig.robot.handle = no_change
        result = await rig.ts._relax_controllers()
        assert not result.ok and rig.ts.controller_fault
        assert LOCOMOTION_CONTROLLER in rig.robot.active()
    run_case(case)


def test_clip_without_completion_feedback_fails():
    async def case(rig):
        trick = rig.ts.tricks["hold_pose"]
        await harness.behaviors.play_policy_trick(rig.ts, trick)
        old_publish = rig.robot.publish
        def playing_forever(topic, msg):
            if topic.endswith("/clip"):
                rig.robot.send(f"/{trick.controller}/clip_status", harness._DataMsg(f"{msg.data} playing frame 0 queued 0"))
            else:
                old_publish(topic, msg)
        rig.robot.publish = playing_forever
        result = await rig.ts.play_clip(trick, "wave_left")
        assert not result.ok and "completion" in result.message
        assert rig.ts.controller_fault and not rig.robot.active()
    run_case(case)


def test_clip_request_after_fall_never_reaches_controller():
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        rig.ts.fallen = trick.controller
        result = await rig.ts.play_clip(trick, 'wave_left')
        assert not result.ok
        assert not rig.robot.clips(trick.controller)
    run_case(case)


def test_stop_cancels_reference_jump_controller_not_just_velocity():
    async def case(rig):
        published = asyncio.Event()
        original = rig.ts.ref_pub.publish
        def observe(msg):
            original(msg)
            published.set()
        rig.ts.ref_pub.publish = observe
        assert (await tool_registry.call(rig.ts, 'play_trick', {'name': 'hop'})).ok
        await published.wait()
        assert rig.ts.reference_active
        result = await tool_registry.call(rig.ts, 'stop', {})
        await rig.settle()
        assert result.ok and 'reference' in result.message
        assert not rig.robot.active()
        assert not rig.ts.reference_active
    run_case(case)


@pytest.mark.parametrize('autonomous', ['follower', 'trick'])
def test_graceful_shutdown_stops_autonomous_work_and_finishes_worker(autonomous):
    async def case(rig):
        if autonomous == 'follower':
            assert (await rig.ts.set_following(True)).ok
        else:
            entered = asyncio.Event()
            async def job():
                result = await rig.ts.switch_to(rig.ts.tricks['bow'].controller)
                assert result.ok
                entered.set()
                await asyncio.Event().wait()
            assert (await rig.ts.queue('active trick', job, 4, then=LOCOMOTION)).ok
            await entered.wait()
        result = await rig.ts.shutdown()
        assert result.ok
        assert not rig.robot.follower_on
        assert rig.ts.worker.done()
        assert rig.ts.commands.empty()
        if autonomous == 'trick':
            assert not rig.robot.active()
        assert not (await tool_registry.call(rig.ts, 'move', {'direction': 'forward'})).ok
    run_case(case)


def test_gamepad_stop_does_not_erase_fall_evidence(monkeypatch):
    async def case(rig):
        scheduled = []
        def remember(coroutine, loop):
            scheduled.append(coroutine)
        monkeypatch.setattr(robot_api.asyncio, 'run_coroutine_threadsafe', remember)
        rig.ts.fallen = LOCOMOTION_CONTROLLER
        rig.ts._on_estop(None)
        assert rig.ts.fallen == LOCOMOTION_CONTROLLER
        await scheduled[0]
        assert rig.ts.fallen == LOCOMOTION_CONTROLLER
        # No IMU evidence: stand-up must still be refused after the e-stop.
        assert not (await rig.ts.switch_to(LOCOMOTION_CONTROLLER)).ok
    run_case(case)


def test_failed_follower_stop_blocks_new_motion_and_stop_relaxes():
    async def case(rig):
        assert (await rig.ts.set_following(True)).ok
        original = rig.robot.handle
        def refuse(service, request):
            if service == "deactivate_person_following":
                return harness._Msg(success=False)
            return original(service, request)
        rig.robot.handle = refuse
        result = await tool_registry.call(rig.ts, "move", {"direction": "forward"})
        assert not result.ok and rig.ts.following
        assert rig.ts.commands.empty() and not harness.walk_twists(rig)
        result = await tool_registry.call(rig.ts, "stop", {})
        assert not result.ok  # don't claim the follower service succeeded
        assert not rig.robot.active()  # continuing follower commands cannot drive relaxed motors
    run_case(case)


def test_stop_can_interrupt_a_missing_service_wait():
    async def case(rig):
        class Missing:
            def wait_for_service(self, timeout_sec):
                assert timeout_sec == 0  # must not block the event loop
                return False
        waiting = asyncio.create_task(rig.ts._call(Missing(), None, timeout=2.0))
        await asyncio.sleep(0)
        assert not waiting.done()
        result = await tool_registry.call(rig.ts, "stop", {})
        assert result.ok and rig.ts.stops == 1
        assert await waiting is None
    run_case(case)


def test_stop_during_pending_switch_removes_late_activation():
    async def case(rig):
        original = rig.ts._call
        switch_started = asyncio.Event()
        finish_switch = asyncio.Event()
        async def delayed(client, request, timeout=3.0):
            if client is rig.ts.switch_client and request.activate_controllers:
                switch_started.set()
                await finish_switch.wait()
            return await original(client, request, timeout)
        rig.ts._call = delayed
        switching = asyncio.create_task(rig.ts.switch_to(rig.ts.tricks["bow"].controller))
        await switch_started.wait()
        await rig.ts.immediate_stop()
        finish_switch.set()
        result = await switching
        assert not result.ok
        assert not rig.robot.active()
    run_case(case)


def test_waiting_request_cannot_outlive_stop():
    async def case(rig):
        await rig.ts.action_lock.acquire()
        request = asyncio.create_task(tool_registry.call(rig.ts, "move", {"direction": "forward"}))
        await asyncio.sleep(0)
        await tool_registry.call(rig.ts, "stop", {})
        rig.ts.action_lock.release()
        assert not (await request).ok
        await rig.settle()
        assert not harness.walk_twists(rig)
    run_case(case)


@pytest.mark.parametrize("cancelled", [False, True])
def test_unknown_switch_quarantines_motion_and_cleans_actual_late_activation(cancelled):
    async def case(rig):
        class Deferred:
            finished = False
            cancelled = False
            def done(self): return self.finished
            def result(self): return harness._Msg(ok=True)
            def cancel(self): self.cancelled = True
        deferred = Deferred()
        client = rig.ts.switch_client
        original = client.call_async
        requests = []
        started = asyncio.Event()
        def delayed(request):
            if request.activate_controllers:
                requests.append(request)
                started.set()
                return deferred
            return original(request)
        client.call_async = delayed
        operation = asyncio.create_task(rig.ts.switch_to(rig.ts.tricks["bow"].controller))
        await started.wait()
        if cancelled:
            operation.cancel()
            with pytest.raises(asyncio.CancelledError):
                await operation
        else:
            result = await operation
            assert not result.ok and "quarantin" in result.message
        assert rig.ts.controller_fault and not deferred.cancelled
        assert not (await tool_registry.call(rig.ts, "move", {"direction": "forward"})).ok
        assert not (await rig.ts.set_following(True)).ok
        assert not (await rig.ts.immediate_stop()).ok  # the uncertainty remains visible
        # The server acts AFTER our client has already timed out or been stopped.
        rig.robot.handle(client.name, requests[0])
        deferred.finished = True
        assert rig.ts.tricks["bow"].controller in rig.robot.active()
        for _ in range(30):
            await asyncio.sleep(0)
            if not rig.ts.activation_cleanup_tasks:
                break
        assert not rig.ts.activation_cleanup_tasks
        assert not rig.robot.active()
        assert rig.ts.controller_fault  # no automatic rearming after an unknown outcome
    run_case(case)


def test_failed_cleanup_after_late_activation_is_not_reported_as_relaxed():
    async def case(rig):
        original = rig.ts._call
        started, proceed = asyncio.Event(), asyncio.Event()
        async def delayed(client, request, timeout=3.0):
            if client is rig.ts.switch_client:
                if request.activate_controllers:
                    started.set()
                    await proceed.wait()
                else:
                    return harness._Msg(ok=False)
            return await original(client, request, timeout)
        rig.ts._call = delayed
        task = asyncio.create_task(rig.ts.switch_to(rig.ts.tricks["bow"].controller))
        await started.wait()
        await rig.ts.immediate_stop()
        proceed.set()
        result = await task
        assert not result.ok and "could not confirm motors relaxed" in result.message
        assert rig.ts.tricks["bow"].controller in rig.robot.active()
    run_case(case)


def test_follow_refuses_a_walking_trick_and_fall():
    async def case(rig):
        trick = rig.ts.tricks["hold_pose"]
        trick.walk_path = Path("walk.json")
        rig.ts.state_after_queue = trick
        result = await rig.ts.set_following(True)
        assert not result.ok and "stand_up" in result.message
        assert "on" not in rig.robot.follow_calls
        rig.ts.state_after_queue = LOCOMOTION
        rig.ts.fallen = LOCOMOTION_CONTROLLER
        assert not (await rig.ts.set_following(True)).ok
        assert "on" not in rig.robot.follow_calls
    run_case(case)


def test_lock_also_revokes_moving_with_a_hidden_walking_policy():
    async def case(rig):
        trick = rig.ts.tricks['hold_pose']
        trick.secret = True
        trick.walk_path = Path('walk.json')
        rig.ts.state_now = rig.ts.state_after_queue = trick
        for tool, args in [('move', {'direction': 'forward'}),
                           ('move', {'direction': 'forward', 'run': True}),
                           ('turn', {'direction': 'left'}),
                           ('arc', {'direction': 'left', 'angle': 30., 'radius': 1.})]:
            result = await tool_registry.call(rig.ts, tool, args)
            assert not result.ok and 'locked' in result.message
            assert 'hold_pose' not in result.message
        assert rig.ts.commands.empty() and not harness.walk_twists(rig)
    run_case(case)


@pytest.mark.parametrize("refuses_stop", [False, True])
def test_server_restart_establishes_follower_shutdown_before_motion(refuses_stop):
    async def run():
        rig = harness.Rig()
        original = rig.robot.handle
        rig.robot.follower_on = True  # the ROS process survived the tool server
        def handle(service, request):
            if refuses_stop and service == "deactivate_person_following":
                return harness._Msg(success=False)
            return original(service, request)
        rig.robot.handle = handle
        try:
            result = await tool_registry.call(rig.ts, "move", {"direction": "forward"})
            await rig.settle()
            if refuses_stop:
                assert not result.ok and rig.ts.follow_state_uncertain
                assert not harness.walk_twists(rig)
                rig.robot.handle = original
                assert (await tool_registry.call(rig.ts, "stop", {})).ok
                assert not rig.ts.follow_state_uncertain
            else:
                assert result.ok
            assert not rig.robot.follower_on
            assert "off" in rig.robot.follow_calls
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


def test_startup_exception_refuses_work_without_a_dead_queue():
    async def run():
        rig = harness.Rig()
        async def broken():
            raise RuntimeError("missing startup transport")
        rig.ts._adopt_running_state = broken
        try:
            result = await tool_registry.call(rig.ts, "move", {"direction": "forward"})
            assert not result.ok and "startup" in result.message
            assert not rig.ts.worker.done()
            assert rig.ts.commands.empty() and not harness.walk_twists(rig)
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
        finally:
            rig.close()
            await asyncio.sleep(0)
    asyncio.run(run())


def test_stop_cleans_up_delayed_follower_activation():
    async def case(rig):
        original = rig.ts._call
        starting, proceed = asyncio.Event(), asyncio.Event()
        async def delayed(client, request, timeout=3.0):
            if client is rig.ts.follow_on:
                starting.set()
                await proceed.wait()
            return await original(client, request, timeout)
        rig.ts._call = delayed
        request = asyncio.create_task(tool_registry.call(rig.ts, "follow", {"enabled": True}))
        await starting.wait()
        assert (await tool_registry.call(rig.ts, "stop", {})).ok
        proceed.set()
        assert not (await request).ok
        assert not rig.robot.follower_on and not rig.ts.following
        assert rig.robot.follow_calls[-2:] == ["on", "off"]
    run_case(case)


def test_unconfirmed_follower_activation_requires_confirmed_shutdown():
    async def case(rig):
        original = rig.ts._call
        async def lost_response(client, request, timeout=3.0):
            response = await original(client, request, timeout)
            return None if client is rig.ts.follow_on else response
        rig.ts._call = lost_response
        result = await tool_registry.call(rig.ts, "follow", {"enabled": True})
        assert not result.ok
        assert not rig.robot.follower_on and not rig.ts.following
        assert rig.robot.follow_calls[-2:] == ["on", "off"]
    run_case(case)


@pytest.mark.parametrize("cancelled", [False, True])
def test_late_follower_start_after_timeout_or_cancellation_is_stopped(cancelled):
    async def case(rig):
        class Deferred:
            finished = False
            cancelled = False
            def done(self): return self.finished
            def result(self): return harness._Msg(success=True)
            def cancel(self): self.cancelled = True
        future = Deferred()
        pending = []
        started = asyncio.Event()
        client = rig.ts.follow_on
        def delayed(request):
            pending.append(request)
            started.set()
            return future
        client.call_async = delayed
        task = asyncio.create_task(rig.ts.set_following(True))
        await started.wait()
        if cancelled:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
        else:
            assert not (await task).ok
        assert rig.ts.controller_fault and not future.cancelled
        assert not (await tool_registry.call(rig.ts, "move", {"direction": "forward"})).ok
        rig.robot.handle(client.name, pending[0])  # remote activation completes late
        future.finished = True
        assert rig.robot.follower_on
        for _ in range(30):
            await asyncio.sleep(0)
            if not rig.ts.activation_cleanup_tasks:
                break
        assert not rig.ts.activation_cleanup_tasks
        assert not rig.robot.follower_on and not rig.ts.following
        assert rig.ts.controller_fault
    run_case(case)


def test_password_gate_never_embeds_the_password(tmp_path):
    password = "a test-only phrase 123"
    path = tmp_path / "password"
    path.write_text(password + "\n")
    gate = SecretGate(path)
    assert not gate.unlock("wrong")
    assert not gate.unlocked
    assert gate.unlock(password)
    gate.lock()
    assert not gate.unlocked
    assert not gate.unlock("")
    assert not gate.unlock("x" * 300)
    assert not SecretGate(tmp_path / "missing").unlock(password)


def test_hidden_tricks_are_enforced_at_acceptance_and_execution(tmp_path):
    password = "a test-only phrase 123"
    file = tmp_path / "password"
    file.write_text(password)
    async def case(rig):
        rig.ts.secret_gate = SecretGate(file)
        trick = rig.ts.tricks["hold_pose"]
        trick.secret = True
        decls = json.dumps(tool_registry.declarations(rig.ts))
        assert "hold_pose" not in decls and "wave_left" not in decls
        assert password not in decls
        denied = await tool_registry.call(rig.ts, "play_trick", {"name": "hold_pose"})
        assert not denied.ok and "unlock" in denied.message
        assert rig.ts.commands.empty()
        wrong = await tool_registry.call(rig.ts, "unlock", {"password": "wrong"})
        assert not wrong.ok and "wrong" not in wrong.message
        before = list(rig.robot.switches)
        unlocked = await tool_registry.call(rig.ts, "unlock", {"password": password})
        assert unlocked.ok and "hold_pose" in unlocked.message and password not in unlocked.message
        assert rig.robot.switches == before  # unlocking never moves
        assert "hold_pose" in json.dumps(tool_registry.declarations(rig.ts))
        assert (await tool_registry.call(rig.ts, "play_trick", {"name": "hold_pose"})).ok
        await tool_registry.call(rig.ts, "lock_tricks", {})
        await rig.settle()
        assert trick.controller not in rig.robot.active()  # queued before lock but not executed
        assert rig.ts.last_outcome["success"] is False
        assert "wave_left" not in rig.ts.action_ids()
    run_case(case)


@pytest.mark.parametrize("reset", ["stop", "lie_down"])
def test_unlock_lifetime_resets_on_stop_or_lie_down(tmp_path, reset):
    file = tmp_path / "password"
    file.write_text("test-only")
    async def case(rig):
        rig.ts.secret_gate = SecretGate(file)
        assert (await tool_registry.call(rig.ts, "unlock", {"password": "test-only"})).ok
        assert (await tool_registry.call(rig.ts, reset, {})).ok
        assert not rig.ts.secret_gate.unlocked
        await rig.settle()
    run_case(case)


def test_explicit_follower_config_never_silently_falls_back(tmp_path, monkeypatch):
    from runtime_config import follower_config
    monkeypatch.setenv("FOLLOW_ME_DIR", str(tmp_path))
    monkeypatch.setenv("PUPPER_FOLLOWER_MODE", "lab5")
    for var in ("FOLLOW_USE_COLOR", "FOLLOW_AVOID_OBSTACLES", "FOLLOW_ALWAYS_DETECT"):
        monkeypatch.delenv(var, raising=False)
    with pytest.raises(ValueError, match="unavailable"):
        follower_config()
    (tmp_path / "robot").mkdir()
    (tmp_path / "robot/follow_service.py").write_text("")
    (tmp_path / "robot/yolov8n_seg.hef").write_bytes(b"test placeholder")
    monkeypatch.setenv("FOLLOW_USE_COLOR", "true")
    monkeypatch.setenv("FOLLOW_AVOID_OBSTACLES", "1")
    config = follower_config()
    assert config.use_color and config.avoid_obstacles
    monkeypatch.setenv("PUPPER_FOLLOWER_MODE", "legacy")
    with pytest.raises(ValueError, match="require"):
        follower_config()


def test_invalid_walking_policy_caps_cannot_be_loaded(tmp_path):
    import yaml
    p = tmp_path / "test_spider"
    p.mkdir()
    (p / "policy.json").write_text(json.dumps({"trick": {"duration_s": 51/50, "fps": 50, "frames": 51,
        "clips": [{"name": "idle", "start": 0, "end": 26, "loop": True},
                  {"name": "up", "start": 25, "end": 51}]},
        "motion": {"joint_pos": [[0.] * 12 for _ in range(51)]}}))
    (p / "walk.json").write_text(json.dumps({"command_clip": {"vx": [-0.05, 0.15], "vy": [-0.1, 0.1], "wz": [-0.5, 0.5]}}))
    spec = {"type": "policy", "policy": "policy.json", "exit_clip": "up",
            "walk": {"policy": "walk.json", "speed": 0.05, "max_forward": 0.15, "max_sideways": 0.1, "turn": 20}}
    (p / "trick.yaml").write_text(yaml.safe_dump(spec))
    with pytest.raises(ValueError, match="trained"):
        load_tricks(tmp_path)  # backward limit must constrain the symmetric API
    spec["walk"]["max_forward"] = 0.05
    spec["walk"]["speed"] = float("nan")
    (p / "trick.yaml").write_text(yaml.safe_dump(spec))
    with pytest.raises(ValueError, match="finite"):
        load_tricks(tmp_path)


def test_huge_number_is_a_refusal_not_an_exception():
    async def case(rig):
        result = await tool_registry.call(rig.ts, "move", {"direction": "forward", "speed": 10**1000})
        assert not result.ok
        assert rig.ts.commands.empty()
    run_case(case)


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_invalid_imu_cannot_authorize_fall_recovery(bad):
    async def case(rig):
        rig.imu(tilt_deg=1)
        rig.ts._on_imu(harness._Msg(orientation=harness._Quat(x=bad), angular_velocity=harness._Vec()))
        assert not rig.ts.upright()[0]
    run_case(case)


def test_recovery_switch_failure_keeps_fall_latched():
    async def case(rig):
        rig.ts.fallen = LOCOMOTION_CONTROLLER
        rig.imu(tilt_deg=1)
        original = rig.robot.handle
        def refuse(service, request):
            if service.endswith("switch_controller") and request.activate_controllers:
                return harness._Msg(ok=False)
            return original(service, request)
        rig.robot.handle = refuse
        result = await rig.ts.switch_to(LOCOMOTION_CONTROLLER)
        assert not result.ok and rig.ts.fallen == LOCOMOTION_CONTROLLER
    run_case(case)


def test_locomotion_policy_must_support_tools_and_reference(tmp_path):
    from tricks import TRICK_DIR, validate_locomotion_contract
    root = ROBOT_DIR.parents[1]
    source = root / "pupper_gait_deploy/policies/default_policy.json"
    references = [load_tricks(TRICK_DIR)["jump"]]
    validate_locomotion_contract(source, references)
    payload = json.loads(source.read_text())
    payload["jump_slot"]["jump_table"][0][0] += 0.1
    path = tmp_path / "mismatch.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="reference"):
        validate_locomotion_contract(path, references)
    payload["command_clip"]["vx"] = [-0.5, 0.5]
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="training range"):
        validate_locomotion_contract(path, [])


def test_fetched_sit_descriptor_is_complete_and_spider_requires_evidence(tmp_path):
    import yaml
    from get_trick import write_template
    dest = tmp_path / "sit_stay_shake"
    dest.mkdir()
    write_template(dest, "sit_stay_shake", {"trick": {"clips": []}})
    spec = yaml.safe_load((dest / "trick.yaml").read_text())
    assert spec["exit_clip"] == "stand_up"
    assert set(spec["actions"]) == {"shake_left", "shake_right", "stand_up"}
    assert "TODO" not in json.dumps(spec)
    spider = tmp_path / "spider_walk"
    spider.mkdir()
    with pytest.raises(ValueError, match="simulation evaluation"):
        write_template(spider, "spider_walk", {}, walks=True)
