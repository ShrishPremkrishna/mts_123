"""The selected in-process walker/kick owner driven on one external field clock."""
import asyncio
from contextlib import asynccontextmanager
import json
from types import SimpleNamespace

import pytest

from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.model import Field
from test_operator_kick import selected_rig
from fc_walk_kick import CombinedRuntime
from robot_api import ESTOP_LOG, MotionBlockedError, Robot

ROBOT = "pupper-1"


@asynccontextmanager
async def field_clock_rig(monkeypatch):
    async with selected_rig(monkeypatch) as rig:
        # All sleepers, including native-service completion, wake on the same
        # clock as simulated frames. Real task scheduling never advances time.
        async def sleep(delay, result=None):
            end = rig.clock.now + delay
            while rig.clock.now < end:
                await asyncio.sleep(0)
            await asyncio.sleep(0)
            return result
        rig.clock.sleep = sleep
        rig.ts.follow_available = False
        yield rig


async def advance(rig, dt=.02):
    rig.clock.now += dt
    for callback in rig.clock.on_advance:
        callback()
    for _ in range(8):
        await asyncio.sleep(0)


def field_tick(runtime, now, seq, pose, *, kind="attack"):
    ex = runtime.executor
    event = {"now": now, "seq": seq, "epoch": "match", "clock": "plant"}
    target = {"target": {"x": -.72, "y": .05, "yaw": .1}} if kind == "move" else {}
    ex.handle({**event, "event": "intent", "robot_id": ROBOT, "kind": kind,
               "issued_at": now, "valid_until": now + .4, "intent_id": kind, **target})
    ex.handle({**event, "event": "frame", "captured_at": now,
               "robots": {ROBOT: {"x": pose[0], "y": pose[1], "yaw": 0}},
               "ball": {"x": 0., "y": 0.}, "goals": {"right": {"x": 1.83, "y": 0.}}})
    return runtime.tick(now)


@pytest.mark.parametrize("side,offset", [("left", -.040), ("right", .046)])
def test_one_runtime_drives_the_walker_then_real_queue_clip_then_walker(monkeypatch, side, offset):
    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            out = []
            ex = Executor(fields={ROBOT: Field()}, kick_side=side)
            runtime = CombinedRuntime(rig.ts, ex, ROBOT, velocity_sink=out.append,
                                      path_problem=lambda active: None)
            await runtime.open()
            assert not rig.robot.switches
            ex.arm("match", "plant", rig.ts.now())
            requests = []
            moved = False
            for i in range(800):
                p = (-.95, offset) if i < 10 else (-.258, offset)
                step = field_tick(runtime, rig.ts.now(), i, p)
                if i < 10:
                    moved |= out[-1].vx > 0.
                if step.request:
                    requests.append(step.request)
                if requests:
                    assert out[-1].vx == out[-1].vy == out[-1].wz == 0.
                if step.decision.phase == "kick_complete":
                    break
                await advance(rig)
            assert step.decision.phase == "kick_complete", (step, rig.ts.last_outcome, runtime.stop_reason)
            assert moved and len(requests) == 1
            assert len(rig.robot.activations("trick_leg_kick")) == 1
            assert rig.ts.last_outcome["operator_kick"]["confirmed_clip"] == "kick_" + side
            assert rig.robot.controllers["gemini_walk"] == "active"
            for seq in range(i + 1, i + 12):
                await advance(rig)
                field_tick(runtime, rig.ts.now(), seq, (-.258, offset), kind="move")
            assert any((out[-1].vx, out[-1].vy, out[-1].wz))
            await runtime.close()
            assert out[-1].vx == out[-1].vy == out[-1].wz == 0.
    asyncio.run(case())


def test_open_does_not_wake_an_inactive_walker(monkeypatch):
    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            rig.robot.controllers["gemini_walk"] = "inactive"
            ex = Executor(fields={ROBOT: Field()}, kick_side="right")
            runtime = CombinedRuntime(rig.ts, ex, ROBOT, velocity_sink=lambda cmd: None,
                                      path_problem=lambda active: None)
            with pytest.raises(RuntimeError, match="sole active"):
                await runtime.open()
            assert not rig.robot.switches
    asyncio.run(case())


def test_rejected_observer_disconnects_without_interrupting_the_current_operator(monkeypatch):
    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            rig.robot.controllers.update(gemini_walk="inactive", trick_leg_kick="active")
            counts = (len(rig.robot.published), len(rig.robot.switches), len(rig.robot.follow_calls))
            observer = Robot(validate_session_manifest=False, observe_only=True)
            ex = Executor(fields={ROBOT: Field()}, kick_side="left")
            runtime = CombinedRuntime(observer, ex, ROBOT, velocity_sink=lambda _: pytest.fail("guard wrote"),
                                      path_problem=lambda _: None, hard_stop=lambda _: pytest.fail("unclaimed Stop"))
            try:
                with pytest.raises(RuntimeError, match="sole active"):
                    await runtime.open()
                runtime.stop("failed observer")
                with pytest.raises(MotionBlockedError, match="unclaimed"):
                    observer.send_speed(0, 0, 0)
                with pytest.raises(MotionBlockedError, match="unclaimed"):
                    await observer._call(observer.follow_off, object())
                observer._on_rosout(SimpleNamespace(name="trick_leg_kick", msg=ESTOP_LOG))
                observer._on_estop(None)
                await asyncio.sleep(0)
                assert observer.fallen == "trick_leg_kick" and observer.motion_block_reason()
            finally:
                await observer.disconnect()
            assert observer.worker.done()
            assert rig.robot.controllers["trick_leg_kick"] == "active"
            assert (len(rig.robot.published), len(rig.robot.switches), len(rig.robot.follow_calls)) == counts
    asyncio.run(case())


@pytest.mark.parametrize("binding", ["during_blend", "two_ticks_after_receipt", "foreign"])
def test_field_loop_survives_expected_discovery_gap_and_waits_for_new_walker(monkeypatch, binding):
    from final_project.behaviors import combined_fc_runner as runner
    from final_project.behaviors.fc_executor_bridge import BridgeCore
    from final_project.behaviors.test_command_path_evidence import CLOCK, SOURCE, authority, graph, status

    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            robot = rig.ts
            fields = {ROBOT: Field(), "pupper-2": Field(attack_sign=-1)}
            core = BridgeCore(ROBOT, "camera", "test", fields, clock=robot.now,
                              command_contract=runner.DEADLINE_CONTRACT, command_clock_id=CLOCK,
                              monotonic_ns=lambda: round(robot.now() * 1e9), source_instance=SOURCE, kick_side="left")
            node, pubs, subs, endpoint = graph()
            pubs[runner.protocol.REQUEST_TOPIC] = [endpoint(runner.NAME)]
            pubs["/fc/obstacle_detections"] = [endpoint("viser_camera")]
            out = []
            path = runner.GuardedPath(core, node, lambda payload: out.append(runner.protocol.decode(payload)))
            runtime = CombinedRuntime(robot, core.executor, ROBOT, velocity_sink=path.write,
                                      path_problem=lambda active: (path.check(active) or (None, None))[1],
                                      walker_ready=path.restored_walker_ready)
            await runtime.open()
            core.executor.arm("match", "plant", robot.now())
            original = robot._call

            async def delayed_switch_reply(client, req, *args, **kwargs):
                reply = await original(client, req, *args, **kwargs)
                if client is robot.switch_client and "trick_leg_kick" in req.activate_controllers:
                    await robot.sleep(.06)  # DDS removes walker before the native switch response.
                return reply

            monkeypatch.setattr(robot, "_call", delayed_switch_reply)
            absent_before_entry = False
            terminal_at = None
            held_after_terminal = 0
            returned_ns = None
            for seq in range(650):
                now = robot.now()
                ns = round(now * 1e9)
                walking = rig.robot.controllers["gemini_walk"] == "active"
                owner = [endpoint("gemini_walk")] if walking else []
                pubs[runner.evidence.NATIVE_AUTHORITY_TOPIC] = owner
                subs[runner.protocol.ADMITTED_TOPIC] = owner
                path.guard_status(json.dumps(status(sequence=seq + 1, sampled_at_monotonic_ns=ns,
                    source_issued_mono_ns=ns-1_000_000, source_expires_mono_ns=ns+140_000_000)))
                if robot.last_outcome is not None and terminal_at is None:
                    terminal_at = seq
                if walking:
                    restored = bool(rig.robot.activations("trick_leg_kick"))
                    if restored and returned_ns is None:
                        returned_ns = ns
                    bound = SOURCE
                    if restored and binding != "during_blend":
                        bound = (SOURCE if binding != "foreign" else "b"*32
                                 ) if terminal_at is not None and seq >= terminal_at+2 else ""
                    path.native_status(json.dumps(authority(generation=3 if restored else 1,
                        activation_mono_ns=returned_ns if restored else 999_000_000_000,
                        sampled_mono_ns=ns, bound_source_instance=bound)))
                elif runtime.handoff.active and not runtime.handoff.clip_started:
                    absent_before_entry = True
                # The CLI spins subscriptions, syncs the guard, marks the active
                # clip's old generation, then advances its only output owner.
                in_kick = runtime.handoff.active is not None
                problem = path.check(in_kick)
                held = bool(problem and problem[0] == "camera")
                if (held or problem is None) and held != core.executor.external_holds[ROBOT]:
                    core.hold(held, problem[1] if held else "camera clear")
                if in_kick and runtime.handoff.clip_started:
                    path.clip_started()
                phase = field_tick(runtime, now, seq, (-.258, -.04)).decision.phase
                if terminal_at is not None and phase == "kick_in_flight":
                    held_after_terminal += 1
                    assert out[-1].is_zero
                if runtime.stop_reason or phase == "kick_complete":
                    break
                await advance(rig)
            assert absent_before_entry and returned_ns and terminal_at is not None
            if binding == "foreign":
                assert runtime.stop_reason and "another" in runtime.stop_reason
            else:
                assert phase == "kick_complete" and runtime.stop_reason is None
                if binding == "two_ticks_after_receipt":
                    assert held_after_terminal >= 2
                for i in range(1, 6):
                    await advance(rig)
                    field_tick(runtime, robot.now(), seq+i, (-.258, -.04), kind="move")
                assert not out[-1].is_zero and path.last_walk_generation == 3
            await runtime.close()
    asyncio.run(case())


@pytest.mark.parametrize("interruption", ["expired", "ball_moved", "stop_after_entry"])
def test_field_veto_at_native_entry_and_during_playback(monkeypatch, interruption):
    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            ex = Executor(fields={ROBOT: Field()}, kick_side="left")
            out = []
            active = []
            hard_stops = []
            runtime = CombinedRuntime(rig.ts, ex, ROBOT, velocity_sink=out.append,
                                      path_problem=lambda in_kick: None, hard_stop=hard_stops.append)
            await runtime.open()
            ex.arm("match", "plant", rig.ts.now())
            permission = asyncio.Event()
            original = rig.ts._set_start_clip

            async def delayed(*args, **kwargs):
                result = await original(*args, **kwargs)
                await permission.wait()
                return result
            if interruption != "stop_after_entry":
                monkeypatch.setattr(rig.ts, "_set_start_clip", delayed)
            ticket = None
            for seq in range(150):
                now = rig.ts.now()
                # Change the pose while the native service is awaiting clip
                # parameters, keeping a current field clock and live intent.
                place = (-.7, -.04) if ticket and interruption == "ball_moved" else (-.258, -.04)
                if interruption == "stop_after_entry" and rig.robot.activations("trick_leg_kick"):
                    ex.set_external_hold(ROBOT, True, now, "person in path")
                step = field_tick(runtime, now, seq, place)
                if step.request is not None:
                    ticket = step.request
                if ticket and ((interruption == "expired" and now > ticket.start_before)
                               or (interruption == "ball_moved" and place[0] == -.7)):
                    permission.set()
                await advance(rig)
                active.append(rig.robot.controllers.get("trick_leg_kick"))
                if runtime.stop_reason and seq > 80:
                    break
            assert ticket is not None
            assert runtime.stop_reason and not ex.armed
            assert out[-1].vx == out[-1].vy == out[-1].wz == 0
            if interruption == "stop_after_entry":
                assert "active" in active
                assert rig.robot.controllers["trick_leg_kick"] != "active"
                assert not rig.robot.activations("gemini_walk")
            else:
                assert not rig.robot.activations("trick_leg_kick")
            await runtime.close()
            assert hard_stops == [runtime.stop_reason]
    asyncio.run(case())


def test_recoverable_camera_hold_does_not_latch_gateway(monkeypatch):
    async def case():
        async with field_clock_rig(monkeypatch) as rig:
            ex = Executor(fields={ROBOT: Field()}, kick_side="left")
            stops = []
            runtime = CombinedRuntime(rig.ts, ex, ROBOT, velocity_sink=lambda _: None,
                                      path_problem=lambda _: None, hard_stop=stops.append)
            await runtime.open()
            now = rig.ts.now()
            ex.arm("match", "plant", now)
            ex.set_external_hold(ROBOT, True, now, "person in front")
            assert field_tick(runtime, now, 1, (-.95, -.04)).decision.phase == "external_hold"
            assert ex.armed and not stops
            await runtime.close()
            assert len(stops) == 1
    asyncio.run(case())
