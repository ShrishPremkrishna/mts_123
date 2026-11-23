"""Software-only live-source contract and closed-loop approach checks."""
from dataclasses import asdict, replace
import math

import pytest

from final_project.behaviors import fc_executor_bridge as bridge
from final_project.behaviors import live_field_attack as attack
from final_project.behaviors import overhead_ingress as ingress
from final_project.overhead_perception.fieldcam.net import REPLY_PROTOCOL, STATE_PROTOCOL
from final_project.pupper_fc.kick_handoff import KickHandoff
from final_project.pupper_fc.model import Field, zero
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, PAW_LEFT_M
from final_project.pupper_fc.simulation import Simulator, SimulatedKick

ROBOT = "pupper-1"
FIELD = Field(length=2.7482693750646883, width=1.442886155351121, goal_width=.6096, ball_radius=.11)


def core_for(field):
    return ingress.IngressCore(source_id="fieldcam", source_session="test-camera", producer_id="test-ingress",
        epoch="attempt", boot_id="test-boot", robot_map={"striker": ROBOT}, calibration_revision=1,
        field=asdict(field))


def sync(source, now):
    probe = source.make_probe(now - .002)
    source.accept_probe({**source.identity(), "nonce": probe["nonce"], "protocol": REPLY_PROTOCOL,
                         "received_mono": 10 + now - .001, "sent_mono": 10 + now - .001}, now)


def packet(source, observation, field, seq, now):
    # Model a frame delivered 20 ms after camera capture. No ROS publisher.
    stamp = observation.captured_at + 1000.
    row = {"visible": True, "measured_this_frame": True, "measured_at": stamp, "age_s": 0.}
    return {"seq": seq, "calibration_valid": True, "calibration_revision": 1, "field": asdict(field),
            "captured_at": stamp, "stamp": stamp, "frame_age_s": 0.,
            "robots": {"striker": {**asdict(observation.robot), **row, "vx": 0., "vy": 0., "wz": 0.}},
            "ball": {**asdict(observation.ball), **row},
            "goals": {name: {"x": sign * field.length / 2, "y_min": -field.goal_width / 2,
                             "y_max": field.goal_width / 2} for name, sign in (("east", 1), ("west", -1))},
            "_transport": {**source.identity(), "protocol": STATE_PROTOCOL, "wire_seq": seq,
                           "clock_valid": True, "live": True, "sent_mono": 10 + now,
                           "sent_wall": 1000 + now, "clock_start_wall": 1000., "pair_span_s": 0.}}


def test_original_capture_and_loss_bound_each_lease():
    source = core_for(FIELD)
    sim = Simulator(FIELD)
    sim.step(zero(0.), .1)
    sync(source, sim.time)
    raw = packet(source, sim.observe(delay=.02), FIELD, 1, sim.time)
    normalized = source.normalize(raw, sim.time)
    play = attack.LiveFieldPlay(ROBOT, FIELD, ingress.MAX_FRAME_AGE_S)
    def lease(frame, now=sim.time):
        return play.plan(frame, now)[0]
    first = lease(normalized)
    assert first["issued_at"] == normalized["captured_at"] < sim.time
    assert first["valid_until"] == normalized["captured_at"] + ingress.MAX_FRAME_AGE_S
    assert first["epoch"] == source.epoch and first["seq"] == normalized["seq"]
    assert lease(normalized, now=first["valid_until"] + .001) is None
    assert lease({**normalized, "goals": {"left": normalized["goals"]["left"]}}) is None
    for entity in ("striker", "ball"):
        sim.step(zero(sim.time), .04)
        raw = packet(source, sim.observe(delay=.02), FIELD, raw["seq"] + 1, sim.time)
        (raw["ball"] if entity == "ball" else raw["robots"]["striker"])["measured_this_frame"] = False
        assert lease(source.normalize(raw, sim.time), sim.time) is None
    revoked = source.revoke("signal lost")
    match = attack.LiveMatchPlay(ROBOT, FIELD, ingress.MAX_FRAME_AGE_S, source.epoch, [ROBOT])
    for mode in (play, match):
        outgoing, intent, reason = mode.events(revoked, sim.time)
        assert intent is None and outgoing["captured_at"] is None
        assert outgoing["source_reason"] == reason and reason


@pytest.mark.parametrize("side,sign", [("left", 1), ("right", -1)])
def test_live_pairs_steer_an_offset_approach_then_dispatch_exactly_one_selected_paw(side, sign):
    field = replace(FIELD, attack_sign=sign)
    sim = Simulator(field, bridge.COMBINED_LIMITS, scenario="offset", bumper_contact=False)
    sim.step(zero(0.), .05)
    source = core_for(field)
    runner = bridge.BridgeCore(ROBOT, source.producer_id, "test-boot",
        {ROBOT: field, "pupper-2": replace(field, attack_sign=-sign)},
        clock=lambda: sim.time, kick_side=side, paw_forward_m=COMBINED_PAW_FORWARD_M[side])
    handoff = KickHandoff(ROBOT)
    backend = SimulatedKick(handoff)
    play = attack.LiveFieldPlay(ROBOT, field, ingress.MAX_FRAME_AGE_S)
    requests = []
    observed_curve = False
    terminal_ticks = 0
    for tick in range(1700):
        if tick % 20 == 0:
            sync(source, sim.time)
        if tick % 2 == 0:
            raw = packet(source, sim.observe(delay=.02), field, tick, sim.time)
            frame = source.normalize(raw, sim.time)
            frame, intent, _ = play.events(frame, sim.time)
            if tick == 0:
                assert not runner.ingest(frame)["accepted"] and not runner.ingest(intent)["accepted"]
                assert not runner.executor.armed
                assert runner.operator({"command": "arm", "epoch": source.epoch})["accepted"]
            assert runner.ingest(frame)["accepted"] and runner.ingest(intent)["accepted"]
        step = runner.executor.kick_step(handoff, sim.time)
        command = step.decision.command
        observed_curve |= abs(command.vx) > .02 and abs(command.wz) > .02
        if step.request:
            job = step.request
            assert job.side == side and job.attempt_id == "live-field-one-shot"
            # Independently measure where the observed ball lies relative to this foot.
            yaw = sim.robot.yaw
            dx, dy = sim.ball.x - sim.robot.x, sim.ball.y - sim.robot.y
            assert dx * math.cos(yaw) + dy * math.sin(yaw) == pytest.approx(COMBINED_PAW_FORWARD_M[side], abs=.020)
            assert -dx * math.sin(yaw) + dy * math.cos(yaw) == pytest.approx(PAW_LEFT_M[side], abs=.015)
            goal_yaw = math.atan2(-sim.ball.y, sign * field.length / 2 - sim.ball.x)
            assert abs(math.remainder(goal_yaw - yaw, math.tau)) < .075
            requests.append(job)
        backend.update(step, sim.time)
        if handoff.active or step.decision.phase == "kick_complete":
            assert (command.vx, command.vy, command.wz) == (0., 0., 0.)
        if step.decision.phase == "kick_complete":
            terminal_ticks += 1
            if terminal_ticks >= 20:
                break
        sim.step(command, .02)
    assert observed_curve and len(requests) == 1 and terminal_ticks == 20


def test_approach_only_follows_ball_clear_standoff_and_withdraws_before_the_old_move_expires():
    sim = Simulator(FIELD, bridge.COMBINED_LIMITS, scenario="offset", bumper_contact=False)
    sim.step(zero(0.), .05)
    source = core_for(FIELD)
    play = attack.LiveFieldPlay(ROBOT, FIELD, ingress.MAX_FRAME_AGE_S, approach_side="right")
    runner = bridge.BridgeCore(ROBOT, source.producer_id, "test-boot",
        {ROBOT: FIELD, "pupper-2": replace(FIELD, attack_sign=-1)}, clock=lambda: sim.time,
        kick_side="right", paw_forward_m=COMBINED_PAW_FORWARD_M["right"])
    assert runner.operator({"command": "arm", "epoch": source.epoch})["accepted"]
    handoff = KickHandoff(ROBOT)
    curved = held = False
    goals = []
    for tick in range(900):
        if tick % 20 == 0:
            sync(source, sim.time)
        if tick == 60:
            sim.move_ball(sim.ball.x + .15, sim.ball.y - .10)
        if tick % 2 == 0:
            raw = packet(source, sim.observe(delay=.02), FIELD, tick, sim.time)
            if tick == 30:
                raw["ball"]["measured_this_frame"] = False
            normalized = source.normalize(raw, sim.time)
            frame, intent, reason = play.events(normalized, sim.time)
            accepted = runner.ingest(frame)["accepted"]
            if tick == 30:
                assert sim.time < runner.executor.intents[ROBOT].valid_until
                assert intent is None and not accepted and runner.executor.frame is None
            else:
                assert accepted and intent["kind"] == "move", reason
                assert intent["target"] and runner.ingest(intent)["accepted"]
                goals.append(intent["target"])
        step = runner.executor.kick_step(handoff, sim.time)
        assert step.request is None and handoff.active is None
        cmd = step.decision.command
        curved |= abs(cmd.vx) > .02 and abs(cmd.wz) > .02
        if tick == 30:
            assert not any((cmd.vx, cmd.vy, cmd.wz))
            held = True
        if tick > 62 and step.decision.phase == "target_reached":
            break
        sim.step(cmd, .02)
    assert held and curved and step.decision.phase == "target_reached"
    assert math.hypot(goals[0]["x"] - goals[-1]["x"], goals[0]["y"] - goals[-1]["y"]) > .10
    assert math.hypot(sim.robot.x - sim.ball.x, sim.robot.y - sim.ball.y) >= .36


@pytest.mark.parametrize("robot_xy,ball_xy,expected", [
    ((.6, 0.), (0., 0.), "enter ball clearance"),
    ((-.6, 0.), (-1.25, 0.), "standoff falls outside"),
    ((-.6, .6), (0., 0.), "robot body or ball centre outside"),
    ((-.6, 0.), (1.38, 0.), "robot body or ball centre outside"),
])
def test_standoff_never_leases_cross_ball_or_out_of_bounds_moves(robot_xy, ball_xy, expected):
    now = 3.
    frame = {"event": "frame", "seq": 7, "epoch": "first", "producer_id": "perception",
             "clock": "boot", "captured_at": now,
             "robots": {ROBOT: dict(x=robot_xy[0], y=robot_xy[1], yaw=0., captured_at=now)},
             "ball": dict(x=ball_xy[0], y=ball_xy[1], vx=0., vy=0.),
             "goals": {"right": {"x": FIELD.length/2, "y": 0.}}}
    play = attack.LiveFieldPlay(ROBOT, FIELD, ingress.MAX_FRAME_AGE_S, "right")
    withdraw, intent, reason = play.events(frame, now)
    assert intent is None and expected in reason
    assert withdraw["captured_at"] is None and withdraw["seq"] == frame["seq"]
    assert withdraw["source_reason"] == reason
