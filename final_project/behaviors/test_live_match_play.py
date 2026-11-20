"""Continuous play through the deployed field/source/runner interfaces with memory-only actuators."""
from dataclasses import asdict, replace
import json
import math

import pytest

from final_project.behaviors import combined_fc_runner as combined
from final_project.behaviors import fc_executor_bridge as bridge
from final_project.behaviors import live_field_attack as attack
from final_project.behaviors import overhead_ingress as ingress
from final_project.behaviors.test_live_field_attack import FIELD, packet, sync
from final_project.pupper_fc.controller import HOME_FRACTION
from final_project.pupper_fc.kick_handoff import KickHandoff
from final_project.pupper_fc.match_director import MatchDirector
from final_project.pupper_fc.model import zero
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M
from final_project.pupper_fc.simulation import SimulatedKick, Simulator


BLUE, RED = "pupper-1", "pupper-2"
SIGNS = {BLUE: 1, RED: -1}
TRACKS = {"striker": BLUE, "keeper": RED}
EPOCH = "league-game"
GOALS = {name: {"x": sign * FIELD.length / 2, "y_min": -.20 + .04 * sign, "y_max": .40 + .04 * sign}
         for name, sign in (("west", -1), ("east", 1))}


class LocalPlayer:
    def __init__(self, robot, tracks, clock):
        field = replace(FIELD, attack_sign=SIGNS[robot])
        self.source = ingress.IngressCore(source_id="fieldcam", source_session="camera",
            producer_id="receiver-" + robot, epoch=EPOCH, boot_id="boot-" + robot,
            robot_map=tracks, calibration_revision=1, field=asdict(field))
        self.play = attack.LiveMatchPlay(robot, field, .30, EPOCH, tracks.values())
        fields = {r: replace(FIELD, attack_sign=s) for r, s in SIGNS.items()}
        self.runner = bridge.BridgeCore(robot, self.source.producer_id, "boot-" + robot, fields, clock=clock,
                                      kick_side="right", paw_forward_m=COMBINED_PAW_FORWARD_M["right"])
        self.handoff = KickHandoff(robot)
        self.backend = SimulatedKick(self.handoff)
        assert self.runner.operator({"command": "arm", "epoch": EPOCH})["accepted"]

    def receive(self, raw, now):
        frame = self.source.normalize(raw, now)
        frame, intent, reason = self.play.events(frame, now)
        result = self.runner.ingest(frame)
        if intent is not None:
            assert result["accepted"], result
            assert self.runner.ingest(intent)["accepted"]
        return frame, intent, reason


def test_repeated_offense_requires_native_completion_and_new_measured_ball():
    tracks = {"striker": BLUE}
    director = MatchDirector(EPOCH, tracks, SIGNS)
    sim = Simulator(FIELD, bridge.COMBINED_LIMITS, scenario="offset", bumper_contact=False)
    sim.step(zero(0.), .05)
    player = LocalPlayer(BLUE, tracks, lambda: sim.time)
    requests, completions = [], []
    after_return = None
    walked_between = False
    for tick in range(2100):
        now = sim.time
        if tick % 20 == 0:
            sync(player.source, now)
        if tick % 2 == 0:
            raw = packet(player.source, sim.observe(delay=.02), FIELD, tick, now)
            raw["fc_match"] = director.update(raw, now=1000. + now)
            assert raw["fc_match"]["attacker"] == BLUE
            frame, intent, reason = player.receive(raw, now)
            if after_return is not None and intent and intent["intent_id"] != requests[-1]:
                assert frame["captured_at"] > after_return
        prior = player.handoff.active
        step = player.runner.executor.kick_step(player.handoff, now)
        edge = combined.kick_event(prior, player.handoff, step, now)
        receipt = edge if edge and edge["state"] == "completed" else None
        if step.request:
            engaged = edge
            assert engaged["state"] == "engaged"
            assert director.observe_kick(engaged) and player.play.completion(engaged, now)
            requests.append(step.request.attempt_id)
            assert requests.count(step.request.attempt_id) == 1
        if receipt:
            assert receipt["attempt_id"] == requests[-1]
            assert not player.play.completion({**receipt, "completed_at": player.play.attempt_capture - .01}, now)
            assert not player.play.completion({**receipt, "epoch": "other"}, now)
            assert player.play.completion(receipt, now)
            assert not player.play.completion(receipt, now)
            assert director.observe_kick(receipt) and not director.observe_kick(engaged)
            completions.append(receipt["attempt_id"])
            after_return = now
            if len(completions) == 2:
                break
            # Both clip receipts are synthetic. Place the ball at a new observed point;
            # the simulator has no paw impact model.
            sim.move_ball(sim.ball.x + .27, sim.ball.y + .12)
        if len(completions) == 1 and step.decision.command.vx > .02:
            walked_between = True
        player.backend.update(step, now)
        sim.step(step.decision.command, .02)
    assert requests == completions == ["r1-p0-k1", "r1-p0-k2"]
    assert walked_between


def test_one_director_two_distinct_pi_clocks_roles_and_independent_goal_return():
    director = MatchDirector(EPOCH, TRACKS, SIGNS, goal_lines=GOALS)
    time_base = [0.1]
    offsets = {BLUE: 0., RED: 25.}
    players = {r: LocalPlayer(r, TRACKS, lambda offset=offsets[r]: time_base[0] + offset) for r in SIGNS}
    sim = Simulator(FIELD)
    seq = 0
    home = {r: -s * HOME_FRACTION * FIELD.length for r, s in SIGNS.items()}

    def update(x, y, *, blue_x=None, red_x=None, skip=(), measured=True):
        nonlocal seq
        seq += 1
        while sim.time < .04 * seq + .08:
            sim.step(zero(sim.time), .02)
        now = time_base[0] = sim.time
        raw = packet(players[BLUE].source, sim.observe(delay=.02), FIELD, seq, now)
        raw["ball"].update(x=x, y=y, vx=0., vy=0., measured_this_frame=measured)
        raw["robots"]["striker"].update(x=home[BLUE] if blue_x is None else blue_x, y=0., yaw=0.)
        raw["robots"]["keeper"] = {**raw["robots"]["striker"], "x":home[RED] if red_x is None else red_x, "yaw":math.pi}
        raw["fc_match"] = director.update(raw, now=now+1000.)
        result = {}
        for robot, player in players.items():
            if robot in skip:
                continue
            # One camera packet; each Pi reconciles to its own OS uptime.
            local = now + offsets[robot]
            probe = player.source.make_probe(local - .002)
            player.source.accept_probe({**player.source.identity(), "nonce": probe["nonce"],
                "protocol": ingress.REPLY_PROTOCOL, "received_mono": 10 + now - .001,
                "sent_mono": 10 + now - .001}, local)
            frame, intent, reason = player.receive(json.loads(json.dumps(raw)), local)
            step = player.runner.executor.kick_step(player.handoff, local)
            assert step.request is None
            result[robot] = (frame, intent, step)
        return raw["fc_match"], result

    state, rows = update(-.30, .32)
    assert state["attacker"] == BLUE
    assert rows[BLUE][0]["goals"]["right"]["y"] == pytest.approx(.14)
    assert rows[RED][1]["kind"] == "defend"
    assert rows[RED][1]["target"]["y"] == pytest.approx(GOALS["east"]["y_max"] - FIELD.robot_radius)
    engaged = {"state": "engaged", "epoch": EPOCH, "robot_id": BLUE, "attempt_id": rows[BLUE][1]["intent_id"]}
    assert director.observe_kick(engaged)
    for _ in range(11):
        state, rows = update(.30, .32)
        assert state["attacker"] == BLUE
    assert not director.observe_kick({**engaged, "state": "completed", "robot_id": RED})
    assert director.observe_kick({**engaged, "state": "completed"})
    # A challenger must maintain its lead. Neither runner gets movement authority during handover.
    for i in range(13):
        state, rows = update(.30, .32)
        if i < 6:
            assert state["phase"] == "holding" and all(record[1] is None for record in rows.values())
    assert state["attacker"] == RED and rows[BLUE][1]["kind"] == "defend"
    east = GOALS["east"]["x"]
    for dx in (-.22, -.15, -.08, 0., .08, .13):
        state, rows = update(east + dx, .20, skip=(RED,) if dx > .06 else ())
    assert state["phase"] == "goal" and state["score"] == {BLUE: 1, RED: 0}
    with pytest.raises(ValueError, match="reset"):
        director.resume(now=time_base[0]+1000.)
    for _ in range(4):
        state, rows = update(east + .14, .2)
    assert state["score"][BLUE] == 1 and rows[RED][0]["robots"][RED]["goal"] == -1
    for _ in range(4):
        state, rows = update(0., 0., red_x=.30)
    assert state["reset_ready"]
    def resume(robot):
        p = players[robot]
        return p.runner.operator({"command": "resume", "frame_seq": p.runner.executor.frame_seq})["accepted"]
    assert resume(BLUE) and not resume(RED)
    state, rows = update(0., 0., red_x=.30)
    assert rows[BLUE][1]["kind"] == "move" and rows[BLUE][2].decision.command.vx == 0.
    for _ in range(3):
        state, rows = update(0., 0.)
    assert resume(RED)
    director.resume(now=time_base[0]+1000.)
    state, rows = update(-.4, 0.)
    assert state["phase"] == "playing" and state["rally"] == 2
    assert rows[BLUE][1]["intent_id"].startswith("r2-") and rows[RED][1]["kind"] == "defend"


@pytest.mark.parametrize("fault", ["graze", "missing", "jump", "new calibration"])
def test_no_goal_from_incomplete_or_discontinuous_crossing(fault):
    director = MatchDirector(EPOCH, {"striker": BLUE}, SIGNS, goal_lines=GOALS)
    sim = Simulator(FIELD)
    player = LocalPlayer(BLUE, director.tracks, lambda: sim.time)
    for seq, dx in enumerate((-.20, -.13, -.04, .04, .09, .14, .18)):
        sim.step(zero(sim.time), .04)
        raw = packet(player.source, sim.observe(delay=.02), FIELD, seq, sim.time)
        raw["ball"].update(x=GOALS["east"]["x"]+dx, y=.40 if fault == "graze" else .1, vx=0., vy=0.)
        if seq == 2:
            if fault == "missing":
                raw["ball"]["measured_this_frame"] = False
            if fault == "jump":
                raw["ball"]["y"] += .80
            if fault == "new calibration":
                raw["calibration_revision"] = 2
        director.update(raw, now=raw["stamp"]+.02)
    assert director.state["phase"] != "goal" and not any(director.score.values())



def test_warm_end_and_resume_ticket_use_the_owning_runner():
    from types import SimpleNamespace
    from final_project.behaviors.operator_drive import OperatorDrive
    player = LocalPlayer(BLUE, {"striker": BLUE}, lambda: 1.)
    core = player.runner
    controls = OperatorDrive(core)
    controls.observe_path(True, 1)
    sent = []
    robot = SimpleNamespace(current=None, commands=SimpleNamespace(empty=lambda:True))
    runtime = SimpleNamespace(robot=robot,handoff=player.handoff,velocity_sink=lambda cmd:sent.append(cmd),stop_reason=None)
    runtime.stop = lambda reason: setattr(runtime,"stop_reason",reason)
    def action(command, ticket):
        return combined.operator_action({"command":command,"robot_id":BLUE,"ticket":ticket},core,None,runtime,None,controls)
    ticket = controls.offer(.9)["ticket"]
    assert action("pause", ticket)["state"] == "standby"
    assert not core.executor.armed and sent[-1].vx == sent[-1].vy == sent[-1].wz == 0
    assert not core.operator({"command":"arm","epoch":EPOCH})["accepted"]
    assert core.operator({"command":"arm","epoch":"game-two"})["accepted"]
    cycle = ("game-two", -1)
    fresh = OperatorDrive(core)
    fresh.observe_path(True,1)
    early = fresh.offer(.81)["ticket"]
    with pytest.raises(ValueError,match="not issued ready"):
        fresh.consume_ticket({"command":"resume","robot_id":BLUE,"ticket":early},1.,resume_cycle=cycle)
    ready = fresh.offer(.92,cycle)
    assert ready["resume_ready"]
    assert fresh.consume_ticket({"command":"resume","robot_id":BLUE,"ticket":ready["ticket"]},1.,resume_cycle=cycle)>1.
    controls.next_issue=0.
    robot.current=object()
    assert action("pause", controls.offer(1.)["ticket"])["state"] == "hard_stop"
    assert runtime.stop_reason



def test_two_phase_kickoff_cannot_move_an_early_armed_robot_or_score_a_pending_goal():
    sim = Simulator(FIELD)
    teams = {r: LocalPlayer(r,TRACKS,lambda:sim.time) for r in SIGNS}
    director = MatchDirector(EPOCH,TRACKS,SIGNS,pending_start=True)
    # Blue admitted its local arm first. Red deliberately remains disarmed.
    red = teams[RED]
    red.runner.executor.stop()
    fresh = False
    for sequence in range(10):
        sim.step(zero(sim.time),.04)
        raw = packet(teams[BLUE].source, sim.observe(delay=.02), FIELD, sequence, sim.time)
        raw["robots"]["keeper"] = {**raw["robots"]["striker"],"x":.66}
        raw["ball"].update(x=FIELD.length/2-.20+sequence*.075,y=0.)
        raw["fc_match"] = director.update(raw,now=1000+sim.time)
        assert raw["fc_match"]["phase"] == "pending" and raw["fc_match"]["attacker"] is None
        for robot, team in teams.items():
            sync(team.source,sim.time)
            normalized = team.source.normalize(raw,sim.time)
            frame, intent, _ = team.play.events(normalized,sim.time)
            assert intent is None
            if team.play.ready_to_arm:
                fresh = True
                assert frame["captured_at"] is not None
            team.runner.ingest(frame)
            step = team.runner.executor.kick_step(team.handoff,sim.time)
            assert step.request is None and not any((step.decision.command.vx,step.decision.command.vy,step.decision.command.wz))
    assert fresh and director.score == {BLUE:0,RED:0}
    with pytest.raises(ValueError,match="fresh"):
        director.commit_start(now=1000+sim.time)
    # Operator starts a second independent lobby transaction after the declined partial arm.
    second = MatchDirector("game-two", TRACKS,SIGNS,pending_start=True)
    for team in teams.values():
        team.runner.executor.stop()
        assert team.runner.operator({"command":"arm","epoch":"game-two"})["accepted"]
        team.source.epoch = "game-two"
        team.play = attack.LiveMatchPlay(team.runner.robot_id,team.play.field,.30,"game-two",TRACKS.values())
    sim.step(zero(sim.time),.04)
    raw = packet(teams[BLUE].source, sim.observe(delay=.02), FIELD, 12,sim.time)
    raw["robots"]["keeper"] = {**raw["robots"]["striker"],"x":.66}
    raw["fc_match"] = second.update(raw,now=1000+sim.time)
    for team in teams.values():
        _, intent, _ = team.receive(raw,sim.time)
        assert intent is None and team.play.ready_to_arm
    with pytest.raises(ValueError,match="fresh"):
        second.commit_start(now=1000+sim.time+1.)
    assert second.commit_start(now=1000+sim.time)["phase"] == "holding"
    sim.step(zero(sim.time),.04)
    raw = packet(teams[BLUE].source, sim.observe(delay=.02), FIELD, 13,sim.time)
    raw["robots"]["keeper"] = {**raw["robots"]["striker"],"x":.66}
    raw["fc_match"] = second.update(raw,now=1000+sim.time)
    kinds = [team.receive(raw,sim.time)[1]["kind"] for team in teams.values()]
    assert sorted(kinds) == ["attack","defend"]


def test_first_runner_frame_after_ball_replacement_recovers_a_missed_goal_pulse():
    team = LocalPlayer(RED, TRACKS, lambda: .22)
    source = team.source
    sim = Simulator(FIELD)
    for seq in (1, 2):
        sim.step(zero(sim.time), .10)
        sync(source, sim.time)
        raw = packet(source, sim.observe(delay=.02), FIELD, seq, sim.time)
        raw["robots"]["keeper"] = {**raw["robots"]["striker"],"x":.82,"y":0.,"yaw":math.pi}
        director = MatchDirector(EPOCH, TRACKS, SIGNS, goal_lines=GOALS)
        info = director.update(raw, now=sim.time+1000.)
        raw["fc_match"] = {**info,"phase":"goal","attacker":None,"goal_sign":1,"reset_ready":seq == 2}
        normalized, intent, _ = team.play.events(source.normalize(raw,sim.time), sim.time)
        if seq == 2:
            assert normalized["robots"][RED]["goal"] == 0
            assert team.runner.ingest(normalized)["accepted"] and team.runner.ingest(intent)["accepted"]
    assert team.runner.executor.kick_step(team.handoff,.22).decision.phase == "await_reset"
    assert team.runner.operator({"command":"resume","frame_seq":team.runner.executor.frame_seq})["accepted"]
