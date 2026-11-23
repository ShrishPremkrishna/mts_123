"""Version-2 match: one selected pitch, independent roles, per-robot kicks, shared referee.

Both robots run the deployed seams: pinned ingress -> LiveMatchPlay -> BridgeCore
(Executor, KickHandoff, KickReporter) with memory-only actuators. The plant is
the approximate 2D sandbox with two bodies and one ball. Its "kick" is a
synthetic ball impulse applied when the simulated clip is confirmed; it stands in
for an unmodelled paw and is not contact evidence. No physical claim.
"""
from dataclasses import asdict, replace
import json
import math

import pytest

from final_project.behaviors import combined_fc_runner as combined
from final_project.behaviors import fc_executor_bridge as bridge
from final_project.behaviors import live_field_attack as attack
from final_project.behaviors import overhead_ingress as ingress
from final_project.overhead_perception.fieldcam.net import REPLY_PROTOCOL, STATE_PROTOCOL
from final_project.overhead_perception.fieldcam.pitch import Pitch
from final_project.pupper_fc.controller import peer_clearance
from final_project.pupper_fc.kick_handoff import KickHandoff
from final_project.pupper_fc.match_director import MatchDirector, directive_ok
from final_project.pupper_fc.model import Field, zero
from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M
from final_project.pupper_fc.simulation import SimulatedKick, Simulator

BLUE, RED = "pupper-1", "pupper-2"
SIGNS = {BLUE: 1, RED: -1}
TRACKS = {"striker": BLUE, "keeper": RED}
EPOCH = "independent-game"
MARKERS = dict(length=3., width=2., goal_width=.6096, ball_radius=.11)
GOALS = {"west": dict(x=-1.80, y_min=-.20, y_max=.41), "east": dict(x=1.75, y_min=-.40, y_max=.21)}
PITCH = Pitch.from_field(MARKERS, GOALS).record()
ENROLLED = Field(**MARKERS)
SELECTED = ENROLLED.with_pitch(PITCH)
OFFSETS = {BLUE: 0., RED: 25.}  # each Pi has its own monotonic clock


class ImpulseKick(SimulatedKick):
    """Synthetic: on clip confirmation push a ball that sits on the paw along the body heading."""

    def __init__(self, handoff, sim, robot_id, speed=1.3):
        super().__init__(handoff)
        self.sim, self.robot_id, self.speed = sim, robot_id, speed
        self.impulses = 0

    def update(self, step, now):
        confirmed = self.clip_confirmed
        super().update(step, now)
        if self.clip_confirmed and not confirmed:
            pose = self.sim.robot if self.robot_id == BLUE else self.sim.others[self.robot_id][0]
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            dx, dy = self.sim.ball.x - pose.x, self.sim.ball.y - pose.y
            if .15 <= dx * c + dy * s <= .40 and abs(-dx * s + dy * c) <= .12:
                self.sim.nudge_ball(self.speed * c, self.speed * s)
                self.impulses += 1


class Player:
    def __init__(self, robot, sim, roster, *, solo_roles="offense"):
        self.robot, self.offset = robot, OFFSETS[robot]
        field = replace(SELECTED, attack_sign=SIGNS[robot])
        tracks = {name: r for name, r in TRACKS.items() if r in roster}
        self.source = ingress.IngressCore(source_id="fieldcam", source_session="camera",
            producer_id="receiver-" + robot, epoch=EPOCH, boot_id="boot-" + robot, robot_map=tracks,
            calibration_revision=1, field=MARKERS, pitch=PITCH)
        self.play = attack.LiveMatchPlay(robot, field, .30, EPOCH, roster, solo_roles=solo_roles)
        self.clock = [0.]
        fields = {r: replace(ENROLLED, attack_sign=s) for r, s in SIGNS.items()}
        self.runner = bridge.BridgeCore(robot, self.source.producer_id, "boot-" + robot, fields,
                                        clock=lambda: self.clock[0], kick_side="right",
                                        paw_forward_m=COMBINED_PAW_FORWARD_M["right"])
        self.handoff = KickHandoff(robot)
        self.kicks = combined.KickReporter(self.handoff)
        self.backend = ImpulseKick(self.handoff, sim, robot)
        assert self.runner.operator({"command": "arm", "epoch": EPOCH, "pitch": PITCH})["accepted"]

    def sync(self, now):
        local = now + self.offset
        probe = self.source.make_probe(local - .002)
        self.source.accept_probe({**self.source.identity(), "nonce": probe["nonce"], "protocol": REPLY_PROTOCOL,
                                  "received_mono": 10 + now - .001, "sent_mono": 10 + now - .001}, local)

    def receive(self, raw, now):
        self.clock[0] = local = now + self.offset
        frame = self.source.normalize(json.loads(json.dumps(raw)), local)
        frame, intent, reason = self.play.events(frame, local)
        result = self.runner.ingest(frame)
        if intent is not None:
            assert result["accepted"], (self.robot, result, reason)
            assert self.runner.ingest(intent)["accepted"], (self.robot, intent)
        return frame, intent, reason

    def step(self, now):
        self.clock[0] = local = now + self.offset
        step = self.runner.executor.kick_step(self.handoff, local)
        return step, self.kicks.observe(step, local), local


def camera(sim, seq, now, *, visible=(BLUE, RED), pitch=PITCH, revision=1):
    obs, peers = sim.observe(delay=.02), sim.peers(delay=.02)
    stamp = obs.captured_at + 1000.
    row = {"visible": True, "measured_this_frame": True, "measured_at": stamp, "age_s": 0.,
           "vx": 0., "vy": 0., "wz": 0.}
    poses = {BLUE: obs.robot, RED: peers.get(RED)}
    robots = {name: {**asdict(poses[robot]), **row} for name, robot in TRACKS.items()
              if robot in visible and poses[robot] is not None}
    selected = Pitch.read(pitch)
    return {"seq": seq, "calibration_valid": True, "calibration_revision": revision, "field": MARKERS,
            "pitch": pitch, "goals": selected.goals, "captured_at": stamp, "stamp": stamp, "frame_age_s": 0.,
            "robots": robots, "ball": {**asdict(obs.ball), **{k: v for k, v in row.items() if k != "wz"},
                                       "vx": obs.ball.vx, "vy": obs.ball.vy},
            "_transport": {"source_id": "fieldcam", "source_session": "camera", "clock_id": "monotonic:camera",
                           "protocol": STATE_PROTOCOL, "wire_seq": seq, "clock_valid": True, "live": True,
                           "sent_mono": 10 + now, "sent_wall": 1000 + now, "clock_start_wall": 1000.,
                           "pair_span_s": 0.}}


class Match:
    def __init__(self, blue, red=None, ball=(0., 0.), *, roster=(BLUE, RED), solo_roles="offense"):
        self.sim = Simulator(SELECTED, bridge.COMBINED_LIMITS, scenario="center", bumper_contact=False)
        self.sim.move_robot(*blue)
        if red is not None:
            self.sim.add_robot(RED, *red)
        self.sim.move_ball(*ball)
        self.sim.step(zero(0.), .05, others={RED: zero(0., RED)} if red is not None else None)
        self.roster = sorted(roster)
        self.director = MatchDirector(EPOCH, {n: r for n, r in TRACKS.items() if r in self.roster}, SIGNS,
                                      pitch=PITCH)
        self.players = {r: Player(r, self.sim, self.roster, solo_roles=solo_roles) for r in self.roster}
        self.held = set()
        self.tick = self.seq = 0
        self.closest = math.inf
        self.requests = {r: [] for r in self.roster}
        self.edges = []
        self.last = {}
        self.directive = None
        self.visible = set(self.roster)

    def run(self, seconds, until=None):
        for _ in range(round(seconds / .02)):
            now = self.sim.time
            if self.tick % 20 == 0:
                for player in self.players.values():
                    player.sync(now)
            if self.tick % 2 == 0:
                self.seq += 1
                raw = camera(self.sim, self.seq, now, visible=self.visible)
                self.directive = raw["fc_match"] = self.director.update(raw, now=1000. + now)
                for robot, player in self.players.items():
                    self.last[robot] = player.receive(raw, now)
            commands = {}
            for robot, player in self.players.items():
                step, edges, local = player.step(now)
                for edge in edges:
                    assert edge["physical"] == combined.NO_CONTACT
                    self.edges.append(edge)
                    if edge["state"] in ("engaged", "completed", "retired"):
                        assert self.director.observe_kick(edge), edge
                        assert player.play.completion(edge, local), edge
                if step.request is not None:
                    self.requests[robot].append(step.request.attempt_id)
                player.backend.update(step, local)
                commands[robot] = zero(now, robot) if robot in self.held else step.decision.command
                self.last[robot] = (*self.last.get(robot, (None, None, None))[:3], step)
            if RED in self.sim.others:
                red = self.sim.others[RED][0]
                self.closest = min(self.closest, math.hypot(self.sim.robot.x - red.x, self.sim.robot.y - red.y))
            self.sim.step(commands.get(BLUE, zero(now)), .02,
                          others={RED: commands.get(RED, zero(now, RED))} if RED in self.sim.others else None)
            self.tick += 1
            if until is not None and until(self):
                return True
        return False

    def phase(self, robot):
        return self.last[robot][3].decision.phase


def test_directive_v2_has_no_attacker_and_lists_per_robot_kick_owners():
    match = Match((-.9, -.2, 0.), (.9, .2, math.pi))
    match.run(.2)
    state = match.directive
    assert state["phase"] == "playing" and state["attacker"] is None and state["kick_owners"] == {}
    assert state["pitch"] == PITCH
    assert directive_ok(state, EPOCH, sorted((BLUE, RED)), state["seq"])
    assert not directive_ok({**state, "attacker": BLUE}, EPOCH, sorted((BLUE, RED)), state["seq"])
    assert not directive_ok({**state, "kick_owners": {"other": "x"}}, EPOCH, sorted((BLUE, RED)), state["seq"])
    # Each robot chose its own role from the same frame; both attack the centre ball.
    roles = {r: match.last[r][3].decision.role for r in (BLUE, RED)}
    assert roles == {BLUE: "offense", RED: "offense"}
    for robot in (BLUE, RED):
        intent = match.last[robot][1]
        assert intent["kind"] == "auto" and intent["roles"] == "match" and intent["role_session"] == "r1"


def test_both_attack_independently_with_one_possession_clearance_and_their_own_kicks():
    match = Match((-.9, -.25, 0.), (.9, .25, math.pi), ball=(0., .0))
    yields = {BLUE: 0, RED: 0}
    simultaneous = False

    def observe(m):
        nonlocal simultaneous
        phases = {r: m.phase(r) for r in (BLUE, RED)}
        for r, p in phases.items():
            yields[r] += p == "yield"
        roles = {m.last[r][3].decision.role for r in (BLUE, RED)}
        simultaneous |= roles == {"offense"} and "yield" not in phases.values()
        return sum(len(v) for v in m.requests.values()) >= 2
    assert match.run(120., observe), {r: (match.phase(r), match.last[r][3].decision.reason) for r in (BLUE, RED)}
    assert simultaneous  # both walked to attack at once before contesting the ball
    assert min(yields.values()) == 0 < max(yields.values())  # one yielded, deterministically the same one
    assert match.closest >= 2 * max(SELECTED.robot_radius, SELECTED.bumper_offset)
    # Each robot used only its own handoff; attempts are per robot and per rally.
    for robot, attempts in match.requests.items():
        assert all(a.startswith("r1-k") for a in attempts)
    engaged = [e for e in match.edges if e["state"] == "engaged"]
    assert {e["robot_id"] for e in engaged} <= {BLUE, RED}


def test_held_peer_beside_the_ball_forfeits_after_its_stall_then_blue_scores_at_the_offset_goal():
    # Red's runner is held (zero commands) standing within contest range of the ball.
    match = Match((.20, -.55, 0.), (1.15, .25, math.pi), ball=(.9, -.15))
    match.held.add(RED)
    first_yield = None

    def scored(m):
        nonlocal first_yield
        if first_yield is None and m.phase(BLUE) == "yield":
            first_yield = m.sim.time
        return m.directive["phase"] == "goal"
    assert match.run(150., scored), (match.phase(BLUE), match.last[BLUE][3].decision.reason)
    assert first_yield is not None  # red was nearer the ball at first
    assert match.director.score == {BLUE: 1, RED: 0} and match.directive["goal_sign"] == 1
    assert match.closest >= 2 * max(SELECTED.robot_radius, SELECTED.bumper_offset)
    east = Pitch.read(PITCH).east
    assert east.y_min + SELECTED.ball_radius <= match.sim.ball.y <= east.y_max - SELECTED.ball_radius
    assert match.sim.ball.x >= east.x  # beyond the physical front, 0.25 m past the marker end
    # Return and deliberate reset: both robots home, ball replaced, one resume cue each.
    match.held.discard(RED)
    assert match.run(60., lambda m: all(m.phase(r) == "await_reset" for r in (BLUE, RED)))
    match.sim.restart_play(0., 0.)
    match.run(.5)
    assert match.directive["reset_ready"]
    for robot, player in match.players.items():
        player.clock[0] = match.sim.time + player.offset
        assert player.runner.operator({"command": "resume", "frame_seq": player.runner.executor.frame_seq})["accepted"]
    match.director.resume(now=1000. + match.sim.time)
    match.run(.3)
    assert match.directive["phase"] == "playing" and match.directive["rally"] == 2
    assert all(match.last[r][1]["role_session"] == "r2" for r in (BLUE, RED))


def test_lost_peer_holds_both_explicitly_and_play_resumes_on_reacquisition():
    match = Match((-.9, -.2, 0.), (.9, .2, math.pi))
    match.run(.4)
    match.visible = {BLUE}
    match.run(.4)
    assert match.directive["phase"] == "holding"
    for robot in (BLUE, RED):
        frame, intent, reason = match.last[robot][:3]
        assert intent is None and frame["captured_at"] is None and reason
        decision = match.last[robot][3].decision
        assert decision.command.vx == decision.command.vy == decision.command.wz == 0.
    match.visible = {BLUE, RED}
    match.run(.4)
    assert match.directive["phase"] == "playing"
    assert all(match.last[r][1] is not None for r in (BLUE, RED))


def test_mismatched_pitch_or_recalibration_holds_instead_of_reinterpreting():
    match = Match((-.9, -.2, 0.), (.9, .2, math.pi))
    match.run(.2)
    other = Pitch.read(PITCH).record()
    other["goals"]["east"].update(y_min=-.30, y_max=.31)
    player = match.players[BLUE]
    raw = camera(match.sim, match.seq + 1, match.sim.time, pitch=other)
    raw["fc_match"] = match.director.update(raw, now=1000. + match.sim.time)
    assert match.director.calibration_changed and raw["fc_match"]["phase"] == "holding"
    with pytest.raises(ValueError, match="pitch geometry differs"):
        player.source.normalize(raw, match.sim.time)
    revoked = player.source.revoke("pitch changed")
    assert player.play.events(revoked, match.sim.time)[1] is None
    raw = camera(match.sim, match.seq + 2, match.sim.time + .04, revision=2)
    with pytest.raises(ValueError, match="calibration revision changed"):
        player.source.normalize(raw, match.sim.time + .04)
    assert player.source.poisoned


def test_solo_version_two_match_is_offense_unless_the_defensive_experiment_is_selected():
    for roles, expected in (("offense", "offense"), ("defense", "defense")):
        match = Match((-1.2, .1, 0.), ball=(-1.0, .3), roster=(BLUE,), solo_roles=roles)
        match.run(.3)
        intent = match.last[BLUE][1]
        assert intent["roles"] == roles and match.last[BLUE][3].decision.role == expected
        if roles == "defense":
            match.run(8.)
            assert not match.requests[BLUE]  # the experiment never kicks
