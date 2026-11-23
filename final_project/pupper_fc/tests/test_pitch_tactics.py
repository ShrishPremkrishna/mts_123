"""One selected pitch through tactics/controller/executor, independent roles and clearance.

Synthetic observations and the approximate 2D plant only; no physical claim.
"""
from dataclasses import replace
import math
import random

import pytest

from final_project.overhead_perception.fieldcam.pitch import Pitch
from final_project.pupper_fc.controller import Controller, peer_clearance
from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.model import Ball, Command, Field, Observation, Pose, zero
from final_project.pupper_fc.paw_approach import paw_pose
from final_project.pupper_fc.tactics import SYNTHETIC_PROFILE, BallForecast, MotionProfile
from final_project.pupper_fc.simulation import Simulator
from final_project.pupper_fc.tactics import Tactics

MARKERS = dict(length=3., width=2., goal_width=.6096, ball_radius=.11)
# Goal fronts 0.30 m and 0.25 m beyond the marker ends, mouths off centre.
GOALS = {"west": dict(x=-1.80, y_min=-.20, y_max=.41), "east": dict(x=1.75, y_min=-.40, y_max=.21)}
PITCH = Pitch.from_field(MARKERS, GOALS).record()
NOMINAL = Field(length=3., width=2., goal_width=.6096, ball_radius=.11)
BLUE = NOMINAL.with_pitch(PITCH)
RED = replace(BLUE, attack_sign=-1)


def observe(robot, ball, now=0., robot_id="one", **kwargs):
    return Observation(now, Pose(*robot), Ball(*ball), robot_id, **kwargs)


def frame(now, seq, robots, ball, *, pitch=PITCH, goals=None):
    selected = Pitch.read(pitch) if pitch is not None else None
    if goals is None:
        goals = ({"left": dict(zip("xy", selected.west.center)), "right": dict(zip("xy", selected.east.center))}
                 if selected else {"left": dict(x=-1.5, y=0), "right": dict(x=1.5, y=0)})
    event = dict(event="frame", now=now, epoch="pitch", clock="replay", seq=seq, captured_at=now,
                 robots={r: dict(zip(("x", "y", "yaw"), p)) if not isinstance(p, dict) else p
                         for r, p in robots.items()},
                 ball=ball if isinstance(ball, dict) or ball is None else dict(zip(("x", "y"), ball)), goals=goals)
    if pitch is not None:
        event["pitch"] = pitch
    return event


def intent(now, seq, robot_id="pupper-1", kind="auto", **changes):
    event = dict(event="intent", now=now, epoch="pitch", clock="replay", seq=seq, issued_at=now,
                 valid_until=now + .5, robot_id=robot_id, intent_id="play", kind=kind)
    event.update(changes)
    return event


def armed(*, pitch=PITCH, **options):
    executor = Executor({"pupper-1": NOMINAL, "pupper-2": replace(NOMINAL, attack_sign=-1)}, **options)
    result = executor.handle(dict(event="arm", now=0., epoch="pitch", clock="replay", pitch=pitch))
    assert result["accepted"], result
    return executor


# ---- one geometry ----------------------------------------------------------
def test_field_keeps_markers_playable_bounds_and_physical_goals_distinct():
    assert (BLUE.length, BLUE.width) == (3., 2.) and BLUE.pitch.record() == PITCH
    assert (BLUE.bounds.x_min, BLUE.bounds.x_max) == (-1.80, 1.75)
    assert BLUE.opponent_goal.center == pytest.approx((1.75, -.095))
    assert BLUE.own_goal.center == pytest.approx((-1.80, .105))
    assert RED.opponent_goal == BLUE.own_goal
    assert BLUE.pitch.mouth_discrepancy == pytest.approx({"west": .61 - .6096, "east": .61 - .6096})
    assert NOMINAL.bounds.x_max == 1.5 and NOMINAL.opponent_goal.center == (1.5, 0.)
    with pytest.raises(ValueError, match="ball radius"):
        Field(length=3., width=2., ball_radius=.06).with_pitch(PITCH)
    assert BLUE.measured_pitch({"length": 3., "width": 2.}).pitch is None


def test_strip_between_markers_and_goal_front_reproduces_the_legacy_hold_and_plays_with_the_pitch():
    ball = (1.62, -.10)  # beyond the 1.5 m marker end, in front of the 1.75 m goal front
    legacy = Tactics(NOMINAL, "one").tick(observe((.8, -.1, 0.), ball), 0.)
    assert legacy.action == "hold" and "outside the field" in legacy.reason
    selected = Tactics(BLUE, "one").tick(observe((.8, -.1, 0.), ball), 0.)
    assert selected.action == "attack"
    controller = Controller(BLUE, kick_side="right")
    decision = controller.tick(observe((.9, -.1, 0.), ball, robot_id="pupper-1"), 0.)
    assert decision.phase in ("approach", "align")
    # The paw stance aims at the actual off-centre mouth, not the marker point (1.5, 0).
    stance = paw_pose(Ball(*ball), (1.75, -.095), "right")
    assert decision.target == pytest.approx((stance.x, stance.y))
    assert Controller(NOMINAL)._reachable(1.62, 0.)[0] == pytest.approx(1.34)
    assert Controller(BLUE)._reachable(1.62, 0.)[0] == pytest.approx(1.59)


def test_role_fraction_and_guard_use_the_physical_fronts():
    tactics = Tactics(BLUE, "one")
    guard = tactics.guard_pose(Ball(-.5, .9))
    assert guard.x == pytest.approx(-1.80 + .35)
    assert -.20 <= guard.y <= .41 and guard.y == pytest.approx(.41 - BLUE.robot_radius)
    nominal_guard = Tactics(NOMINAL, "one").guard_pose(Ball(-.5, .9))
    assert nominal_guard.x == pytest.approx(-1.5 + .35)
    # 30% of the 3.55 m between fronts from the west front is x = -0.735.
    assert Tactics(BLUE, "one").tick(observe((0, 0, 0), (-.70, 0)), 0.).role == "offense"
    assert Tactics(BLUE, "one").tick(observe((0, 0, 0), (-.77, 0)), 0.).role == "defense"


def test_executor_pins_the_pitch_selected_at_arm():
    executor = armed()
    assert executor.controllers["pupper-1"].field == BLUE
    assert executor.handle(frame(.0, 1, {"pupper-1": (-1, 0, 0)}, (0, 0)))["accepted"]
    no_pitch = frame(.02, 2, {"pupper-1": (-1, 0, 0)}, (0, 0))
    del no_pitch["pitch"]
    assert "frame pitch differs" in executor.handle(no_pitch)["reason"]
    moved = frame(.04, 3, {"pupper-1": (-1, 0, 0)}, (0, 0),
                  goals={"left": dict(x=-1.8, y=.105), "right": dict(x=1.75, y=0.)})
    assert "goal differs" in executor.handle(moved)["reason"]
    changed = Pitch.read(PITCH).record()
    changed["goals"]["east"]["x"] = 1.70
    changed["playable_bounds"]["x_max"] = 1.70
    assert "frame pitch differs" in executor.handle(frame(.06, 4, {"pupper-1": (-1, 0, 0)}, (0, 0),
                                                          pitch=changed))["reason"]
    legacy = armed(pitch=None)
    assert "frame pitch differs" in legacy.handle(frame(.0, 1, {"pupper-1": (-1, 0, 0)}, (0, 0)))["reason"]
    assert legacy.handle(frame(.02, 2, {"pupper-1": (-1, 0, 0)}, (0, 0), pitch=None))["accepted"]
    small = Executor({"pupper-1": Field(ball_radius=.06)})
    assert not small.handle(dict(event="arm", now=0, epoch="x", clock="replay", pitch=PITCH))["accepted"]


# ---- role policies and hysteresis ------------------------------------------
def test_solo_offense_never_defends_and_the_defensive_experiment_never_kicks():
    near_own_goal = observe((-1.2, .1, 0.), (-1.4, .1))
    assert Tactics(BLUE, "one", roles="offense").tick(near_own_goal, 0.).action == "attack"
    defense = Tactics(BLUE, "one", roles="defense")
    assert defense.tick(observe((-1.2, .1, 0.), (1.2, .1)), 0.).action == "defend"
    executor = armed(kick_side="right")
    executor.handle(frame(0., 1, {"pupper-1": (.7, .0, 0.)}, (1.0, -.04)))
    executor.handle(intent(0., 1, roles="defense", role_session="experiment"))
    decision = executor.tick(0.)["pupper-1"]
    assert decision.role == "defense" and decision.phase in ("defend", "target_reached")
    assert not executor._kick_permitted("pupper-1", decision)


def test_noisy_ball_near_the_threshold_keeps_a_stable_role():
    rng = random.Random(11)
    tactics = Tactics(BLUE, "one")
    roles = []
    for k in range(400):  # 20 s at 20 Hz around the 30% entry threshold
        now = k * .05
        x = -.735 + rng.uniform(-.06, .06)
        decision = tactics.tick(observe((0., 0., 0.), (x + rng.gauss(0, .01), rng.gauss(0, .01)), now), now)
        roles.append(decision.role)
        assert "rolling" not in decision.reason
    assert sum(a != b for a, b in zip(roles, roles[1:])) <= 1


def test_role_session_keeps_hysteresis_across_new_kick_attempts():
    executor = armed()
    executor.handle(frame(0., 1, {"pupper-1": (-1, 0, 0)}, (-1.5, 0)))
    executor.handle(intent(0., 1, intent_id="r1-k1", role_session="r1"))
    assert executor.tick(0.)["pupper-1"].role == "defense"
    executor.handle(frame(.1, 2, {"pupper-1": (-1, 0, 0)}, (-.4, 0)))
    executor.handle(intent(.1, 2, intent_id="r1-k2", role_session="r1"))
    assert executor.tick(.1)["pupper-1"].role == "defense"  # dwell and hysteresis survived
    executor.handle(intent(.2, 3, intent_id="r1-k3", role_session="r2"))
    assert executor.tick(.2)["pupper-1"].role == "offense"  # a new session starts fresh


# ---- moving ball: defense and offense --------------------------------------
def rolling(tactics, start, velocity, robot, *, frames=8, dt=.05, t0=0., decel=0.):
    decision = None
    speed = math.hypot(*velocity)
    for k in range(frames):
        t = k * dt
        travelled = speed * t - .5 * decel * t * t
        ball = (start[0] + velocity[0] / speed * travelled, start[1] + velocity[1] / speed * travelled)
        decision = tactics.tick(observe(robot, ball, t0 + t), t0 + t)
    return decision


# Deliberately quicker than the sandbox profile so the intercept branch is reachable.
AGILE = MotionProfile("synthetic:test-agile-defender", True, speed=.8, turn_rate=3., acceleration=2.5,
                      braking=2.5, reaction_s=.08, ball_deceleration=.18)


def test_rolling_shot_at_own_goal_intercepts_with_a_profile_and_blocks_without():
    start, velocity, robot = (0., .35), (-1.0, -.10), (-1.3, .05, 0.)
    slow = rolling(Tactics(BLUE, "one", profile=SYNTHETIC_PROFILE), start, velocity, robot, decel=.18)
    assert slow.action == "block" and slow.prediction["intercept"] is None  # too slow to meet it in time
    with_profile = Tactics(BLUE, "one", profile=AGILE)
    decision = rolling(with_profile, start, velocity, robot, decel=AGILE.ball_deceleration)
    assert decision.role == "defense" and "rolling ball" in decision.reason
    assert decision.action == "intercept"
    pose = decision.target
    # On the forecast path, facing the incoming ball.
    slope = velocity[1] / velocity[0]
    assert abs((pose.y - start[1]) - slope * (pose.x - start[0])) < .03
    assert math.cos(pose.yaw - math.atan2(-velocity[1], -velocity[0])) > .99
    record = decision.prediction
    assert record["profile"] == {"source": AGILE.source, "synthetic": True}
    assert record["deceleration_source"].startswith("synthetic") and record["age_s"] == 0.
    assert record["crossing"]["confirmed"] and record["intercept"]["arrival_s"] > 0

    without = Tactics(BLUE, "one")
    decision = rolling(without, start, velocity, robot)
    assert decision.action == "block" and decision.prediction["intercept_inputs_missing"] == ["motion profile"]
    crossing_y = start[1] + velocity[1] / velocity[0] * (-1.80 - start[0])
    assert decision.target.y == pytest.approx(min(max(crossing_y, -.20 + .16), .41 - .16), abs=.03)
    assert decision.target.x == pytest.approx(-1.80 + .35)
    assert decision.prediction["deceleration_source"] == "unmeasured: constant velocity"


def test_a_roll_wide_of_the_mouth_is_not_a_threat_and_uncertainty_guards_conservatively():
    wide = Tactics(BLUE, "one")
    decision = rolling(wide, (.2, .9), (-.6, .0), (-.9, .3, 0.))
    assert decision.role == "offense" and decision.action == "attack"
    uncertain = Tactics(BLUE, "one")
    uncertain.tick(observe((-.9, .3, 0.), (-1.2, .1)), 0.)  # already defending
    uncertain.track.forecast = lambda profile=None: BallForecast(.05, -1.2, .1, -.5, 0., 0., "unmeasured: constant velocity",
                                                                 1.5, .25, .05, 6, .3)
    decision = uncertain.tick(observe((-.9, .3, 0.), (-1.2, .1), .05), .05)
    assert decision.action == "defend" and "uncertain" in decision.reason
    assert decision.prediction["crossing"] is None


def test_offense_meets_a_rolling_ball_then_attacks_once_it_settles():
    tactics = Tactics(BLUE, "one", roles="offense", profile=SYNTHETIC_PROFILE)
    decision = rolling(tactics, (-.6, -.6), (.15, .45), (-.9, -.2, 0.), decel=.18)
    assert decision.action == "intercept" and decision.prediction["intercept"] is not None
    pose = decision.target
    goal = BLUE.opponent_goal.center
    assert math.cos(pose.yaw - math.atan2(goal[1] - pose.y, goal[0] - pose.x)) > .95
    for k in range(12):
        now = 1. + k * .05
        decision = tactics.tick(observe((-.9, -.2, 0.), (-.4, .1), now), now)
    assert decision.action == "attack"


# ---- possession and keep-out ------------------------------------------------
def both(executor, now, seq, blue, red, ball, **rows):
    robots = {"pupper-1": dict(zip(("x", "y", "yaw"), blue)), "pupper-2": dict(zip(("x", "y", "yaw"), red))}
    for robot, extra in rows.items():
        robots[robot].update(extra)
    executor.handle(frame(now, seq, robots, ball))
    for robot in robots:
        executor.handle(intent(now, seq, robot, intent_id="r1-k1", role_session="r1"))
    return executor.tick(now)


def test_symmetric_contest_resolves_identically_for_both_independent_observers():
    executor = armed()
    decisions = both(executor, 0., 1, (-.35, 0., 0.), (.35, 0., math.pi), (0., 0.))
    assert decisions["pupper-1"].phase not in ("yield", "tactical_hold")
    assert decisions["pupper-2"].phase == "yield" and "possession" in decisions["pupper-2"].reason
    # Separate executors (one per Pi) reach the same split from the same frame.
    other = armed()
    mirrored = both(other, 0., 1, (-.35, 0., 0.), (.35, 0., math.pi), (0., 0.))
    assert [mirrored[r].phase == "yield" for r in ("pupper-1", "pupper-2")] == [False, True]


def test_nearer_robot_wins_and_a_yield_persists_through_its_dwell():
    executor = armed()
    decisions = both(executor, 0., 1, (-.40, 0., 0.), (.20, 0., math.pi), (0., 0.))
    assert decisions["pupper-1"].phase == "yield" and decisions["pupper-2"].phase != "yield"
    for seq, now in enumerate((.3, .6, .9), 2):  # peer retreats into a tie: the yield dwell holds
        decisions = both(executor, now, seq, (-.40, 0., 0.), (.45, 0., math.pi), (0., 0.))
        assert decisions["pupper-1"].phase == "yield"
    decisions = both(executor, 1.2, 5, (-.40, 0., 0.), (.45, 0., math.pi), (0., 0.))
    assert decisions["pupper-1"].phase != "yield" and decisions["pupper-2"].phase == "yield"


def test_unresolved_peer_kick_keeps_priority_and_a_static_peer_forfeits():
    executor = armed()
    decisions = both(executor, 0., 1, (-.30, 0., 0.), (.45, 0., math.pi), (0., 0.),
                     **{"pupper-2": {"kick_active": True}})
    assert decisions["pupper-1"].phase == "yield" and "unresolved kick" in decisions["pupper-1"].reason
    tactics = Tactics(BLUE, "pupper-1")
    peer = Observation(0., Pose(.25, .0, math.pi), None, "pupper-2")
    outcomes = []
    for k in range(80):
        now = k * .05
        peer = replace(peer, captured_at=now)
        outcomes.append(tactics.tick(observe((-.45, 0., 0.), (0., .0), now, "pupper-1"), now,
                                     {"pupper-2": peer}).action)
    assert outcomes[0] == "yield" and outcomes[-1] == "attack"
    assert outcomes.index("attack") * .05 >= tactics.config.peer_stall_s


def test_ball_pinned_beside_a_static_peer_is_an_explicit_hold():
    tactics = Tactics(BLUE, "pupper-1")
    for k in range(70):
        now = k * .05
        peer = Observation(now, Pose(-.15, .25, 0.), None, "pupper-2")  # stands where our stance must go
        decision = tactics.tick(observe((-.8, 0., 0.), (.05, 0.), now, "pupper-1"), now, {"pupper-2": peer})
    assert decision.action == "hold" and "static peer" in decision.reason


def test_keepout_removes_closing_motion_and_blocks_a_kick_near_a_peer():
    executor = armed()
    executor.handle(frame(0., 1, {"pupper-1": (0., 0., 0.), "pupper-2": (.45, 0., math.pi)}, (-.9, .8)))
    executor.handle(intent(0., 1, kind="move", target=dict(x=1.0, y=0., yaw=0.)))
    decision = executor.tick(0.)["pupper-1"]
    assert decision.phase == "peer_clearance"
    assert decision.command.vx <= 1e-9  # no motion toward the peer straight ahead
    assert decision.command.vy < 0     # keeps right of the peer
    controller = executor.controllers["pupper-1"]
    ready = replace(decision, phase="kick_ready", command=zero(0.))
    blocked = executor._peer_keepout(controller, executor.frame[1]["pupper-1"], executor.frame[1], ready, 0.)
    assert blocked.phase == "peer_clearance" and "no kick" in blocked.reason


def test_head_on_return_paths_pass_by_keeping_right_in_the_plant():
    field = BLUE
    sim = Simulator(field, scenario="center", bumper_contact=False)
    sim.move_robot(-1.0, .05, 0.)
    sim.add_robot("pupper-2", 1.0, .0, math.pi)
    sim.move_ball(0., .85)
    executor = armed()
    targets = {"pupper-1": dict(x=1.0, y=0., yaw=0.), "pupper-2": dict(x=-1.0, y=0., yaw=math.pi)}
    closest, seq = math.inf, 0
    for tick in range(600):
        now = sim.time
        if tick % 2 == 0:
            seq += 1
            obs = sim.observe()
            red = sim.peers()["pupper-2"]
            executor.handle(frame(now, seq, {"pupper-1": (obs.robot.x, obs.robot.y, obs.robot.yaw),
                                             "pupper-2": (red.x, red.y, red.yaw)}, (obs.ball.x, obs.ball.y)))
            for robot, target in targets.items():
                executor.handle(intent(now, seq, robot, kind="move", intent_id="home", target=target))
        decisions = executor.tick(now)
        red = sim.peers()["pupper-2"]
        closest = min(closest, math.hypot(sim.robot.x - red.x, sim.robot.y - red.y))
        sim.step(decisions["pupper-1"].command, .02, others={"pupper-2": decisions["pupper-2"].command})
        if all(decisions[r].phase == "target_reached" for r in targets):
            break
    assert all(decisions[r].phase == "target_reached" for r in targets), {r: d.reason for r, d in decisions.items()}
    assert closest >= 2 * max(field.robot_radius, field.bumper_offset)


# ---- moving ball in the approximate plant ------------------------------------
def plant_loop(executor, sim, seconds, *, kind="auto", roles="defense", handoff=None, backend=None, held=False):
    seq, trace = 0, []
    for tick in range(round(seconds / .02)):
        now = sim.time
        if tick % 2 == 0:
            seq += 1
            obs = sim.observe(delay=.02)
            ball = dict(x=obs.ball.x, y=obs.ball.y, vx=obs.ball.vx, vy=obs.ball.vy)
            executor.handle(frame(now, seq, {"pupper-1": (obs.robot.x, obs.robot.y, obs.robot.yaw)}, ball))
            executor.handle(intent(now, seq, kind=kind, intent_id="r1-k1", roles=roles, role_session="r1"))
        if handoff is not None:
            step = executor.kick_step(handoff, now)
            backend.update(step, now)
            decision = step.decision
            trace.append((now, decision, step.request, math.hypot(sim.ball.vx, sim.ball.vy)))
        else:
            decision = executor.tick(now)["pupper-1"]
            trace.append((now, decision, None, math.hypot(sim.ball.vx, sim.ball.vy)))
        sim.step(zero(now) if held else decision.command, .02)
        if sim.goal:
            break
    return trace


@pytest.mark.parametrize("profile", [None, SYNTHETIC_PROFILE], ids=["unmeasured-block", "synthetic-profile"])
def test_defensive_experiment_stops_a_rolling_shot_that_otherwise_scores(profile):
    def shot(held):
        sim = Simulator(BLUE, scenario="center", bumper_contact=False)
        sim.move_robot(-1.25, -.15 if not held else -.85, 0.)
        sim.move_ball(.2, .40)
        sim.nudge_ball(-1.1, -.12)
        executor = armed(motion_profile=profile, allow_synthetic_profile=profile is not None)
        return sim, plant_loop(executor, sim, 6., held=held)
    control, _ = shot(held=True)
    assert control.goal == -1  # unopposed, the shot enters the off-centre west mouth
    sim, trace = shot(held=False)
    assert sim.goal == 0
    actions = {d.phase for _, d, _, _ in trace}
    assert actions & {"block", "intercept"}
    used = [d.prediction for _, d, _, _ in trace if d.prediction]
    assert used and all(p["captured_at"] <= now for (now, d, _, _), p in zip(
        [t for t in trace if t[1].prediction], used))
    expected = "synthetic" if profile is not None else "unmeasured"
    assert all(p["deceleration_source"].startswith(expected) for p in used)


def test_offense_meets_a_rolling_ball_and_kicks_only_after_it_settles():
    from final_project.pupper_fc.kick_handoff import KickHandoff
    from final_project.pupper_fc.simulation import SimulatedKick
    from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M
    from final_project.pupper_fc.model import Limits
    limits = Limits(vx=.3, vy=.3, wz=1., acceleration=.65, yaw_acceleration=2.)
    sim = Simulator(BLUE, limits, scenario="center", bumper_contact=False)
    sim.move_robot(-.85, -.35, 0.)
    sim.move_ball(-.3, -.9)
    sim.nudge_ball(0., .45)  # rolls across in front of the robot and stops after about 0.56 m
    executor = Executor({"pupper-1": NOMINAL, "pupper-2": replace(NOMINAL, attack_sign=-1)}, limits,
                        kick_side="right", paw_forward_m=COMBINED_PAW_FORWARD_M["right"],
                        motion_profile=SYNTHETIC_PROFILE, allow_synthetic_profile=True)
    assert executor.handle(dict(event="arm", now=0., epoch="pitch", clock="replay", pitch=PITCH))["accepted"]
    handoff = KickHandoff("pupper-1")
    trace = plant_loop(executor, sim, 30., roles="offense", handoff=handoff, backend=SimulatedKick(handoff))
    phases = [d.phase for _, d, _, _ in trace]
    requests = [(now, speed) for now, _, request, speed in trace if request is not None]
    assert "intercept" in phases and requests
    assert phases.index("intercept") < [i for i, t in enumerate(trace) if t[2] is not None][0]
    assert requests[0][1] < .02  # stationary contact stays the only physical candidate
