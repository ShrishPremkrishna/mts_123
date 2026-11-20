"""kick_ball through the real queue and the fake ROS robot, with a simulated walker
and camera: tool call -> walker -> approach -> leg_kick trick -> kick clip -> walker.

The world integrates the /llm_cmd_vel speeds the server publishes (at an
efficiency the controller does not know). Camera frames arrive at 15 Hz and
carry their capture time; a fake detector returns the ball's true position at
that time, or nothing in the blind zone or outside the 220 deg view. The fake
trick controller starts at the clip that the start_clip parameter names, after
the trick's blend-in, as the native controller does.
"""

import asyncio
import math
import types
from contextlib import asynccontextmanager
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness  # noqa: E402
import tool_registry  # noqa: E402
from ball_vision import BallFix  # noqa: E402
from operator_feedback import receipt_feedback  # noqa: E402
from robot_api import LOCOMOTION, LOCOMOTION_CONTROLLER, MoveLimits, Result, Speeds  # noqa: E402
from tricks import load_tricks  # noqa: E402

LAB = tool_registry.load_lab()
kick = sys.modules["part9_ball_kick"]

CAPTURE_LATENCY = 0.15  # equal to RobotBallIO's fallback when a frame has no usable stamp
CALIBRATED = {"calibrated": True}


class World:
    """Robot pose from the published speeds; the ball fixed in the world frame."""

    def __init__(self, rig, ball=(1.4, 0.0), eff=0.8):
        self.rig, self.ball, self.eff = rig, ball, eff
        self.frames_on = True
        self.next_frame = rig.clock.now
        rig.clock.on_advance.append(self.emit_frame)

    def pose(self, t: float):
        X = Y = th = 0.0
        cmds = [(when, msg) for when, topic, msg in self.rig.robot.published if topic == "/llm_cmd_vel"]
        for i, (when, msg) in enumerate(cmds):
            end = min(t, cmds[i + 1][0] if i + 1 < len(cmds) else t)
            dt = end - when
            if dt <= 0:
                continue
            vx, vy, wz = (msg.linear.x * self.eff, msg.linear.y * self.eff, msg.angular.z * self.eff)
            th_mid = th + wz * dt / 2
            X += (math.cos(th_mid) * vx - math.sin(th_mid) * vy) * dt
            Y += (math.sin(th_mid) * vx + math.cos(th_mid) * vy) * dt
            th += wz * dt
        return X, Y, th

    def relative(self, t: float):
        if self.ball is None:
            return None
        X, Y, th = self.pose(t)
        dx, dy = self.ball[0] - X, self.ball[1] - Y
        return math.cos(th) * dx + math.sin(th) * dy, -math.sin(th) * dx + math.cos(th) * dy

    def emit_frame(self, force: bool = False):
        now = self.rig.clock.now
        if not self.frames_on or (now < self.next_frame and not force):
            return
        self.next_frame = now + 1 / 15
        stamp = harness._Msg(sec=0, nanosec=0)
        msg = harness._Msg(header=harness._Msg(stamp=stamp), data=repr(now - CAPTURE_LATENCY).encode())
        self.rig.robot.send("/camera/image_raw/compressed", msg)


class FakeVision:
    def __init__(self, world):
        self.world = world
        self.fail = False

    def detect(self, data: bytes):
        if self.fail:
            raise RuntimeError("test: detector broken")
        rel = self.world.relative(float(data.decode()))
        if rel is None:
            return None
        r, bearing = math.hypot(*rel), math.atan2(rel[1], rel[0])
        if not 0.45 <= r <= 3.0 or abs(bearing) > math.radians(105):
            return None
        return BallFix(rel[0], rel[1], 0.0, (0.0, 0.0, 1.0, 1.0), 100)


@asynccontextmanager
async def kick_rig(monkeypatch, ball=(1.4, 0.0), eff=0.8, tricks=("leg_kick",), config=CALIBRATED):
    catalog = load_tricks()
    rig = harness.Rig(tricks=lambda: {name: catalog[name] for name in tricks})
    rig.robot.static_loop_start = True
    world = World(rig, ball, eff)
    vision = FakeVision(world)
    monkeypatch.setattr(kick, "make_vision", lambda params: vision)
    monkeypatch.setattr(kick, "load_config", lambda: dict(config))

    async def run_now(fn, *args):  # detection without a worker thread: the virtual clock stays exact
        return fn(*args)
    monkeypatch.setattr(kick, "asyncio", types.SimpleNamespace(to_thread=run_now))
    try:
        assert await rig.settle()
        world.emit_frame(force=True)
        rig.vision = vision
        yield rig, world
    finally:
        rig.close()
        await asyncio.sleep(0)


def last_speed(rig):
    msgs = [m for _, topic, m in rig.robot.published if topic == "/llm_cmd_vel"]
    return (msgs[-1].linear.x, msgs[-1].linear.y, msgs[-1].angular.z) if msgs else None


def kicks(rig, leg):
    """The kick clips the leg kick was activated at (its start_clip values)."""
    return [clip for clip in rig.robot.start_clips.get(leg.controller, []) if clip.startswith("kick_")]


def when_kick_is_requested(rig, leg, action):
    """Run action() right after the leg kick takes the legs at a kick clip."""
    fired = []
    original = rig.robot.handle

    def handle(service, req):
        res = original(service, req)
        if (not fired and service.endswith("switch_controller") and leg.controller in req.activate_controllers
                and kicks(rig, leg)):
            fired.append(rig.clock.now)
            action()
        return res
    rig.robot.handle = handle
    return fired


def switches_after(rig, t):
    return [s for s in rig.robot.switches if s[0] > t]


async def run_until(rig, predicate, limit=400_000):
    for _ in range(limit):
        await harness._real_sleep(0)
        if predicate():
            return True
    return False


# ----------------------------------------------------------------------------- full sequence
@pytest.mark.parametrize("side,ball,expected", [
    ("right", (1.4, 0.0), "right"),
    ("left", (1.2, -0.4), "left"),
    ("auto", (1.0, 0.6), "left"),
    ("auto", (-0.8, -0.5), "left"),  # behind: the left-turning search first sees it on the left
])
def test_kick_ball_full_sequence(monkeypatch, side, ball, expected):
    async def case():
        async with kick_rig(monkeypatch, ball) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            result = await tool_registry.call(rig.ts, "kick_ball", {"side": side})
            assert result.ok, result.message
            assert result.details["submission"]["state"] == "queued"
            assert rig.ts.state_after_queue == LOCOMOTION
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "succeeded", outcome["message"]
            assert f"kick_{expected}" in outcome["message"]
            assert kicks(rig, leg) == [f"kick_{expected}"]
            assert len(rig.robot.activations(leg.controller)) == 1
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER} and rig.ts.state_now == LOCOMOTION
            assert last_speed(rig) == (0.0, 0.0, 0.0)
            # The paw lined up with the ball before the kick (true position, simulated).
            kicked_at = rig.robot.activations(leg.controller)[0]
            x, y = world.relative(kicked_at)
            paw = -0.046 if expected == "right" else 0.040
            assert abs(x - 0.258) <= 0.03 and abs(y - paw) <= 0.02, (x, y)
            # Nothing moved the walker while the trick held the legs.
            during = [m for t, topic, m in rig.robot.published if topic == "/llm_cmd_vel"
                      and t > kicked_at and (m.linear.x or m.linear.y or m.angular.z)]
            assert not during
    asyncio.run(case())


def test_side_defaults_to_auto(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(1.2, -0.5)) as (rig, world):
            assert (await tool_registry.call(rig.ts, "kick_ball", {})).ok
            assert await rig.settle()
            assert kicks(rig, rig.ts.tricks["leg_kick"]) == ["kick_right"]
    asyncio.run(case())


def test_dry_run_lines_up_without_the_kick(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(1.2, 0.2), config={"calibrated": True, "dry_run": True}) as (rig, world):
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "left"})).ok
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "succeeded" and "dry run" in outcome["message"]
            assert rig.robot.activations(rig.ts.tricks["leg_kick"].controller) == []
            x, y = world.relative(rig.clock.now)
            assert abs(x - 0.258) <= 0.03 and abs(y - 0.040) <= 0.02, (x, y)
    asyncio.run(case())


# ----------------------------------------------------------------------------- no kick
def test_no_ball_ends_without_a_kick_and_without_a_fault(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=None) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "auto"})).ok
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            # An expected miss is a completed request (no browser failure fence), never a kick.
            assert outcome["terminal_state"] == "succeeded", outcome["message"]
            assert outcome["message"].startswith("kick_ball ended without a kick:") and "not found" in outcome["message"]
            assert rig.robot.activations(leg.controller) == []
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert last_speed(rig) == (0.0, 0.0, 0.0)
    asyncio.run(case())


# ----------------------------------------------------------------------------- refusals now
def test_kick_needs_a_calibration_file(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, config={}) as (rig, world):
            result = await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})
            assert not result.ok and "calibrat" in result.message and rig.ts.commands.empty()
            located = await tool_registry.call(rig.ts, "locate_ball", {})  # the calibration tool works without it
            assert located.ok and located.details["ball"]["seen"]
    asyncio.run(case())


@pytest.mark.parametrize("bad", [{"calibrated": True, "dry_run": "yes"}, {"calibrated": True, "approach": {"nope": 1}},
                                 {"calibrated": "yes"}])
def test_bad_calibration_file_refuses_before_queueing(monkeypatch, bad):
    async def case():
        async with kick_rig(monkeypatch, config=bad) as (rig, world):
            result = await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})
            assert not result.ok and "calibration" in result.message and rig.ts.commands.empty()
    asyncio.run(case())


def test_speeds_above_the_selected_walker_limits_are_refused(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            rig.ts._locomotion_speeds = Speeds(walk=0.2, turn=40.0, limits=MoveLimits(
                vx_max=0.25, vx_forward_max=0.25, vx_backward_max=0.25, wz_max=40.0))
            result = await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})
            assert not result.ok and "walker limit" in result.message and rig.ts.commands.empty()
    asyncio.run(case())


def test_kick_needs_a_live_camera(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            world.frames_on = False
            rig.clock.now += 2.0  # the newest frame is now 2 s old
            result = await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})
            assert not result.ok and "camera" in result.message and rig.ts.commands.empty()
    asyncio.run(case())


@pytest.mark.parametrize("args", [{"side": "middle"}, {"side": 1}, {"side": "right", "speed": 1.0}])
def test_invalid_arguments_queue_nothing(monkeypatch, args):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            before = list(rig.robot.switches)
            result = await tool_registry.call(rig.ts, "kick_ball", args)
            assert not result.ok
            assert rig.ts.commands.empty() and rig.robot.switches == before
    asyncio.run(case())


# ----------------------------------------------------------------------------- faults in the approach
def test_camera_that_stops_during_the_approach_ends_the_job(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(2.0, 0.0)) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            start = rig.clock.now
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await run_until(rig, lambda: rig.clock.now > start + 4.0)
            world.frames_on = False
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "failed" and "camera frames stopped" in outcome["message"]
            assert rig.robot.activations(leg.controller) == [] and last_speed(rig) == (0.0, 0.0, 0.0)
    asyncio.run(case())


def test_broken_detector_ends_the_job_instead_of_searching(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            rig.vision.fail = True
            start = rig.clock.now
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "failed" and "detector failed" in outcome["message"]
            assert rig.clock.now - start < 10.0  # no full search spin
            assert last_speed(rig) == (0.0, 0.0, 0.0)
    asyncio.run(case())


def test_stop_during_the_approach_cancels_with_zero_speed(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(2.5, 0.0)) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            start = rig.clock.now
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await run_until(rig, lambda: rig.clock.now > start + 8.0)
            assert rig.ts.current is not None
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] in ("cancelled", "failed")
            assert rig.robot.activations(leg.controller) == []
            assert last_speed(rig) == (0.0, 0.0, 0.0)
    asyncio.run(case())


# ----------------------------------------------------------------------------- events while the trick holds the legs
def test_fall_during_the_kick_clip_leaves_the_controllers_alone(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            fired = when_kick_is_requested(rig, leg, lambda: rig.robot.send(
                "/rosout", harness._Msg(name=leg.controller, msg="Emergency stop triggered")))
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await rig.settle()
            assert fired and kicks(rig, leg) == ["kick_right"]
            assert rig.ts.last_outcome["terminal_state"] in ("failed", "cancelled")
            assert rig.ts.fallen == leg.controller
            assert switches_after(rig, fired[0]) == []  # no walker restart, no relax by this server
    asyncio.run(case())


def test_app_stop_during_the_kick_clip_does_not_restart_the_walker(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            stops = []
            fired = when_kick_is_requested(rig, leg, lambda: stops.append(
                asyncio.get_running_loop().create_task(tool_registry.call(rig.ts, "stop", {}))))
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await rig.settle()
            await stops[0]
            assert fired and rig.ts.last_outcome["terminal_state"] in ("failed", "cancelled")
            later = switches_after(rig, fired[0])
            assert all(LOCOMOTION_CONTROLLER not in activated for _, _, activated in later)
            assert rig.robot.controllers[leg.controller] == "inactive"  # Stop relaxed the playing trick
    asyncio.run(case())


@pytest.mark.parametrize("event", ["latched", "heartbeat_gap"])
def test_shared_stop_during_the_kick_clip_sends_no_switch(monkeypatch, event):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]

            def block():
                if event == "latched":
                    rig.robot.stop_status["latched"] = True
                    rig.robot.emit_stop_status()
                else:
                    rig.robot.stop_status_enabled = False  # stale after 0.5 s
            fired = when_kick_is_requested(rig, leg, block)
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await rig.settle()
            assert fired and rig.ts.last_outcome["terminal_state"] in ("failed", "cancelled")
            assert switches_after(rig, fired[0]) == []  # the shared stop owns the legs
    asyncio.run(case())


def test_walker_refusal_after_the_kick_relaxes_the_legs(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            original = rig.ts.switch_to

            async def refuse_walker_after_kick(controller, start_clip=""):
                if controller == LOCOMOTION_CONTROLLER and kicks(rig, leg):
                    return Result(False, "test: walker refused")
                return await original(controller, start_clip)
            monkeypatch.setattr(rig.ts, "switch_to", refuse_walker_after_kick)
            assert (await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})).ok
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "failed"
            assert "walker did not take over" in outcome["message"] and "relaxed" in outcome["message"]
            assert kicks(rig, leg) == ["kick_right"]
            assert rig.robot.active() == set()  # relaxed: no controller holds the legs
    asyncio.run(case())


# ----------------------------------------------------------------------------- tool visibility
def test_without_the_leg_kick_the_tool_is_hidden_and_refused(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, tricks=("sit_stay_shake",)) as (rig, world):
            names = [d["name"] for d in tool_registry.declarations(rig.ts)]
            assert "kick_ball" not in names and "locate_ball" not in names
            result = await tool_registry.call(rig.ts, "kick_ball", {"side": "right"})
            assert not result.ok and "not installed" in result.message
            assert rig.ts.commands.empty()
    asyncio.run(case())


def test_leg_kick_is_not_offered_to_play_trick_or_interactive_action(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, tricks=("leg_kick", "sit_stay_shake")) as (rig, world):
            decl = {d["name"]: d for d in tool_registry.declarations(rig.ts)}
            assert "kick_ball" in decl
            assert "leg_kick" not in str(decl["play_trick"])
            assert "kick_right" not in str(decl.get("interactive_action", ""))
            result = await tool_registry.call(rig.ts, "play_trick", {"name": "leg_kick", "count": 1,
                                                                     "count_unit": "routine_playback"})
            assert not result.ok and rig.ts.commands.empty()
    asyncio.run(case())


# ----------------------------------------------------------------------------- locate_ball
def test_locate_ball_reports_without_moving(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(1.0, 0.3)) as (rig, world):
            published = len(rig.robot.published)
            result = await tool_registry.call(rig.ts, "locate_ball", {})
            assert result.ok, result.message
            ball = result.details["ball"]
            assert ball["seen"] and abs(ball["x"] - 1.0) < 0.01 and abs(ball["y"] - 0.3) < 0.01
            assert ball["distance"] == 1.04 and ball["bearing_deg"] == 17 and isinstance(ball["bearing_deg"], int)
            assert "submission" not in result.details and rig.ts.commands.empty()
            assert not [m for _, topic, m in rig.robot.published[published:] if topic == "/llm_cmd_vel"]
            assert receipt_feedback("locate_ball", result.as_dict()) == "Ball seen 1.04 metres away, 17 degrees left."
    asyncio.run(case())


def test_locate_ball_without_a_ball_or_camera(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=None) as (rig, world):
            result = await tool_registry.call(rig.ts, "locate_ball", {})
            assert result.ok and result.details["ball"]["seen"] is False
            assert receipt_feedback("locate_ball", result.as_dict()) == "Ball not seen."
            world.frames_on = False
            rig.clock.now += 2.0
            result = await tool_registry.call(rig.ts, "locate_ball", {})
            assert not result.ok and "camera" in result.message
    asyncio.run(case())


# ----------------------------------------------------------------------------- kick_in_place (operator tool)
@pytest.mark.parametrize("side", ["right", "left"])
def test_kick_in_place_plays_the_clip_and_returns_to_the_walker(monkeypatch, side):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            result = await tool_registry.call(rig.ts, "kick_in_place", {"side": side})
            assert result.ok, result.message
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "succeeded", outcome["message"]
            assert kicks(rig, leg) == [f"kick_{side}"]
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
            assert not [m for _, topic, m in rig.robot.published if topic == "/llm_cmd_vel"
                        and (m.linear.x or m.linear.y or m.angular.z)]  # no approach motion
    asyncio.run(case())


def test_kick_in_place_is_hidden_from_gemini_and_checks_its_side(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            assert "kick_in_place" not in [d["name"] for d in tool_registry.declarations(rig.ts)]
            for bad in ({"side": "auto"}, {}, {"side": "middle"}):
                assert not (await tool_registry.call(rig.ts, "kick_in_place", bad)).ok
            assert rig.ts.commands.empty()
    asyncio.run(case())


def test_kick_in_place_starts_at_the_kick_clip_and_leaves_at_the_end_of_the_motion(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            start = rig.clock.now
            assert (await tool_registry.call(rig.ts, "kick_in_place", {"side": "right"})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] == "succeeded", rig.ts.last_outcome["message"]
            taken = rig.robot.activations(leg.controller)
            assert len(taken) == 1 and taken[0] - start <= kick.KICK_SETTLE_S + 0.11
            assert rig.robot.clips(leg.controller) == []  # no clip request: the activation names the clip
            back = rig.robot.activations(LOCOMOTION_CONTROLLER)[-1]
            motion_end = taken[0] + leg.blend_time + kick.KICK_MOTION_END_S
            assert motion_end - 0.02 <= back <= motion_end + 0.15, (back - taken[0], leg.blend_time)
            assert back < taken[0] + leg.blend_time + leg.clip_length("kick_right")  # before the clip's still tail ends
    asyncio.run(case())


# ----------------------------------------------------------------------------- kick_at (one job from coordinates)
def paw_y(side):
    return -0.046 if side == "right" else 0.040


@pytest.mark.parametrize("ball,side,expected", [
    ((0.60, 0.10), "auto", "left"),
    ((0.55, -0.04), "auto", "right"),
    ((0.80, 0.30), "right", "right"),
    ((0.20, -0.046), "right", "right"),  # nearer than the chest: backs up, then kicks
])
def test_kick_at_turns_walks_and_kicks_in_one_job(monkeypatch, ball, side, expected):
    async def case():
        eff = 1.2
        async with kick_rig(monkeypatch, ball=ball, eff=eff) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            jobs = rig.ts._job_sequence
            # The fake walker has no start delay; its turn rate also has the gain eff.
            monkeypatch.setattr(kick.kick_plan, "TURN_GAIN", eff)
            result = await tool_registry.call(rig.ts, "kick_at", {"x": ball[0], "y": ball[1], "side": side,
                                                                  "gain": eff, "delay": 0.0})
            assert result.ok, result.message
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "succeeded", outcome["message"]
            assert rig.ts._job_sequence == jobs + 1  # the turn, the walk and the kick are one job
            assert kicks(rig, leg) == [f"kick_{expected}"] and rig.robot.clips(leg.controller) == []
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER} and rig.ts.state_now == LOCOMOTION
            taken = rig.robot.activations(leg.controller)[0]
            x, y = world.relative(taken)
            assert abs(x - kick.kick_plan.TARGET_X) <= 0.01 and abs(y - paw_y(expected)) <= 0.01, (x, y)
            # The trick takes the legs KICK_SETTLE_S after the last walk command, with no other wait.
            moving = [t for t, m in rig.robot.twists() if m.linear.x or m.linear.y or m.angular.z]
            assert moving and taken - moving[-1] <= 1 / 20 + kick.KICK_SETTLE_S + 0.11, taken - moving[-1]
            assert not [t for t in moving if t > taken]
    asyncio.run(case())


def test_kick_at_dry_run_walks_without_the_kick(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(0.7, 0.0), eff=1.31) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            result = await tool_registry.call(rig.ts, "kick_at", {"x": 0.7, "y": 0.0, "dry_run": True, "delay": 0.0})
            assert result.ok and await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "succeeded" and "dry run" in outcome["message"]
            assert rig.robot.activations(leg.controller) == [] and last_speed(rig) == (0.0, 0.0, 0.0)
            assert abs(world.relative(rig.clock.now)[0] - kick.kick_plan.TARGET_X) <= 0.01
    asyncio.run(case())


def test_kick_at_skips_a_turn_and_a_walk_that_are_too_small(monkeypatch):
    async def case():
        ball = (kick.kick_plan.TARGET_X + 0.005, paw_y("right"))
        async with kick_rig(monkeypatch, ball=ball) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": ball[0], "y": ball[1], "side": "right"})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] == "succeeded", rig.ts.last_outcome["message"]
            assert not [t for t, m in rig.robot.twists() if m.linear.x or m.linear.y or m.angular.z]
            assert kicks(rig, leg) == ["kick_right"]
    asyncio.run(case())


def test_kick_at_refuses_bad_requests_and_is_hidden_from_gemini(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            assert "kick_at" not in [d["name"] for d in tool_registry.declarations(rig.ts)]
            for bad in ({"x": 5.0, "y": 0.0}, {"x": 0.0, "y": 0.03}, {"x": float("nan"), "y": 0.0},
                        {"x": 0.6, "y": 0.0, "side": "middle"}, {"x": 0.6, "y": 0.0, "gain": 0.1},
                        {"x": 0.6, "y": 0.0, "delay": 2.0}, {"x": 0.6}):
                assert not (await tool_registry.call(rig.ts, "kick_at", bad)).ok, bad
            assert rig.ts.commands.empty() and rig.robot.twists() == []
    asyncio.run(case())


def test_stop_during_the_kick_at_walk_cancels_before_the_kick(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=(2.0, 0.0)) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            start = rig.clock.now
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": 2.0, "y": 0.0, "side": "right"})).ok
            assert await run_until(rig, lambda: rig.clock.now > start + 2.0)
            assert rig.ts.current is not None
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] in ("cancelled", "failed")
            assert rig.robot.activations(leg.controller) == [] and last_speed(rig) == (0.0, 0.0, 0.0)
    asyncio.run(case())


# ----------------------------------------------------------------------------- kick_at with a pose (no ball)
@pytest.mark.parametrize("x,y,angle,side", [
    (2 * 0.3048, -1 * 0.3048, 0.0, "right"),  # 2 ft ahead, 1 ft to the right, kick straight ahead
    (0.5, 0.4, 90.0, "left"),
    (0.6, 0.0, -30.0, "right"),
])
def test_kick_at_goes_to_a_pose_and_kicks_on_the_heading(monkeypatch, x, y, angle, side):
    async def case():
        eff = 1.2
        async with kick_rig(monkeypatch, ball=None, eff=eff) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            jobs = rig.ts._job_sequence
            monkeypatch.setattr(kick.kick_plan, "TURN_GAIN", eff)  # the fake walker turns with the same gain
            result = await tool_registry.call(rig.ts, "kick_at", {"x": x, "y": y, "angle": angle, "point": "robot",
                                                                  "side": side, "gain": eff, "delay": 0.0})
            assert result.ok, result.message
            assert await rig.settle()
            outcome = rig.ts.last_outcome
            assert outcome["terminal_state"] == "succeeded", outcome["message"]
            assert rig.ts._job_sequence == jobs + 1 and kicks(rig, leg) == [f"kick_{side}"]
            taken = rig.robot.activations(leg.controller)[0]
            X, Y, heading = world.pose(taken)
            assert abs(X - x) <= 0.01 and abs(Y - y) <= 0.01, (X, Y)
            assert abs(math.degrees(heading) - angle) <= 1.0, math.degrees(heading)
            moving = [t for t, m in rig.robot.twists() if m.linear.x or m.linear.y or m.angular.z]
            assert taken - moving[-1] <= 1 / 20 + kick.KICK_SETTLE_S + 0.11 and not [t for t in moving if t > taken]
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER}
    asyncio.run(case())


def test_kick_at_settle_zero_switches_right_after_the_last_command(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=None) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": 0.5, "y": 0.0, "point": "robot", "angle": 0.0,
                                                                 "side": "left", "settle": 0.0})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] == "succeeded", rig.ts.last_outcome["message"]
            taken = rig.robot.activations(leg.controller)[0]
            moving = [t for t, m in rig.robot.twists() if m.linear.x or m.linear.y or m.angular.z]
            assert taken - moving[-1] <= 1 / 20 + 0.11, taken - moving[-1]
    asyncio.run(case())


def test_kick_at_speed_shortens_the_walk_and_obeys_the_walker_limit(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch, ball=None) as (rig, world):
            limit = rig.ts.speeds(LOCOMOTION).limits
            top = limit.vx_forward_max if limit.vx_forward_max is not None else limit.vx_max
            for bad in (top + 0.5, 0.0, -0.2):
                assert not (await tool_registry.call(rig.ts, "kick_at", {"x": 0.6, "y": 0.0, "point": "robot",
                                                                         "speed": bad})).ok
            for bad in ({"settle": 5.0}, {"point": "goal"}, {"angle": 400.0}):
                assert not (await tool_registry.call(rig.ts, "kick_at", {"x": 0.6, "y": 0.0, **bad})).ok, bad
            assert rig.ts.commands.empty()
            fast = min(top, 0.4)
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": 0.6, "y": 0.0, "point": "robot", "angle": 0.0,
                                                                 "dry_run": True, "speed": fast, "delay": 0.0})).ok
            assert await rig.settle()
            walk = [t for t, m in rig.robot.twists() if m.linear.x]
            assert abs((walk[-1] - walk[0]) - 0.6 / (kick.kick_plan.FWD_GAIN * fast)) <= 0.11
    asyncio.run(case())


# ----------------------------------------------------------------------------- fast walker start (tricks/walker.yaml)
STAND_FIRST = "neural_controller_default"


def fast_walker(monkeypatch, ready=0.7, stand_s=2.3):
    """The settings that tricks/walker.yaml gives: gemini_walk ready in ``ready`` seconds, and
    the course controller stands the robot up when it does not stand."""
    import robot_api as api
    monkeypatch.setattr(api, "LOCOMOTION_READY_S", ready)
    monkeypatch.setattr(api, "WALK_STAND_FIRST", STAND_FIRST)
    monkeypatch.setattr(api, "WALK_STAND_FIRST_S", stand_s)
    monkeypatch.setattr(kick, "LOCOMOTION_READY_S", ready)


def test_fast_walker_takes_the_legs_back_quickly_after_a_kick(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            fast_walker(monkeypatch)
            leg = rig.ts.tricks["leg_kick"]
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": 0.553, "y": 0.040, "side": "left"})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] == "succeeded", rig.ts.last_outcome["message"]
            back = rig.robot.activations(LOCOMOTION_CONTROLLER)[-1]
            assert back - rig.robot.activations(leg.controller)[0] == pytest.approx(leg.blend_time + kick.KICK_MOTION_END_S, abs=0.15)
            assert rig.clock.now - back == pytest.approx(0.7, abs=0.15)  # not the 4.2 s of the course values
            assert rig.robot.activations(STAND_FIRST) == []  # the robot stands after a kick: no slow stand-up
            # The next job starts at once on the walker that already holds the legs.
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": 0.553, "y": -0.046, "side": "right"})).ok
            assert await rig.settle()
            assert kicks(rig, leg) == ["kick_left", "kick_right"] and rig.robot.activations(STAND_FIRST) == []
    asyncio.run(case())


def test_kick_at_leave_gives_the_legs_back_when_the_paw_is_down(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            leg = rig.ts.tricks["leg_kick"]
            for bad in (1.0, 3.5):
                assert not (await tool_registry.call(rig.ts, "kick_at", {"x": 0.553, "y": 0.04, "leave": bad})).ok
            assert (await tool_registry.call(rig.ts, "kick_at", {"x": 0.553, "y": 0.040, "side": "left", "leave": 2.1})).ok
            assert await rig.settle()
            assert rig.ts.last_outcome["terminal_state"] == "succeeded", rig.ts.last_outcome["message"]
            back = rig.robot.activations(LOCOMOTION_CONTROLLER)[-1]
            assert back - rig.robot.activations(leg.controller)[0] == pytest.approx(leg.blend_time + 2.1, abs=0.15)
    asyncio.run(case())


def test_fast_walker_never_lifts_the_robot_the_course_controller_stands_it_up_first(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            fast_walker(monkeypatch)
            assert (await rig.call("lie_down")).ok and rig.robot.active() == set()
            mark = rig.clock.now
            assert (await rig.call("stand_up")).ok
            assert rig.ts.last_outcome["terminal_state"] == "succeeded", rig.ts.last_outcome["message"]
            after = [switch for switch in rig.robot.switches if switch[2]][-2:]  # the two activations of stand_up
            assert [(off, on) for _, off, on in after] == [([], [STAND_FIRST]), ([STAND_FIRST], [LOCOMOTION_CONTROLLER])]
            assert after[0][0] >= mark
            assert after[1][0] - after[0][0] >= 2.3 - 1e-6  # the slow stand-up gets its time
            assert rig.robot.active() == {LOCOMOTION_CONTROLLER} and rig.ts.state_now == LOCOMOTION
    asyncio.run(case())


def test_stop_during_the_slow_stand_up_does_not_start_the_walker(monkeypatch):
    async def case():
        async with kick_rig(monkeypatch) as (rig, world):
            fast_walker(monkeypatch)
            assert (await rig.call("lie_down")).ok
            mark = rig.clock.now
            assert (await tool_registry.call(rig.ts, "stand_up", {})).ok
            assert await run_until(rig, lambda: rig.robot.activations(STAND_FIRST) and rig.clock.now > mark + 1.0)
            assert (await tool_registry.call(rig.ts, "stop", {})).ok
            assert await rig.settle()
            assert [t for t in rig.robot.activations(LOCOMOTION_CONTROLLER) if t > mark] == []
            assert LOCOMOTION_CONTROLLER not in rig.robot.active()
    asyncio.run(case())


def test_walker_file_is_optional_and_refuses_a_short_start_with_no_stand_up_controller(tmp_path):
    import tricks
    assert tricks.walker_config(tmp_path / "none.yaml") == {"init_duration": 2.0, "fade_in_duration": 2.0,
                                                            "stand_first": None, "stand_first_s": 2.3}
    if not (tricks.TRICK_DIR / "walker.yaml").is_file():  # a robot release can install the file
        assert tricks.LOCOMOTION_READY_S == pytest.approx(tricks.WALK_READY_S) and tricks.WALK_STAND_FIRST is None
    path = tmp_path / "walker.yaml"
    path.write_text("init_duration: 0.2\nfade_in_duration: 0.3\nstand_first: neural_controller_default\n")
    assert tricks.walker_config(path) == {"init_duration": 0.2, "fade_in_duration": 0.3,
                                          "stand_first": "neural_controller_default", "stand_first_s": 2.3}
    for bad in ("init_duration: 0.2\n", "init_duration: 0.01\nstand_first: a\n", "speed: 3\n",
                "init_duration: 0.2\nstand_first: 'bad name'\n"):
        path.write_text(bad)
        with pytest.raises(ValueError):
            tricks.walker_config(path)
