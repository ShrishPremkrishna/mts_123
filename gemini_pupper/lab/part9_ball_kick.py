"""Final project: "go find the ball, walk over to it, and kick it" as one tool.

kick_ball(side) queues one job, shaped like shake_hand (part5):

  1. the walker (gemini_walk) takes the legs;
  2. robot/ball_kick.py finds the ball (look, then turn in steps), walks to it,
     lines the chosen front paw up with stop-and-look pulses, and creeps the last
     0.24 m blind (the camera cannot see the ball from 0.30 to 0.45 m);
  3. the leg_kick trick takes the legs (STRICT switch) and starts at the kick_right
     or kick_left clip (start_clip), so the kick begins when the blend-in ends;
  4. the walker takes the legs back when the kick motion ends (2.6 s into the
     3.0 s clip): the trick policy trained only with its feet on the ground and
     jitters when the robot is lifted.

kick_at(x, y, side) is the same kick for a ball at known coordinates (the field
camera): one job turns, walks and kicks, with no wait between the walk and the
kick. With an angle or point="robot" it goes to a pose first (turn, walk, turn)
and kicks on that heading. kick_in_place(side) is only steps 3 and 4.

Camera fixes come from robot/ball_vision.py, which runs only during step 2, on
the newest frame at each control tick, in a worker thread of this server (no
detector node, no Hailo). A Stop, a fall or
a motion block ends the job at the next 50 ms tick with a zero speed command.

Calibration (ball radius, range scale, sideways bias, stage distances) can be
changed without code: put a YAML file at PUPPER_BALL_KICK_CONFIG (default
~/.config/gemini-pupper/ball_kick.yaml) with the keys of robot/ball_kick.py's
config_from_mapping().
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
from pathlib import Path

import kick_at as kick_plan  # robot/kick_at.py: the turn-and-walk plan and the walker model
from ball_kick import Fix, approach, config_from_mapping
from part3_follow import prepare_to_move
from robot_api import (FELL_MESSAGE, LLM_CMD_RATE, LOCOMOTION, LOCOMOTION_CONTROLLER, WALK_READY_S,
                       MotionBlockedError, Result, Robot, with_note)
from tool_registry import Param, kick_ball_available, tool

log = logging.getLogger("kick_ball")

LEG_KICK = "leg_kick"  # tricks/leg_kick: clips stand_idle (loop), kick_right, kick_left
SIDES = ("right", "left", "auto")
APPROACH_ESTIMATE_S = 45.0  # simulated approach: median 31 s, 95th percentile 43 s, maximum 55 s
# At most 6 detections per second: one cluttered frame cost 0.4 s of CPU on the Pi
# (2026-11-18), and the Pi throttles near 85 deg C. A 1.5 s measure still gets about 9 fixes.
DETECT_PERIOD_S = 1.0 / 6.0
CAMERA_MAX_AGE_S = 1.0  # a newer frame must arrive within this time while the detector runs
DETECTOR_FAILURES_MAX = 5  # consecutive detector exceptions that end the approach
# Kick clip timeline (tricks/leg_kick/policy.json, both sides): still to 0.40 s, weight
# shift to 0.9 s, wind-up to 1.3 s, swing 1.3-1.5 s (contact), leg down by 2.1 s, weight
# back by 2.56 s, then still to 3.0 s.
KICK_MOTION_END_S = 2.6  # into the clip: the walker takes the legs here, not at 3.0 s
KICK_SETTLE_S = 0.15  # zero speed on the walker before the trick takes the legs (tool option settle)
STEP_SETTLE_S = 0.2  # zero speed between kick_at's turn and walk steps, so that each starts from rest
CONFIG_PATH = Path(os.environ.get("PUPPER_BALL_KICK_CONFIG", "~/.config/gemini-pupper/ball_kick.yaml")).expanduser()


def kick_trick(robot: Robot):
    """The installed leg kick trick (tool_only: kick_ball, both kick clips), or None."""
    return robot.tricks.get(LEG_KICK) if kick_ball_available(robot) else None


def load_config() -> dict:
    """The calibration overrides (an empty mapping if there is no file)."""
    if not CONFIG_PATH.is_file():
        return {}
    import yaml

    data = yaml.safe_load(CONFIG_PATH.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{CONFIG_PATH}: expected a mapping")
    return data


def make_vision(params):
    """The camera detector (OpenCV, imported only when a kick runs). Tests replace this."""
    from ball_vision import BallVision

    return BallVision(params)


class RobotBallIO:
    """ApproachIO on the Robot: walker speeds out; poll() runs the detector on the
    newest camera frame in a worker thread, so the event loop and Stop stay responsive."""

    def __init__(self, robot: Robot, vision):
        self.robot, self.vision = robot, vision
        self.blocked: str | None = None
        self.fix: Fix | None = None
        self.frames = self.fixes = 0
        self.found = None  # the newest ball_vision.BallFix
        self._last_msg = None
        self._last_run = -1e9
        self._failures = 0

    def now(self) -> float:
        return self.robot.now()

    async def sleep(self, seconds: float) -> None:
        await self.robot.sleep(seconds)

    def send(self, vx: float, vy: float, wz: float) -> None:
        try:
            self.robot.send_speed(vx, vy, math.degrees(wz))
        except MotionBlockedError as error:  # send_speed already published a zero
            self.blocked = str(error)

    def check(self) -> str | None:
        if self.robot.fallen:
            return FELL_MESSAGE
        if self.robot.stopped_during_call():
            return "stopped"
        return self.blocked or self.robot.motion_block_reason()

    def capture_time(self, msg, received: float) -> float:
        """The frame's capture time on robot.now()'s clock (the ROS stamp is on the same machine)."""
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.robot.node.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp > 0 and 0.0 <= age <= 2.0:
                return self.robot.now() - age
        except (AttributeError, TypeError):
            pass
        return received - 0.15  # no usable stamp: the measured latency was 115 to 166 ms

    async def poll(self) -> Fix | None:
        frame = self.robot.latest_camera_frame
        age = None if frame is None else self.robot.now() - frame[1]
        if age is None or age > CAMERA_MAX_AGE_S:
            # The camera stopped: end the approach (check() reports it), never search blind.
            self.blocked = self.blocked or ("no camera frame has arrived" if age is None
                                            else f"camera frames stopped ({age:.1f} s since the last frame)")
            return self.fix
        if frame[0] is not self._last_msg and self.robot.now() - self._last_run >= DETECT_PERIOD_S:
            msg, received = frame
            self._last_msg, self._last_run = msg, self.robot.now()
            try:
                found = await asyncio.to_thread(self.vision.detect, bytes(msg.data))
                self._failures = 0
            except Exception as error:
                self._failures += 1
                log.exception("ball detection failed on one frame")
                if self._failures >= DETECTOR_FAILURES_MAX:
                    self.blocked = self.blocked or f"ball detector failed: {type(error).__name__}: {error}"
                found = None
            self.frames += 1
            if found is not None:
                self.fixes += 1
                self.found = found
                self.fix = Fix(found.x, found.y, self.capture_time(msg, received),
                               precise=getattr(found, "method", "fit") == "fit")
        return self.fix


def camera_problem(robot: Robot) -> str | None:
    """Why the camera cannot be used now, or None."""
    frame = robot.latest_camera_frame
    if frame is None:
        return "no camera frame has arrived"
    age = robot.now() - frame[1]
    return f"the newest camera frame is {age:.1f} s old" if age > CAMERA_MAX_AGE_S else None


def speed_problem(robot: Robot, approach) -> str | None:
    """A configured approach speed above the selected walker limits, or None."""
    limits = robot.speeds(LOCOMOTION).limits
    forward = limits.vx_forward_max if limits.vx_forward_max is not None else limits.vx_max
    backward = limits.vx_backward_max if limits.vx_backward_max is not None else limits.vx_max
    turn = max(approach.max_wz, approach.search_wz, approach.min_turn_wz)
    if max(approach.approach_speed, approach.pulse_vx) > forward or approach.pulse_vx > backward:
        return f"approach speed above the walker limit ({forward:g} m/s forward, {backward:g} m/s backward)"
    if approach.pulse_vy > limits.vy_max:
        return f"sideways pulse speed above the walker limit ({limits.vy_max:g} m/s)"
    if math.degrees(turn) > limits.wz_max:
        return f"turn rate above the walker limit ({limits.wz_max:g} deg/s)"
    return None


async def kick_ball(robot: Robot, side: str) -> Result:
    """Find the ball, walk to it, line up the chosen paw and kick (finished)."""
    if side not in SIDES:
        return Result(False, "side must be right, left or auto")
    trick = kick_trick(robot)
    if trick is None:
        return Result(False, "The leg kick trick is not installed, so kick_ball is unavailable.")
    if robot.fallen:
        return Result(False, FELL_MESSAGE)
    try:
        config = config_from_mapping(load_config())
    except (OSError, ValueError, TypeError) as error:
        return Result(False, f"kick_ball calibration file is invalid: {error}")
    if not config.calibrated:
        return Result(False, f"kick_ball needs a ball calibration file with calibrated: true ({CONFIG_PATH});"
                             " see docs/kick-ball.md stage R3. locate_ball works without it.")
    if problem := speed_problem(robot, config.approach):
        return Result(False, f"kick_ball calibration refused: {problem}")
    if problem := camera_problem(robot):
        return Result(False, f"kick_ball needs the camera: {problem}")
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note

    async def job() -> Result:
        if robot.fallen:
            return Result(False, FELL_MESSAGE)
        if kick_trick(robot) is not trick:
            return Result(False, "The leg kick trick changed before the kick.")
        res = await robot.switch_to(LOCOMOTION_CONTROLLER)  # stand (and leave a seated trick) first
        if not res.ok:
            return res
        io = RobotBallIO(robot, make_vision(config.vision))
        try:
            outcome = await approach(io, side, config.geometry, config.approach)
        finally:
            robot.send_speed(0, 0, 0)
        for line in outcome.log:
            log.info("kick_ball %s", line)
        seen = f"{io.fixes} ball fixes in {io.frames} frames"
        if not outcome.ok:
            details = {"kick_ball": {"kicked": False, "stage": "approach", "reason": outcome.reason,
                                     "no_ball": outcome.no_ball, "log": outcome.log[-12:]}}
            if outcome.no_ball:
                # The ball was not found, lost or moved: a normal end of the request, not a fault.
                return Result(True, f"kick_ball ended without a kick: {outcome.reason} ({seen}); zero speed"
                                    " requested, walking controller still active.", details)
            return Result(False, f"kick_ball stopped before the kick: {outcome.reason} ({seen}); zero speed requested",
                          details)
        clip = f"kick_{outcome.side}"
        details = {"kick_ball": {"kicked": False, "side": outcome.side, "requested": side,
                                 "before_creep": outcome.before_creep, "creep_m": round(outcome.creep_m, 3),
                                 "efficiency": round(outcome.efficiency, 3), "dry_run": config.dry_run,
                                 "log": outcome.log[-12:]}}
        if config.dry_run:
            return Result(True, f"dry run: lined up for {clip} ({seen}); no kick played, walker still active.", details)
        kicked = await play_kick(robot, trick, clip)
        if not kicked.ok:
            return Result(False, kicked.message, details)
        details["kick_ball"]["kicked"] = True
        return Result(True, f"{clip} controller playback confirmed after the line-up ({seen}); "
                            f"locomotion controller active. Ball contact and travel were not measured.", details)

    estimate = robot.transition_time_estimate(LOCOMOTION_CONTROLLER) + APPROACH_ESTIMATE_S + kick_time(trick)
    return with_note(note, await robot.queue(f"kick_ball {side}", job, estimate, then=LOCOMOTION))


def _legs_owned_elsewhere(robot: Robot) -> bool:
    """A stop, a fall, a shared-stop block or a controller fault owns the legs now."""
    return bool(robot.fallen or robot.stopped_during_call() or robot.controller_fault
                or robot.motion_block_reason())


def kick_time(trick) -> float:
    """Seconds play_kick takes: blend-in, the kick motion, then the walker's own start."""
    return KICK_SETTLE_S + trick.blend_time + KICK_MOTION_END_S + WALK_READY_S + 0.5


async def play_kick(robot: Robot, trick, clip: str) -> Result:
    """STRICT switch into the leg kick at the kick clip itself (start_clip), wait for
    the end of the kick motion, then give the legs back to the walker (or relax them
    if the walker refuses them for a reason other than a stop).

    The controller holds the clip's first frame (a stand) during its blend-in and
    starts the clip when the blend-in ends, so no stand_idle pass, no settle and no
    clip request come before the kick.
    """
    entry = await robot.switch_to(trick.controller, start_clip=clip)
    if not entry.ok:
        return Result(False, f"leg kick did not take the legs: {entry.message}")
    result = Result(False, "kick not played")
    try:
        await robot.sleep(trick.blend_time + KICK_MOTION_END_S - 0.1)
        if robot.fallen:
            result = Result(False, f"leg kick ended in a fall: {FELL_MESSAGE}")
            return result
        result = await robot.wait_start_clip(trick, clip, KICK_MOTION_END_S)
        if result.ok:
            result = Result(True, f"played {clip}")
        return result
    finally:
        # Leave the trick at once, also after a failed clip (the policy jitters when
        # the robot is lifted). The shared stop, a fall or a fault owns the legs
        # otherwise: this server then sends no switch request at all.
        if robot.state_now is trick and not _legs_owned_elsewhere(robot):
            back = await robot.switch_to(LOCOMOTION_CONTROLLER)
            if not back.ok:
                if _legs_owned_elsewhere(robot):
                    extra = f"the walker did not take over ({back.message}); the stop owns the legs"
                else:
                    relaxed = await robot.switch_to(None)
                    extra = (f"the walker did not take over ({back.message}); this server relaxed the leg"
                             f" controllers: {relaxed.message}")
                log.warning("kick_ball: %s", extra)
                result.message = (f"{clip} played, but {extra}" if result.ok else f"{result.message}; {extra}")
                result.ok = False


def _round_half_away(value: float) -> int:
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


async def locate_ball(robot: Robot, seconds: float = 1.2) -> Result:
    """Look without moving and report the median camera fix (no queue, no motion)."""
    if problem := camera_problem(robot):
        return Result(False, f"The ball cannot be located: {problem}.")
    try:
        config = config_from_mapping(load_config())
    except (OSError, ValueError, TypeError) as error:
        return Result(False, f"kick_ball calibration file is invalid: {error}")
    io = RobotBallIO(robot, make_vision(config.vision))
    start = robot.now()
    fixes, seen = [], set()
    while robot.now() - start < seconds:
        fix = await io.poll()
        if io.blocked:
            return Result(False, f"The ball cannot be located: {io.blocked}.")
        if fix is not None and fix.stamp >= start - 0.2 and fix.stamp not in seen:
            seen.add(fix.stamp)
            fixes.append(io.found)
        await robot.sleep(0.05)
    if config.snapshot_path and hasattr(io.vision, "save_snapshot"):
        await asyncio.to_thread(io.vision.save_snapshot, config.snapshot_path)
    if len(fixes) < 3:
        why = getattr(io.vision, "last_note", "") or ""
        return Result(True, f"Ball not seen ({len(fixes)} fixes in {io.frames} frames{'; ' + why if why else ''})."
                            " The camera does not see a ball close to the robot's nose or behind it.",
                      {"ball": {"schema": "pupper_ball_fix_v1", "seen": False, "frames": io.frames,
                                "fixes": len(fixes)}})

    def median(values):
        values = sorted(values)
        return values[len(values) // 2]
    x, y, z = median(f.x for f in fixes), median(f.y for f in fixes), median(f.z for f in fixes)
    distance = round(math.hypot(x, y), 2)
    bearing = _round_half_away(math.degrees(math.atan2(y, x)))
    where = "ahead" if abs(bearing) < 5 else f"{abs(bearing)} degrees {'left' if bearing > 0 else 'right'}"
    methods = sorted({getattr(f, "method", "fit") for f in fixes})
    rho = median(getattr(f, "rho", 0.0) for f in fixes)
    return Result(True, f"Ball seen {distance:g} metres away, {where} (camera estimate x={x:.3f} m, y={y:+.3f} m,"
                        f" z={z:+.3f} m, method {'/'.join(methods)}; not a physical measurement).",
                  {"ball": {"schema": "pupper_ball_fix_v1", "seen": True, "x": round(x, 3), "y": round(y, 3),
                            "z": round(z, 3), "distance": distance, "bearing_deg": bearing,
                            "method": "/".join(methods), "rho_deg": round(math.degrees(rho), 2),
                            "frames": io.frames, "fixes": len(fixes)}})


async def drive(robot: Robot, vx: float, vy: float, wz_deg: float, seconds: float) -> str | None:
    """Send one speed to the walker for ``seconds``, then zero speed. Returns why it
    ended early (a fall, a stop, a motion block), or None."""
    end = robot.now() + seconds
    try:
        while robot.now() < end - 1e-6:
            if robot.fallen:
                return FELL_MESSAGE
            if robot.stopped_during_call():
                return "stopped"
            robot.send_speed(vx, vy, wz_deg)
            await robot.sleep(min(1.0 / LLM_CMD_RATE, end - robot.now()))
    except MotionBlockedError as error:
        return str(error)
    finally:
        robot.send_speed(0, 0, 0)
    if robot.fallen:
        return FELL_MESSAGE
    return "stopped" if robot.stopped_during_call() else None


async def kick_now(robot: Robot, trick, clip: str, settle: float = KICK_SETTLE_S) -> Result:
    """Zero speed for ``settle`` seconds, then play_kick. The caller holds the walker."""
    if early := await drive(robot, 0.0, 0.0, 0.0, settle):
        return Result(False, f"no kick: {early}")
    return await play_kick(robot, trick, clip)


async def kick_in_place(robot: Robot, side: str) -> Result:
    """Play the leg kick now, from where Pupper stands (finished). The caller lined the
    ball up first; no approach and no camera here."""
    if side not in ("right", "left"):
        return Result(False, "side must be right or left")
    trick = kick_trick(robot)
    if trick is None:
        return Result(False, "The leg kick trick is not installed, so kick_in_place is unavailable.")
    if robot.fallen:
        return Result(False, FELL_MESSAGE)
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note
    clip = f"kick_{side}"

    async def job() -> Result:
        if robot.fallen:
            return Result(False, FELL_MESSAGE)
        if kick_trick(robot) is not trick:
            return Result(False, "The leg kick trick changed before the kick.")
        res = await robot.switch_to(LOCOMOTION_CONTROLLER)  # stand on the walker first
        if not res.ok:
            return res
        kicked = await kick_now(robot, trick, clip)
        if not kicked.ok:
            return kicked
        return Result(True, f"{clip} controller playback confirmed; locomotion controller active."
                            " Ball contact was not measured.", {"kick_ball": {"kicked": True, "side": side}})

    estimate = robot.transition_time_estimate(LOCOMOTION_CONTROLLER) + kick_time(trick)
    return with_note(note, await robot.queue(f"kick_in_place {side}", job, estimate, then=LOCOMOTION))


async def kick_at(robot: Robot, x: float, y: float, side: str = "auto", dry_run: bool = False,
                  gain: float = kick_plan.FWD_GAIN, delay: float = kick_plan.DEAD_S, angle: float | None = None,
                  point: str = "ball", speed: float = kick_plan.SPEED, settle: float = KICK_SETTLE_S) -> Result:
    """Go to a kick position and kick, as one job: the kick clip is requested in the
    same tick in which the last motion step ends (finished).

    x, y: a point ahead and to the left of base_link, in metres, from outside the robot
    (the field camera). With no angle and point "ball": the ball centre; one turn onto
    the paw line, then a walk until the ball is at the chest. With an angle (degrees from
    the heading at the start, left positive) or point "robot": turn to the stand point,
    walk to it, turn to the kick heading. gain, delay: the walker model (travel =
    gain x speed x (time - delay)), measured at 0.2 m/s.
    """
    numbers = (x, y, gain, delay, speed, settle) + (() if angle is None else (angle,))
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in numbers):
        return Result(False, "x, y, angle, gain, delay, speed and settle must be finite numbers")
    if side not in SIDES or type(dry_run) is not bool:
        return Result(False, "side must be right, left or auto, and dry_run true or false")
    if not 0.5 <= gain <= 2.0 or not 0.0 <= delay <= 0.5 or not 0.0 <= settle <= 2.0:
        return Result(False, "gain must be from 0.5 to 2, delay from 0 to 0.5 s and settle from 0 to 2 s")
    trick = kick_trick(robot)
    if trick is None:
        return Result(False, "The leg kick trick is not installed, so kick_at is unavailable.")
    if robot.fallen:
        return Result(False, FELL_MESSAGE)
    speeds = robot.speeds(LOCOMOTION)
    forward = speeds.limits.vx_forward_max if speeds.limits.vx_forward_max is not None else speeds.limits.vx_max
    if not speeds.limits.vx_min <= speed <= forward:
        return Result(False, f"kick_at refused: speed must be from {speeds.limits.vx_min:g} to {forward:g} m/s")
    try:
        side = kick_plan.choose_side(y, side)
        path = kick_plan.steps(x, y, side, angle, point)
    except ValueError as error:
        return Result(False, f"kick_at refused: {error}")
    timed = []  # (kind, value, seconds); steps too small to command are dropped
    for kind, value in path:
        if kind == "turn" and abs(value) >= kick_plan.MIN_TURN_DEG:
            timed.append((kind, value, abs(value) / (speeds.turn * kick_plan.TURN_GAIN)))
        elif kind == "walk" and abs(value) >= kick_plan.MIN_WALK_M:
            timed.append((kind, value, delay + abs(value) / (gain * speed)))
    if any(seconds > speeds.limits.duration_max for _, _, seconds in timed):
        return Result(False, f"kick_at refused: a turn or the walk is longer than {speeds.limits.duration_max:g} s")
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note
    clip = f"kick_{side}"
    moves = ", ".join(f"turn {value:+.1f} deg" if kind == "turn" else f"walk {value:+.3f} m"
                      for kind, value, _ in timed) or "no motion"
    details = {"kick_at": {"kicked": False, "side": side, "point": point, "angle_deg": angle, "dry_run": dry_run,
                           "steps": [{"kind": kind, "value": round(value, 3), "seconds": round(seconds, 2)}
                                     for kind, value, seconds in timed]}}

    async def job() -> Result:
        if robot.fallen:
            return Result(False, FELL_MESSAGE)
        if kick_trick(robot) is not trick:
            return Result(False, "The leg kick trick changed before the kick.")
        res = await robot.switch_to(LOCOMOTION_CONTROLLER)
        if not res.ok:
            return res
        early = None
        for index, (kind, value, seconds) in enumerate(timed):
            if index and not early:  # each step starts from rest; the kick has its own settle
                early = await drive(robot, 0.0, 0.0, 0.0, STEP_SETTLE_S)
            if not early:
                early = (await drive(robot, 0.0, 0.0, math.copysign(speeds.turn, value), seconds) if kind == "turn"
                         else await drive(robot, math.copysign(speed, value), 0.0, 0.0, seconds))
        if early:
            return Result(False, f"kick_at stopped before the kick: {early}; zero speed requested", details)
        if dry_run:
            return Result(True, f"dry run: {moves} for {clip}; no kick played, walker still active.", details)
        kicked = await kick_now(robot, trick, clip, settle)
        if not kicked.ok:
            return Result(False, kicked.message, details)
        details["kick_at"]["kicked"] = True
        return Result(True, f"{clip} controller playback confirmed after {moves}; locomotion controller active."
                            " Ball contact was not measured.", details)

    estimate = (robot.transition_time_estimate(LOCOMOTION_CONTROLLER) + sum(seconds for _, _, seconds in timed)
                + STEP_SETTLE_S * max(0, len(timed) - 1) + (0.0 if dry_run else kick_time(trick)))
    return with_note(note, await robot.queue(f"kick_at {point} {x:.2f} {y:+.2f} {side}", job, estimate, then=LOCOMOTION))


@tool(
    "kick_in_place",
    "Operator tool, not offered to Gemini: play the leg kick now with the named front leg, from where Pupper"
    " stands, then return to the walking controller. Only after the ball was lined up in front of that paw.",
    side=Param("string", "Which front leg kicks: right or left.", enum=["right", "left"]),
    _ends_following=True,
)
async def kick_in_place_tool(robot: Robot, side: str) -> Result:
    return await kick_in_place(robot, side)


@tool(
    "kick_at",
    "Operator tool, not offered to Gemini: go to a point given relative to Pupper (from the field camera) and"
    " play the leg kick with no wait between the last motion and the kick, then return to the walking"
    " controller. With no angle the point is the ball: one turn onto the kicking paw's line, then a walk until"
    " the ball is at the chest. With an angle, or point robot: turn to the point, walk to it, turn to the kick"
    " heading.",
    x=Param("number", "Point ahead of Pupper's centre (base_link), metres."),
    y=Param("number", "Point left of Pupper's centre, metres; right is negative."),
    side=Param("string", "Which front leg kicks: right, left, or auto (the side the point is on).",
               enum=list(SIDES), required=False),
    dry_run=Param("boolean", "Turn and walk, no kick. Default false.", required=False),
    gain=Param("number", "Walker travel per commanded metre at 0.2 m/s. Default 1.31.", required=False),
    delay=Param("number", "Walker start delay, seconds. Default 0.14.", required=False),
    angle=Param("number", "Kick heading in degrees from the heading at the start, left positive.", required=False),
    point=Param("string", "The point is the ball centre (default) or where Pupper stands.",
                enum=list(kick_plan.POINTS), required=False),
    speed=Param("number", "Walk speed command, m/s. Default 0.2 (the gain was measured there).", required=False),
    settle=Param("number", "Zero-speed time before the kick, seconds. Default 0.15.", required=False),
    _ends_following=True,
)
async def kick_at_tool(robot: Robot, x: float, y: float, side: str = "auto", dry_run: bool = False,
                       gain: float = kick_plan.FWD_GAIN, delay: float = kick_plan.DEAD_S, angle: float | None = None,
                       point: str = "ball", speed: float = kick_plan.SPEED, settle: float = KICK_SETTLE_S) -> Result:
    return await kick_at(robot, x, y, side, dry_run, gain, delay, angle, point, speed, settle)


@tool(
    "locate_ball",
    "Look for the soccer ball with the camera for about one second without moving, and report its distance and"
    " direction. Use when asked where the ball is or whether Pupper can see it. It does not turn to search;"
    " kick_ball searches. Not a physical measurement.",
)
async def locate_ball_tool(robot: Robot) -> Result:
    if not kick_ball_available(robot):
        return Result(False, "The ball tools are unavailable: the leg kick trick is not installed.")
    return await locate_ball(robot)


@tool(
    "kick_ball",
    "Find the soccer ball with the camera, walk to it, line up one front paw and kick it, then stand on the"
    " walker. Use for requests such as go find the ball and kick it, or kick the ball with your left leg."
    " One call does the whole sequence; do not add move, turn, play_trick or stand_up. It searches by"
    " turning in place (commanded up to about one full turn), and it ends without a kick if the ball is not"
    " found or is lost. Not collision avoidance: a person must keep the area clear. Queued is acceptance; the"
    " final result reports the controller sequence, not whether the ball was hit.",
    side=Param("string", "Which front leg kicks: right, left, or auto (the side the ball is first seen on;"
                         " a centred ball gets the right leg). Use auto unless the person names a leg.",
               enum=list(SIDES), required=False),
    _ends_following=True,
)
async def kick_ball_tool(robot: Robot, side: str = "auto") -> Result:
    return await kick_ball(robot, side)
