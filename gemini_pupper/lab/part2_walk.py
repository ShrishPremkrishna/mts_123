"""Part 2: Walk. TODO 2 (check_move), TODO 3 (the turn and move tools), then TODO 4 (arc).

Read it top to bottom: check_move (yours) decides whether a move is allowed;
walk (finished: read it) does a move on the robot; turn, move and arc (yours)
are what Gemini calls, and they call walk.

    "walk forward" -> move (TODO 3) -> walk -> check_move (TODO 2) -> the job, later
"""

from __future__ import annotations

import math

from basics import PREPARES_MOVE
from part3_follow import prepare_to_move
from robot_api import FELL_MESSAGE, LIMITS, LLM_CMD_RATE, LOCOMOTION, LOCOMOTION_CONTROLLER, MoveLimits, Result, Robot, not_written, with_note  # noqa: F401
from tool_registry import Param, tool

# "Run" means at least this fast, in m/s (the locomotion policy switches to its
# running gait above about 0.5 m/s).
RUN_MIN_SPEED = 1.0
RUN_DEFAULT_SPEED = 1.2
# The default walking and turning speeds come from robot.speeds(): 0.4 m/s and
# 60 deg/s in locomotion mode, and a walking trick's own (the spider's are slower).


# ============================================================ what the robot does
def check_move(vx: float, vy: float, wz_deg: float, duration: float, limits: MoveLimits = LIMITS) -> str | None:
    """None if the robot may make this move, else the reason it may not.

    vx forward m/s, vy left m/s, wz_deg counter-clockwise deg/s, duration in s.

    TODO 2 (Part 2) ------------------------------------------------------------
    Gemini reads:        the string you return, word for word, as the tool's
                         reply. "too fast" leaves Gemini guessing; "too fast:
                         |vx|<=1.5 m/s" lets it retry at a speed that works.
    Must always be true: refuse anything too fast, too slow to move at all, a
                         duration that isn't more than 0 and at most
                         duration_max, and a move that would travel farther
                         than distance_max. Look at every field of MoveLimits
                         (robot/robot_api.py).
    Why here:            the tool descriptions ask Gemini to stay within the
                         limits; this makes sure of it.
    Tests check:         - good moves (even exactly at a limit) -> None
                         - each bad move -> a non-empty reason
                         - the reason contains the limit's number, as Python
                           prints it: vx_max "1.5", vy_max "0.5", wz_max
                           "114.0", duration_max "20.0", distance_max "7.0".
                           Passes: f"... at most {limits.duration_max} s"
                           Fails:  "too long", or "{limits.duration_max:.0f}" ("20")
    Test:                python3 lab/test.py todo2
    """
    # ========== TODO 2: YOUR CODE HERE ==========
    # Check the rules one at a time, and return the first reason you find.
    # Each reason says what was wrong AND the limit, e.g. f"too fast: limits are |vx|<={limits.vx_max} m/s, ..."
    # 1. Too fast: abs(vx) > limits.vx_max, or abs(vy) > limits.vy_max, or abs(wz_deg) > limits.wz_max.
    # 2. Too slow to move at all: vx, vy AND wz_deg are ALL below their minimums (vx_min, vy_min, wz_min).
    #    (Only all three: a turn in place has vx = vy = 0, and that is fine.)
    # 3. The duration must be more than 0 and at most limits.duration_max.
    # 4. Too far: speed * duration > limits.distance_max, where speed = (vx**2 + vy**2) ** 0.5.
    # 5. Nothing wrong: return None.
    values = (vx, vy, wz_deg, duration)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return "motion values must be finite numbers (m/s, degrees/s and seconds)"
    forward = limits.vx_forward_max if limits.vx_forward_max is not None else limits.vx_max
    backward = limits.vx_backward_max if limits.vx_backward_max is not None else limits.vx_max
    if vx > forward or vx < -backward or abs(vy) > limits.vy_max or abs(wz_deg) > limits.wz_max:
        if limits.vx_forward_max is not None or limits.vx_backward_max is not None:
            return f"too fast: -{backward}<=vx<={forward} m/s, |vy|<={limits.vy_max} m/s, |wz|<={limits.wz_max} deg/s"
        return f"too fast: |vx|<={limits.vx_max} m/s, |vy|<={limits.vy_max} m/s, |wz|<={limits.wz_max} deg/s"
    if abs(vx) < limits.vx_min and abs(vy) < limits.vy_min and abs(wz_deg) < limits.wz_min:
        return f"too slow: at least |vx|>={limits.vx_min} m/s, |vy|>={limits.vy_min} m/s or |wz|>={limits.wz_min} deg/s"
    if not 0 < duration <= limits.duration_max:
        return f"duration must be more than 0 and at most {limits.duration_max} s"
    if math.hypot(vx, vy) * duration > limits.distance_max:
        return f"too far: one move travels at most {limits.distance_max} m"
    return None
    # ====================================


async def walk(robot: Robot, vx: float, vy: float, wz_deg: float, duration: float) -> Result:
    """Walk at (vx, vy, wz_deg) for duration seconds, then stand still (finished: read it).

    vx forward m/s, vy left m/s, wz_deg counter-clockwise deg/s. Your turn and
    move tools (TODO 3) call this. It is stand_up's shape (basics.py), with a
    loop in the job. Two things to notice:

    - Why send the speed again and again? The locomotion policy doesn't take
      "walk for 2 seconds"; it takes a speed, over and over. In between sits
      cmd_vel_mux (the node that lets the joystick win over Gemini), and it
      forgets a speed it hasn't heard for 0.5 s, so a crashed program can't
      leave Pupper walking forever. So the job sends it LLM_CMD_RATE (20) times
      a second for the whole walk, then a zero to stand still.
    - Why check robot.fallen three times? The robot may already have fallen when
      Gemini asks (1), may fall while this job waits in line (2), or while it
      walks (3). Each time, Gemini should hear FELL_MESSAGE, not "moved".
    - Who walks, and how fast? Usually the locomotion policy. But in a trick
      that walks in its own stance (the spider, Part 6), its own walking policy
      does, with its own limits (robot.speeds(): from its trick.yaml), so a move
      that is too fast for it is refused with its numbers. Both depend on the
      state Pupper will be in when the job runs: robot.state_after_queue, the
      same now-question as Part 5.
    """
    # NOW, before anything is queued:
    state = robot.state_after_queue
    if getattr(state, "walks", False) and not robot.trick_available(state):
        return Result(False, "This walking stance is locked; stand_up leaves it safely, or unlock before moving in it.")
    speeds = robot.speeds()  # the limits where Pupper will be (a walking trick has its own)
    refused = check_move(vx, vy, wz_deg, duration, speeds.limits)  # your TODO 2: is this move allowed?
    if refused:
        return Result(False, refused + speeds.where)  # Gemini hears why; nothing is queued
    note = await prepare_to_move(robot)  # get ready: does nothing until your TODO 5 (Part 3)
    if isinstance(note, Result):
        return note
    state = robot.state_after_queue  # where Pupper will be when this job runs
    walker = robot.walking_controller(state)  # the locomotion policy, or a walking trick's own
    then = state if walker != LOCOMOTION_CONTROLLER else LOCOMOTION  # a walking trick stays in its stance
    if robot.fallen:  # (1)
        return Result(False, FELL_MESSAGE)

    async def job() -> Result:  # LATER, when the queue gets to it
        if robot.fallen:  # (2)
            return Result(False, FELL_MESSAGE)
        if getattr(state, "walks", False) and not robot.trick_available(state):
            return Result(False, "The walking stance was locked before this motion started; stand_up leaves it safely.")
        # A previous job may have failed or a person may have changed controllers.
        # A gait-specific command must never silently run on a different gait.
        if walker != LOCOMOTION_CONTROLLER and robot.state_now is not state:
            return Result(False, f"{state.name} is no longer running; play it first")
        refused_now = check_move(vx, vy, wz_deg, duration, robot.speeds(state).limits)
        if refused_now:
            return Result(False, refused_now)
        res = await robot.switch_to(walker)  # the policy that walks takes the legs (if it hasn't already)
        if not res.ok:
            return res
        end = robot.now() + duration
        try:
            while robot.now() < end - 1e-6:
                if robot.fallen:  # (3)
                    return Result(False, FELL_MESSAGE)
                robot.send_speed(vx, vy, wz_deg)
                await robot.sleep(min(1.0 / LLM_CMD_RATE, end - robot.now()))
        finally:
            robot.send_speed(0, 0, 0)  # also stop on cancellation or a failed publish/sleep
        # A sensor/stop callback can arrive during the last sleep, after the
        # loop's final check and before asynchronous cancellation cleans up.
        if robot.fallen:
            return Result(False, FELL_MESSAGE)
        if robot.stopped_during_call():
            return Result(False, "movement interrupted by a stop")
        return Result(True, f"moved {what} for {duration:.2f} s")  # the outcome (debug log)

    # What Gemini hears NOW: the job was accepted.
    what = f"vx={vx:g} vy={vy:g} wz={wz_deg:.3g}"  # 45.836623610465864 -> 45.8: Gemini reads this
    return with_note(note, await robot.queue(f"move {what} {duration:.3g}s", job, duration, then=then))


# ============================================================ what Gemini reads
# TODO 3: write turn first, then move: what Gemini reads (the description and every
# parameter's description) and their bodies.
#   - turn is the warm-up: one number to work out (which way), one call. It shows
#     you the whole path: Gemini's words -> numbers -> walk -> the robot.
#   - move is the real challenge: four parameters, defaults, and the "run" rule.
#   - Put the robot's real limits in the words: LIMITS.vx_max and the others
#     (robot/robot_api.py), RUN_MIN_SPEED, and the defaults. Add PREPARES_MOVE.
#   Tests check:
#   - each tool's description is 20+ characters, and every parameter is described
#   - turn left sends +wz, right -wz, about the angle asked for, with vx = vy = 0
#   - move's words contain the number LIMITS.vx_max ("1.5") and the word "follow"
#     (from PREPARES_MOVE). Passes: f"... up to {LIMITS.vx_max} m/s" + PREPARES_MOVE
#   - the move examples below send exactly those speeds
#   - move("right", run=True) says it walked: the reply says "walk".
#     Passes: "Sideways never runs; walking instead. move vx=0 vy=-0.4 ... queued"
#     Fails:  "move vx=0 vy=-0.4 ... queued" (Gemini then tells them it's running)
#   - a move check_move refuses comes back as a failure with check_move's reason
#   Test: python3 lab/test.py todo3  (turn is checked first, on its own)
# ========== TODO 3: YOUR CODE HERE ==========
@tool(
    "turn",
    "Turn in place left or right by a positive angle in degrees (default 90). Uses the current stance's turning speed."
    + PREPARES_MOVE,
    direction=Param("string", "Left or right relative to Pupper.", enum=["left", "right"]),
    angle=Param("number", "Positive degrees to turn, default 90; the duration limit still applies.", required=False),
)
async def turn_tool(robot: Robot, direction: str, angle: float = 90.0) -> Result:
    # Turn in place: no forward or sideways speed, just a turning speed, for as long
    # as the angle takes.
    # 1. The turning speed: turn_speed = robot.speeds().turn (60 deg/s walking; a
    #    walking trick has its own). wz_deg: +turn_speed to the left (counter-clockwise),
    #    -turn_speed to the right.
    # 2. How long it takes: seconds = abs(angle) / turn_speed
    #    (90 degrees at 60 deg/s takes 1.5 s)
    # 3. return await walk(robot, 0.0, 0.0, wz_deg, seconds)
    if direction not in ("left", "right"):
        return Result(False, "direction must be left or right")
    if not math.isfinite(angle) or angle <= 0:
        return Result(False, "angle must be a finite number greater than 0 degrees")
    turn_speed = robot.speeds().turn
    return await walk(robot, 0.0, 0.0, turn_speed if direction == "left" else -turn_speed, angle / turn_speed)


@tool(
    "move",
    "Move in one direction for a duration (default 1.5 s). {locomotion_motion_contract}"
    " Sideways always walks. Walking tricks use their own lower limits and may refuse running."
    + PREPARES_MOVE,
    direction=Param("string", "Direction relative to Pupper; left is positive sideways.", enum=["forward", "backward", "left", "right"]),
    run=Param("boolean", "Request a run forward/backward; sideways walks instead. Default false.", required=False),
    speed=Param("number", "Positive speed in m/s; omit for the current stance's default.", required=False),
    duration=Param("number", "Time in seconds, greater than 0 and at most 20; default 1.5.", required=False),
)
async def move_tool(
    robot: Robot,
    direction: str,
    run: bool = False,
    speed: float | None = None,
    duration: float = 1.5,
) -> Result:
    # Gemini may leave out run, speed and duration (then they have the defaults above:
    # speed is None when Gemini gave none). Work out ONE speed, then split it into vx and vy.
    # 1. The speed:
    #    - running (run is True AND the direction is forward or backward): at least
    #      RUN_MIN_SPEED; if Gemini gave no speed, RUN_DEFAULT_SPEED.
    #      (max(a, b) is the bigger of two numbers: max(0.6, RUN_MIN_SPEED) is 1.0.)
    #    - otherwise (walking, or any sideways move): speed, or robot.speeds().walk if
    #      Gemini gave none (0.4 m/s walking; a walking trick has its own).
    # 2. Split it by direction (the robot's convention: +x forward, +y LEFT):
    #        forward: vx = +speed, vy = 0      backward: vx = -speed, vy = 0
    #        left:    vx = 0, vy = +speed      right:    vx = 0, vy = -speed
    # 3. If Gemini asked to run sideways, you walk instead: say so, or Gemini will
    #    tell the person it's running. Start the reply with a note:
    #      note = "Sideways never runs; walking instead." (or "" otherwise)
    #      return with_note(note, await walk(robot, vx, vy, 0.0, duration))
    # Examples: move("forward") -> vx 0.4 | move("backward", run=True) -> vx -1.2 |
    #   move("forward", run=True, speed=0.6) -> vx 1.0 | move("right", run=True) -> vy -0.4
    if direction not in ("forward", "backward", "left", "right"):
        return Result(False, "direction must be forward, backward, left or right")
    if speed is not None and (not math.isfinite(speed) or speed <= 0):
        return Result(False, "speed must be a finite number greater than 0 m/s")
    state = robot.state_after_queue
    if getattr(state, "walks", False) and not robot.trick_available(state):
        return Result(False, "This walking stance is locked; stand_up leaves it safely, or unlock before moving in it.")
    if run and getattr(state, "walks", False) and not state.walk.run:
        limits = robot.speeds().limits
        return Result(False, f"{state.name} cannot run; it walks at most {limits.vx_max} m/s forward and {limits.vy_max} m/s sideways")
    sideways = direction in ("left", "right")
    magnitude = (max(RUN_MIN_SPEED, RUN_DEFAULT_SPEED if speed is None else speed)
                 if run and not sideways else robot.speeds().walk if speed is None else speed)
    vx = magnitude if direction == "forward" else -magnitude if direction == "backward" else 0.0
    vy = magnitude if direction == "left" else -magnitude if direction == "right" else 0.0
    note = "Sideways never runs; walking instead." if run and sideways else ""
    return with_note(note, await walk(robot, vx, vy, 0.0, duration))
# ====================================


# TODO 4: the arc tool: walk forward and turn at the same time, along part of a
# circle. turn sends only a turning speed and move only a straight one, so
# "go in a circle" was move, stop, turn, move. walk takes vx and wz_deg together,
# and the locomotion policy can do both at once: arc is how Gemini asks for it.
#   Now:                 work out vx and wz_deg from the radius and the speed, then
#                        walk (walk checks the move with check_move and queues it).
#   Gemini reads:        the words, to pick a radius the robot can do; then the
#                        reply: walk's, or why there is no arc (a negative radius,
#                        a speed of 0 or less, or too tight a circle for that
#                        speed), or check_move's reason (too long, too far).
#   Must always be true: an arc the robot can't do never moves it and never crashes.
#   Radius 0 is a sharp corner, not an error: Gemini plans a whole shape and sends
#   every call at once, and it often asks for corners as arc(radius=0). Refuse
#   them, and the sides still run: a rectangle becomes a straight line. So radius
#   0 turns in place (like turn), and the reply says so.
#   The math: around a circle of radius r (m) at speed v (m/s),
#     wz_deg = math.degrees(v / r)   (+ to the left, - to the right)
#     seconds = abs(angle) / abs(wz_deg)
#     e.g. 90 deg at 0.4 m/s around r = 0.5 m: wz = 45.8 deg/s for 1.96 s (0.79 m)
#   Put the real limits in the words: the robot turns at most LIMITS.wz_max deg/s,
#   so at speed v the radius must be at least v / math.radians(LIMITS.wz_max); one
#   arc goes at most LIMITS.distance_max m. Add PREPARES_MOVE.
#   Tests check:
#   - arc's description is 20+ characters, and every parameter is described
#   - its words contain LIMITS.distance_max ("7.0") and "follow" (from PREPARES_MOVE).
#     Passes: f"... One arc goes at most {LIMITS.distance_max} m." + PREPARES_MOVE
#     Fails:  "Walk in a curve." (Gemini then asks for circles the robot refuses)
#   - arc("left", angle=90, radius=0.5) sends vx = +0.4, vy = 0 and wz = +45.8 deg/s,
#     and turns about 90 deg; arc("right", ...) sends a negative wz, still forward
#   - arc("left", speed=0.6, radius=1.0) sends vx = 0.6 and wz = +34.4 deg/s
#   - arc("left", angle=90, radius=0) turns in place, left, about 90 deg, and the
#     reply says "place".
#     Passes: "Radius 0: turned in place (a sharp corner). move vx=0 ... queued"
#     Fails:  a crash (division by zero), or a refusal
#   - arc("left", radius=-1) is a failure that says "radius", and sends nothing
#   - arc("left", radius=0.1) is too tight (229 deg/s): a failure that says "radius"
#     and gives the smallest radius that works at that speed (0.2-0.3 m at 0.4 m/s).
#     Passes: "too tight: at 0.4 m/s the radius must be at least 0.25 m"
#     Fails:  check_move's "too fast: ... |wz|<=114.0 deg/s" (Gemini can't tell the
#             radius is the problem, and sends the same arc again), or a success
#             that quietly turned slower
#   Test: python3 lab/test.py todo4
# ========== TODO 4: YOUR CODE HERE ==========
@tool(
    "arc",
    "Walk forward along a left/right circular arc. Specify the positive angle in degrees (360 for a circle)."
    " Radius defaults to 0.5 m; radius 0 turns in place. {locomotion_arc_contract}"
    " Walking tricks have their own lower limits." + PREPARES_MOVE,
    direction=Param("string", "Which way the arc bends, relative to Pupper.", enum=["left", "right"]),
    angle=Param("number", "Positive degrees around the arc; required (90 is a quarter circle)."),
    radius=Param("number", "Radius in metres, default 0.5; zero makes a corner in place.", required=False),
    speed=Param("number", "Positive forward speed in m/s; omit for the current stance's walking speed.", required=False),
)
async def arc_tool(
    robot: Robot,
    direction: str,
    angle: float,
    radius: float = 0.5,
    speed: float | None = None,
) -> Result:
    # speed is None when Gemini gave none: use robot.speeds().walk (0.4 m/s walking).
    # 1. No circle to follow: if radius < 0 or speed <= 0, return Result(False, ...)
    #    saying which, and what works: "radius must be 0 m or more".
    #    Radius 0 is a sharp corner: turn in place like your turn tool (+-turn_speed,
    #    for abs(angle) / turn_speed s), and say so in front of walk's reply:
    #      return with_note("Radius 0: turned in place (a sharp corner).", await walk(...))
    # 2. Too tight: if math.degrees(speed / radius) > wz_max, return a failure
    #    with the smallest radius that works: speed / math.radians(wz_max), where
    #    wz_max = robot.speeds().limits.wz_max (LIMITS.wz_max walking).
    #    (check_move would refuse it too, but "|wz|<=114.0" doesn't tell Gemini that
    #    the radius is the problem.) Round up: f"{math.ceil(r * 100) / 100} m"
    # 3. wz_deg = math.degrees(speed / radius): + to the left, - to the right.
    # 4. seconds = abs(angle) / abs(wz_deg)
    # 5. return await walk(robot, speed, 0.0, wz_deg, seconds)  (walk checks it)
    # angle has no default: Gemini must say how much of the circle (left out, it
    # walked a quarter when asked for a circle). The tool registry refuses a call without it.
    # Examples: arc("left", 90) -> vx 0.4, wz +45.8 for 1.96 s |
    #   arc("right", angle=180, radius=1.0, speed=0.6) -> vx 0.6, wz -34.4 for 5.24 s
    if direction not in ("left", "right"):
        return Result(False, "direction must be left or right")
    if not math.isfinite(angle) or angle <= 0:
        return Result(False, "angle must be a finite number greater than 0 degrees")
    if not math.isfinite(radius) or radius < 0:
        return Result(False, "radius must be finite and at least 0 m")
    speed = robot.speeds().walk if speed is None else speed
    if not math.isfinite(speed) or speed <= 0:
        return Result(False, "speed must be finite and greater than 0 m/s")
    if radius == 0:
        return with_note("Radius 0: turning in place (a sharp corner).", await turn_tool(robot, direction, angle))
    wz_max = robot.speeds().limits.wz_max
    minimum_radius = speed / math.radians(wz_max)
    if radius < minimum_radius:
        return Result(False, f"radius too tight: at {speed:g} m/s use at least {math.ceil(minimum_radius * 100) / 100} m")
    wz = math.degrees(speed / radius)
    return await walk(robot, speed, 0.0, wz if direction == "left" else -wz, angle / wz)
# ====================================
