"""Final project: line up on a near ball with the camera and kick it, as one job.

kick_seen(side) walks to a ball that the camera sees, stops with the ball at the
kick point of one front paw (lab/ball_lineup.py), and plays the leg kick with no
wait. The far position of the ball is the field camera's task; this tool does
the last metre. look_ball() reports what the detector sees, with no motion.

Two detectors find the ball (values file: detector, or the tool option):
  fc   (default) The Pupper FC ball detector of the camera node
       (follow_me_pupper/robot/pink_ball.py: pink color and the Hailo YOLO
       "sports ball" boxes) publishes one row for each frame on /fc/onboard_ball.
       Its box in the 220 deg panorama is the seed of the Ball Eye outline fit on
       the same frame, which gives the position. That detector does not report
       a ball nearer than approximately 0.28 m ahead (the ball is stretched at
       the pole of the panorama), so near the kick point the seed is the
       position that the track predicts.
  eye  Ball Eye's own YOLO on the floor view (lab/ball_eye.py; needs the model file).

The measurement runs in a worker thread. A Stop, a fall or a motion block ends
the job at the next 50 ms tick with a zero speed command.

Files on the Pupper (no code change to set them):
  model   PUPPER_BALL_EYE_MODEL, default ~/.config/gemini-pupper/ball_eye.onnx
          (.hef: Hailo-8; .onnx: CPU through OpenCV)
  values  ~/.config/gemini-pupper/ball_eye.yaml: detector (fc or eye), need_yolo
          (fc: use only rows that the YOLO box confirms), ball_radius, conf, view_size,
          yolo_size (320 with the 320 model on the CPU; the view stays 640),
          reduce, mount {x, y, z, tilt_up_deg, yaw_deg}, lineup {LineupParams fields}
  record  PUPPER_BALL_EYE_RECORD=<folder>: each job writes its fixes and commands
          (JSON lines) and every fifth floor view (JPEG, training data) there.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import threading
import time
from collections import Counter, deque
from dataclasses import replace
from pathlib import Path
from statistics import median

import numpy as np

from ball_eye import BallEye, BallTrack, EyeParams, Mount, Pano, View, load_backend, load_lens
from ball_lineup import Lineup, LineupParams
from part3_follow import prepare_to_move
from part9_ball_kick import KICK_MOTION_END_S, kick_time, kick_trick, play_kick
from robot_api import (FELL_MESSAGE, LLM_CMD_RATE, LOCOMOTION, LOCOMOTION_CONTROLLER, MotionBlockedError, Result,
                       Robot, with_note)
from tool_registry import Param, tool

log = logging.getLogger("kick_seen")

MODEL_PATH = Path(os.environ.get("PUPPER_BALL_EYE_MODEL", "~/.config/gemini-pupper/ball_eye.onnx")).expanduser()
CONFIG_PATH = Path(os.environ.get("PUPPER_BALL_EYE_CONFIG", "~/.config/gemini-pupper/ball_eye.yaml")).expanduser()
RECORD_DIR = os.environ.get("PUPPER_BALL_EYE_RECORD", "")
CAMERA_MAX_AGE_S = 1.0
FRAME_LATENCY_S = 0.13  # used only when a frame has no usable stamp (measured 115 to 166 ms at 10 Hz)
DETECTORS = ("fc", "eye")
FC_BALL_TOPIC, FC_BALL_SCHEMA = "/fc/onboard_ball", "fc_onboard_ball_v1"  # follow_me_pupper/robot/onboard_ball.py
FC_ROW_WAIT_S = 0.20  # a row comes 62 ms (median) after the capture (Pupper, 2026-11-20); later: the frame has no row
FC_ROWS_MAX_AGE_S = 1.5  # no row for this long: the detector does not run
FC_HOLD_S = 2.0  # with no fresh track, the last fix is the seed for this long
_EYE = None  # ((params, detector), BallEye): one detector for the server, the model loads one time
_FC_ROWS = None  # one subscription for the server


def load_params() -> tuple[EyeParams, LineupParams]:
    data = {}
    if CONFIG_PATH.is_file():
        import yaml

        data = yaml.safe_load(CONFIG_PATH.read_text()) or {}
    eye = EyeParams()
    eye = replace(eye, **{k: data[k] for k in ("ball_radius", "conf", "reduce", "refine", "base_height", "yolo_size") if k in data},
                  mount=replace(Mount(), **data.get("mount", {})), view=View(size=int(data.get("view_size", 640))))
    line = replace(LineupParams(ball_radius=eye.ball_radius), **data.get("lineup", {}))
    return eye, line


def load_source() -> tuple[str, bool]:
    """(detector, need_yolo) from the values file."""
    data = {}
    if CONFIG_PATH.is_file():
        import yaml

        data = yaml.safe_load(CONFIG_PATH.read_text()) or {}
    detector = str(data.get("detector", "fc"))
    if detector not in DETECTORS:
        raise ValueError(f"detector must be fc or eye, not {detector!r}")
    return detector, bool(data.get("need_yolo", False))


def get_eye(params: EyeParams, detector: str = "eye") -> BallEye:
    global _EYE
    if _EYE is None or _EYE[0] != (params, detector):
        if detector == "fc":  # the Pupper FC detector finds the ball: no model of this module
            eye = BallEye(None, params, load_lens())
        elif not MODEL_PATH.is_file():
            raise FileNotFoundError(f"no Ball Eye model at {MODEL_PATH}")
        else:
            eye = BallEye(load_backend(MODEL_PATH, params.conf), params, load_lens())
        _EYE = ((params, detector), eye)
    return _EYE[1]


class EyeIO:
    """Newest camera frame -> BallFix, in a worker thread. poll() returns each new fix one time."""

    def __init__(self, robot: Robot, eye: BallEye, record: Path | None = None):
        self.robot, self.eye, self.record = robot, eye, record
        self.frames = self.fixes = 0
        self.ages: list[float] = []  # capture to fix, s
        self.ms: list[dict] = []
        self._last_msg = None
        self._busy = None  # the running detection
        self._lines = []

    def capture_time(self, msg, received: float) -> float:
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.robot.node.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp > 0 and 0.0 <= age <= 2.0:
                return self.robot.now() - age
        except (AttributeError, TypeError):
            pass
        return received - FRAME_LATENCY_S

    def problem(self) -> str | None:
        frame = self.robot.latest_camera_frame
        if frame is None:
            return "no camera frame has arrived"
        age = self.robot.now() - frame[1]
        return f"camera frames stopped ({age:.1f} s since the last frame)" if age > CAMERA_MAX_AGE_S else None

    def note(self, **row) -> None:
        if self.record is not None:
            self._lines.append(json.dumps({"t": round(self.robot.now(), 3), **row}))

    async def poll(self):
        """The fix of a finished detection (or None), and start the next detection."""
        fix = None
        if self._busy is not None and self._busy.done():
            task, self._busy = self._busy, None
            fix = task.result()
        frame = self.robot.latest_camera_frame
        if self._busy is None and frame is not None and frame[0] is not self._last_msg:
            msg, received = frame
            self._last_msg = msg
            self._busy = self._start(bytes(msg.data), self.capture_time(msg, received))
        return fix

    def _start(self, jpeg: bytes, stamp: float):
        """One detection in a worker thread: the event loop and Stop stay responsive. Tests replace this."""
        return asyncio.ensure_future(asyncio.to_thread(self._detect, jpeg, stamp))

    def _measure(self, jpeg: bytes, stamp: float):
        return self.eye.detect(jpeg, stamp)

    def _detect(self, jpeg: bytes, stamp: float):
        try:
            fix = self._measure(jpeg, stamp)
        except Exception:
            log.exception("ball detection failed on one frame")
            return None
        self.frames += 1
        if self.record is not None and self.frames % 5 == 1 and self.eye.view_rgb is not None:
            import cv2

            cv2.imwrite(str(self.record / f"view_{self.frames:05d}.jpg"), cv2.cvtColor(self.eye.view_rgb, cv2.COLOR_RGB2BGR))
        if fix is None:
            self.note(frame=self.frames, fix=None, why=self.eye.note)
            return None
        self.fixes += 1
        self.ages.append(self.robot.now() - stamp)
        self.ms.append(fix.ms)
        self.note(frame=self.frames, x=round(fix.x, 4), y=round(fix.y, 4), z=round(fix.z, 4), method=fix.method,
                  source=fix.source, score=round(fix.score, 2), stamp=round(stamp, 3),
                  ms={k: round(v, 1) for k, v in fix.ms.items()})
        return fix

    def summary(self) -> dict:
        out = {"frames": self.frames, "fixes": self.fixes}
        if self.ages:
            out["age_ms_median"] = round(1e3 * median(self.ages))
            out["stage_ms_median"] = {k: round(median(m[k] for m in self.ms), 1) for k in self.ms[0]}
        return out

    def close(self) -> None:
        if self.record is not None and self._lines:
            (self.record / "log.jsonl").write_text("\n".join(self._lines) + "\n")


def stamp_ns(msg) -> int:
    try:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return 0


class FcRows:
    """Rows of the Pupper FC onboard ball detector, by the capture stamp of their frame [ns]."""

    def __init__(self, robot: Robot):
        from std_msgs.msg import String

        self.robot, self.node = robot, robot.node
        self.rows: dict[int, dict] = {}
        self.count, self.states = 0, Counter()
        self.since, self.last = robot.now(), None
        self._lock = threading.Lock()
        robot.node.create_subscription(String, FC_BALL_TOPIC, self._on_row, 5)

    def _on_row(self, msg) -> None:
        try:
            row = json.loads(msg.data)
            stamp, state = row["captured_at_ros_ns"], str(row["state"])
            if row["schema"] != FC_BALL_SCHEMA or type(stamp) is not int:
                return
            keep = {"state": state}
            if state == "tracked":
                box = [float(v) for v in row["box"]]
                if len(box) != 4 or not (0.0 <= box[0] < box[2] <= 1.0 and 0.0 <= box[1] < box[3] <= 1.0):
                    return
                keep.update(box=box, width=int(row["projection_width"]), source=str(row["source"]))
        except (KeyError, ValueError, TypeError):
            return
        with self._lock:
            self.rows[stamp] = keep
            while len(self.rows) > 16:
                del self.rows[next(iter(self.rows))]
            self.count += 1
            self.states[state] += 1
            self.last = self.robot.now()

    def get(self, stamp: int) -> dict | None:
        with self._lock:
            return self.rows.get(stamp)

    def problem(self) -> str | None:
        last = self.since if self.last is None else self.last
        if self.robot.now() - last <= FC_ROWS_MAX_AGE_S:
            return None
        return (f"the Pupper FC ball detector publishes no row on {FC_BALL_TOPIC}" if self.last is None
                else f"the rows on {FC_BALL_TOPIC} stopped")


def fc_rows(robot: Robot) -> FcRows:
    global _FC_ROWS
    if _FC_ROWS is None or _FC_ROWS.node is not robot.node:
        _FC_ROWS = FcRows(robot)
    return _FC_ROWS


class FcBallIO(EyeIO):
    """The ball that the Pupper FC detector finds, measured by the Ball Eye outline fit.

    Each camera frame waits for the detector's row of the same capture stamp. A
    tracked row gives the seed (its box as a cone of camera rays). With no ball
    in the row, or no row in time, the seed is the position that the track of
    the earlier fixes predicts: only near the camera, and only with an outline fit."""

    def __init__(self, robot: Robot, eye: BallEye, record: Path | None = None, need_yolo: bool = False):
        super().__init__(robot, eye, record)
        self.rows, self.need_yolo = fc_rows(robot), need_yolo
        self.track, self.last_fix = BallTrack(), None
        self.pending: deque = deque(maxlen=6)  # frames with no measurement yet: (message, time received)
        self.seeds, self.sources = Counter(), Counter()
        self._seeds = []  # the seeds of the running measurement

    def problem(self) -> str | None:
        return super().problem() or self.rows.problem()

    async def poll(self):
        fix = None
        if self._busy is not None and self._busy.done():
            task, self._busy = self._busy, None
            fix = task.result()
            if fix is not None:
                self.track.update(fix)
                self.last_fix = fix
                self.sources[f"{fix.source}/{fix.method}"] += 1
        frame = self.robot.latest_camera_frame
        if frame is not None and frame[0] is not self._last_msg:
            self._last_msg = frame[0]
            self.pending.append(frame)
        if self._busy is None and (job := self._next()) is not None:
            msg, stamp, self._seeds = job
            self._busy = self._start(bytes(msg.data), stamp)
        return fix

    def _next(self):
        """The newest frame that has its row, or whose row did not come in time: (message, capture time, seeds)."""
        now = self.robot.now()
        for i in range(len(self.pending) - 1, -1, -1):
            msg, received = self.pending[i]
            row = self.rows.get(stamp_ns(msg))
            if row is None and now - received < FC_ROW_WAIT_S:
                continue  # the row can come; an older frame can have its row
            for _ in range(i + 1):
                self.pending.popleft()
            stamp = self.capture_time(msg, received)
            seeds = [seed for seed in (self._row_seed(row), self._track_seed(stamp)) if seed is not None]
            state = row["state"] if row is not None else "no row"
            self.seeds[seeds[0][4] if seeds else f"none ({state})"] += 1
            if not seeds:
                self.eye.note = f"no seed: detector row {state}, no near track"
                self.note(fix=None, why=self.eye.note)
                return None
            return msg, stamp, seeds
        return None

    def _row_seed(self, row):
        if row is None or row["state"] != "tracked" or (self.need_yolo and row["source"] != "pink+yolo"):
            return None
        axis, rho, whole = Pano(width=row["width"]).cone(row["box"])
        return axis, rho, whole, False, row["source"]

    def _track_seed(self, stamp: float):
        p = self.eye.p
        s = self.track.predict(stamp) if self.track.hits >= 2 else None
        if s is None and self.last_fix is not None and 0.0 <= stamp - self.last_fix.stamp <= FC_HOLD_S:
            s = (self.last_fix.x, self.last_fix.y)
        if s is None:
            return None
        ray = p.mount.to_optical((s[0], s[1], p.ball_radius - p.base_height))
        rng = float(np.linalg.norm(ray))
        if rng > p.seed_fit_range:  # far from the camera the detector's rows are the only source
            return None
        return ray / rng, math.asin(min(1.0, p.ball_radius / rng)), False, True, "track"

    def _measure(self, jpeg: bytes, stamp: float):
        for axis, rho, whole, must_fit, source in self._seeds:
            fix = self.eye.detect_seed(jpeg, axis, rho, stamp, whole=whole, must_fit=must_fit,
                                       score=1.0 if source == "pink+yolo" else 0.5, source=source)
            if fix is not None:
                return fix
        return None

    def summary(self) -> dict:
        return {**super().summary(), "rows": self.rows.count, "row_states": dict(self.rows.states),
                "seeds": dict(self.seeds), "fix_sources": dict(self.sources)}


def make_io(robot: Robot, eye: BallEye, detector: str, need_yolo: bool) -> EyeIO:
    if detector == "fc":
        return FcBallIO(robot, eye, record_folder(), need_yolo)
    return EyeIO(robot, eye, record_folder())


def pick_detector(detector: str) -> tuple[str, bool]:
    """The tool option, or the values file when the option is empty."""
    source, need_yolo = load_source()
    if detector:
        if detector not in DETECTORS:
            raise ValueError(f"detector must be fc or eye, not {detector!r}")
        source = detector
    return source, need_yolo


def record_folder() -> Path | None:
    if not RECORD_DIR:
        return None
    path = Path(RECORD_DIR).expanduser() / time.strftime("%Y%m%d-%H%M%S")
    path.mkdir(parents=True, exist_ok=True)
    return path


async def look_ball(robot: Robot, seconds: float = 1.5, detector: str = "") -> Result:
    """Detect without motion and report the median fix, the fix rate and the delay."""
    try:
        eye_params, _ = load_params()
        source, need_yolo = pick_detector(detector)
        io = make_io(robot, get_eye(eye_params, source), source, need_yolo)
    except Exception as error:
        return Result(False, f"Ball Eye is not ready: {type(error).__name__}: {error}")
    if problem := io.problem():
        return Result(False, f"Ball Eye needs the camera: {problem}")
    fixes, start = [], robot.now()
    while robot.now() - start < seconds:
        fix = await io.poll()
        if fix is not None:
            fixes.append(fix)
        await robot.sleep(0.01)
    io.close()
    seen = {"detector": source, **io.summary()}
    if problem := io.problem():  # the detector's rows did not come during the look
        return Result(False, f"Ball Eye needs the camera: {problem}", {"ball_eye": {"seen": False, **seen}})
    if len(fixes) < 2:
        return Result(True, f"Ball not seen ({seen}; last note: {io.eye.note or 'none'}).", {"ball_eye": {"seen": False, **seen}})
    x, y, z = (median(getattr(f, k) for f in fixes) for k in "xyz")
    spread = max(math.hypot(f.x - x, f.y - y) for f in fixes)
    methods = sorted({f.method for f in fixes})
    return Result(True, f"Ball at x {x:.3f} m, y {y:+.3f} m (z {z:+.3f}); spread {spread * 1e3:.0f} mm; {'/'.join(methods)};"
                        f" {seen}. Camera estimate, not a physical measurement.",
                  {"ball_eye": {"seen": True, "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
                                "spread_mm": round(spread * 1e3, 1), "methods": methods, **seen}})


async def kick_seen(robot: Robot, side: str = "auto", dry_run: bool = False, leave: float = KICK_MOTION_END_S,
                    detector: str = "") -> Result:
    """Line up on the ball that the camera sees and kick (finished)."""
    if side not in ("right", "left", "auto") or type(dry_run) is not bool:
        return Result(False, "side must be right, left or auto, and dry_run true or false")
    trick = kick_trick(robot)
    if trick is None:
        return Result(False, "The leg kick trick is not installed, so kick_seen is unavailable.")
    if robot.fallen:
        return Result(False, FELL_MESSAGE)
    try:
        eye_params, line_params = load_params()
        source, need_yolo = pick_detector(detector)
        eye = get_eye(eye_params, source)
    except Exception as error:
        return Result(False, f"Ball Eye is not ready: {type(error).__name__}: {error}")
    limits = robot.speeds(LOCOMOTION).limits
    forward = limits.vx_forward_max if limits.vx_forward_max is not None else limits.vx_max
    line_params = replace(line_params, v_far=min(line_params.v_far, forward), v_near=min(line_params.v_near, forward),
                          v_side=min(line_params.v_side, limits.vy_max),
                          w_max=min(line_params.w_max, math.radians(limits.wz_max)))
    note = await prepare_to_move(robot)
    if isinstance(note, Result):
        return note

    async def job() -> Result:
        if robot.fallen:
            return Result(False, FELL_MESSAGE)
        if kick_trick(robot) is not trick:
            return Result(False, "The leg kick trick changed before the kick.")
        res = await robot.switch_to(LOCOMOTION_CONTROLLER)
        if not res.ok:
            return res
        io = make_io(robot, eye, source, need_yolo)
        line = Lineup(line_params, side)
        early, cmd = None, None
        try:
            while early is None:
                if robot.fallen:
                    early = FELL_MESSAGE
                elif robot.stopped_during_call():
                    early = "stopped"
                elif problem := (io.problem() or robot.motion_block_reason()):
                    early = problem
                else:
                    fix = await io.poll()
                    if fix is not None:
                        line.add_fix(fix)
                    cmd = line.step(robot.now())
                    if cmd.kick or cmd.done:
                        break
                    robot.send_speed(cmd.vx, cmd.vy, math.degrees(cmd.wz))
                    io.note(cmd=[round(cmd.vx, 3), round(cmd.vy, 3), round(math.degrees(cmd.wz), 1)], phase=cmd.phase)
                    await robot.sleep(1.0 / LLM_CMD_RATE)
        except MotionBlockedError as error:
            early = str(error)
        finally:
            robot.send_speed(0, 0, 0)
        for text in line.log:
            log.info("kick_seen %s", text)
        io.note(log=line.log)
        io.close()
        details = {"kick_seen": {"kicked": False, "detector": source, "side": line.side, "pulses": line.pulses,
                                 "log": line.log[-10:],
                                 "gain": {k: round(v, 2) for k, v in line.gain.items()}, **io.summary(),
                                 **({k: round(v, 4) if isinstance(v, float) else v for k, v in cmd.info.items()} if cmd else {})}}
        if early:
            return Result(False, f"kick_seen stopped before the kick: {early}; zero speed requested", details)
        if cmd.done:  # the ball was not seen, or was lost: a normal end, not a fault
            return Result(True, f"kick_seen ended without a kick: {cmd.done}; walking controller still active.", details)
        clip = f"kick_{cmd.kick}"
        where = f"ball at x {cmd.info['x']:.3f} m, y {cmd.info['y']:+.3f} m"
        if dry_run:
            return Result(True, f"dry run: lined up for {clip} ({where}); no kick played.", details)
        kicked = await play_kick(robot, trick, clip, leave)  # the robot is at rest: no settle time
        if not kicked.ok:
            return Result(False, kicked.message, details)
        details["kick_seen"]["kicked"] = True
        return Result(True, f"{clip} controller playback confirmed after the camera line-up ({where}, {line.pulses}"
                            f" pulses); locomotion controller active. Ball contact was not measured.", details)

    estimate = robot.transition_time_estimate(LOCOMOTION_CONTROLLER) + 10.0 + (0.0 if dry_run else kick_time(trick))
    return with_note(note, await robot.queue(f"kick_seen {side}", job, estimate, then=LOCOMOTION))


@tool(
    "look_ball",
    "Operator tool, not offered to Gemini: detect the pink ball with the camera for 1.5 seconds without motion"
    " and report its position from Pupper's centre, the fix rate and the frame delay.",
    detector=Param("string", "What finds the ball: fc (the Pupper FC detector's rows) or eye (Ball Eye's YOLO)."
                             " Default: the values file, else fc.", enum=list(DETECTORS), required=False),
)
async def look_ball_tool(robot: Robot, detector: str = "") -> Result:
    return await look_ball(robot, detector=detector)


@tool(
    "kick_seen",
    "Operator tool, not offered to Gemini: walk to the pink ball that the camera sees (within approximately"
    " 1.5 m ahead), stop with the ball at the kick point of one front paw, and play the leg kick, then return"
    " to the walking controller. A person must keep the area clear.",
    side=Param("string", "Which front leg kicks: right, left, or auto (the side the ball is on).",
               enum=["right", "left", "auto"], required=False),
    dry_run=Param("boolean", "Line up, no kick. Default false.", required=False),
    leave=Param("number", "Seconds into the kick clip at which the walker takes the legs back. Default 2.6.",
                required=False),
    detector=Param("string", "What finds the ball: fc (the Pupper FC detector's rows) or eye (Ball Eye's YOLO)."
                             " Default: the values file, else fc.", enum=list(DETECTORS), required=False),
    _ends_following=True,
)
async def kick_seen_tool(robot: Robot, side: str = "auto", dry_run: bool = False,
                         leave: float = KICK_MOTION_END_S, detector: str = "") -> Result:
    return await kick_seen(robot, side, dry_run, leave, detector)
