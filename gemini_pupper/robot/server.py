"""Gemini Pupper robot server: HTTP + WebSocket API on port 8000, and the web app.

Run on the Pupper after the robot stack is up (see start.sh):
    python3 robot/server.py

The web app gets Gemini's instructions and tools from GET /tools (lab/prompt.md
and the @tool functions in lab/) and sends every tool call to POST /tools/<name>. Every call
returns {"success": bool, "message": str}, which goes straight back to Gemini,
so Gemini hears about refused speeds, unknown tricks and failed controller
switches. The other endpoints are for the debug panel and for testing by hand.
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import re
import socket
import subprocess
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from time import monotonic

import psutil
import uvicorn
from fastapi import Body, Depends, FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from starlette.requests import HTTPConnection
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

import tool_registry as gemini_tools
from robot_api import CALL_STOPS, Robot
from runtime_config import component_dir, follower_config

behaviors = gemini_tools.load_lab()  # lab/: every tool, and the behaviors they call

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("server")

REPO = Path(__file__).resolve().parent.parent
WEB_DIST = REPO / "web" / "dist"
PROMPT_FILE = REPO / "lab" / "prompt.md"
BATTERY_SCRIPT = component_dir("pupperv3-monorepo", "PUPPER_MONOREPO_DIR") / "robot/utils/check_batt_voltage.py"

tools: Robot | None = None
FAKE_MODE = False  # only the explicit localhost fake_server.py entrypoint sets this


def runtime_info() -> dict:
    # A constructed API object does not prove a physical robot is connected.
    return {"runtime": "fake" if FAKE_MODE else "robot", "hardware_connected": False if FAKE_MODE else None,
            "command_epoch": getattr(tools, "command_epoch", None), "command_epoch_required": True}


class Detections:
    """Forwards person detections (ZMQ on localhost:5556) to browser WebSockets.

    The follower publishes them while it follows (it keeps the Hailo off
    otherwise); the monorepo detector, when used instead, publishes them always.
    """

    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()

    async def run(self) -> None:
        try:
            import zmq
            import zmq.asyncio
        except ImportError:
            log.warning("pyzmq missing; eye tracking from robot detections disabled")
            return
        sock = zmq.asyncio.Context().socket(zmq.SUB)
        sock.connect("tcp://127.0.0.1:5556")
        sock.subscribe(b"")
        while True:
            payload = (await sock.recv_multipart())[-1].decode("utf-8", errors="ignore")
            if payload.startswith("{"):
                for ws in list(self.clients):
                    try:
                        await ws.send_text(payload)
                    except Exception:
                        self.clients.discard(ws)


detections = Detections()


@asynccontextmanager
async def lifespan(app: FastAPI):
    global tools
    if FAKE_MODE:
        from gemini_chat import DeskBackend
        backend = DeskBackend(realtime=True)
        try:
            backend.setup()
            tools = backend.rig.ts
            await tools.ready.wait()
            log.warning("FAKE runtime: real queue timing, no physics, ROS transport or hardware")
            yield
        finally:
            try:
                await backend.aclose()
            finally:
                tools = None
        return
    follow_available = follower_config().mode != "disabled"
    tools = Robot()
    tools.follow_available = follow_available
    task = asyncio.create_task(detections.run())
    try:
        yield
    finally:
        task.cancel()
        try:
            with suppress(asyncio.CancelledError):
                await task
        except Exception:
            log.exception("detection forwarding failed; continuing robot shutdown")
        try:
            stopped = await tools.shutdown()
            (log.info if stopped.ok else log.error)("tool-server shutdown: %s", stopped.message)
        finally:
            tools = None


class CommandEpochError(HTTPException):
    """A received HTTP request was rejected before any command was submitted."""


async def require_command_epoch(connection: HTTPConnection):
    """Fence old HTTP intents; Stop and read-only calls stay independent.

    FastAPI resolves this dependency after parsing the request body. The caller's
    opaque token therefore rejects a delayed body or an entirely delayed request.
    Preserve that generation through dispatch as Stop may race the action lock.
    """
    if (connection.scope["type"] != "http"
            or connection.scope["method"] not in {"POST", "PUT", "PATCH", "DELETE"}
            or (connection.scope["method"] == "POST" and connection.url.path.rstrip("/") in {
                "/stop/immediate", "/queue/stop", "/tools/stop", "/tools/robot_status"})):
        yield
        return
    robot = T()
    expected = connection.headers.get("x-pupper-command-epoch")
    if expected != robot.command_epoch:
        reason = "missing" if expected is None else "stale"
        raise CommandEpochError(status_code=428 if expected is None else 409, detail={
            "success": False,
            "message": f"{reason} command epoch; no command was submitted. Refresh status and make a new request; do not retry this action automatically.",
            "command_epoch": robot.command_epoch, "command_epoch_required": True,
        })
    token = CALL_STOPS.set(robot.stops)
    try:
        yield
    finally:
        CALL_STOPS.reset(token)


app = FastAPI(title="Gemini Pupper robot server", lifespan=lifespan,
              dependencies=[Depends(require_command_epoch)])


@app.exception_handler(CommandEpochError)
async def command_epoch_error(_request, error: CommandEpochError):
    return JSONResponse(status_code=error.status_code, content=error.detail)

# The web app is served from this same origin; also allow a dev server on the laptop.
app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=r"https?://(localhost|127\.0\.0\.1)(:\d+)?",
    allow_methods=["*"],
    allow_headers=["*"],
)


def T() -> Robot:
    if tools is None:
        raise HTTPException(503, "tool server not ready")
    return tools


# ------------------------------------------------------------------ models
class MoveVelocity(BaseModel):
    forward_backward_velocity: float = Field(..., description="m/s, positive = forward")
    right_left_velocity: float = Field(..., description="m/s, positive = right")
    turning_velocity: float = Field(..., description="deg/s, positive = turn right")
    duration: float = Field(..., description="seconds")


class MoveDirection(BaseModel):
    heading: float = Field(..., description="degrees, 0 = forward, 90 = right")
    speed: float
    duration: float


class Trick(BaseModel):
    animation_name: str


class TrickAction(BaseModel):
    action: str


class Wait(BaseModel):
    duration: float


class Volume(BaseModel):
    volume: int = Field(..., ge=0, le=150)


# ------------------------------------------------------------------ Gemini's tools
def system_prompt() -> str:
    """lab/prompt.md without its <!-- comments -->."""
    text = PROMPT_FILE.read_text() if PROMPT_FILE.exists() else ""
    return re.sub(r"<!--.*?-->", "", text, flags=re.S).strip()


@app.get("/tools")
def tool_list():
    """What Gemini is told: the system prompt and the tool declarations."""
    return {"prompt": system_prompt(), "tools": gemini_tools.declarations(T())}


@app.post("/tools/{name}")
async def tool_call(name: str, args: dict = Body(default={})):
    res = await gemini_tools.call(T(), name, args)
    log.info("tool %s(%s) -> %s: %s", name, "[redacted]" if name == "unlock" else args,
             "ok" if res.ok else "FAILED", res.message)
    return {**res.as_dict(), **({"command_epoch": T().command_epoch, "command_epoch_required": True} if name == "stop" else {})}


# ------------------------------------------------------------------ commands
@app.post("/walking/activate")
async def walking_activate():
    return (await gemini_tools.call(T(), "stand_up", {})).as_dict()


@app.post("/walking/deactivate")
async def walking_deactivate():
    return (await gemini_tools.call(T(), "lie_down", {})).as_dict()


@app.post("/move/velocity")
async def move_velocity(req: MoveVelocity):
    # The browser uses right/clockwise-positive; ROS uses left/counter-clockwise-positive.
    res = await gemini_tools.dispatch(T(), lambda: behaviors.walk(T(), req.forward_backward_velocity,
                -req.right_left_velocity, -req.turning_velocity, req.duration), name="move/velocity")
    return res.as_dict()


@app.post("/move/direction")
async def move_direction(req: MoveDirection):
    async def run():
        # Validate both segments before queuing either; a rejected second segment
        # must not leave the robot executing half of the requested movement.
        limits = T().speeds().limits
        issue = behaviors.check_move(req.speed, 0.0, 0.0, req.duration, limits)
        if issue:
            from robot_api import Result
            return Result(False, issue)
        if req.heading:
            turn_speed = T().speeds().turn
            turn = await behaviors.walk(T(), 0.0, 0.0, -turn_speed if req.heading > 0 else turn_speed, abs(req.heading) / turn_speed)
            if not turn.ok:
                return turn
        return await behaviors.walk(T(), req.speed, 0.0, 0.0, req.duration)
    return (await gemini_tools.dispatch(T(), run, name="move/direction")).as_dict()


@app.post("/animation")
async def animation(req: Trick):
    return (await gemini_tools.call(T(), "play_trick", {"name": req.animation_name})).as_dict()


@app.post("/interactive_action")
async def interactive_action(req: TrickAction):
    return (await gemini_tools.call(T(), "interactive_action", {"action": req.action})).as_dict()


@app.get("/interactive_actions")
def interactive_actions():
    """Actions of interactive tricks (e.g. shake a paw while sitting): id -> trick and description."""
    return {aid: {"trick": t.name, "description": t.actions[clip]} for aid, (t, clip) in T().action_ids().items()}


@app.get("/animations")
def animations():
    return available_tricks()


def available_tricks() -> dict:
    """The tricks in <repo>/tricks, every one played by a policy."""
    return {
        name: {"description": t.description, "duration_s": round(t.total_time, 1), "kind": t.kind}
        for name, t in T().available_tricks().items()
    }


@app.post("/queue/wait")
async def queue_wait(req: Wait):
    return (await gemini_tools.dispatch(T(), lambda: T().wait(req.duration), name="wait")).as_dict()


@app.post("/queue/reset")
async def queue_reset():
    return (await gemini_tools.dispatch(T(), lambda: T().clear_queue(), name="clear_queue")).as_dict()


@app.post("/queue/stop")
@app.post("/stop/immediate")
async def stop_immediate():
    result = await T().immediate_stop(relax_playing_trick=True)
    return {**result.as_dict(), "command_epoch": T().command_epoch, "command_epoch_required": True}


@app.get("/queue/status")
def queue_status():
    return {**T().status(), **runtime_info()}


@app.post("/following/activate")
async def following_on():
    return (await gemini_tools.call(T(), "follow", {"enabled": True})).as_dict()


@app.post("/following/deactivate")
async def following_off():
    return (await gemini_tools.call(T(), "follow", {"enabled": False})).as_dict()


@app.get("/system/mode")
async def system_mode():
    return (await T().mode()).as_dict()


@app.get("/limits")
def limits():
    return T().speeds().limits.__dict__


# ------------------------------------------------------------------ sensing
# What Gemini sees: the fisheye frame undistorted to an equirectangular view, as
# Lab 5 hands the person detector (same lens model, FOV and 700-pixel width), so
# people look like people instead of bending around the edge of the lens.
CAMERA_PARAMS = Path(__file__).resolve().parent / "camera_params.yaml"
EQUIRECT_WIDTH = 700
EQUIRECT_FOV_DEG = 220.0
# The browser polls at 1 Hz; the course camera's configured frame interval is
# much shorter. Reject a stopped feed after two seconds without a ROS receipt.
# This is receipt age, not exposure time or proof a publisher is not replaying.
CAMERA_MAX_AGE_S = 2.0
camera_model = None


def undistort_jpeg(data: bytes) -> bytes | None:
    import cv2
    import numpy as np
    from fisheye_converter import equirect_height, fisheye_to_equirectangular, load_camera_model
    global camera_model
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    h, w = frame.shape[:2]
    if camera_model is None or (camera_model.width, camera_model.height) != (w, h):
        camera_model = load_camera_model(CAMERA_PARAMS, w, h)
    equirect = fisheye_to_equirectangular(frame, camera_model, EQUIRECT_WIDTH,
                                          equirect_height(EQUIRECT_WIDTH, EQUIRECT_FOV_DEG),
                                          h_fov_deg=EQUIRECT_FOV_DEG)
    ok, jpeg = cv2.imencode(".jpg", equirect, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return jpeg.tobytes() if ok else None


@app.get("/camera/image")
def camera_image():
    headers = {"Cache-Control": "no-store"}
    def unavailable(reason: str):
        return Response(status_code=503, content=reason, headers=headers)
    if FAKE_MODE:
        return unavailable("camera unavailable in fake mode")
    # The ROS callback stores message and receipt time together. Reading this
    # snapshot cannot consume a new frame or put an old one back over it.
    snapshot = getattr(T(), "latest_camera_frame", None)
    if not isinstance(snapshot, tuple) or len(snapshot) != 2:
        return unavailable("no timestamped camera frame yet")
    msg, received = snapshot
    def age():
        if isinstance(received, bool) or not isinstance(received, (int, float)):
            return None
        value = monotonic() - received
        return value if math.isfinite(value) and 0 <= value <= CAMERA_MAX_AGE_S else None
    if age() is None:
        return unavailable("camera frame stale or receipt time invalid; no fresh frame available")
    try:
        jpeg = undistort_jpeg(bytes(msg.data))
    except Exception:
        return unavailable("could not decode the camera frame")
    if not jpeg:
        return unavailable("could not decode the camera frame")
    final_age = age()  # expensive image processing must not bypass the age bound
    if final_age is None:
        return unavailable("camera frame became stale during image processing")
    return Response(content=jpeg, media_type="image/jpeg",
                    headers={**headers, "X-Pupper-Camera-Age-Seconds": f"{final_age:.3f}"})


@app.websocket("/ws/detections")
async def ws_detections(ws: WebSocket):
    await ws.accept()
    detections.clients.add(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        detections.clients.discard(ws)


@app.get("/system/status")
def system_status():
    if FAKE_MODE:
        return {
            **runtime_info(),
            "battery": {"percentage": None, "is_charging": None, "voltage": None},
            "cpu": {"usage": None, "temperature": None},
            "services": {"robot": "simulated" if tools else "inactive", "internet": "not_checked"},
        }
    pct = volts = None
    if BATTERY_SCRIPT.exists():
        try:
            reading = subprocess.run(["python3", str(BATTERY_SCRIPT)], capture_output=True, text=True, timeout=2)
            # The class helper reports both from one I2C sample, using its own
            # pack calibration. Never combine two independent sensor readings.
            match = re.fullmatch(r"Battery:\s*(\d+(?:\.\d+)?)%\s+(\d+(?:\.\d+)?)V\s*", reading.stdout)
            if reading.returncode == 0 and match:
                percentage, voltage = map(float, match.groups())
                if 0 <= percentage <= 100 and 0 <= voltage < float("inf"):
                    pct, volts = percentage, voltage
        except (OSError, subprocess.SubprocessError, ValueError):
            pass  # unavailable is different from a measured zero percent
    temps = psutil.sensors_temperatures()
    cpu_temp = temps["cpu_thermal"][0].current if "cpu_thermal" in temps else None
    try:
        socket.create_connection(("1.1.1.1", 53), timeout=1).close()
        internet = "online"
    except OSError:
        internet = "offline"
    return {
        **runtime_info(),
        "battery": {"percentage": pct, "is_charging": False, "voltage": volts},
        "cpu": {"usage": psutil.cpu_percent(), "temperature": cpu_temp},
        "services": {"robot": "active" if tools else "inactive", "internet": internet},
    }


@app.post("/system/volume")
def system_volume(req: Volume):
    if FAKE_MODE:
        return {"success": False, "message": "volume control unavailable in fake mode"}
    level = max(0.0, min(req.volume / 100.0, 1.5))
    r = subprocess.run(["wpctl", "set-volume", "@DEFAULT_SINK@", str(level)], capture_output=True, text=True)
    return {"success": r.returncode == 0, "message": f"volume set to {req.volume}%" if r.returncode == 0 else r.stderr}


@app.get("/health")
def health():
    return {"status": "running", "tools": tools is not None, **runtime_info()}


# The built web app (npm run build in web/ -> web/dist/) is served at /, so the robot's own
# browser, or a laptop through an SSH tunnel, opens everything from one address.
if WEB_DIST.is_dir():
    app.mount("/", StaticFiles(directory=WEB_DIST, html=True), name="web")


if __name__ == "__main__":
    uvicorn.run(app, host=os.environ.get("HOST", "127.0.0.1"), port=int(os.environ.get("PORT", "8000")))
