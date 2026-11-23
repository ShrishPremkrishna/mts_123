"""Synthetic in-process rehearsal endpoints for the laptop lobby. Nothing here is physical.

Replay reuses the live Lobby, Peer, FieldFeed and MatchDirector transactions. Only the
process-shaped endpoints change: no SSH, discovery, camera device, Home or UDP publisher.
Every event carries ``synthetic: true`` and ``physical: false``; acknowledgments describe
the replay transport only and never claim robot motion, Stop or board health.
"""
import asyncio
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import uuid

from .arena_camera import ArenaCamera
from .field_transport import FieldFeed
from .lobby import PROFILE, Lobby

REPLAY_HOSTS = {"pupper-1": "replay-blue", "pupper-2": "replay-red"}
ATTACK_SIGNS = {"pupper-1": -1, "pupper-2": 1}
LIMITATION = "Replay · synthetic transport. No robot moves; board health and Stop are simulated, never physical."
SYNTHETIC = {"synthetic": True, "physical": False}


def _profile_field():
    from final_project.overhead_perception.fieldcam.config import SIM_CONFIG, load_config
    cfg = load_config(SIM_CONFIG)
    return dict(cfg["field"], ball_radius=cfg["ball"]["radius"])


def replay_profile(robot):
    return dict(event="profile", robot_id=robot, robot_host="127.0.0.1", source_host="127.0.0.1", port=5005,
                match_pitch_version=1, field_config=dict(field=_profile_field(), attack_signs=dict(ATTACK_SIGNS)),
                imu_recovery="replay_synthetic", match_capable=True, limitation=LIMITATION, **SYNTHETIC)


def _replay_host(host, robot):
    if REPLAY_HOSTS.get(robot) != host:
        raise ValueError("Replay endpoints accept only their own synthetic team slot")


class ReplayRunner:
    """A process-shaped stand-in for ``fc_session.py match --runner-only``."""
    synthetic = True
    PERIOD = .05

    def __init__(self, robot):
        self.robot, self.mode, self.ticket = robot, "autonomous", 0
        self.board = dict(state="healthy", board_session=f"replay-{robot}-{uuid.uuid4().hex[:8]}", generation=1,
                          incident=0, epoch=None, reason="Synthetic replay board", **SYNTHETIC)
        self.armed_epoch = None
        self.commands, self.motion = [], dict(forward=0, left=0, turn=0)
        self.returncode = None
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(write=self._write, drain=self._drain, close=self.close)
        self._emit(replay_profile(robot))
        self._ticks = asyncio.create_task(self._heartbeat())

    def _emit(self, event):
        if self.returncode is None:
            self.stdout.feed_data((json.dumps(dict(event, robot_id=self.robot, **SYNTHETIC)) + "\n").encode())

    async def _heartbeat(self):
        while self.returncode is None:
            self.ticket += 1
            self._emit(dict(event="controls", mode=self.mode, ticket=self.ticket, ready=True, resume_ready=False,
                            imu=dict(self.board), limitation=LIMITATION))
            await asyncio.sleep(self.PERIOD)

    def _reply(self, msg, accepted, reason="", **fields):
        self._emit(dict(event="operator", command=msg.get("command"), ticket=msg.get("ticket"),
                        accepted=accepted, reason=reason, **fields))

    def _write(self, data):
        msg = json.loads(data)
        self.commands.append(msg)
        command = msg.get("command")
        if command == "stop":
            self.armed_epoch, self.board["epoch"] = None, None
            self.motion = dict(forward=0, left=0, turn=0)
            self._emit(dict(event="stop_result", confirmed=False,
                            reason="Replay Stop recorded by the synthetic transport. No physical Stop occurred."))
            return
        if msg.get("robot_id") != self.robot or type(msg.get("ticket")) is not int:
            return self._reply(msg, False, "Replay runner refused a command for another team or without a ticket")
        if command == "arm":
            self.armed_epoch = self.board["epoch"] = msg["epoch"]
            self.board.update(state="healthy", hold_id=None)
            return self._reply(msg, True, state="armed", epoch=msg["epoch"])
        if command == "pause":
            self.armed_epoch, self.board["epoch"] = None, None
            self.motion = dict(forward=0, left=0, turn=0)
            return self._reply(msg, True, state="standby")
        if command == "mode" and msg.get("mode") in ("autonomous", "teleop") and self.armed_epoch is None:
            self.mode = msg["mode"]
            return self._reply(msg, True)
        if command == "onboard_ball_mode":
            return self._reply(msg, True)
        if command == "imu_pause" and msg.get("epoch") == self.armed_epoch:
            hold = uuid.uuid4().hex
            self.board.update(state="paused", hold_id=hold)
            return self._reply(msg, True, state="paused", epoch=msg["epoch"], hold_id=hold, imu=dict(self.board))
        if command == "drive":
            axes = {key: msg.get(key, 0) for key in ("forward", "left", "turn")}
            current = self.armed_epoch is not None and msg.get("epoch") == self.armed_epoch and self.mode == "teleop"
            if current or not any(axes.values()):
                # Synthetic receipt only. A brake from any epoch is accepted, as on the Pi.
                self.motion = axes if current else dict(forward=0, left=0, turn=0)
                return self._reply(msg, True, applied=dict(self.motion))
            return self._reply(msg, False, "Replay runner refused drive outside its current match epoch")
        self._reply(msg, False, f"Replay does not model {command}")

    async def _drain(self):
        pass

    def close(self):
        if self.returncode is None:
            self.returncode = 0
            self._ticks.cancel()
            self.stdout.feed_eof()

    async def wait(self):
        return self.returncode

    terminate = kill = close


async def open_replay_runner(host, robot):
    _replay_host(host, robot)
    return ReplayRunner(robot)


class ReplayFieldProc:
    """A process-shaped stand-in for ``fc_session.py field``. Read by the real FieldFeed."""
    synthetic = True

    def __init__(self, robot, epoch, session):
        self.returncode = None
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(close=self.close)
        self._send(dict(event="source_open", mode="publish", robot_id=robot, epoch=epoch,
                        source={"source_session": session}))
        self._ticks = asyncio.create_task(self._status())

    def _send(self, event):
        if self.returncode is None:
            self.stdout.feed_data((json.dumps(dict(event, **SYNTHETIC)) + "\n").encode())

    async def _status(self):
        frames = 0
        while self.returncode is None:
            frames += 1
            self._send(dict(mode="publish", frames=frames, pairs=frames, latest=dict(paired=True, **SYNTHETIC)))
            await asyncio.sleep(.1)

    def close(self):
        if self.returncode is None:
            self.returncode = 0
            self._ticks.cancel()
            self.stdout.feed_eof()

    async def wait(self):
        return self.returncode

    terminate = kill = close


async def open_replay_field(host, robot, info, epoch, session, revision, tracks):
    _replay_host(host, robot)
    return FieldFeed(ReplayFieldProc(robot, epoch, session), robot, epoch, session)


async def replay_describe(host, robot):
    _replay_host(host, robot)
    return replay_profile(robot)


class ReplayPublisher:
    """Counts referee frames in memory. Opens no socket."""
    synthetic = True

    def __init__(self, host, port, *, source_session, live):
        self.host, self.port, self.session, self.sent, self.closed = host, port, source_session, 0, False

    def send(self, state):
        if not self.closed:
            self.sent += 1

    def close(self):
        self.closed = True


class ReplayCamera(ArenaCamera):
    """The maintained rendered SimSource + Perception path, fenced to replay.

    ``live`` is the existing Lobby current-frame predicate, not physical provenance.
    Synthetic provenance remains explicit and Live lobbies reject this class.
    Runner acknowledgments exercise I/O transactions, not the robot executor.
    """
    synthetic = True

    def __init__(self, profile, source="sim", *, appearance=False):
        if source != "sim":
            raise ValueError("The replay field never opens a camera device")
        super().__init__(profile, "sim", appearance=appearance, synthetic_clock=lambda _: time.time())
        self.state["synthetic"] = True

    def snapshot(self):
        snap = super().snapshot()
        frame = snap.get("frame")
        current = bool(frame and not self.done.is_set() and 0 <= time.time() - frame["captured_at"] <= .30)
        snap.update(live=current, synthetic=True, physical=False,
                    status="Synthetic camera · " + snap["status"])
        if frame:
            snap["frame"] = dict(frame, **SYNTHETIC)
        return snap

    def capture_setup(self):
        return dict(super().capture_setup(), **SYNTHETIC)

    def set_match(self, director, publishers=()):
        if any(not getattr(pub, "synthetic", False) for pub in publishers):
            raise ValueError("The rendered rehearsal accepts only in-memory publishers")
        with self.lock:
            super().set_match(director)
            self.publishers = tuple(publishers)


def replay_lobby(settings=None):
    """A Lobby wired exclusively to replay endpoints, with throwaway settings."""
    scratch = None
    if settings is None:
        scratch = tempfile.TemporaryDirectory(prefix="pupper-fc-replay-")
        settings = Path(scratch.name) / "arena.json"
    lobby = Lobby(replay=True, camera_factory=ReplayCamera, describe_peer=replay_describe,
                  field_factory=open_replay_field, publisher_factory=ReplayPublisher, settings=settings)
    lobby.scratch = scratch  # removed with the lobby
    return lobby
