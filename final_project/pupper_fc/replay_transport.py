"""Synthetic rehearsal endpoints for the laptop lobby. Nothing here is physical.

Replay uses the live Lobby, Peer, FieldFeed and MatchDirector transactions and the
selected robot-side code. Only external I/O is substituted:

* the rendered SimSource + Perception camera stands in for the overhead camera,
  following the kinematic plant in ``ReplayWorld``;
* each referee frame is encoded by the real ``StatePublisher.send`` and delivered in
  memory, never through a socket;
* ``ReplayFieldProc`` stands in for ``fc_session.py field``: the selected
  ``IngressCore`` admits each datagram and the selected ``LiveMatchPlay`` turns the
  referee directive into this robot's frame and attack/defend/reset lease;
* ``replay_robot`` (one child interpreter per team) stands in for
  ``fc_session.py match --runner-only``: the selected RunnerSession, OperatorDrive,
  CombinedRuntime, BridgeCore/Executor and board reader run over a synthetic
  controller manager, kick clip, Stop gateway, guard, board and walker.

A per-team socket pair stands in for the Pi's ROS topics between the field
producer and the runner (normalized events in, kick receipts out) and carries the
walker's executed commands to the plant. The plant moves robots exactly as
commanded and kicks the ball when the synthetic clip reaches contact with the ball
in reach. Every runner event carries ``synthetic: true`` and ``physical: false``.
"""
import asyncio
from collections import deque
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import uuid

from .arena_camera import ArenaCamera
from .field_transport import FieldFeed
from .lobby import Lobby, TRACKS
from .model import Field

ROOT = Path(__file__).resolve().parents[2]
REPLAY_HOSTS = {"pupper-1": "replay-blue", "pupper-2": "replay-red"}
# Routing labels for the in-memory network. No socket is ever opened to them.
ROBOT_HOSTS = {"pupper-1": "127.0.0.11", "pupper-2": "127.0.0.12"}
ATTACK_SIGNS = {"pupper-1": -1, "pupper-2": 1}
PRODUCER = "fc-field"
KICK_SIDE = "right"
ROBOT_MODULE = "final_project.pupper_fc.replay_robot"
LIMITATION = ("Replay · synthetic I/O. The selected runner and executor decide; robot motion, kick clips, "
              "board health, guard and Stop gateway are simulated, never physical.")
SYNTHETIC = {"synthetic": True, "physical": False}


def _profile_field():
    from final_project.overhead_perception.fieldcam.config import SIM_CONFIG, load_config
    cfg = load_config(SIM_CONFIG)
    return dict(cfg["field"], ball_radius=cfg["ball"]["radius"])


def field_config():
    """The enrolled field both replay robots report, in the Pi's field-config shape."""
    raw, enrolled = _profile_field(), Field()
    field = {key: float(raw[key]) for key in ("length", "width", "goal_width", "ball_radius")}
    field.update(robot_radius=enrolled.robot_radius, bumper_offset=enrolled.bumper_offset)
    return dict(field=field, attack_signs=dict(ATTACK_SIGNS))


def match_field_config(info):
    """As ``fc_session.session_field``: apply this match's measured pitch to the enrolled field."""
    config = json.loads(json.dumps(info["field_config"]))
    pitch = info.get("pitch")
    if pitch is not None:
        measured = Field(**config["field"]).measured_pitch(pitch)
        config["field"].update({key: getattr(measured, key) for key in pitch})
    return config


def replay_profile(robot):
    return dict(event="profile", robot_id=robot, robot_host=ROBOT_HOSTS[robot], source_host="127.0.0.1", port=5005,
                producer_id=PRODUCER, match_pitch_version=1, field_config=field_config(),
                imu_recovery="strict", match_capable=True, limitation=LIMITATION, **SYNTHETIC)


def _replay_host(host, robot):
    if REPLAY_HOSTS.get(robot) != host:
        raise ValueError("Replay endpoints accept only their own synthetic team slot")


class ReplayWorld:
    """Synthetic kinematic plant behind the rendered overhead camera.

    Unexpired walker commands move each robot exactly; there is no gait, slip,
    latency or braking model. The ball rolls with constant friction and bounces off
    walls outside the goal mouths. A kick moves the ball only when the synthetic
    kick clip reaches contact with the ball inside a fixed reach in front of the
    robot. None of these values is a measured Pupper or ball property.
    """
    SCENE = {"striker": [.95, -.35, 2.8], "keeper": [-.95, .35, -.35]}
    BALL = (.2, .1)
    FRICTION = .45      # m/s^2
    KICK_SPEED = 1.4    # m/s
    REACH = (.08, .45, .18)  # forward min, forward max, lateral half-width (m)
    NET = .18           # how far behind the goal line the synthetic net stops the ball (m)
    WALL = .03          # clearance beyond the body radius, so measured centres stay kickoff-valid (m)

    def __init__(self, field=None):
        f = field or _profile_field()
        self.length, self.width = float(f["length"]), float(f["width"])
        self.goal_width, self.ball_radius = float(f["goal_width"]), float(f["ball_radius"])
        self.robot_radius = Field().robot_radius
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.robots = {k: list(v) for k, v in self.SCENE.items()}
            self.ball, self.ball_v = list(self.BALL), [0., 0.]
            self.commands = {}
            self.last = None
            self.kicks = 0

    def command(self, track, vx, vy, wz, expires_at):
        with self.lock:
            self.commands[track] = (float(vx), float(vy), float(wz), float(expires_at))

    def halt(self, track):
        with self.lock:
            self.commands.pop(track, None)

    def place_ball(self, x=0., y=0.):
        with self.lock:
            self.ball, self.ball_v = [float(x), float(y)], [0., 0.]

    def ball_inside(self):
        with self.lock:
            return abs(self.ball[0]) <= self.length / 2 and abs(self.ball[1]) <= self.width / 2

    def kick(self, track):
        with self.lock:
            x, y, yaw = self.robots[track]
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = self.ball[0] - x, self.ball[1] - y
            forward, lateral = dx * c + dy * s, -dx * s + dy * c
            low, high, half = self.REACH
            if not (low <= forward <= high and abs(lateral) <= half):
                return False
            self.ball_v = [self.KICK_SPEED * c, self.KICK_SPEED * s]
            self.kicks += 1
            return True

    def pose(self, track):
        with self.lock:
            return tuple(self.robots[track])

    def advance(self, source, now=None):
        """Integrate since the previous render, then write the scene into SimSource."""
        now = time.monotonic() if now is None else now
        with self.lock:
            dt = 0. if self.last is None else min(max(now - self.last, 0.), .1)
            self.last = now
            margin = self.robot_radius + self.WALL
            for track, (vx, vy, wz, expires) in list(self.commands.items()):
                step = min(dt, max(0., expires - (now - dt)))  # never integrate past the command's deadline
                if now >= expires:
                    del self.commands[track]
                if step <= 0 or track not in self.robots:
                    continue
                x, y, yaw = self.robots[track]
                c, s = math.cos(yaw), math.sin(yaw)
                x = min(max(x + (c * vx - s * vy) * step, -self.length / 2 + margin), self.length / 2 - margin)
                y = min(max(y + (s * vx + c * vy) * step, -self.width / 2 + margin), self.width / 2 - margin)
                yaw = math.atan2(math.sin(yaw + wz * step), math.cos(yaw + wz * step))
                self.robots[track] = [x, y, yaw]
            self._roll(dt)
            source.robots = {k: list(v) for k, v in self.robots.items()}
            source.ball[:] = self.ball

    def _roll(self, dt):
        r, half_l, half_w, mouth = self.ball_radius, self.length / 2, self.width / 2, self.goal_width / 2
        # Keep the ball out of robot bodies (a crude synthetic push, not contact physics).
        for x, y, _ in self.robots.values():
            dx, dy = self.ball[0] - x, self.ball[1] - y
            gap = math.hypot(dx, dy)
            reach = self.robot_radius + r - .03
            if 1e-6 < gap < reach:
                self.ball = [x + dx / gap * reach, y + dy / gap * reach]
        speed = math.hypot(*self.ball_v)
        if speed <= .02:
            self.ball_v = [0., 0.]
            return
        scale = max(speed - self.FRICTION * dt, 0.) / speed
        self.ball_v = [v * scale for v in self.ball_v]
        bx, by = self.ball[0] + self.ball_v[0] * dt, self.ball[1] + self.ball_v[1] * dt
        if abs(by) > half_w - r:
            by = math.copysign(half_w - r, by)
            self.ball_v[1] *= -.6
        if abs(bx) > half_l - r:
            if abs(by) < mouth - r:
                if abs(bx) > half_l + self.NET:  # resting in the synthetic net
                    bx, self.ball_v = math.copysign(half_l + self.NET, bx), [0., 0.]
            elif abs(self.ball[0]) <= half_l - r:
                bx = math.copysign(half_l - r, bx)
                self.ball_v[0] *= -.6
        self.ball = [bx, by]


class _MemorySocket:
    """The publisher's socket surface, delivering datagrams in this process only."""

    def __init__(self, network, host):
        self.network, self.host = network, host
        self.inbox = deque()

    def sendto(self, data, addr):
        self.network.deliver(addr[0], bytes(data))

    def recvfrom(self, size):
        if not self.inbox:
            raise BlockingIOError
        return self.inbox.popleft(), (self.host, 5005)

    def close(self):
        self.inbox.clear()


def _publisher_class():
    from final_project.overhead_perception.fieldcam.net import StatePublisher

    class ReplayPublisher(StatePublisher):
        """The real fieldcam state encoding and clock replies over an in-memory socket."""
        synthetic = True

        def __init__(self, network, host, port, *, source_session, live):
            # Mirror StatePublisher's fields without constructing a UDP socket.
            if not isinstance(source_session, str) or not 0 < len(source_session) <= 128 or type(live) is not bool:
                raise ValueError("source session and live flag are required")
            self.addr, self.port, self.host = (host, port), port, host
            self.source_id, self.source_session, self.live = "fieldcam", source_session, live
            self.session = source_session
            self.clock_id = "monotonic:" + source_session
            self.monotonic, self.wall = time.monotonic, time.time
            self.sock = _MemorySocket(network, host)
            self.wire_seq, self.clock_valid = 0, True
            self._last_pair = self._pair()
            self.clock_start_wall = self._last_pair[1] if self._last_pair is not None else None
            self.sent, self.closed = 0, False
            self.network = network

        def send(self, state):
            if not self.closed:
                self.sent += 1
                super().send(state)

        def close(self):
            self.closed = True
            self.network.publishers.pop(self.host, None)
            self.sock.close()

    return ReplayPublisher


class ReplayFieldProc:
    """A process-shaped stand-in for ``fc_session.py field``, read by the real FieldFeed.

    Datagrams pass ``IngressCore`` as ``UDPIngress.poll`` would deliver them, and each
    admitted frame (or revocation) goes through the selected ``LiveMatchPlay`` exactly
    as ``live_field_attack.stream`` does. Its outgoing frame and lease are published
    to this robot's runner; the runner's kick receipts return through ``completion``.
    """
    synthetic = True
    REPORT_S = 1.  # live_field_attack.stream's status cadence

    def __init__(self, network, robot, info, epoch, session, revision, tracks):
        from final_project.behaviors.fc_executor_bridge import load_fields
        from final_project.behaviors.live_field_attack import LiveMatchPlay
        from final_project.behaviors.overhead_ingress import IngressCore, MAX_FRAME_AGE_S
        self.network, self.robot, self.epoch = network, robot, epoch
        self.host = info["robot_host"]
        self.returncode = None
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(close=self.close)
        config = match_field_config(info)
        self.ingress = IngressCore(source_id="fieldcam", source_session=session, producer_id=info["producer_id"],
                                   epoch=epoch, boot_id=network.boot_id, robot_map=dict(tracks),
                                   calibration_revision=revision, field=config["field"])
        self.play = LiveMatchPlay(robot, load_fields(config)[robot], MAX_FRAME_AGE_S, epoch, tracks.values())
        self.inbox, self.wake = deque(), asyncio.Event()
        self.peer, self.next_probe = None, 0.
        self.frames = self.pairs = self.revocations = 0
        self.latest = None
        self.completions = 0
        network.fields[self.host] = self
        self._send(dict(event="source_open", mode="publish", source=self.ingress.identity(), epoch=epoch,
                        producer_id=info["producer_id"], robot_id=robot, goal=self.play.goal_name,
                        approach_only=False, side=None, field=config["field"]))
        self._task = asyncio.create_task(self._stream())

    def _send(self, event):
        if self.returncode is None:
            self.stdout.feed_data((json.dumps(dict(event, **SYNTHETIC), allow_nan=False) + "\n").encode())

    def _publish(self, event):
        link = self.network.robots.get(self.robot)
        if link is not None:
            link.publish(event)

    def receive(self, data):
        if self.returncode is None:
            self.inbox.append(data)
            self.wake.set()

    def completion(self, receipt):
        """The producer's /fc/kick_result subscription."""
        if self.returncode is None and self.play.completion(receipt, time.monotonic()):
            self.completions += 1

    def _poll(self):
        """``UDPIngress.poll`` over the in-memory datagrams; the publisher answers probes."""
        from final_project.behaviors.overhead_ingress import PROBE_PERIOD_S
        from final_project.overhead_perception.fieldcam.net import MAX_DRAIN, REPLY_PROTOCOL
        events = []
        for _ in range(MAX_DRAIN):
            if not self.inbox:
                break
            raw, now = self.inbox.popleft(), time.monotonic()
            try:
                packet = json.loads(raw)
                if not isinstance(packet, dict):
                    raise ValueError("packet must be an object")
                if packet.get("protocol") == REPLY_PROTOCOL:
                    if self.peer is not None:
                        self.ingress.accept_probe(packet, now)
                    continue
                self.ingress._identity(packet.get("_transport"))
                self.peer = self.peer or self.host
                events.append(self.ingress.normalize(packet, now))
            except (ValueError, TypeError, KeyError, UnicodeError) as exc:
                event = self.ingress.revoke(exc)
                if event is not None:
                    events.append(event)
        now = time.monotonic()
        publisher = self.network.publishers.get(self.host)
        if self.peer is not None and publisher is not None and now >= self.next_probe:
            publisher.sock.inbox.append(json.dumps(self.ingress.make_probe(now)).encode())
            publisher.service_clock()
            self.next_probe = now + PROBE_PERIOD_S
        event = self.ingress.expire(now)
        if event is not None:
            events.append(event)
        return events

    async def _stream(self):
        """The body of ``live_field_attack.stream`` with publish to this robot's runner."""
        report_at, reported_ready = 0., False
        while self.returncode is None:
            for frame in self._poll():
                now = time.monotonic()
                outgoing, intent, reason = self.play.events(frame, now)
                self._publish(outgoing)
                if intent is not None:
                    self._publish(intent)
                self.frames += 1
                self.pairs += int(intent is not None)
                self.revocations += int(outgoing["captured_at"] is None)
                self.latest = {"seq": frame["seq"], "captured_at": frame["captured_at"], "paired": intent is not None,
                               "ready_to_arm": self.play.ready_to_arm, "reason": reason,
                               "kind": intent.get("kind") if intent else None,
                               "target": intent.get("target") if intent else None,
                               "pose": frame["robots"].get(self.robot), "ball": frame["ball"],
                               "opponent_goal": frame["goals"].get(self.play.goal_name)}
            now = time.monotonic()
            ready = bool(self.latest and self.latest["ready_to_arm"])
            if now >= report_at or ready != reported_ready:
                reported_ready = ready
                self._send(dict(mode="publish", reason=self.ingress.last_reason, frames=self.frames, pairs=self.pairs,
                                revocations=self.revocations, latest=self.latest))
                report_at = now + self.REPORT_S
            deadline = now + .020
            if self.ingress.last_capture is not None and not self.ingress.held:
                deadline = min(deadline, self.ingress.last_capture + self.play.max_age)
            self.wake.clear()
            try:
                await asyncio.wait_for(self.wake.wait(), max(0., deadline - time.monotonic()))
            except TimeoutError:
                pass

    def close(self):
        if self.returncode is None:
            # An orderly source exit clears the runner frame immediately.
            revoke = self.ingress.revoke("live field attack source closed")
            if revoke is not None:
                self._publish(revoke)
            self.returncode = 0
            self._task.cancel()
            if self.network.fields.get(self.host) is self:
                del self.network.fields[self.host]
            self.stdout.feed_eof()

    async def wait(self):
        return self.returncode

    terminate = kill = close


class ReplayRobotProcess:
    """The lobby's handle on one replay robot interpreter, plus its synthetic ROS link.

    stdin/stdout are the child's own pipes, so Stop, EOF, terminate and exit are
    process semantics, as over SSH. The link carries field events to the runner,
    kick receipts to the field producer and walker commands to the plant.
    """
    synthetic = True

    def __init__(self, network, robot, proc, reader, writer):
        self.network, self.robot, self.proc = network, robot, proc
        self.track = TRACKS[robot]
        self.stdin, self.stdout = proc.stdin, proc.stdout
        self.pid = proc.pid
        self.reader, self.writer = reader, writer
        self.walk_commands = 0
        self.contacts = deque(maxlen=8)
        self.link = asyncio.create_task(self._link())

    @property
    def returncode(self):
        return self.proc.returncode

    async def wait(self):
        code = await self.proc.wait()
        await asyncio.gather(self.link, return_exceptions=True)
        return code

    def terminate(self):
        self.proc.terminate()

    def kill(self):
        self.proc.kill()

    def publish(self, event):
        from final_project.behaviors.overhead_ingress import EVENT_TOPIC
        self._write(dict(topic=EVENT_TOPIC, data=event))

    def _write(self, message):
        if not self.writer.is_closing():
            self.writer.write((json.dumps(message, allow_nan=False) + "\n").encode())

    async def _link(self):
        world = self.network.world
        try:
            while line := await self.reader.readline():
                try:
                    message = json.loads(line)
                except ValueError:
                    continue
                topic = message.get("topic") if isinstance(message, dict) else None
                if topic == "walk":
                    world.command(self.track, message["vx"], message["vy"], message["wz"],
                                  message["expires_mono_ns"] / 1e9)
                    self.walk_commands += 1
                elif topic == "kick_contact":
                    hit = world.kick(self.track)
                    self.contacts.append(dict(attempt_id=message.get("attempt_id"), hit=hit, at=time.monotonic()))
                    self._write(dict(topic="kick_contact_result", attempt_id=message.get("attempt_id"), hit=hit))
                elif topic == "/fc/kick_result":
                    field = self.network.fields.get(ROBOT_HOSTS[self.robot])
                    if field is not None:
                        field.completion(message.get("data"))
        except (ConnectionError, OSError):
            pass
        finally:
            world.halt(self.track)
            self.writer.close()
            if self.network.robots.get(self.robot) is self:
                del self.network.robots[self.robot]


def robot_command(robot, boot_id, bus_fd):
    return [sys.executable, "-B", "-m", ROBOT_MODULE, "--robot-id", robot, "--side", KICK_SIDE, "--boot-id", boot_id,
            "--bus-fd", str(bus_fd), "--profile", json.dumps(replay_profile(robot), separators=(",", ":"))]


class ReplayNetwork:
    """In-memory routing between the laptop referee, the replay robots and the plant."""

    def __init__(self):
        self.world = ReplayWorld()
        self.boot_id = "replay-" + uuid.uuid4().hex[:12]
        self.robots, self.fields, self.publishers = {}, {}, {}
        self.loop = None

    def _bind_loop(self):
        self.loop = asyncio.get_running_loop()

    def deliver(self, host, data):
        """Thread-safe datagram delivery; an absent receiver drops it, as UDP would."""
        loop = self.loop
        if loop is None or loop.is_closed():
            return

        def dispatch():
            field = self.fields.get(host)
            if field is not None:
                field.receive(data)
        try:
            loop.call_soon_threadsafe(dispatch)
        except RuntimeError:
            pass

    async def open_runner(self, host, robot, operator_owner=None):
        _replay_host(host, robot)
        if operator_owner is not None:
            raise ValueError("Replay has no standing operator to attach")
        if robot in self.robots:
            raise ValueError("This replay robot already has match controls open")
        self._bind_loop()
        parent, child = socket.socketpair()
        try:
            proc = await asyncio.create_subprocess_exec(
                *robot_command(robot, self.boot_id, child.fileno()), pass_fds=(child.fileno(),), cwd=ROOT,
                env=dict(os.environ, PYTHONPATH=str(ROOT)), stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        reader, writer = await asyncio.open_unix_connection(sock=parent)
        link = self.robots[robot] = ReplayRobotProcess(self, robot, proc, reader, writer)
        return link

    async def open_field(self, host, robot, info, epoch, session, revision, tracks):
        _replay_host(host, robot)
        self._bind_loop()
        return FieldFeed(ReplayFieldProc(self, robot, info, epoch, session, revision, tracks), robot, epoch, session)

    async def describe(self, host, robot):
        _replay_host(host, robot)
        return replay_profile(robot)

    def publisher(self, host, port, *, source_session, live):
        if host not in ROBOT_HOSTS.values():
            raise ValueError("Replay publishers route only to replay robots")
        self._bind_loop()
        publisher = _publisher_class()(self, host, port, source_session=source_session, live=live)
        self.publishers[host] = publisher
        return publisher

    def place_ball_problem(self, lobby):
        """The operator may replace the ball between matches, at a goal reset, or when it left the field."""
        if lobby.phase == "idle":
            return None
        match = ((lobby.camera.snapshot() if lobby.camera else {}).get("match")) or {}
        if lobby.phase == "playing" and match.get("phase") == "goal":
            return None
        if not self.world.ball_inside():
            return None
        return "The ball is in play. Place it between matches, after a goal, or when it leaves the field."

    def place_ball(self, lobby):
        problem = self.place_ball_problem(lobby)
        if problem:
            raise ValueError(problem)
        self.world.place_ball()

    def summary(self, lobby):
        world = self.world
        return dict(place_ball=self.place_ball_problem(lobby) is None, ball_inside=world.ball_inside(),
                    robots={robot: dict(pid=link.pid, walk_commands=link.walk_commands,
                                        contacts=list(link.contacts)[-3:])
                            for robot, link in self.robots.items()},
                    plant="Kinematic plant: exact commanded motion, constant-friction ball, fixed kick reach. "
                          "No gait, contact or latency model.",
                    timing="Ideal-capture comparison; selected camera delay is not applied. Timing qualification pending.",
                    **SYNTHETIC)


class ReplayCamera(ArenaCamera):
    """The maintained rendered SimSource + Perception path, fenced to replay.

    ``live`` is the existing Lobby current-frame predicate, not physical provenance.
    Synthetic provenance remains explicit and Live lobbies reject this class. The
    rendered scene follows the synthetic plant in ``ReplayWorld``.
    """
    synthetic = True

    def __init__(self, profile, source="sim", *, appearance=False, network=None):
        if source != "sim":
            raise ValueError("The replay field never opens a camera device")
        self.network = network or ReplayNetwork()
        super().__init__(profile, "sim", appearance=appearance, synthetic_clock=lambda _: time.time())
        self.state["synthetic"] = True

    def _sim_source(self, cfg, size):
        from final_project.overhead_perception.scripts.run_perception import SimSource
        world = self.network.world
        world.reset()

        class WorldSource(SimSource):
            def read(self, last_n=-1):
                world.advance(self)
                return super().read(last_n)
        return WorldSource(cfg, size)

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
    network = ReplayNetwork()

    def camera(profile, source, *, appearance=False):
        return ReplayCamera(profile, source, appearance=appearance, network=network)
    lobby = Lobby(replay=True, camera_factory=camera, describe_peer=network.describe,
                  field_factory=network.open_field, publisher_factory=network.publisher, settings=settings)
    lobby.scratch = scratch  # removed with the lobby
    lobby.replay_network = network
    return lobby
