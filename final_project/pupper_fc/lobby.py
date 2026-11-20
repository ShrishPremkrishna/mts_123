"""Coordinate one camera, one field referee and one or two already homed robot sessions."""
import asyncio
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid

from .arena_camera import ArenaCamera
from .field_transport import describe, open_field, validate_profile
from .match_director import MatchDirector, goal_geometry

TRACKS = {"pupper-1": "striker", "pupper-2": "keeper"}
PROFILE = Path(__file__).with_name("arena.json")


class Peer:
    def __init__(self, robot, host, owner, proc):
        self.robot, self.host, self.owner, self.proc = robot, host, owner, proc
        self.offer = {}
        self.field_info = None
        self.received = 0.
        self.changed = asyncio.Event()
        self.answers = {}
        self.closed = False

    def observe(self, event):
        if event.get("event") == "profile":
            self.field_info = validate_profile(event, self.robot)
        elif event.get("event") == "controls":
            if event.get("robot_id") != self.robot:
                raise ValueError("Runner identity does not match this team")
            if (event.get("mode") not in ("autonomous", "teleop") or type(event.get("ready")) is not bool
                    or type(event.get("ticket")) is not int or not 0 < event["ticket"] < 2**53):
                raise ValueError("Runner sent an invalid control offer")
            self.offer, self.received = event, time.monotonic()
            self.changed.set()
        elif event.get("event") == "operator":
            key = (event.get("command"), event.get("ticket"))
            future = self.answers.get(key)
            if future and not future.done() and event.get("robot_id") == self.robot:
                future.set_result(event)

    async def send(self, message):
        self.proc.stdin.write((json.dumps(message, allow_nan=False) + "\n").encode())
        await asyncio.wait_for(self.proc.stdin.drain(), .5)

    async def request(self, action, **fields):
        async with asyncio.timeout(3):
            # A newly issued offer gives the Pi the complete local input lease.
            issued = self.offer.get("ticket")
            while True:
                self.changed.clear()
                if self.closed:
                    raise ValueError(f"{self.robot} disconnected")
                offer = self.offer
                ready = offer.get("resume_ready") if action == "resume" else offer.get("ready")
                if action == "pause":
                    ready = True
                if ready and offer.get("ticket") != issued and time.monotonic() - self.received < .18:
                    break
                await self.changed.wait()
            key = (action, offer["ticket"])
            result = asyncio.get_running_loop().create_future()
            self.answers[key] = result
            try:
                await self.send(dict(command=action, robot_id=self.robot, ticket=key[1], **fields))
                response = await result
                if response.get("accepted") is not True:
                    raise ValueError(str(response.get("reason") or "Robot declined " + action))
                return response
            finally:
                self.answers.pop(key, None)

    def disconnect(self):
        self.closed = True
        self.changed.set()
        for answer in self.answers.values():
            if not answer.done():
                answer.set_exception(ValueError(f"{self.robot} disconnected"))


class Lobby:
    def __init__(self, *, camera_factory=ArenaCamera, describe_peer=describe, field_factory=open_field,
                 publisher_factory=None, settings=None):
        self.profile = json.loads(PROFILE.read_text())
        self.settings = settings or Path.home() / ".config/pupper-fc/arena.json"
        self.camera_factory, self.describe, self.field = camera_factory, describe_peer, field_factory
        self.publisher = publisher_factory
        self.camera, self.owner = None, None
        self.source, self.appearance, self.goals = "phone", False, None
        try:
            saved = json.loads(self.settings.read_text())
            if saved.get("profile") == self.profile["tags"] and self.valid_goals(saved.get("goal_lines")):
                self.goals = saved["goal_lines"]
        except (OSError, ValueError, AttributeError):
            pass
        self.peers, self.feeds, self.publishers, self.players, self.autonomous = {}, {}, [], [], []
        self.phase, self.message = "idle", ""
        self.lock = asyncio.Lock()
        self.watcher = self.transition = self.ending = self.aborted = None

    def valid_goals(self, goals):
        f = self.profile["field"]
        return goal_geometry(goals, f["length"], f["width"], f["ball_radius"])

    def state(self):
        camera = self.camera.snapshot() if self.camera else None
        message = self.message
        if self.phase == "playing" and camera:
            if not camera.get("live"):
                message = "Play held · camera feed lost"
            elif (camera.get("match") or {}).get("phase") == "holding":
                message = "Play held · waiting for current field measurements or the attacker handover"
        return dict(phase=self.phase, message=message, source=self.source, appearance=self.appearance,
                    resume_ready=self.phase == "playing" and bool(camera and (camera.get("match") or {}).get("reset_ready"))
                    and all(p.offer.get("resume_ready") for p in self.autonomous),
                    goal_lines=self.goals, camera=camera)

    def check_owner(self, token):
        if self.owner is not None and self.owner != token:
            raise ValueError("A different window owns the match")

    async def attach_camera(self, data, token):
        async with self.lock:
            self.check_owner(token)
            if self.phase != "idle":
                raise ValueError("End the match before changing the camera")
            source, appearance = data.get("source"), data.get("appearance", False)
            if source not in ("phone","0","1","2","3","sim") or type(appearance) is not bool:
                raise ValueError("Select a camera from settings")
            if self.camera:
                await asyncio.to_thread(self.camera.close)
            self.camera = self.camera_factory(self.profile, source, appearance=appearance)
            self.source, self.appearance = source, appearance
            self.camera.start()
            return self.state()

    async def survey(self, data, token):
        async with self.lock:
            self.check_owner(token)
            if self.phase != "idle" or self.camera is None:
                raise ValueError("Connect the camera and end the current match before surveying goals")
            goals = self.camera.survey(data)
            if not self.valid_goals(goals):
                raise ValueError("Choose the two west posts first, then the two east posts, at floor level")
            self.settings.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode="w", dir=self.settings.parent, delete=False) as out:
                tmp = out.name
                json.dump({"profile": self.profile["tags"], "goal_lines": goals}, out, allow_nan=False)
            try:
                os.replace(tmp, self.settings)
            finally:
                Path(tmp).unlink(missing_ok=True)
            self.goals = goals
            return self.state()

    def camera_ready(self, players):
        snap = self.camera.snapshot() if self.camera else {}
        f = snap.get("frame") or {}
        if not (snap.get("live") and f.get("calibration_valid") and 0 <= time.time()-f["captured_at"] <= .30):
            raise ValueError("Connect a live camera with a current field calibration")
        visible = snap["visible"]
        if not visible.get("ball") or any(not visible["robots"].get(TRACKS[p.robot]) for p in players):
            raise ValueError("Place the ball and each connected robot in view")
        return snap, f

    async def await_camera(self, players, *, commit=False):
        end = time.monotonic() + 2
        while True:
            try:
                scene = self.camera_ready(players)
                if commit:
                    self.camera.commit_start()
                return scene
            except ValueError:
                if time.monotonic() >= end:
                    raise
                await asyncio.sleep(.035)

    async def start(self, token):
        if self.phase != "idle":
            raise ValueError("A match is already open")
        async with self.lock:
            self.check_owner(token)
            if self.phase != "idle":
                raise ValueError("A match is already open")
            players = list(self.peers.values())
            if not players or any(p.owner != token or not p.offer or p.closed for p in players):
                raise ValueError("Connect your team in this window first")
            if not self.camera or not self.camera.snapshot().get("live"):
                raise ValueError("Connect a live camera with a current field calibration")
            modes = {p.robot: p.offer["mode"] for p in players}
            auto = [p for p in players if modes[p.robot] == "autonomous"]
            self.players, self.phase, self.owner = players, "starting", token
            self.autonomous = auto
            self.transition = asyncio.current_task()
            self.message = "Connecting the field to the robots"
            attempted = False
            try:
                async with asyncio.timeout(18):
                    snap, f = await self.await_camera(players)
                    async def profile(p):
                        if getattr(p, "field_info", None) is None:
                            p.field_info = await self.describe(p.host, p.robot)
                        return p.field_info
                    infos = await asyncio.gather(*(profile(p) for p in players))
                    cfg = infos[0]["field_config"]
                    if any(i["field_config"] != cfg for i in infos):
                        raise ValueError("Robots have different field profiles")
                    for key in ("length","width","ball_radius","goal_width"):
                        if not math.isclose(cfg["field"][key], self.profile["field"][key], rel_tol=1e-5):
                            raise ValueError(f"Camera and Pi disagree on field {key}")
                    epoch = uuid.uuid4().hex
                    tracks = {TRACKS[p.robot]: p.robot for p in players}
                    director = MatchDirector(epoch, tracks, cfg["attack_signs"], goal_lines=self.goals, pending_start=True)
                    for p, info in zip(players, infos):
                        if p in auto:
                            if self.publisher is None:
                                from final_project.overhead_perception.fieldcam.net import StatePublisher
                                self.publisher = StatePublisher
                            self.publishers.append(self.publisher(info["robot_host"], info["port"],
                                                   source_session=snap["source_session"], live=True))
                    self.camera.set_match(director, self.publishers)
                    # Keep each newly opened resource even if another Pi fails to open.
                    async def open_one(p, info):
                        self.feeds[p.robot] = await self.field(p.host, p.robot, info, epoch, snap["source_session"],
                                                               f["calibration_revision"], tracks)
                        await self.feeds[p.robot].wait_ready()
                    async with asyncio.TaskGroup() as group:
                        for p, info in zip(players, infos):
                            if p in auto:
                                group.create_task(open_one(p, info))
                    await self.await_camera(players)
                    attempted = True
                    results = await asyncio.gather(*(p.request("arm", epoch=epoch, mode=modes[p.robot]) for p in players), return_exceptions=True)
                    for result in results:
                        if isinstance(result, BaseException):
                            raise result
                    await self.await_camera(players, commit=True)
                self.phase, self.message = "playing", "Match in progress"
                self.watcher = asyncio.create_task(self._monitor())
                return self.state()
            except BaseException as exc:
                self.transition = None
                notes = await asyncio.shield(self._finish(players if attempted else []))
                while isinstance(exc, BaseExceptionGroup):
                    exc = exc.exceptions[0]
                self.message = " · ".join([str(exc) or "Match start interrupted or timed out", *notes])
                if isinstance(exc, (KeyboardInterrupt, SystemExit)) or (
                        isinstance(exc, asyncio.CancelledError) and self.aborted is not asyncio.current_task()):
                    raise
                raise ValueError(self.message) from None
            finally:
                self.transition = self.aborted = None

    def drive_allowed(self, peer, epoch):
        if self.phase != "playing" or peer not in self.players or peer.offer.get("mode") != "teleop":
            return False
        snap = self.camera.snapshot()
        match = snap.get("match") or {}
        frame = snap.get("frame") or {}
        return (snap.get("live") and match.get("phase") == "playing" and match.get("epoch") == epoch
                and 0 <= time.time() - frame.get("captured_at", 0) <= .30)

    async def _monitor(self):
        while self.phase == "playing":
            await asyncio.sleep(.25)
            problem = next((feed.problem for feed in self.feeds.values() if feed.problem), None)
            if problem:
                await self.end(self.owner, problem)
                return

    async def _neutral(self, players):
        async def pause(p):
            if p.closed:
                return f"{p.robot} disconnected"
            try:
                reply = await p.request("pause")
                if reply.get("state") == "hard_stop":
                    return f"{p.robot} used Stop during a native action"
            except (ValueError, TimeoutError, OSError) as exc:
                await p.send({"command":"stop"})
                return f"{p.robot} sent Stop after standby failed: {str(exc) or 'no reply'}"
        reports = await asyncio.gather(*(pause(p) for p in players), return_exceptions=True)
        return [str(item) or "Robot did not confirm standby" for item in reports if item is not None]

    async def _release(self):
        if self.watcher and self.watcher is not asyncio.current_task():
            self.watcher.cancel()
        self.watcher = None
        await asyncio.gather(*(v.close() for v in self.feeds.values()), return_exceptions=True)
        for publisher in self.publishers:
            publisher.close()
        self.feeds.clear()
        self.publishers.clear()
        self.players, self.autonomous, self.owner, self.phase = [], [], None, "idle"

    async def _finish(self, players):
        self.phase = "stopping"
        if self.camera:
            self.camera.set_match(None)
        try:
            return await self._neutral(players)
        finally:
            await self._release()

    async def end(self, token, reason="Match ended"):
        self.check_owner(token)
        if self.transition and self.transition is not asyncio.current_task():
            self.aborted, self.transition = self.transition, None
            self.aborted.cancel()
        async def shutdown():
            async with self.lock:
                self.check_owner(token)
                notes = []
                if self.phase != "idle":
                    self.phase = "stopping"
                    notes = await self._finish(self.players)
                self.message = " · ".join([reason, *notes])
                return self.state()
        if self.ending is None or self.ending.done():
            self.ending = asyncio.create_task(shutdown())
        return await asyncio.shield(self.ending)

    async def resume(self, token):
        async with self.lock:
            self.check_owner(token)
            if self.phase != "playing" or not (self.camera.snapshot().get("match") or {}).get("reset_ready"):
                raise ValueError("Return the ball to the field before resuming")
            if not all(p.offer.get("resume_ready") for p in self.autonomous):
                raise ValueError("Waiting for the autonomous robots to reach their home positions")
            self.transition = asyncio.current_task()
            try:
                answers = await asyncio.gather(*(p.request("resume") for p in self.autonomous), return_exceptions=True)
                for a in answers:
                    if isinstance(a, BaseException):
                        raise a
                self.camera.resume()
            except (OSError, TimeoutError, ValueError, asyncio.CancelledError) as exc:
                self.transition = None
                await asyncio.shield(self._finish(self.players))
                if isinstance(exc, asyncio.CancelledError) and self.aborted is not asyncio.current_task():
                    raise
                raise ValueError(str(exc) or "Goal reset timed out. Start a new match.") from None
            finally:
                self.transition = self.aborted = None
            return self.state()

    async def remove(self, peer):
        peer.disconnect()
        if self.peers.get(peer.robot) is peer:
            del self.peers[peer.robot]
        if peer in self.players:
            await self.end(peer.owner, f"{peer.robot} disconnected · match ended")

