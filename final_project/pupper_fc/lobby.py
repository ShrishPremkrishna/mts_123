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

from .arena_camera import ArenaCamera, camera_settings
from .balance_match import BalanceHold, fresh, identity, play_problem, proof
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
        elif event.get("event") == "imu" and self.offer:
            if event.get("robot_id") != self.robot or not isinstance(event.get("imu"), dict):
                raise ValueError("Runner returned invalid board health")
            # Immediate fault edge, independent of the periodic control ticket.
            self.offer = dict(self.offer, imu=event["imu"])
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
            initial_board = {self.robot: proof(self)} if action == "arm" else None
            while True:
                self.changed.clear()
                if self.closed:
                    raise ValueError(f"{self.robot} disconnected")
                offer = self.offer
                ready = offer.get("resume_ready") if action == "resume" else offer.get("ready")
                if action == "imu_resume":
                    imu = proof(self)
                    ready = imu.get("state") == "ready_to_resume" and imu.get("hold_id") == fields["hold_id"]
                if action in ("pause", "imu_pause"):
                    ready = True
                if ready and offer.get("ticket") != issued and time.monotonic() - self.received < .18:
                    if initial_board and (reason := play_problem(self, initial_board)):
                        raise ValueError(f"{self.robot}: {reason}")
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


def synthetic(item):
    """Replay endpoints declare themselves. Live endpoints never carry this marker."""
    return getattr(item, "synthetic", False) is True


class Lobby:
    def __init__(self, *, camera_factory=ArenaCamera, describe_peer=describe, field_factory=open_field,
                 publisher_factory=None, settings=None, replay=False):
        self.profile = json.loads(PROFILE.read_text())
        default_settings = Path.home() / ".config/pupper-fc/arena.json"
        if replay and (camera_factory is ArenaCamera or describe_peer is describe or field_factory is open_field
                       or publisher_factory is None or settings is None or Path(settings) == default_settings):
            raise ValueError("Replay requires injected replay endpoints and temporary settings")
        self.mode = "replay" if replay else "live"
        self.settings = settings or default_settings
        self.camera_factory, self.describe, self.field = camera_factory, describe_peer, field_factory
        self.publisher = publisher_factory
        self.camera, self.owner = None, None
        self.source, self.appearance, self.goals = "sim" if replay else "phone", False, None
        self.setup, self.tracking = {"mode": "layout"}, "tags"
        self.surveys = {}
        try:
            saved = json.loads(self.settings.read_text())
            self.setup, self.tracking = camera_settings(saved.get("camera_settings", {}), self.profile)
            if saved.get("profile") == self.profile["tags"]:
                if isinstance(saved.get("surveys"), dict):
                    self.surveys = saved["surveys"]
                elif self.valid_goals(saved.get("goal_lines")):
                    self.surveys[self.geometry_key()] = saved["goal_lines"]
                goals = self.surveys.get(self.geometry_key())
                self.goals = goals if self.valid_goals(goals) else None
        except (OSError, ValueError, TypeError, AttributeError):
            pass
        self.peers, self.feeds, self.publishers, self.players, self.autonomous = {}, {}, [], [], []
        self.phase, self.message = "idle", ""
        self.lock = asyncio.Lock()
        self.watcher = self.transition = self.ending = self.aborted = None
        self.balance = self.pause_task = self.binding = None
        self.board_anchors, self.match_infos, self.match_modes, self.pitch = {}, [], {}, None
        self.epoch, self.referee, self.last_score = None, None, {}

    def geometry_key(self):
        geometry = {k: v for k, v in self.setup.items() if k != "patch_height"}
        return json.dumps(geometry, sort_keys=True, separators=(",", ":"))

    def settings_data(self):
        return dict(field_mode=self.setup["mode"], robot_tracking=self.tracking, patch_height=self.setup.get("patch_height", .18),
                    anchor=self.setup if self.setup["mode"] == "corner" else None)

    def save_settings(self):
        self.settings.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=self.settings.parent, delete=False) as out:
            tmp = out.name
            json.dump(dict(profile=self.profile["tags"], camera_settings=self.settings_data(),
                           surveys=self.surveys), out, allow_nan=False)
        try:
            os.replace(tmp, self.settings)
        finally:
            Path(tmp).unlink(missing_ok=True)

    def valid_goals(self, goals):
        f = dict(self.profile["field"], **self.setup)
        return goal_geometry(goals, f["length"], f["width"], f["ball_radius"])

    def state(self):
        camera = self.camera.snapshot() if self.camera else None
        message = self.message
        if self.phase == "playing" and camera:
            if not camera.get("live"):
                message = "Play held · camera feed lost"
            elif (camera.get("match") or {}).get("phase") == "holding":
                message = "Play held · waiting for current field measurements or the attacker handover"
        balance = self.balance_state() if self.balance else None
        score = self.referee.state.get("score") if self.referee else self.last_score
        return dict(phase=self.phase, message=message, source=self.source, appearance=self.appearance,
                    score=score, mode=self.mode, synthetic=self.mode == "replay",
                    balance=balance,
                    resume_ready=self.phase == "playing" and bool(camera and (camera.get("match") or {}).get("reset_ready"))
                    and all(p.offer.get("resume_ready") for p in self.autonomous),
                    goal_lines=self.goals, camera=camera, camera_settings=self.settings_data(),
                    field_defaults=self.profile["field"], floor_size=self.profile["floor_size"])

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
            if self.mode == "replay" and source != "sim":
                raise ValueError("Replay mode uses only the synthetic field. It never opens a camera.")
            setup, tracking = camera_settings(data, self.profile)
            if self.camera:
                await asyncio.to_thread(self.camera.close)
                self.camera = None
            self.setup, self.tracking = setup, tracking
            saved = self.surveys.get(self.geometry_key())
            self.goals = saved if source != "sim" and self.valid_goals(saved) else None
            profile = dict(self.profile, field_setup=setup, robot_tracking=tracking, surveyed_goals=self.goals)
            camera = self.camera_factory(profile, source, appearance=appearance)
            if synthetic(camera) != (self.mode == "replay"):
                raise ValueError("The synthetic replay field is available only in replay mode")
            self.camera = camera
            self.save_settings()
            self.source, self.appearance = source, appearance
            try:
                self.camera.start()
            except BaseException:
                self.camera = None
                with suppress(Exception):
                    await asyncio.to_thread(camera.close)
                raise
            return self.state()

    async def setup_snapshot(self, action, data, token):
        async with self.lock:
            self.check_owner(token)
            if self.phase != "idle" or not self.camera:
                raise ValueError("Connect a camera and end the match before checking the field.")
            if action == "camera/snapshot":
                return self.camera.capture_setup()
            if action == "camera/far-corner":
                self.camera.mark_far_corner(data)
            else:
                self.camera.confirm_field(data)
            return self.state()

    async def use_configured_goals(self, token):
        async with self.lock:
            self.check_owner(token)
            if self.phase != "idle" or not self.camera:
                raise ValueError("Connect the field camera and end the match before changing goals.")
            self.surveys.pop(self.geometry_key(), None)
            self.save_settings()
            self.goals = None
            self.camera.set_goals(None)
            return self.state()

    async def survey(self, data, token):
        async with self.lock:
            self.check_owner(token)
            if self.phase != "idle" or self.camera is None:
                raise ValueError("Connect the camera and end the current match before surveying goals")
            goals = self.camera.survey(data)
            if not self.valid_goals(goals):
                raise ValueError("Choose the two west posts first, then the two east posts, at floor level")
            self.surveys[self.geometry_key()] = goals
            self.save_settings()
            self.goals = goals
            self.camera.set_goals(goals)
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

    async def await_camera(self, players, *, before_commit=None, binding=None):
        end = time.monotonic() + 2
        while True:
            try:
                scene = self.camera_ready(players)
                if binding is not None and binding != (scene[0]["source_session"], scene[1]["calibration_revision"]):
                    raise ValueError("The field calibration changed during kickoff. Start again.")
                if before_commit is not None:
                    before_commit()
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
            self._check_mode(players)
            if not self.camera or not self.camera.snapshot().get("live"):
                raise ValueError("Connect a live camera with a current field calibration")
            self.board_anchors = {p.robot:dict(proof(p)) for p in players}
            self._check_boards(players)
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
                    binding = snap["source_session"], f["calibration_revision"]
                    async def profile(p):
                        if getattr(p, "field_info", None) is None:
                            p.field_info = await self.describe(p.host, p.robot)
                        return p.field_info
                    infos = await asyncio.gather(*(profile(p) for p in players))
                    cfg = infos[0]["field_config"]
                    if any(i["field_config"] != cfg for i in infos):
                        raise ValueError("Robots have different field profiles")
                    camera_geometry = f.get("field", self.profile["field"])
                    if not math.isclose(cfg["field"]["ball_radius"], camera_geometry["ball_radius"], rel_tol=0., abs_tol=1e-6):
                        raise ValueError("Camera and Pi disagree on ball radius")
                    pitch = {key: camera_geometry[key] for key in ("length", "width")}
                    if not math.isclose(camera_geometry["goal_width"], cfg["field"]["goal_width"], rel_tol=0., abs_tol=1e-6):
                        pitch["goal_width"] = camera_geometry["goal_width"]
                    if any(not math.isclose(value, cfg["field"][key], rel_tol=0., abs_tol=1e-6)
                           for key, value in pitch.items()):
                        if any(info.get("match_pitch_version") != 1 for info in infos):
                            raise ValueError("Update the Pi release to use this measured pitch")
                        infos = [dict(info, pitch=pitch) for info in infos]
                    else:
                        pitch = None
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
                    self.referee = director
                    self.camera.set_match(director, self.publishers)
                    self.match_infos, self.match_modes, self.pitch = infos, modes, pitch
                    self.binding, self.epoch = binding, epoch
                    await self._open_fields(epoch, tracks)
                    await self.await_camera(players, binding=binding)
                    self._check_boards(players)
                    attempted = True
                    results = await asyncio.gather(*(p.request("arm", epoch=epoch, mode=modes[p.robot], **({"pitch": pitch} if pitch else {})) for p in players), return_exceptions=True)
                    for result in results:
                        if isinstance(result, BaseException):
                            raise result
                    if any(r.get("state") == "paused" for r in results):
                        self._begin_balance_hold("Kickoff paused. A previous board incident needs explicit recovery.")
                        self.watcher = asyncio.create_task(self._monitor())
                        return self.state()
                    await self.await_camera(players, binding=binding, before_commit=lambda: self._check_boards(players, epoch))
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

    def _check_mode(self, players):
        """Replay plays only replay teams on the synthetic field; live play refuses both."""
        replay_parts = [synthetic(self.camera), *(synthetic(getattr(p, "proc", None)) for p in players)]
        if self.mode == "replay" and not all(replay_parts):
            raise ValueError("Replay matches use only replay teams and the synthetic field")
        if self.mode != "replay" and any(replay_parts):
            raise ValueError("Live matches refuse replay teams and the synthetic field")

    def _check_boards(self, players, epoch=None):
        for peer in players:
            reason = play_problem(peer, self.board_anchors, epoch)
            if reason:
                raise ValueError(f"{peer.robot}: {reason}")

    def drive_allowed(self, peer, epoch):
        if (self.phase != "playing" or peer not in self.players or peer.offer.get("mode") != "teleop"
                or any(play_problem(p, self.board_anchors, self.epoch) for p in self.players)):
            return False
        snap = self.camera.snapshot()
        match = snap.get("match") or {}
        frame = snap.get("frame") or {}
        return (snap.get("live") and match.get("phase") == "playing" and match.get("epoch") == epoch
                and 0 <= time.time() - frame.get("captured_at", 0) <= .30)

    async def _open_fields(self, epoch, tracks):
        async def open_one(peer, info):
            self.feeds[peer.robot] = await self.field(peer.host, peer.robot, info, epoch, self.binding[0],
                                                      self.binding[1], tracks)
            await self.feeds[peer.robot].wait_ready()
        async with asyncio.TaskGroup() as group:
            for p, info in zip(self.players, self.match_infos):
                if p in self.autonomous:
                    group.create_task(open_one(p, info))

    def balance_state(self):
        current = False
        try:
            snap, frame = self.camera_ready(self.players)
            current = self.binding == (snap["source_session"], frame["calibration_revision"])
            if not current and not self.balance.blocked:
                self.balance.blocked = "The field camera or calibration changed. End the match and check the field."
            if current:
                current = self._inside_field(frame)
        except ValueError:
            pass
        return self.balance.report(self.players, current)

    def _inside_field(self, frame):
        field, robots = frame.get("field") or {}, frame.get("robots") or {}
        points = [frame.get("ball"), *(robots.get(TRACKS[p.robot]) for p in self.players)]
        return all(isinstance(pt, dict) and type(pt.get(k)) in (int, float)
                   and type(field.get(dim)) in (int, float) and abs(pt[k]) <= field[dim] / 2
                   for pt in points for k,dim in (("x", "length"), ("y", "width")))

    def _begin_balance_hold(self, reason):
        self.camera.pause_balance()
        self.phase, self.message = "balance_hold", "Match paused for balance feedback"
        self.balance = BalanceHold(self.epoch, self.board_anchors, reason)
        if self.referee.kick_owner is not None:
            self.balance.blocked = "A native kick was active at the balance hold. End the match and check team setup."
        if any(not identity(self.board_anchors.get(p.robot, {})) for p in self.players):
            self.balance.blocked = "A team cannot verify board continuity. End the match and check team setup."
        self.pause_task = asyncio.create_task(self._pause_roster(self.balance))

    def observe_controls(self, peer):
        if peer not in self.players:
            return
        if self.phase == "playing":
            reason = play_problem(peer, self.board_anchors, self.epoch)
            if reason:
                self._begin_balance_hold(f"{peer.robot} · {reason}")
        if self.balance and self.phase in ("balance_hold", "balance_resuming"):
            row = proof(peer)
            if row.get("state") == "home_required":
                self.balance.blocked = f"{peer.robot} requires supported Home. End the match and open team setup."
            elif peer.robot in self.board_anchors and identity(row) and identity(row) != identity(self.board_anchors[peer.robot]):
                self.balance.blocked = f"{peer.robot} changed board session or Home generation. End the match and check setup."
            elif (self.phase == "balance_hold" and not self.balance.preparing and not self.balance.blocked
                  and row.get("state") in ("paused", "ready_to_resume") and row.get("epoch") == self.epoch
                  and isinstance(row.get("hold_id"), str) and row["hold_id"]
                  and row["hold_id"] != self.balance.paused.get(peer.robot)):
                self._begin_balance_hold(f"{peer.robot} reported another balance incident. Rechecking both board and field readiness.")

    def stop_requested(self, peer):
        if peer not in self.players or self.phase in ("idle", "stopping"):
            return
        if self.balance:
            self.balance.blocked = f"{peer.robot} requested Stop. End the match and check team setup."
        self.camera.pause_balance()
        self.phase, self.message = "stopping", f"{peer.robot} requested Stop"
        async def stop_and_end():
            try:
                await self._stop_roster()
            finally:
                await self.end(peer.owner, self.message)
        asyncio.create_task(stop_and_end())

    async def _stop_roster(self):
        await asyncio.gather(*(p.send({"command":"stop"}) for p in self.players if not p.closed), return_exceptions=True)

    async def _pause_roster(self, hold):
        try:
            async def one(p):
                if not hold.anchors.get(p.robot):
                    raise ValueError(f"{p.robot} has no board continuity protocol")
                reply = await p.request("imu_pause", epoch=hold.epoch)
                hold.record_pause(p, reply)
            results = await asyncio.gather(*(one(p) for p in self.players), return_exceptions=True)
            failed = next((r for r in results if isinstance(r, BaseException)), None)
            if failed is not None:
                hold.blocked = (hold.blocked or f"Roster pause unconfirmed. {str(failed)}") + " Stop sent to all teams. End the match."
                await self._stop_roster()
        finally:
            hold.preparing = False

    async def _monitor(self):
        while self.phase in ("playing", "balance_hold", "balance_resuming"):
            await asyncio.sleep(.10)
            problem = next((feed.problem for feed in self.feeds.values() if feed.problem), None)
            if problem:
                await self.end(self.owner, problem)
                return
            for p in self.players:
                self.observe_controls(p)

    async def _close_fields(self, *, require_closed=False):
        feeds, self.feeds = self.feeds, {}
        results = await asyncio.gather(*(v.close() for v in feeds.values()), return_exceptions=True)
        if require_closed and any(isinstance(result, BaseException) for result in results):
            raise ValueError("An old field connection did not close")

    async def resume_balance(self, token, incident_id):
        async with self.lock:
            self.check_owner(token)
            hold = self.balance
            if (self.phase != "balance_hold" or not hold or hold.id != incident_id
                    or not self.balance_state()["ready"]):
                raise ValueError("Resume requires the current incident, verified boards and a fresh field view for every team")
            self.phase, self.message = "balance_resuming", "Reconnecting field play for all teams"
            self.transition = asyncio.current_task()
            epoch = uuid.uuid4().hex
            try:
                async with asyncio.timeout(15):
                    director = self.camera.after_balance(epoch)
                    await self._close_fields(require_closed=True)
                    self.referee = director
                    self.camera.set_match(director, self.publishers)
                    await self._open_fields(epoch, director.tracks)
                    await self.await_camera(self.players, binding=self.binding)
                    if hold.blocked or not all(hold.review(p)[1] for p in self.players):
                        raise ValueError("Board continuity changed while preparing the field")
                    incidents = {p.robot:proof(p).get("incident") for p in self.players}
                    async def arm(p):
                        board, generation = identity(hold.anchors[p.robot])
                        reply = await p.request("imu_resume", epoch=epoch, previous_epoch=hold.epoch,
                                                hold_id=hold.paused[p.robot], board_session=board, generation=generation,
                                                mode=self.match_modes[p.robot], **({"pitch":self.pitch} if self.pitch else {}))
                        native = reply.get("imu") or {}
                        if (reply.get("state") != "armed" or reply.get("epoch") != epoch
                                or reply.get("hold_id") != hold.paused[p.robot] or native.get("state") != "healthy"
                                or native.get("epoch") != epoch
                                or identity(native) != (board,generation) or native.get("incident") != incidents[p.robot]):
                            raise ValueError(f"{p.robot} did not confirm the new match and board generation")
                    results = await asyncio.gather(*(arm(p) for p in self.players), return_exceptions=True)
                    for r in results:
                        if isinstance(r, BaseException):
                            raise r
                    # The camera stays pending and keyboard drive stays closed until
                    # a fresh healthy heartbeat follows *every* new-epoch receipt.
                    deadline = time.monotonic() + .65
                    while not all(fresh(p) and proof(p).get("epoch") == epoch and proof(p).get("state") == "healthy"
                                  for p in self.players):
                        if hold.blocked or time.monotonic() >= deadline:
                            raise ValueError("Waiting for current post-resume board feedback")
                        await asyncio.sleep(.015)
                    def validate_roster():
                        for p in self.players:
                            row = proof(p)
                            if (hold.blocked or not fresh(p) or row.get("state") != "healthy"
                                    or row.get("epoch") != epoch or identity(row) != identity(hold.anchors[p.robot])
                                    or row.get("incident") != incidents[p.robot]):
                                raise ValueError("Board continuity changed before roster release")
                    await self.await_camera(self.players, binding=self.binding, before_commit=validate_roster)
                    self.epoch, self.board_anchors = epoch, {p.robot:dict(proof(p)) for p in self.players}
                    self.phase, self.message, self.balance = "playing", "Match in progress", None
            except BaseException as exc:
                self.camera.pause_balance()
                hold.blocked = "Resume did not complete for all teams. Stop sent to all teams. End the match."
                self.phase, self.message = "balance_hold", hold.blocked
                await asyncio.shield(self._stop_roster())
                if isinstance(exc,(KeyboardInterrupt,SystemExit)) or (isinstance(exc,asyncio.CancelledError) and self.aborted is not asyncio.current_task()):
                    raise
                raise ValueError(hold.blocked) from None
            finally:
                self.transition = self.aborted = None
            return self.state()

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
        if self.pause_task and not self.pause_task.done():
            self.pause_task.cancel()
            await asyncio.gather(self.pause_task, return_exceptions=True)
        self.balance = self.pause_task = self.referee = None
        await self._close_fields()
        for publisher in self.publishers:
            publisher.close()
        self.feeds.clear()
        self.publishers.clear()
        self.players, self.autonomous, self.owner, self.phase = [], [], None, "idle"

    async def _finish(self, players):
        self.phase = "stopping"
        if self.camera:
            score = ((self.camera.snapshot().get("match") or {}).get("score")
                     or (self.referee.state.get("score") if self.referee else None))
            if score:
                self.last_score = dict(score)
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
                if self.balance:
                    raise ValueError("Balance recovery takes priority over a goal reset")
                self.camera.resume()
            except (OSError, TimeoutError, ValueError, asyncio.CancelledError) as exc:
                self.transition = None
                if not self.balance:
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
