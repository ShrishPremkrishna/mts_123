"""Replay mode through the app's own routes: the production match and runner path over synthetic I/O.

Each connected team is a replay_robot interpreter running the selected RunnerSession,
OperatorDrive, CombinedRuntime and Executor; each field feed runs the selected
IngressCore and LiveMatchPlay. Assertions use what the app and its robots report
(offers, operator replies, kick receipts, telemetry, referee state, plant pose),
never a second copy of their logic. Nothing here touches a device or SSH.
"""
import asyncio
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import types
import uuid

from fastapi.testclient import TestClient
import pytest

from final_project.pupper_fc import discovery, field_transport, setup_session, teleop
from final_project.pupper_fc.lobby import Lobby, Peer
from final_project.pupper_fc.replay_transport import ROBOT_MODULE, ReplayCamera, replay_lobby, replay_profile
from final_project.pupper_fc.teleop import create_app, source_identity

OWNER = str(uuid.uuid4())
LOCAL = {"origin": "http://localhost", "host": "localhost"}
HEADERS = {"x-pupper-fc": OWNER, "origin": "http://localhost"}
ROBOT = {"blue": "pupper-1", "red": "pupper-2"}
IDENTITY = dict(kind="git", commit="0" * 40, dirty=False, detail="test")
PRODUCTION = {"session": "final_project.behaviors.combined_fc_runner.RunnerSession",
              "runtime": "fc_walk_kick.CombinedRuntime",
              "drive": "final_project.behaviors.operator_drive.OperatorDrive",
              "executor": "final_project.pupper_fc.integration.Executor",
              "gateway": "foundation.hardware.software_start.ResidentStart",
              "robot": "robot_api.Robot"}
NO_CONTACT = "no ball contact or goal is observed by this receipt"


@pytest.fixture
def no_devices(monkeypatch):
    """SSH, discovery, camera devices, Home, socket publishers and any other child process fail the test.

    The only process replay may start is its own synthetic robot module.
    """
    def forbidden(name):
        def fail(*args, **kwargs):
            raise AssertionError(f"replay reached {name}")
        return fail
    spawned = []
    real_spawn = asyncio.create_subprocess_exec

    async def only_replay_robot(*argv, **kwargs):
        if list(argv[:4]) != [sys.executable, "-B", "-m", ROBOT_MODULE]:
            raise AssertionError(f"replay spawned {argv[:4]}")
        spawned.append(argv)
        return await real_spawn(*argv, **kwargs)
    monkeypatch.setattr(asyncio, "create_subprocess_exec", only_replay_robot)
    monkeypatch.setattr(discovery, "discover", forbidden("discovery"))
    monkeypatch.setattr(field_transport, "describe", forbidden("SSH describe"))
    monkeypatch.setattr(teleop, "open_runner", forbidden("SSH runner"))
    monkeypatch.setattr(setup_session, "open_setup", forbidden("SSH setup/Home"))
    pytest.importorskip("cv2")
    monkeypatch.setattr("final_project.overhead_perception.fieldcam.capture.FrameGrabber", forbidden("a camera device"))
    monkeypatch.setattr("cv2.VideoCapture", forbidden("a camera device"))
    from final_project.overhead_perception.fieldcam import net
    # Replay reuses StatePublisher's encoding but must never construct its UDP socket.
    monkeypatch.setattr(net, "socket", types.SimpleNamespace(
        socket=forbidden("a socket publisher"), AF_INET=0, SOCK_DGRAM=0, SOL_SOCKET=0, SO_BROADCAST=0))
    return spawned


def replay_client(tmp_path):
    lobby = replay_lobby(tmp_path / "replay.json")
    app = create_app(mode="replay", lobby=lobby, identity=IDENTITY)
    return TestClient(app, base_url="http://localhost", headers=HEADERS), lobby


class Team:
    """One /play socket, drained in the background as the browser does."""

    def __init__(self, client, slot):
        reply = client.post("/api/discover", json={"slot": slot})
        assert reply.status_code == 200 and reply.json()["synthetic"] is True, reply.text
        self.robot = ROBOT[slot]
        self.context = client.websocket_connect(f"/play/{slot}?owner={OWNER}", headers=LOCAL)
        self.ws = self.context.__enter__()
        self.events, self.closed = [], False
        self.changed = threading.Condition()
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self):
        while True:
            try:
                event = self.ws.receive_json()
            except Exception:
                with self.changed:
                    self.closed = True
                    self.changed.notify_all()
                return
            with self.changed:
                self.events.append(event)
                self.changed.notify_all()

    def wait(self, check, timeout=8., after=0):
        """The first event at index >= after that satisfies check."""
        deadline = time.monotonic() + timeout
        with self.changed:
            while True:
                for event in self.events[after:]:
                    if check(event):
                        return event
                after = len(self.events)
                remaining = deadline - time.monotonic()
                assert remaining > 0 and not self.closed, (
                    f"{self.robot}: no matching event; closed={self.closed}; recent={json.dumps(self.events[-8:])[-6000:]}")
                self.changed.wait(remaining)

    def offer(self):
        """A control offer issued after this call, as the browser answers each ticket."""
        return self.wait(lambda e: e.get("event") == "controls", after=len(self.events))

    def send(self, **message):
        self.ws.send_json(dict(message, robot_id=self.robot))

    def request(self, command, *, ticket=None, **fields):
        mark = len(self.events)
        ticket = self.offer()["ticket"] if ticket is None else ticket
        self.send(command=command, ticket=ticket, **fields)
        return self.wait(lambda e: e.get("event") == "operator" and e.get("command") == command
                         and e.get("ticket") == ticket, after=mark)

    def until_accepted(self, command, timeout=6., **fields):
        deadline = time.monotonic() + timeout
        while True:
            reply = self.request(command, **fields)
            if reply["accepted"]:
                return reply
            assert time.monotonic() < deadline, reply

    def close(self):
        self.context.__exit__(None, None, None)


def wait_for(check, timeout=3., message="condition not reached"):
    deadline = time.monotonic() + timeout
    while not (value := check()):
        assert time.monotonic() < deadline, message() if callable(message) else message
        time.sleep(.03)
    return value


def open_field(client, lobby):
    reply = client.post("/api/camera", json={"source": "sim"})
    assert reply.status_code == 200, reply.text
    wait_for(lambda: (lobby.camera.snapshot().get("frame") or {}).get("calibration_valid"), 6)
    return reply.json()


def team_state(client, robot):
    return client.get("/api/state").json()["teams"].get(robot) or {}


def planner(client, robot):
    return ((team_state(client, robot).get("telemetry") or {}).get("planner") or {}).get("phase")


def healthy(team):
    """A fresh offer with current board evidence, as the lobby requires before kickoff."""
    return team.wait(lambda e: e.get("event") == "controls" and e["ready"] and e["imu"]["state"] == "healthy", 6)


def select_teleop(team):
    """Change mode on a robot ticket, then wait until its offers report it, as the Settings view does."""
    assert team.until_accepted("mode", mode="teleop")["mode"] == "teleop"
    return team.wait(lambda e: e.get("event") == "controls" and e["mode"] == "teleop" and e["ready"],
                     after=len(team.events))


def start(client, lobby, *teams):
    for team in teams:
        healthy(team)
    wait_for(lambda: not client.get("/api/state").json()["kickoff"]["field"], 4)
    reply = client.post("/api/match/start")
    assert reply.status_code == 200, reply.text
    assert reply.json()["phase"] == "playing"
    return reply.json()["camera"]["match"]["epoch"]


def test_replay_refuses_cameras_addresses_ssh_and_home(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    with client:
        assert client.get("/health").json() == {"app": "pupper-fc-lobby", "mode": "replay"}
        for source in ("phone", "0", "3"):
            refused = client.post("/api/camera", json={"source": source})
            assert refused.status_code == 409 and "never opens a camera" in refused.text
        assert lobby.camera is None
        for request in ({"slot": "blue", "host": "pi@robot.local"}, {"slot": "red", "host": "", "save_address": True}):
            refused = client.post("/api/discover", json=request)
            assert refused.status_code == 409 and "never contacts" in refused.text
        assert client.post("/api/discover", json={"slot": "blue"}).json()["host"] == "replay-blue"
        with client.websocket_connect(f"/setup/blue?owner={OWNER}", headers=LOCAL) as ws:
            event = ws.receive_json()
            assert event["stage"] == "error" and "skips Prepare and Home" in event["message"]
        state = open_field(client, lobby)
        assert state["mode"] == "replay" and state["synthetic"] and state["camera"]["synthetic"]
        wait_for(lambda: bool(lobby.camera.jpeg()))
        image = client.get("/api/frame.jpg")
        assert image.status_code == 200 and image.content.startswith(b"\xff\xd8")
        snapshot = client.post("/api/camera/snapshot", json={})
        assert snapshot.status_code == 200 and snapshot.json()["synthetic"] is True
        assert set(range(6)).issubset(lobby.camera.snapshot()["visible"]["tags"])
    assert not no_devices  # no robot was connected, so no synthetic robot started
    assert not Path.home().joinpath(".config/pupper-fc/replay.json").exists()
    assert (tmp_path / "replay.json").exists()  # throwaway settings, never the user's profile

    with pytest.raises(ValueError, match="never accepts SSH hosts"):
        create_app({"blue": "pi@robot.local"}, mode="replay", identity=IDENTITY)
    async def runner(*_):
        pass
    for override in ({"connect": runner}, {"scan": lambda: {}}, {"setup": runner}):
        with pytest.raises(ValueError, match="synthetic transport"):
            create_app(mode="replay", identity=IDENTITY, **override)
    with pytest.raises(ValueError, match="requires a replay lobby"):
        create_app(mode="replay", lobby=Lobby(settings=tmp_path / "live.json"), identity=IDENTITY)
    with pytest.raises(ValueError, match="requires a live lobby"):
        create_app(lobby=replay_lobby(tmp_path / "other.json"), identity=IDENTITY)
    with pytest.raises(ValueError, match="injected replay endpoints"):
        Lobby(replay=True, settings=tmp_path / "x.json")
    with pytest.raises(ValueError, match="mode"):
        create_app(mode="sim", identity=IDENTITY)


def test_a_replay_team_is_the_production_runner_and_unknown_health_is_never_ready(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    with client:
        blue = Team(client, "blue")
        try:
            profile = blue.wait(lambda e: e.get("event") == "profile")
            assert profile == {**replay_profile("pupper-1"), "event": "profile"}
            first = blue.wait(lambda e: e.get("event") == "controls")
            # The first offer precedes three advancing board pairs: never ready on unknown health.
            assert first["ready"] is False and first["imu"]["state"] == "unknown"
            assert first["synthetic"] is True and first["physical"] is False and type(first["ticket"]) is int
            offer = healthy(blue)
            assert offer["robot_id"] == "pupper-1" and offer["imu"]["board_session"] and offer["imu"]["generation"] == 1
            status = blue.wait(lambda e: e.get("event") == "fc_status")
            assert status["implementation"] == PRODUCTION and status["synthetic"] is True
            assert status["native"]["active"] == ["gemini_walk"] and not status["native"]["gateway_latched"]
            assert len(no_devices) == 1 and no_devices[0][no_devices[0].index("--robot-id") + 1] == "pupper-1"

            # RunnerSession's own ticket rules answer: a consumed ticket cannot be replayed.
            reply = blue.request("onboard_ball_mode", mode="confirmed")
            assert reply["accepted"] and reply["requested_mode"] == "confirmed"
            assert "decision_mono_ns" in reply and "guard_sample_mono_ns" in reply
            again = blue.request("onboard_ball_mode", ticket=reply["ticket"], mode="off")
            assert not again["accepted"] and again["reason"] == "control ticket expired or already consumed"
            mark = len(blue.events)
            blue.send(command="arm", ticket=blue.offer()["ticket"], epoch="x")  # only the lobby arms a match
            refused = blue.wait(lambda e: e.get("event") == "error", after=mark)
            assert refused["reason"] == "Use match controls for this action"
            state = client.get("/api/state").json()
            assert state["replay"]["robots"]["pupper-1"]["pid"] and state["teams"]["pupper-1"]["synthetic"]
        finally:
            blue.close()
        wait_for(lambda: not lobby.peers and not lobby.replay_network.robots, 6)


def test_autonomous_rehearsal_approaches_kicks_scores_resets_resumes_ends_and_stops(tmp_path, no_devices):
    """The acceptance rehearsal through UI routes. Decisions are production output; I/O is synthetic."""
    client, lobby = replay_client(tmp_path)
    world = lobby.replay_network.world
    with client:
        blue = Team(client, "blue")
        try:
            open_field(client, lobby)
            epoch = start(client, lobby, blue)
            arm = blue.wait(lambda e: e.get("event") == "operator" and e.get("command") == "arm")
            assert arm["accepted"] and arm["robot_id"] == "pupper-1"
            assert set(lobby.feeds) == {"pupper-1"} and all(p.synthetic for p in lobby.publishers)
            before = world.pose("striker")
            wait_for(lambda: planner(client, "pupper-1") in ("approach", "align"), 5, lambda: planner(client, "pupper-1"))
            lease = team_state(client, "pupper-1")["field"]
            assert lease["kind"] == "attack" and lease["paired"] and lease["reason"] == "paired attack"

            # The production KickReporter's receipts, in order, each stating it observes no contact.
            done = blue.wait(lambda e: e.get("event") == "kick" and e["state"] in ("completed", "retired", "aborted"), 40)
            kicks = [e for e in blue.events if e.get("event") == "kick" and e["attempt_id"] == done["attempt_id"]]
            assert [k["state"] for k in kicks] == ["engaged", "clip_started", "completed"], kicks
            assert all(k["physical"] == NO_CONTACT and k["synthetic"] and k["epoch"] == epoch for k in kicks)
            assert done["attempt_id"].startswith("r1-p0-k")
            assert math.dist(before[:2], world.pose("striker")[:2]) > .1  # the walker executed executor commands
            telemetry = wait_for(lambda: (row := team_state(client, "pupper-1")["telemetry"])["kick"].get("state")
                                 == "completed" and row, 2)
            assert telemetry["implementation"] == PRODUCTION and telemetry["ingress"]["frames"] > 20
            assert [r["source"] for r in telemetry["kick"]["receipts"]] == ["runner", "synthetic native receipt",
                                                                             "synthetic native receipt"]
            assert telemetry["labels"]["native"].startswith("Synthetic")

            # The synthetic impulse scores; the referee holds for a reset and the executor returns home.
            wait_for(lambda: (client.get("/api/state").json()["camera"]["match"] or {}).get("phase") == "goal", 12)
            assert client.get("/api/state").json()["score"]["pupper-1"] == 1
            wait_for(lambda: planner(client, "pupper-1") == "await_reset", 15, lambda: planner(client, "pupper-1"))
            assert client.post("/api/match/resume").status_code == 409  # the ball is still in the net
            assert client.get("/api/state").json()["replay"]["place_ball"]
            assert client.post("/api/replay/place-ball", json={}).status_code == 200
            wait_for(lambda: client.get("/api/state").json()["camera"]["match"]["reset_ready"], 3)
            # The executor's read-only query allows the reset; the runner then offers a resume-ready
            # ticket (RunnerSession samples the offer after the runtime tick) and consumes it.
            wait_for(lambda: resume_check(client) == "ready", 4, lambda: resume_check(client))
            blue.wait(lambda e: e.get("event") == "controls" and e["resume_ready"], 3)
            wait_for(lambda: client.get("/api/state").json()["resume_ready"], 2)
            resumed = client.post("/api/match/resume")
            assert resumed.status_code == 200, resumed.text
            assert blue.wait(lambda e: e.get("event") == "operator" and e.get("command") == "resume")["accepted"]
            assert client.post("/api/match/resume").status_code == 409  # the reset is consumed once
            wait_for(lambda: planner(client, "pupper-1") in ("approach", "align"), 6, lambda: planner(client, "pupper-1"))
            wait_for(lambda: team_state(client, "pupper-1")["field"]["kind"] == "attack", 3)

            ended = client.post("/api/match/end")
            assert ended.status_code == 200 and ended.json()["phase"] == "idle"
            pause = blue.wait(lambda e: e.get("event") == "operator" and e.get("command") == "pause")
            assert pause["accepted"] and pause["state"] in ("standby", "hard_stop")
            assert not lobby.feeds and not lobby.publishers and not lobby.replay_network.fields
            wait_for(lambda: team_state(client, "pupper-1")["telemetry"]["armed"] is False, 3)
            blue.offer()  # the runner keeps offering tickets between matches

            if pause["state"] == "standby":
                again = start(client, lobby, blue)
                assert again != epoch
                assert client.post("/api/match/end").json()["phase"] == "idle"

            mark = len(blue.events)
            blue.send(command="stop")
            result = blue.wait(lambda e: e.get("event") == "stop_result", 6, after=mark)
            assert result["synthetic"] and result["confirmed"] and not result["uncertain"]
            assert result["scope"].startswith("gateway latch acknowledgement only")
            wait_for(lambda: blue.closed and not lobby.peers and lobby.phase == "idle", 6)
            pose = world.pose("striker")
            time.sleep(.3)
            assert world.pose("striker") == pose  # nothing moves after Stop
        finally:
            blue.close()


def resume_check(client, robot="pupper-1"):
    return ((team_state(client, robot).get("telemetry") or {}).get("planner") or {}).get("resume")


def test_two_autonomous_teams_take_referee_roles_through_production_match_play(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    with client:
        blue, red = Team(client, "blue"), Team(client, "red")
        try:
            open_field(client, lobby)
            start(client, lobby, blue, red)

            def roles():
                state = client.get("/api/state").json()
                leases = {robot: (state["teams"][robot].get("field") or {}) for robot in ROBOT.values()}
                if not all(row.get("paired") for row in leases.values()):
                    return None
                return state["camera"]["match"]["attacker"], {robot: row["kind"] for robot, row in leases.items()}
            attacker, kinds = wait_for(roles, 5)
            defender = next(robot for robot in ROBOT.values() if robot != attacker)
            assert kinds == {attacker: "attack", defender: "defend"}
            assert team_state(client, defender)["field"]["reason"] == "guard own goal"
            wait_for(lambda: planner(client, defender) == "defend", 4, lambda: planner(client, defender))
            # Two runner processes, two board sessions, one shared referee.
            pids = client.get("/api/state").json()["replay"]["robots"]
            assert pids["pupper-1"]["pid"] != pids["pupper-2"]["pid"]
            boards = {team.robot: team.offer()["imu"]["board_session"] for team in (blue, red)}
            assert boards["pupper-1"] != boards["pupper-2"]
            assert client.post("/api/match/end").json()["phase"] == "idle"
        finally:
            blue.close()
            red.close()


def test_teleop_uses_production_tickets_deadlines_and_kickoff_epoch(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    world = lobby.replay_network.world
    with client:
        red = Team(client, "red")
        try:
            healthy(red)
            select_teleop(red)
            open_field(client, lobby)
            epoch = start(client, lobby, red)
            assert not lobby.feeds and not lobby.publishers  # a keyboard team receives no field lease
            wait_for(lambda: (lobby.camera.snapshot().get("match") or {}).get("phase") == "playing", 4)
            assert team_state(client, "pupper-2")["telemetry"]["armed"] is False  # executor stays disarmed

            stale = red.offer()["ticket"] - 3
            refused = red.request("drive", ticket=stale, epoch=epoch, forward=1, left=0, turn=0)
            assert refused["accepted"] is False and refused["reason"] == "control ticket expired or already consumed"
            assert red.request("drive", epoch=epoch, forward=0, left=0, turn=0)["accepted"]  # release first
            before = world.pose("keeper")
            moved = red.request("drive", epoch=epoch, forward=1, left=0, turn=0)
            assert moved["accepted"], moved
            reused = red.request("drive", ticket=moved["ticket"], epoch=epoch, forward=1, left=0, turn=0)
            assert reused["accepted"] is False and "consumed" in reused["reason"]
            # One ticket drives at most its 0.25 s lifetime, then the plant is still.
            time.sleep(.45)
            settled = world.pose("keeper")
            assert math.dist(before[:2], settled[:2]) > .005
            time.sleep(.3)
            assert world.pose("keeper") == settled
            wrong = red.request("drive", epoch="another-match", forward=1, left=0, turn=0)
            assert wrong["accepted"] is False and wrong["reason"] == "drive needs the current accepted kickoff"

            assert client.post("/api/match/end").json()["phase"] == "idle"
            after = red.request("drive", epoch=epoch, forward=1, left=0, turn=0)
            assert after["accepted"] is False and after["reason"] == "drive needs the current accepted kickoff"
        finally:
            red.close()


def test_stop_is_terminal_and_only_an_explicit_reconnect_opens_a_new_runner(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    with client:
        red = Team(client, "red")
        try:
            healthy(red)
            select_teleop(red)
            open_field(client, lobby)
            old = start(client, lobby, red)
            first_pid = client.get("/api/state").json()["replay"]["robots"]["pupper-2"]["pid"]
            red.send(command="stop")
            result = red.wait(lambda e: e.get("event") == "stop_result", 6)
            assert result["synthetic"] and result["request"]["reason"] == "operator Stop or input closed"
            wait_for(lambda: red.closed and lobby.phase == "idle" and not lobby.peers, 6)
            assert "requested Stop" in client.get("/api/state").json()["message"]
            assert "pupper-2" not in client.get("/api/state").json()["replay"]["robots"]
        finally:
            red.close()

        again = Team(client, "red")  # the operator reconnects explicitly
        try:
            offer = healthy(again)
            assert offer["mode"] == "autonomous"  # a new runner starts from its own defaults
            assert client.get("/api/state").json()["replay"]["robots"]["pupper-2"]["pid"] != first_pid
            select_teleop(again)
            stale = again.request("drive", epoch=old, forward=1, left=0, turn=0)
            assert stale["accepted"] is False and stale["reason"] == "drive needs the current accepted kickoff"
            new = start(client, lobby, again)
            assert new != old
            assert client.post("/api/match/end").json()["phase"] == "idle"
        finally:
            again.close()


def test_a_ball_left_outside_the_field_blocks_kickoff_until_the_operator_places_it(tmp_path, no_devices):
    """After a goal and End match the ball rests in the net. Readiness says so; Place ball is the remedy."""
    client, lobby = replay_client(tmp_path)
    world = lobby.replay_network.world
    with client:
        blue = Team(client, "blue")
        try:
            open_field(client, lobby)
            healthy(blue)
            wait_for(lambda: not client.get("/api/state").json()["kickoff"]["field"], 4)
            world.place_ball(-(world.length / 2 + world.NET), 0.)  # where a scored ball comes to rest

            def blocked():
                problems = client.get("/api/state").json()["kickoff"]["field"]
                return problems and problems[0].startswith("the ball is outside the field")
            wait_for(blocked, 3)
            state = client.get("/api/state").json()
            assert state["replay"]["place_ball"] and not state["replay"]["ball_inside"]
            began = time.monotonic()
            refused = client.post("/api/match/start")
            assert refused.status_code == 409 and "the ball is outside the field" in refused.json()["detail"]
            assert time.monotonic() - began < 5 and lobby.phase == "idle"
            assert not [e for e in blue.events if e.get("command") == "arm"]  # no robot was armed

            assert client.post("/api/replay/place-ball", json={}).status_code == 200
            wait_for(lambda: not client.get("/api/state").json()["kickoff"]["field"], 3)
            start(client, lobby, blue)
            # In play, the operator may not move a ball that is inside the field.
            refused = client.post("/api/replay/place-ball", json={})
            assert refused.status_code == 409 and "in play" in refused.json()["detail"]
            assert client.post("/api/match/end").json()["phase"] == "idle"
        finally:
            blue.close()


def test_live_default_keeps_physical_gates_and_refuses_replay_endpoints(tmp_path):
    from final_project.pupper_fc.replay_transport import ReplayNetwork
    network = ReplayNetwork()
    app = create_app(identity=IDENTITY)
    with TestClient(app, base_url="http://localhost", headers=HEADERS) as client:
        assert client.get("/health").json()["mode"] == "live"
        info = client.get("/api/app").json()
        assert info["mode"] == "live" and info["hardware_qualified"] is False and not info["synthetic"]
        state = client.get("/api/state").json()
        assert state["mode"] == "live" and "replay" not in state
        assert client.post("/api/replay/place-ball", json={}).status_code == 409

    async def check():
        lobby = Lobby(camera_factory=ReplayCamera, describe_peer=network.describe, field_factory=network.open_field,
                      publisher_factory=network.publisher, settings=tmp_path / "live.json")
        with pytest.raises(ValueError, match="only in replay mode"):
            await lobby.attach_camera({"source": "sim"}, OWNER)
        assert lobby.camera is None
        # A replay robot smuggled into a live lobby is refused before any arm.
        live = Lobby(settings=tmp_path / "live2.json")
        live.camera = types.SimpleNamespace(snapshot=lambda: {"live": True})
        writes = []
        proc = types.SimpleNamespace(synthetic=True, stdin=types.SimpleNamespace(write=writes.append))
        peer = Peer("pupper-1", "replay-blue", OWNER, proc)
        peer.offer = {"mode": "autonomous", "ready": True, "ticket": 1}
        live.peers[peer.robot] = peer
        with pytest.raises(ValueError, match="Live matches refuse replay"):
            await live.start(OWNER)
        assert not writes and live.phase == "idle"
    asyncio.run(check())


def test_source_identity_reports_release_stamp_git_head_or_unknown(tmp_path):
    release = tmp_path / "release"
    release.mkdir()
    (release / "release-source.json").write_text(json.dumps({"source_commit": "abc1234def"}))
    assert source_identity(release) == dict(kind="release", commit="abc1234def", dirty=None, detail="release-source.json")
    (release / "release-source.json").write_text("not json")
    assert source_identity(release)["commit"] is None
    bare = tmp_path / "bare"
    bare.mkdir()
    assert source_identity(bare)["kind"] == "unknown"
    # Git identity is tested in its own repository so this check also works in
    # an installed archive, whose correct identity is its release stamp.
    root = tmp_path / "git-source"
    subprocess.run(["git", "init", "--quiet", "--template=", str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "-c", "user.name=FC test", "-c", "user.email=fc-test@example.invalid",
                    "-c", "commit.gpgsign=false", "-c", "core.hooksPath=/dev/null", "commit", "--quiet",
                    "--allow-empty", "-m", "Source identity fixture"], check=True)
    head = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    current = source_identity(root)
    assert current["kind"] == "git" and current["commit"] == head and type(current["dirty"]) is bool

    client, _ = replay_client(tmp_path)
    with client:
        info = client.get("/api/app").json()
        assert info["mode"] == "replay" and info["synthetic"] and info["hardware_qualified"] is False
        assert info["source"] == IDENTITY and info["native_candidate"]["status"] == "pending"
        assert "never physical" in info["limitation"] and "LiveMatchPlay" in info["executor"]["planner"]
        state = client.get("/api/state").json()
        assert state["app"] == info and state["replay"]["synthetic"] and not state["replay"]["robots"]


def test_offline_check_is_labelled_software_only(tmp_path, monkeypatch):
    fake = types.ModuleType("final_project.pupper_fc.self_test")
    fake.check = lambda: {"ok": True, "summary": "Synthetic field passed."}
    monkeypatch.setitem(sys.modules, fake.__name__, fake)
    monkeypatch.setattr("final_project.pupper_fc.self_test", fake, raising=False)
    client, _ = replay_client(tmp_path)
    with client:
        report = client.post("/api/self-test", json={}).json()
    assert report == {"ok": True, "summary": "Synthetic field passed.", "scope": "software-only", "hardware_qualified": False}


def test_shell_navigation_keeps_existing_controls():
    html = (Path(teleop.STATIC) / "controls.html").read_text()
    for name in ("home", "field", "robot", "settings", "play"):
        assert f'data-view-target="{name}"' in html
    for existing in ("stop-all", "teams", "camera-connect", "start-match", "end-match", "self-test",
                     "self-test-result", "robot-setup", "fieldpanel", "camera-source", "error"):
        assert f'id="{existing}"' in html
    for added in ("mode-badge", "app-source", "app-mode-label", "app-hardware", "app-native", "app-limitation",
                  "play-view", "onboards", "telemetry", "lifecycles", "team-settings", "place-ball", "resume-match"):
        assert f'id="{added}"' in html
    assert 'class="t-lease"' in html
    # Stop lives in the sticky header, outside every switchable view.
    header = html[html.index('<header class="bar">'):html.index("</header>")]
    assert 'id="stop-all"' in header and 'id="mode-badge"' in header and "data-views" not in header


def test_failed_camera_start_detaches_and_closes(tmp_path):
    cameras = []
    class BrokenCamera:
        def __init__(self, *args, **kwargs):
            self.closed = False
            cameras.append(self)
        def start(self):
            raise RuntimeError("camera failed to start")
        def close(self):
            self.closed = True
    async def check():
        lobby = Lobby(camera_factory=BrokenCamera, settings=tmp_path / "settings.json")
        with pytest.raises(RuntimeError, match="failed to start"):
            await lobby.attach_camera({"source": "0"}, OWNER)
        assert lobby.camera is None and cameras[0].closed
    asyncio.run(check())


def test_synthetic_clock_cannot_rewrite_a_device_capture():
    from final_project.pupper_fc.arena_camera import ArenaCamera
    with pytest.raises(ValueError, match="only valid for the rendered simulation"):
        ArenaCamera({}, "phone", synthetic_clock=lambda _: time.time())


def test_field_producer_is_ingress_and_live_match_play_without_sockets(no_devices):
    """A StatePublisher encoding, a measured clock probe and IngressCore admission, then LiveMatchPlay."""
    from final_project.behaviors.live_field_attack import LiveMatchPlay
    from final_project.pupper_fc.replay_transport import ROBOT_HOSTS, ReplayNetwork

    async def check():
        network = ReplayNetwork()
        publisher = network.publisher(ROBOT_HOSTS["pupper-1"], 5005, source_session="s1", live=True)
        info = replay_profile("pupper-1")
        feed = await network.open_field("replay-blue", "pupper-1", info, "ep", "s1", 1, {"striker": "pupper-1"})
        field = network.fields[ROBOT_HOSTS["pupper-1"]]
        assert type(field.play) is LiveMatchPlay
        captured = time.time()
        f = info["field_config"]["field"]
        row = {"visible": True, "measured_this_frame": True, "measured_at": captured, "age_s": 0., "x": -.5, "y": .1,
               "yaw": 0., "yaw_at_capture": 0., "vx": 0., "vy": 0., "wz": 0.}
        state = {"seq": 1, "captured_at": captured, "stamp": captured, "frame_age_s": 0., "calibration_valid": True,
                 "calibration_revision": 1, "field": dict(f), "robots": {"striker": row},
                 "goals": {name: {"x": sign * f["length"] / 2, "y_min": -f["goal_width"] / 2, "y_max": f["goal_width"] / 2}
                           for name, sign in (("west", -1), ("east", 1))},
                 "ball": dict(row, x=0., y=0.), "fc_match": None}
        publisher.send(state)  # the first datagram binds the peer; the next poll measures the clock
        await asyncio.sleep(.1)
        assert field.ingress.sync is not None
        publisher.send(dict(state, seq=2, captured_at=time.time(), stamp=time.time()))
        await asyncio.sleep(.1)
        # Admitted through IngressCore; held by LiveMatchPlay because no referee directive exists yet.
        assert field.frames >= 1 and field.latest["paired"] is False and "directive" in field.latest["reason"]
        assert publisher.wire_seq == 2
        publisher.close()
        await feed.close()
        assert ROBOT_HOSTS["pupper-1"] not in network.fields and not network.publishers
    asyncio.run(check())
