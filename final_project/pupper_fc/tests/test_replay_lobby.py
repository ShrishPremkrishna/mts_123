"""Replay mode: the real Lobby/Peer/FieldFeed transactions over synthetic endpoints that never touch devices."""
import asyncio
import json
from pathlib import Path
import subprocess
import sys
import time
import types
import uuid

from fastapi.testclient import TestClient
import pytest
from starlette.websockets import WebSocketDisconnect

from final_project.pupper_fc import discovery, field_transport, setup_session, teleop
from final_project.pupper_fc.lobby import Lobby
from final_project.pupper_fc.replay_transport import ReplayCamera, ReplayRunner, replay_lobby
from final_project.pupper_fc.teleop import create_app, source_identity

OWNER = str(uuid.uuid4())
LOCAL = {"origin": "http://localhost", "host": "localhost"}
HEADERS = {"x-pupper-fc": OWNER, "origin": "http://localhost"}
ROBOT = {"blue": "pupper-1", "red": "pupper-2"}
IDENTITY = dict(kind="git", commit="0" * 40, dirty=False, detail="test")


@pytest.fixture
def no_devices(monkeypatch):
    """Any SSH, discovery, camera device, Home or socket publisher use fails the test."""
    def forbidden(name):
        def fail(*args, **kwargs):
            raise AssertionError(f"replay reached {name}")
        return fail
    async def forbidden_async(*args, **kwargs):
        raise AssertionError("replay spawned a subprocess")
    monkeypatch.setattr(asyncio, "create_subprocess_exec", forbidden_async)
    monkeypatch.setattr(discovery, "discover", forbidden("discovery"))
    monkeypatch.setattr(field_transport, "describe", forbidden("SSH describe"))
    monkeypatch.setattr(teleop, "open_runner", forbidden("SSH runner"))
    monkeypatch.setattr(setup_session, "open_setup", forbidden("SSH setup/Home"))
    pytest.importorskip("cv2")
    monkeypatch.setattr("final_project.overhead_perception.fieldcam.capture.FrameGrabber", forbidden("a camera device"))
    monkeypatch.setattr("cv2.VideoCapture", forbidden("a camera device"))
    publisher = types.ModuleType("final_project.overhead_perception.fieldcam.net")
    publisher.StatePublisher = forbidden("a socket publisher")
    monkeypatch.setitem(sys.modules, publisher.__name__, publisher)


def replay_client(tmp_path):
    lobby = replay_lobby(tmp_path / "replay.json")
    app = create_app(mode="replay", lobby=lobby, identity=IDENTITY)
    return TestClient(app, base_url="http://localhost", headers=HEADERS), lobby


def join(client, slot):
    reply = client.post("/api/discover", json={"slot": slot})
    assert reply.status_code == 200 and reply.json()["synthetic"] is True, reply.text
    return client.websocket_connect(f"/play/{slot}?owner={OWNER}", headers=LOCAL)


def controls(sock, mode=None):
    while True:
        event = sock.receive_json()
        if event.get("event") == "controls" and (mode is None or event["mode"] == mode):
            return event


def wait_for(check, timeout=3.):
    deadline = time.monotonic() + timeout
    while not check():
        assert time.monotonic() < deadline, "condition not reached"
        time.sleep(.02)


def open_field(client, lobby):
    reply = client.post("/api/camera", json={"source": "sim"})
    assert reply.status_code == 200, reply.text
    wait_for(lambda: (lobby.camera.snapshot().get("frame") or {}).get("seq", 0) > 1)
    return reply.json()


def drive(sock, robot, epoch, forward=1):
    """Send one keyed drive on a fresh ticket; return the replay runner's receipt."""
    offer = controls(sock)
    sock.send_json(dict(command="drive", robot_id=robot, ticket=offer["ticket"], epoch=epoch,
                        forward=forward, left=0, turn=0))
    while (event := sock.receive_json()).get("command") != "drive":
        pass
    return event


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


def test_replay_start_and_end_run_through_the_lobby_with_two_independent_peers(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    with client, join(client, "blue") as blue, join(client, "red") as red:
        first = {slot: controls(sock) for slot, sock in (("blue", blue), ("red", red))}
        for slot, offer in first.items():
            assert offer["robot_id"] == ROBOT[slot] and offer["synthetic"] is True and offer["physical"] is False
            assert offer["imu"]["synthetic"] is True
        red.send_json(dict(command="mode", robot_id="pupper-2", mode="teleop", ticket=controls(red)["ticket"]))
        controls(red, "teleop")
        assert client.post("/api/match/start").status_code == 409  # no field yet
        open_field(client, lobby)
        peers = dict(lobby.peers)
        assert set(peers) == set(ROBOT.values()) and peers["pupper-1"] is not peers["pupper-2"]
        runners = {robot: peer.proc for robot, peer in peers.items()}
        assert all(isinstance(r, ReplayRunner) for r in runners.values())
        assert runners["pupper-1"].board["board_session"] != runners["pupper-2"].board["board_session"]

        started = client.post("/api/match/start")
        assert started.status_code == 200, started.text
        state = started.json()
        epoch = state["camera"]["match"]["epoch"]
        assert state["phase"] == "playing" and lobby.players == [peers["pupper-1"], peers["pupper-2"]]
        for robot, runner in runners.items():
            arms = [c for c in runner.commands if c["command"] == "arm"]
            assert [(a["epoch"], a["robot_id"]) for a in arms] == [(epoch, robot)]
            assert runner.armed_epoch == epoch
        # Only the autonomous team receives a field feed and an in-memory publisher.
        assert set(lobby.feeds) == {"pupper-1"} and lobby.feeds["pupper-1"].opened
        publishers = list(lobby.publishers)
        assert len(publishers) == 1 and publishers[0].synthetic
        wait_for(lambda: (lobby.camera.snapshot().get("match") or {}).get("phase") == "playing")
        receipt = drive(red, "pupper-2", epoch)
        assert receipt["accepted"] and receipt["applied"]["forward"] == 1 and receipt["synthetic"]
        assert not [c for c in runners["pupper-1"].commands if c["command"] == "drive"]  # independent slots
        feeds = list(lobby.feeds.values())

        ended = client.post("/api/match/end")
        assert ended.status_code == 200 and ended.json()["phase"] == "idle"
        assert all(f.proc.returncode == 0 for f in feeds) and all(p.closed for p in publishers)
        assert all(r.armed_epoch is None and r.returncode is None for r in runners.values())
        assert [c["command"] for c in runners["pupper-2"].commands if c["command"] in ("arm", "pause")] == ["arm", "pause"]
    assert all(r.returncode == 0 for r in runners.values())


def test_stop_and_restart_cannot_revive_prior_authority(tmp_path, no_devices):
    client, lobby = replay_client(tmp_path)
    with client, join(client, "red") as red:
        red.send_json(dict(command="mode", robot_id="pupper-2", mode="teleop", ticket=controls(red)["ticket"]))
        controls(red, "teleop")
        open_field(client, lobby)
        runner = lobby.peers["pupper-2"].proc
        old = client.post("/api/match/start").json()["camera"]["match"]["epoch"]
        wait_for(lambda: (lobby.camera.snapshot().get("match") or {}).get("phase") == "playing")
        assert drive(red, "pupper-2", old)["applied"]["forward"] == 1

        red.send_json(dict(command="stop", robot_id="pupper-2"))
        while (event := red.receive_json()).get("event") != "stop_result":
            pass
        assert event["synthetic"] and event["confirmed"] is False and "No physical Stop" in event["reason"]
        wait_for(lambda: lobby.phase == "idle")
        assert runner.armed_epoch is None and "requested Stop" in lobby.message
        # The lobby brakes stale drive, and the runner independently refuses the old epoch.
        assert drive(red, "pupper-2", old)["applied"]["forward"] == 0
        runner.stdin.write(json.dumps(dict(command="drive", robot_id="pupper-2", ticket=runner.ticket,
                                           epoch=old, forward=1, left=0, turn=0)).encode())
        while (refused := red.receive_json()).get("command") != "drive":
            pass
        assert refused["accepted"] is False and "current match epoch" in refused["reason"]
        assert runner.motion["forward"] == 0

        restart = client.post("/api/match/start")
        assert restart.status_code == 200, restart.text
        new = restart.json()["camera"]["match"]["epoch"]
        assert new != old and runner.armed_epoch == new
        wait_for(lambda: (lobby.camera.snapshot().get("match") or {}).get("phase") == "playing")
        assert drive(red, "pupper-2", old)["applied"]["forward"] == 0
        assert drive(red, "pupper-2", new)["applied"]["forward"] == 1
        assert client.post("/api/match/end").json()["phase"] == "idle"


def test_live_default_keeps_physical_gates_and_refuses_replay_endpoints(tmp_path):
    from final_project.pupper_fc.replay_transport import ReplayPublisher, open_replay_field, replay_describe
    app = create_app(identity=IDENTITY)
    with TestClient(app, base_url="http://localhost", headers=HEADERS) as client:
        assert client.get("/health").json()["mode"] == "live"
        info = client.get("/api/app").json()
        assert info["mode"] == "live" and info["hardware_qualified"] is False and not info["synthetic"]
        assert client.get("/api/state").json()["mode"] == "live"

    async def check():
        lobby = Lobby(camera_factory=ReplayCamera, describe_peer=replay_describe, field_factory=open_replay_field,
                      publisher_factory=ReplayPublisher, settings=tmp_path / "live.json")
        with pytest.raises(ValueError, match="only in replay mode"):
            await lobby.attach_camera({"source": "sim"}, OWNER)
        assert lobby.camera is None
        # A replay runner smuggled into a live lobby is refused before any arm.
        live = Lobby(settings=tmp_path / "live2.json")
        live.camera = types.SimpleNamespace(snapshot=lambda: {"live": True})
        runner = ReplayRunner("pupper-1")
        from final_project.pupper_fc.lobby import Peer
        peer = Peer("pupper-1", "replay-blue", OWNER, runner)
        peer.offer = {"mode": "autonomous", "ready": True, "ticket": 1}
        live.peers[peer.robot] = peer
        with pytest.raises(ValueError, match="Live matches refuse replay"):
            await live.start(OWNER)
        assert not [c for c in runner.commands if c["command"] == "arm"] and live.phase == "idle"
        runner.close()
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
    root = Path(teleop.__file__).resolve().parents[2]
    head = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    current = source_identity(root)
    assert current["kind"] == "git" and current["commit"] == head and type(current["dirty"]) is bool

    client, _ = replay_client(tmp_path)
    with client:
        info = client.get("/api/app").json()
        assert info["mode"] == "replay" and info["synthetic"] and info["hardware_qualified"] is False
        assert info["source"] == IDENTITY and info["native_candidate"]["status"] == "pending"
        assert "never physical" in info["limitation"]
        assert client.get("/api/state").json()["app"] == info


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
    for added in ("mode-badge", "app-source", "app-mode-label", "app-hardware", "app-native", "app-limitation"):
        assert f'id="{added}"' in html


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
