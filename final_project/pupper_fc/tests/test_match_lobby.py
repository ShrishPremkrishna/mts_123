"""Rehearse real HTTP + WebSockets and referee wiring with in-memory Pi and camera endpoints."""
import asyncio
import json
import time
import uuid
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest

from final_project.pupper_fc.discovery import teams
from final_project.pupper_fc.field_transport import FieldFeed
from final_project.pupper_fc.lobby import Lobby
from final_project.pupper_fc.match_director import goal_geometry
from final_project.pupper_fc.teleop import create_app

OWNER = str(uuid.uuid4())
HEADERS = {"x-pupper-fc": OWNER, "origin":"http://localhost"}
ROBOT = {"blue":"pupper-1","red":"pupper-2"}


class LabRunner:
    def __init__(self, robot, reject=False):
        self.robot, self.reject = robot, reject
        self.commands, self.ticket = [], 0
        self.closed = self.armed = False
        self.mode = "autonomous"
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(write=self.write, drain=self.drain, close=self.close)
        self.ticks = asyncio.create_task(self.heartbeat())

    def emit(self, event):
        self.stdout.feed_data((json.dumps(dict(event, robot_id=self.robot))+"\n").encode())

    async def heartbeat(self):
        while not self.closed:
            self.ticket += 1
            self.emit(dict(event="controls",mode=self.mode,ticket=self.ticket,ready=True,resume_ready=True))
            await asyncio.sleep(.02)

    def write(self, data):
        msg = json.loads(data)
        self.commands.append(msg)
        ok = not (self.reject and msg["command"] == "arm")
        if ok:
            if msg["command"] == "arm":
                self.armed = True
            elif msg["command"] in ("pause","stop"):
                self.armed = False
            elif msg["command"] == "mode":
                self.mode = msg["mode"]
        self.emit(dict(event="operator",command=msg["command"],ticket=msg.get("ticket"),
                       accepted=ok,reason="injected missing zero command" if not ok else ""))

    async def drain(self):
        pass

    def close(self):
        if not self.closed:
            self.closed, self.armed = True, False
            self.ticks.cancel()
            self.stdout.feed_eof()

    async def wait(self):
        assert self.closed

    terminate = kill = close


class Camera:
    def __init__(self, profile, source, *, appearance):
        self.source = source
        self.session = uuid.uuid4().hex
        self.director, self.pubs = None, ()
        self.playing = False
        self.reset = False
        self.started = False

    def start(self):
        self.started = True

    def close(self):
        self.started = False

    def jpeg(self):
        return b"jpeg"

    def snapshot(self):
        return dict(live=self.source != "sim", source_session=self.session,
                    frame=dict(calibration_valid=True, calibration_revision=2,captured_at=time.time(),seq=1),
                    visible=dict(ball=True,tags=list(range(6)),robots=dict(striker=True,keeper=True)),
                    match=dict(reset_ready=self.reset, epoch=self.director.epoch, phase="playing" if self.playing else "pending") if self.director else None)

    def set_match(self, director, pubs=()):
        self.director, self.pubs = director, tuple(pubs)
        self.playing = False

    def resume(self):
        self.reset = False

    def commit_start(self):
        assert self.director is not None
        self.playing = True


class Feed:
    def __init__(self):
        self.closed = False
        self.problem = None

    async def wait_ready(self):
        pass

    async def close(self):
        self.closed = True


class Publisher:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def close(self):
        self.closed = True


def rig(tmp_path,reject=None):
    runners, feeds, destinations = {}, [], []
    async def connect(host, robot):
        runners[robot] = LabRunner(robot, reject == robot)
        return runners[robot]
    async def describe(host, robot):
        return dict(match_pitch_version=1, field_config=dict(field=lobby.profile["field"], attack_signs={"pupper-1":-1,"pupper-2":1}),
                    robot_host=host, source_host="camera", port=5005)
    async def field(host, robot, info, epoch, session, revision, tracks):
        assert revision == 2 and robot in tracks.values()
        feeds.append(Feed())
        return feeds[-1]
    def publisher(host, port, **kwargs):
        assert kwargs["live"]
        destinations.append((host,Publisher()))
        return destinations[-1][1]
    lobby = Lobby(camera_factory=Camera,describe_peer=describe,field_factory=field,publisher_factory=publisher,
                  settings=tmp_path/"settings.json")
    app = create_app(connect=connect,lobby=lobby,scan=lambda: {robot:[slot+"-pi"] for slot,robot in ROBOT.items()})
    return TestClient(app,base_url="http://localhost",headers=HEADERS),lobby,runners,feeds,destinations


def enter(client,slot):
    assert client.post("/api/discover",json={"slot":slot}).status_code == 200
    return client.websocket_connect("/play/"+slot+"?owner="+OWNER,headers={"origin":"http://localhost","host":"localhost"})


@pytest.mark.parametrize("slots,modes", [
    (("blue",), ("autonomous",)), (("red",), ("autonomous",)), (("red",), ("teleop",)),
    (("blue", "red"), ("autonomous", "autonomous")), (("blue", "red"), ("teleop", "autonomous")),
    (("blue", "red"), ("teleop", "teleop")),
])
def test_start_end_and_restart_with_current_team(tmp_path,slots,modes):
    from contextlib import ExitStack
    client,lobby,robots,feeds,dests = rig(tmp_path)
    with client, ExitStack() as stack:
        for slot, mode in zip(slots, modes):
            sock = stack.enter_context(enter(client,slot))
            offer = sock.receive_json()
            assert offer["robot_id"] == ROBOT[slot]
            if mode == "teleop":
                sock.send_json(dict(command="mode", robot_id=ROBOT[slot], mode=mode, ticket=offer["ticket"]))
                while (event := sock.receive_json()).get("event") != "controls" or event["mode"] != mode:
                    pass
        assert client.post("/api/camera",json={"source":"0"}).status_code == 200
        for i in range(2):
            reply = client.post("/api/match/start")
            assert reply.status_code == 200, reply.text
            assert reply.json()["phase"]=="playing" and all(p.armed for p in robots.values())
            assert set(lobby.camera.director.tracks.values()) == set(robots)
            assert client.post("/api/match/end",headers={"x-pupper-fc":str(uuid.uuid4())}).status_code == 409
            lobby.camera.reset = True
            assert client.post("/api/match/resume").status_code == 200
            assert client.post("/api/match/end").json()["phase"] == "idle"
            assert not any(p.armed or p.closed for p in robots.values())
            assert all(f.closed for f in feeds) and all(p.closed for _,p in dests)
        for p in robots.values():
            arms = [cmd["epoch"] for cmd in p.commands if cmd["command"] == "arm"]
            assert len(set(arms))==2
            assert [cmd["mode"] for cmd in p.commands if cmd["command"] == "arm"] == [p.mode] * 2


def test_rollback_partial_arm_and_reject_simulated_camera(tmp_path):
    client,lobby,robots,feeds,dests = rig(tmp_path,reject="pupper-2")
    with client, enter(client,"blue") as blue, enter(client,"red") as red:
        blue.receive_json()
        red.receive_json()
        assert client.post("/api/camera",json={"source":"sim"}).status_code==200
        assert client.post("/api/match/start").status_code==409
        assert not feeds and not dests
        client.post("/api/camera",json={"source":"0"})
        reply=client.post("/api/match/start")
        assert reply.status_code==409 and "zero command" in reply.text
        assert lobby.phase=="idle" and all(not p.armed for p in robots.values())
        assert all(f.closed for f in feeds) and all(p.closed for _,p in dests)
        assert not lobby.camera.director


def test_cross_origin_and_discovery_collisions(tmp_path):
    client, *_ = rig(tmp_path)
    with client:
        assert client.post("/api/discover",json={"slot":"red"},headers={"origin":"http://other.invalid"}).status_code==403
    props={b"robot_id":b"pupper-1",b"protocol":b"1"}
    records=[SimpleNamespace(server=host,port=22,properties=props) for host in ("one.local.","two.local.","two.local","-x.local","x.bad")]
    assert teams(records)=={"pupper-1":["pi@one.local","pi@two.local"],"pupper-2":[]}


def test_actual_field_status_contract():
    async def check():
        out=asyncio.StreamReader()
        f=FieldFeed(SimpleNamespace(stdout=out),"pupper-1","game","camera")
        def emit(row):
            out.feed_data((json.dumps(row)+"\n").encode())
        emit(dict(event="source_open", mode="publish",robot_id="pupper-1",epoch="game",source={"source_session":"camera"}))
        emit(dict(mode="publish",frames=52,pairs=50,latest=dict(paired=False,reason="missing current ball")))
        with pytest.raises(ValueError, match="missing current ball"):
            await f.wait_ready(.015)
        emit(dict(mode="publish",frames=55,pairs=51,latest=dict(paired=True)))
        await f.wait_ready(.2)
        out.feed_eof()
        await f.reader
        assert "closed" in f.problem
    asyncio.run(check())


def test_stop_preempts_arm_and_cancelled_http_end_still_releases_everything(tmp_path):
    async def run():
        entered, pause_started, finish_pause = asyncio.Event(), asyncio.Event(), asyncio.Event()
        class Peer:
            owner, robot, host, closed = OWNER, "pupper-1", "blue-pi", False
            offer = {"mode":"autonomous","ready":True}
            async def request(self, cmd, **fields):
                if cmd == "arm":
                    entered.set()
                    await asyncio.Future()
                if cmd == "pause":
                    pause_started.set()
                    await finish_pause.wait()
                    return {"state":"standby"}
        client,lobby,_,feeds,dest = rig(tmp_path)
        await lobby.attach_camera({"source":"0"},OWNER)
        p = Peer()
        lobby.peers[p.robot] = p
        kickoff = asyncio.create_task(lobby.start(OWNER))
        await asyncio.wait_for(entered.wait(), .5)
        assert lobby.camera.director.pending_start
        end = asyncio.create_task(lobby.end(OWNER))
        await asyncio.wait_for(pause_started.wait(), .5)
        end.cancel()
        await asyncio.gather(end,return_exceptions=True)
        finish_pause.set()
        with pytest.raises(ValueError):
            await asyncio.wait_for(kickoff,.5)
        await asyncio.wait_for(lobby.ending,.5)
        assert lobby.phase == "idle" and lobby.camera.director is None
        assert all(f.closed for f in feeds) and all(p.closed for _,p in dest)
        client.close()
    asyncio.run(run())



def test_teleop_commit_gates_keyed_drive_and_disconnect_ends_teammate_play(tmp_path):
    client, lobby, robots, _, _ = rig(tmp_path)
    with client, enter(client, "blue") as blue:
        first = blue.receive_json()
        blue.send_json(dict(command="mode", ticket=first["ticket"], robot_id=ROBOT["blue"], mode="teleop"))
        def drive(epoch):
            while True:
                event = blue.receive_json()
                if event.get("event") == "controls" and event["mode"] == "teleop":
                    break
            blue.send_json(dict(command="drive", ticket=event["ticket"], epoch=epoch, robot_id=ROBOT["blue"],
                                forward=1, left=0, turn=0))
            while blue.receive_json().get("command") != "drive":
                pass
            return robots[ROBOT["blue"]].commands[-1]["forward"]
        assert drive("old") == 0
        client.post("/api/camera", json={"source":"0"}).raise_for_status()
        with enter(client, "red") as red:
            red.receive_json()
            state = client.post("/api/match/start").json()
            epoch = state["camera"]["match"]["epoch"]
            assert drive("old") == 0 and drive(epoch) == 1
            lobby.camera.playing = False
            assert drive(epoch) == 0
        assert lobby.phase == "idle" and not any(p.armed for p in robots.values())
        assert not robots[ROBOT["blue"]].closed and drive(epoch) == 0


def test_runner_profile_removes_the_kickoff_ssh_probe(tmp_path):
    client, lobby, *_ = rig(tmp_path)
    async def unnecessary(*_):
        pytest.fail("starting the match opened another SSH profile connection")
    lobby.describe = unnecessary
    with client, enter(client, "blue") as sock:
        sock.receive_json()
        peer = lobby.peers["pupper-1"]
        info = dict(event="profile", robot_id=peer.robot, robot_host="192.0.2.70", source_host="192.0.2.5",
                    port=5005, field_config=dict(field=lobby.profile["field"], attack_signs={"pupper-1": -1, "pupper-2": 1}))
        with pytest.raises(ValueError, match="identity"):
            peer.observe({**info, "robot_id": "pupper-2"})
        peer.observe(info)
        assert client.post("/api/camera", json={"source": "0"}).status_code == 200
        for _ in range(2):
            start = client.post("/api/match/start")
            assert start.status_code == 200, start.text
            assert client.post("/api/match/end").status_code == 200


@pytest.mark.parametrize("change", [
    {"length": 3.01, "width": 1.82},
    # A small round-off disagreement can exceed the Pi ingress's one micrometre tolerance.
    {"length": 2.748275, "width": 1.442886155351121},
])
def test_measured_pitch_reaches_each_runner_and_field_feed(tmp_path, change):
    from contextlib import ExitStack
    client, lobby, robots, _, _ = rig(tmp_path)
    pitches = []
    orig = lobby.field
    async def open_field(host, robot, info, *args):
        pitches.append(info["pitch"])
        return await orig(host, robot, info, *args)
    lobby.field = open_field
    with client, ExitStack() as stack:
        for slot in ROBOT:
            stack.enter_context(enter(client,slot)).receive_json()
        assert client.post("/api/camera",json={"source":"0"}).status_code == 200
        expected = change
        snap = lobby.camera.snapshot
        def measured():
            result = snap()
            result["frame"]["field"] = dict(lobby.profile["field"], **expected)
            return result
        lobby.camera.snapshot = measured
        reply = client.post("/api/match/start")
        assert reply.status_code == 200, reply.text
        assert pitches == [expected, expected]
        assert all([m["pitch"] for m in r.commands if m["command"] == "arm"] == [expected]
                   for r in robots.values())
        assert client.post("/api/match/end").status_code == 200


def test_surveyed_posts_reach_match_while_base_map_still_passes_pi_ingress():
    pytest.importorskip("cv2")
    from final_project.behaviors.overhead_ingress import IngressCore
    from final_project.pupper_fc.arena_camera import ArenaCamera
    from final_project.pupper_fc.lobby import PROFILE
    from final_project.pupper_fc.match_director import MatchDirector

    camera = ArenaCamera(json.loads(PROFILE.read_text()), "sim")
    camera.start()
    try:
        deadline = time.monotonic() + 4
        while not camera.snapshot()["frame"] and time.monotonic() < deadline:
            time.sleep(.02)
        snap = camera.snapshot()
        field = snap["frame"]["field"]
        # Keep measured posts displaced and mouths laterally offset.
        lines = {name: dict(x=sign * (field["length"] / 2 - .22), y_min=-.22, y_max=.44)
                 for name, sign in (("west", -1), ("east", 1))}
        camera.set_goals(lines)
        director = MatchDirector("pitch-test", {"striker": "pupper-1"}, {"pupper-1": 1, "pupper-2": -1},
                                 goal_lines=lines, pending_start=True)
        camera.set_match(director)
        while not camera.snapshot()["match"] and time.monotonic() < deadline:
            time.sleep(.02)
        frame = camera.snapshot()["frame"]
        assert frame["fc_match"]["goal_lines"] == lines and frame["goals"] != lines
        ingress = IngressCore(source_id="camera", source_session=camera.session, producer_id="test",
                              epoch="pitch-test", boot_id="test", robot_map={"striker": "pupper-1"},
                              calibration_revision=frame["calibration_revision"], field=field)
        assert ingress._geometry(frame)["right"] == {"x": field["length"] / 2, "y": 0.}
    finally:
        camera.close()
