import asyncio
import json
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest
from starlette.websockets import WebSocketDisconnect

from final_project.pupper_fc.teleop import create_app, operator_message


class Runner:
    def __init__(self, robot_id):
        self.id = robot_id
        self.writes = []
        self.closed = False
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(write=self.write, drain=self.drain, close=self.close)
        self.reply({"event": "controls", "ticket": 1, "robot_id": robot_id, "mode": "autonomous", "ready": True})

    def write(self, wire):
        message = json.loads(wire)
        self.writes.append(message)
        self.reply({"accepted": True, "robot_id": self.id, "echo": message["command"]})

    def reply(self, message):
        self.stdout.feed_data((json.dumps(message) + "\n").encode())

    async def drain(self):
        pass

    def close(self):
        self.closed = True
        self.stdout.feed_eof()

    async def wait(self):
        assert self.closed

    terminate = kill = close


def test_two_sessions_are_independent_and_stop_releases_only_its_owner():
    runners = {}

    async def connect(host, robot_id):
        runners[host] = Runner(robot_id)
        return runners[host]

    app = create_app({"blue": "left-host", "red": "right-host"}, connect)
    headers = {"origin": "http://localhost", "host": "localhost"}
    with TestClient(app, base_url="http://localhost") as client:
        assert client.get("/").status_code == 200
        assert client.get("/slots").json()["red"]["robot_id"] == "pupper-2"
        assert not runners
        with client.websocket_connect("/play/blue", headers=headers) as blue:
            assert blue.receive_json()["robot_id"] == "pupper-1"
            with pytest.raises(WebSocketDisconnect):
                with client.websocket_connect("/play/blue", headers=headers):
                    pass
            with client.websocket_connect("/play/red", headers=headers) as red:
                assert red.receive_json()["robot_id"] == "pupper-2"
                blue.send_json(dict(command="mode", ticket=1, robot_id="pupper-1", mode="teleop"))
                assert blue.receive_json()["echo"] == "mode"
                assert not runners["right-host"].writes
                red.send_json(dict(command="stop", robot_id="pupper-2"))
                assert red.receive_json()["echo"] == "stop"
            blue.send_json(dict(command="drive", epoch="closed", ticket=2, robot_id="pupper-1", forward=1, left=0, turn=0))
            assert blue.receive_json()["echo"] == "drive"
            assert runners["left-host"].writes[-1]["forward"] == 0
        assert all(peer.closed for peer in runners.values())
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/play/blue", headers={**headers, "origin": "https://example.com"}):
                pass


def test_keyboard_control_rejects_mismatches_and_unbounded_input():
    for patch in ({"robot_id": "pupper-2"}, {"ticket": True}, {"forward": float("nan")}, {"left": 1}):
        with pytest.raises(ValueError):
            operator_message(dict(command="drive", epoch="game", ticket=1, robot_id="pupper-1",
                                  forward=1, left=0, turn=0) | patch, "pupper-1")
    with pytest.raises(ValueError):
        create_app({"blue": "mts", "red": "mts"})


OWNER = "1e384ddf-4b3f-4aea-a762-876e33ca5400"
LOCAL = {"origin": "http://localhost", "host": "localhost"}
BLUE_SETUP = "/setup/blue?owner=" + OWNER


class SetupPi:
    """A process-shaped Pi fixture. Reads/writes run on the server event loop."""
    def __init__(self, robot, action, home_only, *, wrong_robot=False, delayed=False):
        self.robot = "pupper-2" if wrong_robot else robot
        self.action, self.home_only = action, home_only
        self.stdout = asyncio.StreamReader()
        self.writes = []
        self.stdin = SimpleNamespace(write=self.write, drain=self.drain, close=lambda: None)
        self.code = 0
        self.returncode = None
        if action == "home" and not home_only:
            prompt = b'{"event":"stand_prompt"}\r\n'
            if delayed:
                asyncio.get_running_loop().call_later(.1, self.stdout.feed_data, prompt)
            else:
                self.stdout.feed_data(prompt)
        else:
            self.finish()

    def finish(self):
        receipt = dict(robot_id=self.robot, stage=self.action, home_only=self.home_only,
                       match_armed=False, run="/selected/session")
        self.stdout.feed_data(json.dumps(receipt).encode() + b"\r\n")
        self.stdout.feed_eof()

    def write(self, line):
        self.writes.append(line)
        if line == b"\n":
            self.finish()
        else:
            self.code = 1
            self.stdout.feed_data(b"Pupper FC: standing cancelled\r\n")
            self.stdout.feed_eof()

    async def drain(self):
        pass

    async def wait(self):
        self.returncode = self.code
        return self.code


def test_dashboard_uses_pi_receipts_and_a_fresh_operator_standing_cue():
    launches = []
    async def launch(host, robot, action, home_only):
        assert (host, robot) == ("pi-blue", "pupper-1")
        proc = SetupPi(robot, action, home_only)
        launches.append(proc)
        return proc
    app = create_app({"blue": "pi-blue"}, setup=launch)
    with TestClient(app, base_url="http://localhost") as client:
        for request in ({"action":"prepare"}, {"action":"home","supported":True,"home_only":True}):
            with client.websocket_connect(BLUE_SETUP, headers=LOCAL) as ws:
                ws.send_json(request)
                assert ws.receive_json()["stage"] == "running"
                reply = ws.receive_json()
                assert reply["stage"] == "complete" and reply["run"] == "/selected/session"
                assert reply["home_only"] == request.get("home_only",False)
        with client.websocket_connect(BLUE_SETUP, headers=LOCAL) as ws:
            ws.send_json({"action":"home","supported":True})
            assert ws.receive_json()["stage"] == "running"
            cue = ws.receive_json()
            assert cue["stage"] == "stand_prompt" and not any(p.writes for p in launches)
            with pytest.raises(WebSocketDisconnect):
                with client.websocket_connect("/play/blue", headers=LOCAL):
                    pass
            result = client.post("/api/match/start", headers={**LOCAL, "x-pupper-fc":OWNER}, json={})
            assert result.status_code == 409 and "setup" in result.json()["detail"]
            ws.send_json({"action":"stand","cue":cue["cue"]})
            assert ws.receive_json()["stage"] == "complete"
        assert [p.writes for p in launches] == [[],[],[b"\n"]]


def test_dashboard_rejects_wrong_team_missing_support_and_untrusted_browser():
    launches = []
    async def launch(host, robot, action, home_only):
        launches.append(robot)
        return SetupPi(robot, action, home_only, wrong_robot=True)
    app = create_app({"blue":"pi-blue"}, setup=launch)
    with TestClient(app, base_url="http://localhost") as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect(BLUE_SETUP, headers={**LOCAL, "origin":"http://elsewhere"}):
                pass
        with client.websocket_connect(BLUE_SETUP, headers=LOCAL) as ws:
            ws.send_json({"action":"home"})
            assert ws.receive_json()["stage"] == "error"
        assert not launches
        with client.websocket_connect(BLUE_SETUP, headers=LOCAL) as ws:
            ws.send_json({"action":"prepare"})
            assert ws.receive_json()["stage"] == "running"
            reply = ws.receive_json()
            assert reply["stage"] == "error" and "confirming" in reply["message"]


def test_home_startup_fault_reaches_operator_before_cleanup_and_requires_fresh_request():
    import threading
    release = threading.Event()
    reason = "Home stopped before motor enable (stationary startup window timed out)."
    processes = []

    async def launch(host, robot, action, home_only):
        if action == "prepare":
            return SetupPi(robot, action, home_only)
        proc = SetupPi(robot, action, home_only)
        proc.stdout = asyncio.StreamReader()
        proc.stdout.feed_data((json.dumps({"event":"home_startup_fault", "message":reason}) + "\n").encode())
        async def cleanup():
            await asyncio.to_thread(release.wait, 4)
            # Impossible/stale success data must not open a standing cue or mark a failed run homed.
            proc.stdout.feed_data(b'{"event":"stand_prompt"}\n')
            proc.stdout.feed_data((reason + " Owned setup processes exited. Select Prepare.\n").encode())
            proc.finish()
        asyncio.create_task(cleanup())
        processes.append(proc)
        return proc

    app = create_app({"blue":"pi-blue"}, setup=launch)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect(BLUE_SETUP, headers=LOCAL) as ws:
            ws.send_json({"action":"home", "home_only":True, "supported":True})
            assert ws.receive_json()["stage"] == "running"
            fault = ws.receive_json()
            assert fault["stage"] == "home_startup_fault" and fault["message"] == reason
            release.set()
            done = ws.receive_json()
            assert done["stage"] == "error" and "Select Prepare" in done["message"]
        with client.websocket_connect(BLUE_SETUP, headers=LOCAL) as ws:
            ws.send_json({"action":"prepare"})
            assert ws.receive_json()["stage"] == "running"
            assert ws.receive_json()["stage"] == "complete"
        assert len(processes) == 1 and processes[0].writes == []


@pytest.mark.parametrize("leave", ["early disconnect", "stale cue", "disconnect at cue"])
def test_dashboard_abandoned_home_finishes_without_enabling_standing(leave):
    launches = []
    async def launch(host, robot, action, home_only):
        proc = SetupPi(robot, action, home_only, delayed=leave == "early disconnect")
        launches.append(proc)
        return proc
    with TestClient(create_app({"blue":"pi-blue"},setup=launch), base_url="http://localhost") as client:
        with client.websocket_connect(BLUE_SETUP,headers=LOCAL) as ws:
            ws.send_json({"action":"home","supported":True})
            assert ws.receive_json()["stage"] == "running"
            if leave != "early disconnect":
                assert ws.receive_json()["stage"] == "stand_prompt"
            if leave == "stale cue":
                ws.send_json({"action":"stand","cue":"stale"})
                assert ws.receive_json()["stage"] == "error"
    assert launches[0].writes == [b"cancel\n"]


def test_setup_requests_expected_ssh_command_and_a_terminal_only_for_standing(monkeypatch):
    from final_project.pupper_fc.setup_session import open_setup
    from unittest.mock import AsyncMock
    compatible = AsyncMock()
    monkeypatch.setattr("final_project.pupper_fc.setup_session.require_session", compatible)
    seen = []
    async def spawn(*args, **kwargs):
        seen.append(args)
    monkeypatch.setattr(asyncio,"create_subprocess_exec",spawn)
    for action, home_only in [("prepare",False),("home",True),("home",False)]:
        asyncio.run(open_setup("blue-pi", "pupper-1", action, home_only))
    assert compatible.await_count == 3
    assert [a[1] for a in seen] == ["-T","-T","-tt"]
    assert all("BatchMode=yes" in a and "StrictHostKeyChecking=yes" in a for a in seen)
    assert [a[-1].split()[2:] for a in seen] == [
        ["prepare","--robot-id","pupper-1","--dashboard"],
        ["home","--robot-id","pupper-1","--dashboard","--supported","--home-only"],
        ["home","--robot-id","pupper-1","--dashboard","--supported"]]


def test_setup_reaps_child_after_overlong_remote_output_and_releases_slot():
    import sys
    processes = []
    async def launch(*args):
        # A bad remote endpoint fills the pipe and remains alive awaiting EOF.
        child = await asyncio.create_subprocess_exec(
            sys.executable, "-c", "import sys;sys.stdout.write('x'*300000);sys.stdout.flush();sys.stdin.read()",
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        processes.append(child)
        return child
    app = create_app({"blue":"pi-blue"}, setup=launch)
    with TestClient(app,base_url="http://localhost") as client:
        for _ in range(2):
            with client.websocket_connect(BLUE_SETUP,headers=LOCAL) as ws:
                ws.send_json({"action":"prepare"})
                assert ws.receive_json()["stage"] == "running"
                assert ws.receive_json()["stage"] == "error"
            assert processes[-1].returncode == 0


def test_setup_reaps_unwritable_child_when_operator_leaves_before_standing_prompt():
    import sys
    processes = []
    async def launch(*args):
        program = ("import os,time; os.close(0); time.sleep(.06); "
                   "print('{\"event\":\"stand_prompt\"}',flush=True); time.sleep(.12)")
        child = await asyncio.create_subprocess_exec(
            sys.executable, "-c", program, stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        processes.append(child)
        return child
    with TestClient(create_app({"blue":"pi-blue"},setup=launch),base_url="http://localhost") as client:
        with client.websocket_connect(BLUE_SETUP,headers=LOCAL) as ws:
            ws.send_json({"action":"home","supported":True})
            assert ws.receive_json()["stage"] == "running"
    assert processes[0].returncode == 0


def test_saved_team_address_scopes_discovery_and_connect_to_owning_window():
    import uuid
    from final_project.pupper_fc.lobby import Lobby

    scans, connections = [], []
    async def connect(host, robot_id):
        connections.append((host, robot_id))
        return Runner(robot_id)
    def scan():
        scans.append(True)
        return {"pupper-1":["blue-pi.local"],"pupper-2":["red-pi.local"]}
    lobby = Lobby()
    owner, other = (str(uuid.uuid4()) for _ in range(2))
    local = {"origin":"http://localhost","host":"localhost"}
    headers = {**local,"x-pupper-fc":owner}
    with TestClient(create_app(lobby=lobby, scan=scan, connect=connect), base_url="http://localhost") as c:
        def choose(slot, host, **kw):
            return c.post("/api/discover", json={"slot":slot,"host":host,**kw}, headers=headers)
        assert choose("blue","-oCommand=x",save_address=True).status_code == 409
        assert choose("blue","mts",save_address=True).json()["host"] == "mts"
        assert not scans and not connections
        assert choose("red","pi@MTS.",save_address=True).status_code == 409
        assert c.post("/api/discover",json={"slot":"blue","host":"other"},headers={**headers,"origin":"http://other.invalid"}).status_code == 403
        with pytest.raises(WebSocketDisconnect), c.websocket_connect("/play/blue?owner="+other, headers=local):
            pass
        assert choose("blue","mts").is_success
        with c.websocket_connect("/play/blue?owner="+owner, headers=local) as ws:
            assert ws.receive_json()["robot_id"] == "pupper-1"
            assert choose("blue","other",save_address=True).status_code == 409
        assert connections == [("mts","pupper-1")] and not scans
        assert choose("blue","",save_address=True).is_success and not scans
        assert choose("blue","").json()["host"] == "blue-pi.local" and scans == [True]
        lobby.owner = other
        assert choose("red","red-ssh",save_address=True).status_code == 409
        lobby.owner, lobby.phase = None, "starting"
        assert choose("red","red-ssh",save_address=True).status_code == 409
        lobby.phase = "idle"



@pytest.mark.parametrize("action", ["connect", "prepare", "home"])
@pytest.mark.parametrize("failure", ["missing", "unsupported"])
def test_incompatible_pi_receives_only_a_read_only_profile_request(monkeypatch, action, failure):
    from final_project.pupper_fc.field_transport import SESSION
    from final_project.pupper_fc.setup_session import open_setup
    from final_project.pupper_fc.teleop import open_runner

    commands = []

    async def spawn(*argv, **kwargs):
        commands.append(argv[-1])
        profile = dict(robot_id="pupper-1", source_host="192.168.1.10", robot_host="192.168.1.11",
                       port=5005, field_config={"field": dict(length=2, width=1, ball_radius=.05, goal_width=.4)})
        async def communicate():
            if failure == "missing":
                return b"", f"python3: can't open file '{SESSION}': [Errno 2] No such file or directory".encode()
            return json.dumps(profile).encode(), b""
        return SimpleNamespace(returncode=int(failure == "missing"), communicate=communicate)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", spawn)
    with pytest.raises(ValueError, match="no Pupper FC session helper|field protocol is incompatible"):
        if action == "connect":
            asyncio.run(open_runner("blue", "pupper-1"))
        else:
            asyncio.run(open_setup("blue", "pupper-1", action, True))
    assert commands == [f"python3 {SESSION} describe --robot-id pupper-1"]


@pytest.mark.parametrize("identity", [{}, {"robot_id": "pupper-2"}])
def test_a_kick_receipt_without_this_teams_identity_ends_the_session(identity):
    """Runner kick receipts reach the referee only when they name this robot."""
    runners = []

    async def connect(host, robot_id):
        runners.append(Runner(robot_id))
        runners[-1].reply({"event": "kick", "state": "engaged", "attempt_id": "r1-p0-k1", "epoch": "e", **identity})
        return runners[-1]

    app = create_app({"blue": "left-host"}, connect)
    headers = {"origin": "http://localhost", "host": "localhost"}
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect("/play/blue", headers=headers) as blue:
            while (event := blue.receive_json()).get("event") != "error":
                pass
            assert "wrong robot identity" in event["reason"]
        assert runners[0].closed
