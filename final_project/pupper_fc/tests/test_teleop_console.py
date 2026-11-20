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
    seen = []
    async def spawn(*args, **kwargs):
        seen.append(args)
    monkeypatch.setattr(asyncio,"create_subprocess_exec",spawn)
    for action, home_only in [("prepare",False),("home",True),("home",False)]:
        asyncio.run(open_setup("blue-pi", "pupper-1", action, home_only))
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
