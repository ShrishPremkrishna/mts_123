"""Browser ownership, the challenge relay and leased match controls at both WebSockets."""
import asyncio
import json
import time
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest
from starlette.websockets import WebSocketDisconnect

from final_project.pupper_fc import operator_link
from final_project.pupper_fc.operator_link import CHALLENGE_WINDOW_S, OperatorLink, browser_input
from final_project.pupper_fc.teleop import create_app
from final_project.pupper_fc.tests.test_teleop_console import LOCAL, OWNER, Runner


class OperatorPi:
    """The selected ``fc_session.py operator`` stream: challenges, status, standing, Stop."""

    def __init__(self, robot, period=.03):
        self.robot = robot
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(write=self.write, drain=self.drain, close=self.close)
        self.writes = []
        self.standing = self.closed = False
        self.tick = asyncio.create_task(self.challenge(period))

    async def challenge(self, period):
        sequence = 0
        while not self.closed:
            sequence += 1
            self.emit(event="operator_challenge", challenge=f"native-instance 3 {sequence}")
            await asyncio.sleep(period)

    def emit(self, **row):
        self.stdout.feed_data((json.dumps(dict(row, robot_id=self.robot)) + "\n").encode())

    def write(self, data):
        row = json.loads(data)
        self.writes.append(row)
        if row["action"] == "stand":
            self.standing = True
            self.emit(event="operator_standing")
        elif row["action"] == "stop":
            self.emit(event="operator_stop", confirmed=True, reason="Operator pressed Stop")
        else:
            self.emit(event="operator_status", ready=True, standing=self.standing)

    async def drain(self):
        pass

    def close(self):
        if not self.closed:
            self.closed = True
            self.tick.cancel()
            self.stdout.feed_eof()

    async def wait(self):
        assert self.closed

    terminate = kill = close


def receive(ws, kind):
    while (row := ws.receive_json())["event"] != kind:
        pass
    return row


def test_lease_stand_runner_transfer_and_terminal_stop():
    processes, attaches = [], []

    async def operator(host, robot, owner):
        assert (host, robot, owner) == ("blue-pi", "pupper-1", OWNER)
        processes.append(OperatorPi(robot))
        return processes[-1]

    async def connect(host, robot, owner=None):
        attaches.append((host, robot, owner))
        return Runner(robot)

    path = "/operator/blue?owner=" + OWNER
    play = "/play/blue?owner=" + OWNER + "&lease=1"
    app = create_app({"blue": "blue-pi"}, operator=operator, connect=connect)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect(play, headers=LOCAL) as runner:
            assert "standing" in runner.receive_json()["reason"]
        with client.websocket_connect(path, headers=LOCAL) as ws:
            # One operator per robot, and only the owning window.
            with pytest.raises(WebSocketDisconnect), client.websocket_connect(path, headers=LOCAL):
                pass
            with pytest.raises(WebSocketDisconnect), client.websocket_connect(
                    path.replace(OWNER, "dfaba465-923b-49fa-93d4-feb1dc98f7ab"), headers=LOCAL):
                pass
            # Setup, discovery changes and unleased match controls wait for the operator.
            with pytest.raises(WebSocketDisconnect), client.websocket_connect("/setup/blue?owner=" + OWNER, headers=LOCAL):
                pass
            with client.websocket_connect("/play/blue?owner=" + OWNER, headers=LOCAL) as runner:
                assert "Finish supported standing" in runner.receive_json()["reason"]
            challenge = receive(ws, "operator_challenge")["challenge"]
            heartbeat = {"action": "heartbeat", "challenge": challenge}
            ws.send_json(heartbeat)
            assert receive(ws, "operator_status")["ready"]
            ws.send_json(heartbeat)  # already echoed
            ws.send_json({"action": "heartbeat", "challenge": "made-up"})
            with client.websocket_connect(play, headers=LOCAL) as runner:
                assert runner.receive_json()["event"] == "error"
            assert not attaches and processes[0].writes == [heartbeat]
            state = client.get("/api/state").json()
            assert state["operators"]["blue"]["ready"] and not state["operators"]["blue"]["standing"]
            stand = {"action": "stand", "challenge": challenge, "supported": True}
            ws.send_json(stand)
            receive(ws, "operator_standing")
            ws.send_json(stand)  # a click may use its displayed challenge exactly once
            with client.websocket_connect(play, headers=LOCAL) as runner:
                assert receive(runner, "controls")["robot_id"] == "pupper-1"
                assert attaches == [("blue-pi", "pupper-1", OWNER)]
                team = client.get("/api/state").json()["teams"]["pupper-1"]
                assert team["operator"]["standing"] and team["mode"] == "autonomous"
                ws.send_json({"action": "stop"})
                stop = receive(ws, "operator_stop")
                assert stop["confirmed"] and stop["reason"] == "Operator pressed Stop"
            assert processes[0].writes == [heartbeat, stand, {"action": "stop"}]
        assert processes[0].closed
        assert client.get("/api/state").json()["operators"] == {}


def test_no_laptop_generated_renewal_and_expired_or_unsupported_stand():
    processes = []

    async def launch(host, robot, owner):
        processes.append(OperatorPi(robot, period=5))
        return processes[-1]

    app = create_app({"blue": "blue-pi"}, operator=launch)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect("/operator/blue?owner=" + OWNER, headers=LOCAL) as ws:
            challenge = receive(ws, "operator_challenge")["challenge"]
            time.sleep(CHALLENGE_WINDOW_S + .1)  # the browser does no work; the laptop renews nothing
            assert not processes[0].writes
            ws.send_json({"action": "stand", "challenge": challenge, "supported": True})  # expired: dropped
            ws.send_json({"action": "stand", "challenge": challenge})  # unsupported: ends the relay
            assert "Invalid" in receive(ws, "error")["reason"]
        assert processes[0].closed and not processes[0].writes


def test_losing_the_operator_stops_its_match_controls():
    runners = []

    async def operator(host, robot, owner):
        return OperatorPi(robot)

    async def connect(host, robot, owner=None):
        runners.append(Runner(robot))
        return runners[-1]

    app = create_app({"blue": "blue-pi"}, operator=operator, connect=connect)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect("/play/blue?owner=" + OWNER + "&lease=1", headers=LOCAL) as runner:
            pass  # refused before any operator exists
        assert not runners
        ws = client.websocket_connect("/operator/blue?owner=" + OWNER, headers=LOCAL).__enter__()
        challenge = receive(ws, "operator_challenge")["challenge"]
        ws.send_json({"action": "stand", "challenge": challenge, "supported": True})
        receive(ws, "operator_standing")
        with client.websocket_connect("/play/blue?owner=" + OWNER + "&lease=1", headers=LOCAL) as runner:
            receive(runner, "controls")
            ws.__exit__(None, None, None)  # the browser's Stop link closes
            deadline = time.monotonic() + 2
            while {"command": "stop"} not in runners[0].writes:
                assert time.monotonic() < deadline, runners[0].writes
                time.sleep(.02)


def test_replay_has_no_operator_or_lease():
    from final_project.pupper_fc.replay_transport import replay_lobby
    import tempfile
    from pathlib import Path
    with tempfile.TemporaryDirectory() as scratch:
        app = create_app(mode="replay", lobby=replay_lobby(Path(scratch) / "r.json"),
                         identity=dict(kind="git", commit="0" * 40, dirty=False, detail="t"))
        headers = {"x-pupper-fc": OWNER, "origin": "http://localhost"}
        with TestClient(app, base_url="http://localhost", headers=headers) as client:
            client.post("/api/discover", json={"slot": "blue"})
            with client.websocket_connect("/operator/blue?owner=" + OWNER, headers=LOCAL) as ws:
                assert "no Pi Stop gateway" in ws.receive_json()["reason"]
            with client.websocket_connect("/play/blue?owner=" + OWNER + "&lease=1", headers=LOCAL) as ws:
                assert "standing" in ws.receive_json()["reason"]
    with pytest.raises(ValueError, match="synthetic transport"):
        async def fake(*_):
            pass
        create_app(mode="replay", operator=fake)


def test_browser_input_and_liveness_are_strict():
    assert browser_input({"action": "stop"}) == "stop"
    assert browser_input({"action": "heartbeat", "challenge": "a 1 2"}) == "heartbeat"
    assert browser_input({"action": "stand", "challenge": "a 1 2", "supported": True}) == "stand"
    for bad in ({"action": "stand", "challenge": "a 1 2", "supported": 1}, {"action": "stand", "challenge": "a 1 2"},
                {"action": "heartbeat", "challenge": ""}, {"action": "heartbeat", "challenge": "x" * 257},
                {"action": "heartbeat", "challenge": "a", "extra": 1}, {"action": "recover-imu"}, ["stop"]):
        with pytest.raises(ValueError):
            browser_input(bad)
    now = [0.]
    link = OperatorLink(OWNER, "pupper-1", clock=lambda: now[0])
    link.standing, link.last_echo = True, 0.
    assert link.current()
    now[0] = operator_link.ECHO_LIVENESS_S
    assert not link.current()
    link.last_echo, link.stopping = now[0], True
    assert not link.current()


def test_native_connection_commands_name_the_operator(monkeypatch):
    from final_project.pupper_fc.operator_link import open_operator
    from final_project.pupper_fc import teleop

    calls = []

    async def spawn(*argv, **kwargs):
        calls.append(argv)

    async def enrolled(host, robot):
        return None

    monkeypatch.setattr(asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(teleop, "require_session", enrolled)
    asyncio.run(open_operator("blue-pi", "pupper-1", OWNER))
    asyncio.run(teleop.open_runner("blue-pi", "pupper-1", OWNER))
    asyncio.run(teleop.open_runner("blue-pi", "pupper-1"))
    assert calls[0][-1].split()[2:] == ["operator", "--robot-id", "pupper-1", "--owner", OWNER.replace("-", "")]
    assert calls[1][-1].endswith("--operator-owner " + OWNER.replace("-", ""))
    assert "--operator-owner" not in calls[2][-1]
    assert all(argv[1] == "-T" and "BatchMode=yes" in argv for argv in calls)


def test_setup_receipt_carries_operator_control_and_home_fields():
    class Home:
        def __init__(self):
            self.stdout = asyncio.StreamReader()
            self.stdin = SimpleNamespace(write=lambda data: None, drain=self.drain, close=lambda: None)
            self.returncode = None
            receipt = {"run": "/run/fc", "robot_id": "pupper-1", "stage": "home", "match_armed": False,
                       "home_only": True, "fc_operator_control": True, "homed": True, "imu_recovery": "strict"}
            self.stdout.feed_data((json.dumps(receipt) + "\n").encode())
            self.stdout.feed_eof()

        async def drain(self):
            pass

        async def wait(self):
            self.returncode = 0
            return 0

    async def launch(host, robot, action, home_only):
        assert (action, home_only) == ("home", True)
        return Home()

    sent = []

    class Socket:
        def __init__(self):
            self.inbox = [{"action": "home", "supported": True, "home_only": True}]

        async def receive_json(self):
            if self.inbox:
                return self.inbox.pop(0)
            await asyncio.sleep(10)

        async def send_json(self, row):
            sent.append(row)

    from final_project.pupper_fc.setup_session import run_setup
    asyncio.run(run_setup(Socket(), "blue-pi", "pupper-1", launch))
    complete = sent[-1]
    assert complete["stage"] == "complete" and complete["fc_operator_control"] is True and complete["homed"] is True
    assert complete["match_capable"] is True and "posture" not in complete["message"]
    assert complete["message"].startswith("Pi reports")


@pytest.mark.parametrize("row,reason", [
    ({"event": "operator_challenge", "challenge": "native-instance 3 1"}, "did not identify this team"),
    ({"event": "operator_standing", "robot_id": "pupper-2"}, "belongs to a different team"),
    ({"event": "operator_status", "robot_id": "pupper-1", "ready": "yes", "standing": False}, "Invalid Pi operator status"),
    ({"event": "operator_stop", "robot_id": "pupper-1", "reason": "Operator pressed Stop"}, "without a valid acknowledgement"),
])
def test_operator_events_fail_closed_without_their_strict_identity_and_fields(row, reason):
    """Every selected fc_operator event names its robot; a missing or wrong identity ends the relay."""
    processes = []

    async def operator(host, robot, owner):
        processes.append(OperatorPi(robot, period=5))
        processes[-1].stdout.feed_data((json.dumps(row) + "\n").encode())
        return processes[-1]

    app = create_app({"blue": "blue-pi"}, operator=operator)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect("/operator/blue?owner=" + OWNER, headers=LOCAL) as ws:
            event = receive(ws, "error")
            assert reason in event["reason"]
        assert processes[0].closed and client.get("/api/state").json()["operators"] == {}


def test_only_selected_operator_events_are_relayed():
    """The Pi reports failures as diagnostic text before EOF; an unselected 'error' row is never relayed."""
    async def operator(host, robot, owner):
        pi = OperatorPi(robot, period=5)
        pi.emit(event="error", reason="not part of the selected stream")
        pi.emit(event="operator_status", ready=True, standing=False)
        return pi

    app = create_app({"blue": "blue-pi"}, operator=operator)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect("/operator/blue?owner=" + OWNER, headers=LOCAL) as ws:
            first = ws.receive_json()
            while first["event"] == "operator_challenge":
                first = ws.receive_json()
            assert first["event"] == "operator_status" and first["robot_id"] == "pupper-1"
            assert client.get("/api/state").json()["operators"]["blue"]["ready"] is True


def test_setup_receipt_reports_a_previous_session_fault():
    class Prepare:
        def __init__(self):
            self.stdout = asyncio.StreamReader()
            self.stdin = SimpleNamespace(write=lambda data: None, drain=self.drain, close=lambda: None)
            self.returncode = None
            receipt = {"run": "/run/fc2", "robot_id": "pupper-1", "stage": "prepare", "match_armed": False,
                       "home_only": False, "fc_operator_control": True, "homed": False, "imu_recovery": "strict",
                       "previous_fault": "Board deactivated after a primary IMU fault"}
            self.stdout.feed_data((json.dumps(receipt) + "\n").encode())
            self.stdout.feed_eof()

        async def drain(self):
            pass

        async def wait(self):
            self.returncode = 0
            return 0

    async def launch(host, robot, action, home_only):
        return Prepare()

    sent = []

    class Socket:
        def __init__(self):
            self.inbox = [{"action": "prepare"}]

        async def receive_json(self):
            if self.inbox:
                return self.inbox.pop(0)
            await asyncio.sleep(10)

        async def send_json(self, row):
            sent.append(row)

    from final_project.pupper_fc.setup_session import run_setup
    asyncio.run(run_setup(Socket(), "blue-pi", "pupper-1", launch))
    complete = sent[-1]
    assert complete["stage"] == "complete" and complete["previous_fault"] == "Board deactivated after a primary IMU fault"
    assert "previous session ended: Board deactivated" in complete["message"]
