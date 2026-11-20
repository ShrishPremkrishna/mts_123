"""Exercise browser ownership and the byte-for-byte challenge relay at both WebSockets."""
import asyncio
import json
import time
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest
from starlette.websockets import WebSocketDisconnect

from final_project.pupper_fc.teleop import create_app
from final_project.pupper_fc.tests.test_teleop_console import LOCAL, OWNER, Runner


class OperatorPi:
    def __init__(self, robot):
        self.robot = robot
        self.stdout = asyncio.StreamReader()
        self.stdin = SimpleNamespace(write=self.write, drain=self.drain, close=self.close)
        self.writes = []
        self.standing = self.closed = False
        self.tick = asyncio.create_task(self.challenge())

    async def challenge(self):
        sequence = 0
        while not self.closed:
            sequence += 1
            self.emit(event="operator_challenge", challenge=f"native-instance 3 {sequence}")
            await asyncio.sleep(.03)

    def emit(self, **row):
        self.stdout.feed_data((json.dumps(dict(row, robot_id=self.robot)) + "\n").encode())

    def write(self, data):
        row = json.loads(data)
        self.writes.append(row)
        if row["action"] == "stand":
            self.standing = True
            self.emit(event="operator_standing")
        elif row["action"] == "stop":
            self.emit(event="operator_stop", confirmed=True, reason="browser Stop")
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

    async def connect(host, robot, owner):
        attaches.append((host, robot, owner))
        return Runner(robot)

    path = "/operator/blue?owner=" + OWNER
    play = "/play/blue?owner=" + OWNER + "&lease=1"
    app = create_app({"blue": "blue-pi"}, operator=operator, connect=connect)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect(play, headers=LOCAL) as runner:
            assert runner.receive_json()["event"] == "error"
        with client.websocket_connect(path, headers=LOCAL) as ws:
            with pytest.raises(WebSocketDisconnect), client.websocket_connect(path, headers=LOCAL):
                pass
            with pytest.raises(WebSocketDisconnect), client.websocket_connect(
                    path.replace(OWNER, "dfaba465-923b-49fa-93d4-feb1dc98f7ab"), headers=LOCAL):
                pass
            challenge = receive(ws, "operator_challenge")["challenge"]
            heartbeat = {"action": "heartbeat", "challenge": challenge}
            ws.send_json(heartbeat)
            assert receive(ws, "operator_status")["ready"]
            ws.send_json(heartbeat)  # echoed already
            ws.send_json({"action": "heartbeat", "challenge": "made-up"})
            with client.websocket_connect(play, headers=LOCAL) as runner:
                assert runner.receive_json()["event"] == "error"
            assert not attaches and processes[0].writes == [heartbeat]
            stand = {"action": "stand", "challenge": challenge, "supported": True}
            ws.send_json(stand)
            receive(ws, "operator_standing")
            ws.send_json(stand)  # a click may use its displayed challenge exactly once
            with client.websocket_connect(play, headers=LOCAL) as runner:
                assert receive(runner, "controls")["robot_id"] == "pupper-1"
                assert attaches == [("blue-pi", "pupper-1", OWNER)]
                ws.send_json({"action": "stop"})
                assert receive(ws, "operator_stop")["confirmed"]
            assert processes[0].writes == [heartbeat, stand, {"action": "stop"}]
        assert processes[0].closed


def test_no_laptop_generated_renewal_and_expired_or_unsupported_stand():
    processes = []

    async def launch(host, robot, owner):
        processes.append(OperatorPi(robot))
        return processes[-1]

    app = create_app({"blue": "blue-pi"}, operator=launch)
    with TestClient(app, base_url="http://localhost") as client:
        with client.websocket_connect("/operator/blue?owner=" + OWNER, headers=LOCAL) as ws:
            challenge = receive(ws, "operator_challenge")["challenge"]
            time.sleep(.82)  # the browser does no work
            assert not processes[0].writes
            ws.send_json({"action": "stand", "challenge": challenge, "supported": True})
            ws.send_json({"action": "stand", "challenge": challenge})
            assert "Invalid" in receive(ws, "error")["reason"]
        assert processes[0].closed and not processes[0].writes


def test_native_connection_commands_name_the_operator(monkeypatch):
    from final_project.pupper_fc.operator_link import open_operator
    from final_project.pupper_fc.teleop import open_runner

    calls = []

    async def spawn(*argv, **kwargs):
        calls.append(argv)

    monkeypatch.setattr(asyncio, "create_subprocess_exec", spawn)
    asyncio.run(open_operator("blue-pi", "pupper-1", OWNER))
    asyncio.run(open_runner("blue-pi", "pupper-1", OWNER))
    assert calls[0][-1].split()[2:] == ["operator", "--robot-id", "pupper-1", "--owner", OWNER.replace("-", "")]
    assert calls[1][-1].endswith("--operator-owner " + OWNER.replace("-", ""))
    assert all(argv[1] == "-T" and "BatchMode=yes" in argv for argv in calls)
