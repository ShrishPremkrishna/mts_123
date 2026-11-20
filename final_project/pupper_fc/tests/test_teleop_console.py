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
