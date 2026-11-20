import json

from fastapi.testclient import TestClient

from final_project.pupper_fc.runtime import Sandbox
from final_project.pupper_fc.server import create_app


def test_default_practice_shows_one_paw_handoff_and_allows_speed_changes():
    sandbox = create_app().state.sandbox
    assert sandbox.kick_side == "left"
    sandbox.control({"action": "start"})
    for tick in range(250):
        if tick == 12:
            sandbox.control({"action": "configure", "settings": {"max_speed": .45}})
        sandbox.advance()
        if sandbox.decision.phase == "kick_complete":
            break
    state = sandbox.snapshot()
    assert state["decision"]["phase"] == "kick_complete"
    assert state["metrics"]["shots"] == 1 and state["metrics"]["goals"] == 0
    assert state["settings"]["max_speed"] == .45


def test_viewer_and_api_use_same_offline_session():
    sandbox = Sandbox()
    # No lifespan here: step explicitly instead of relying on wall-clock timing.
    client = TestClient(create_app(sandbox))
    assert client.get("/").status_code == 200
    assert client.get("/app.js").status_code == 200
    assert {item["id"] for item in client.get("/api/scenarios").json()} >= {"center", "wrong_side"}
    response = client.post("/api/control", json={"action": "step"})
    assert response.status_code == 200
    assert response.json()["time"] == .05
    assert response.json()["observation"]["captured_at"] == 0
    assert response.json()["execution"]["frame_captured_at"] == 0
    assert response.json()["execution"]["frame_seq"] == 1
    assert client.get("/api/state").json() == json.loads(json.dumps(sandbox.snapshot()))
    trace = client.get("/api/trace")
    assert trace.status_code == 200
    assert trace.json()["schema"] == "pupper_fc_trace_v1"


def test_invalid_request_is_actionable_and_does_not_move():
    sandbox = Sandbox()
    client = TestClient(create_app(sandbox))
    response = client.post("/api/control", json={"action": "configure", "settings": {"max_speed": 200}})
    assert response.status_code == 422
    assert "max_speed" in response.json()["detail"]
    assert sandbox.sim.time == 0
    assert not sandbox.running


def test_api_exposes_real_role_lease_and_external_hold_in_existing_trace():
    sandbox = Sandbox()
    client = TestClient(create_app(sandbox))
    client.post("/api/control", json={"action": "move_ball", "x": -1.2, "y": .6})
    state = client.post("/api/control", json={"action": "step"}).json()
    assert state["decision"]["role"] == "defense"
    assert state["execution"]["armed"] and state["execution"]["action"] == "auto"
    assert 0 < state["execution"]["lease_remaining_s"] < .5
    before = client.get("/api/state").json()
    assert client.get("/api/state").json() == before  # GET never renews the local producer
    held = client.post("/api/control", json={"action": "external_hold", "held": True}).json()
    assert held["execution"]["external_hold"]
    assert held["decision"]["phase"] == "external_hold"
    client.post("/api/control", json={"action": "configure", "settings": {"renew_intent": False}})
    for _ in range(12):
        client.post("/api/control", json={"action": "step"})
    client.post("/api/control", json={"action": "external_hold", "held": False})
    lost = client.post("/api/control", json={"action": "step"}).json()
    assert lost["decision"]["phase"] == "intent_lost"
    trace = client.get("/api/trace").json()
    assert trace["schema"] == "pupper_fc_trace_v1"
    assert trace["frames"][-1]["execution"] == lost["execution"]
    assert {"integration.py", "tactics.py", "obstacles.py"} <= set(trace["source_sha256"])


def test_api_resume_refusal_keeps_goal_ball_and_current_session():
    sandbox = Sandbox()
    client = TestClient(create_app(sandbox))
    client.post("/api/control", json={"action": "start"})
    for _ in range(800):
        sandbox.advance()
        if sandbox.snapshot()["execution"]["resume_available"]:
            break
    else:
        raise AssertionError("actual sandbox did not score and return")
    client.post("/api/control", json={"action": "pause"})
    response = client.post("/api/control", json={"action": "move_robot", "x": 0, "y": .7, "yaw": 1.57})
    assert not response.json()["execution"]["resume_available"]
    before = client.get("/api/state").json()
    refusal = client.post("/api/control", json={"action": "resume"})
    assert refusal.status_code == 422 and "home position and heading" in refusal.json()["detail"]
    assert client.get("/api/state").json() == before
