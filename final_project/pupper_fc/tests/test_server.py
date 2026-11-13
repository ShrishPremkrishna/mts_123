import json

from fastapi.testclient import TestClient

from final_project.pupper_fc.runtime import Sandbox
from final_project.pupper_fc.server import create_app


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
