"""The laptop archive must run rehearsal without imports from a nearby checkout."""
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[3]


def test_extracted_laptop_runs_the_selected_robot(tmp_path):
    packager = runpy.run_path(str(ROOT / "pupper-fc-package-laptop"))
    archive = tmp_path / "laptop.zip"
    revision = "a" * 40
    # Wheel downloading is independent of source closure; use the verified test
    # interpreter while executing only the extracted application sources.
    packager["write_laptop"](archive, {}, revision)
    with zipfile.ZipFile(archive) as bundle:
        bundle.extractall(tmp_path / "unpacked")
    extracted = tmp_path / "unpacked/Pupper FC"
    script = r'''
import json, pathlib, sys, uuid
from fastapi.testclient import TestClient
from final_project.pupper_fc.teleop import create_app

root = pathlib.Path.cwd().resolve()
owner = str(uuid.uuid4())
headers = {"x-pupper-fc": owner, "origin": "http://localhost"}
app = create_app(mode="replay")
with TestClient(app, base_url="http://localhost", headers=headers) as client:
    release = client.get("/api/release").json()
    assert client.post("/api/discover", json={"slot": "blue"}).status_code == 200
    with client.websocket_connect("/play/blue?owner=" + owner,
                                  headers={"origin": "http://localhost", "host": "localhost"}) as socket:
        while True:
            event = socket.receive_json()
            if event.get("event") == "error":
                raise AssertionError(event)
            if event.get("event") == "controls" and event.get("ready"):
                assert event["synthetic"] is True
                break
    assert client.get("/").status_code == 200
for name, module in list(sys.modules.items()):
    if name.startswith(("final_project.", "gemini_pupper.", "foundation.", "follow_me_pupper.")):
        filename = getattr(module, "__file__", None)
        if filename:
            assert pathlib.Path(filename).resolve().is_relative_to(root), (name, filename)
assert "rclpy" not in sys.modules, "Synthetic ROS belongs only to the isolated robot child"
print(json.dumps({"release": release, "ready": True}))
'''
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    result = subprocess.run([sys.executable, "-B", "-c", script], cwd=extracted,
                            env=environment, text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout.splitlines()[-1])
    assert report["ready"] is True
    assert revision in json.dumps(report["release"])
