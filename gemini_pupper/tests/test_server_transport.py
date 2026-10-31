"""In-process HTTP/SDK boundary tests: fake ROS, no sockets, camera or credentials.

Run with FastAPI/httpx/google-genai installed. Missing optional transport packages
skip this module; they are not counted as passing transport validation. The test
imports the course fake ROS harness BEFORE server/robot_api, never runs uvicorn,
and uses httpx.ASGITransport rather than a listening server.
"""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager, suppress
import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import sys
import types

import pytest

pytest.importorskip("fastapi")
httpx = pytest.importorskip("httpx")
pytest.importorskip("uvicorn")
pytest.importorskip("psutil")
pytest.importorskip("numpy")
pytest.importorskip("yaml")
genai_types = pytest.importorskip("google.genai.types")
genai = pytest.importorskip("google.genai")

ROBOT_DIR = Path(__file__).resolve().parents[1] / "robot"
sys.path.insert(0, str(ROBOT_DIR))
import test_tools as harness  # installs fake ROS before importing robot_api
import robot_api
import tool_registry


def forbidden(*args, **kwargs):
    raise AssertionError("transport smoke must not open sockets, run commands or access a camera")


@pytest.fixture
def server(monkeypatch):
    # Fail before Robot construction if another importer cached real ROS bindings.
    assert robot_api.Node is harness._Node
    assert robot_api.SingleThreadedExecutor is harness._Executor
    assert robot_api.rclpy is sys.modules["rclpy"]
    monkeypatch.setattr(genai, "Client", forbidden)
    # Patch outbound/bound network and subprocess entry points even on a Pi.
    # asyncio's local socketpair remains usable; no TCP server or client is used.
    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(socket.socket, "connect", forbidden)
    monkeypatch.setattr(socket.socket, "connect_ex", forbidden)
    monkeypatch.setattr(socket.socket, "bind", forbidden)
    monkeypatch.setattr(subprocess, "run", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    for name in ("GOOGLE_API_KEY", "GEMINI_API_KEY"):
        monkeypatch.delenv(name, raising=False)
    previous_fisheye = sys.modules.get("fisheye_converter")
    camera = types.ModuleType("cv2")
    camera.__getattr__ = forbidden
    monkeypatch.setitem(sys.modules, "cv2", camera)
    name = "pupper_server_transport_under_test"
    spec = importlib.util.spec_from_file_location(name, ROBOT_DIR / "server.py")
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    # ASGITransport does not enter lifespan automatically. Accidental entry must
    # fail before constructing Robot; the dedicated lifecycle test installs its
    # own explicit fake constructor instead.
    monkeypatch.setattr(module, "Robot", forbidden)
    monkeypatch.setattr(module.detections, "run", forbidden)
    yield module
    assert module.tools is None
    if previous_fisheye is None:
        sys.modules.pop("fisheye_converter", None)


def epoch_hook(server):
    async def attach(request):
        # Test clients issue a fresh intent for each ordinary call. Explicit stale
        # headers and the dedicated transport-delay tests remain unchanged.
        if server.tools is not None and "X-Pupper-Command-Epoch" not in request.headers:
            request.headers["X-Pupper-Command-Epoch"] = server.tools.command_epoch
    return attach


@asynccontextmanager
async def client_with_rig(server, *, epochs=True):
    previous = {name: getattr(robot_api, name) for name in ("time", "asyncio", "load_tricks")}
    rig = harness.Rig()
    server.tools = rig.ts
    try:
        assert await rig.settle()
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),
                                     base_url="http://in-process.invalid",
                                     event_hooks={"request": [epoch_hook(server)]} if epochs else {}) as client:
            yield client, rig
    finally:
        server.tools = None
        rig.close()
        with suppress(asyncio.CancelledError):
            await rig.ts.worker
        for name, value in previous.items():
            setattr(robot_api, name, value)


def test_http_declarations_and_tool_response_round_trip_through_sdk_types(server):
    async def case():
        async with client_with_rig(server) as (client, rig):
            response = await client.get("/tools")
            assert response.status_code == 200
            document = response.json()
            assert document["prompt"] and "<!--" not in document["prompt"]
            # The real installed SDK validates the exact schema obtained over HTTP.
            # No genai.Client is created and no API key is read or supplied.
            config = genai_types.LiveConnectConfig.model_validate({
                "systemInstruction": document["prompt"],
                "tools": [{"functionDeclarations": document["tools"]}],
            })
            declarations = config.tools[0].function_declarations
            assert any(d.name == "move" for d in declarations)
            call = genai_types.FunctionCall(id="transport-call-1", name="move", args={
                "direction": "left", "speed": 0.2, "duration": 0.1,
            })
            wire_call = json.loads(call.model_dump_json(by_alias=True, exclude_none=True))
            reply = await client.post(f"/tools/{wire_call['name']}", json=wire_call["args"])
            payload = reply.json()
            assert reply.status_code == 200
            assert set(payload) == {"success", "message", "details"}
            assert payload["details"] == {"submission": {"schema": "pupper_submission_v1", "state": "queued"}}
            assert payload["success"] is True and isinstance(payload["message"], str)
            await rig.settle()
            moving = harness.walk_twists(rig)
            assert moving and all(message.linear.y == pytest.approx(0.2) for message in moving)
            # Browser geminiLive.ts wraps the robot payload in response.result and
            # retains the original function-call id/name. Check that SDK boundary.
            envelope = genai_types.LiveClientToolResponse(function_responses=[
                genai_types.FunctionResponse(id=call.id, name=call.name, response={"result": payload})
            ])
            wire = json.loads(envelope.model_dump_json(by_alias=True, exclude_none=True))
            echoed = wire["functionResponses"][0]
            assert echoed["id"] == call.id and echoed["name"] == "move"
            assert echoed["response"]["result"] == payload
    asyncio.run(case())


@pytest.mark.parametrize("name,args,reason", [
    ("does_not_exist", {}, "unknown tool"),
    ("move", {}, "needs direction"),
    ("move", {"direction": "forward", "speed": True}, "must be a number"),
    ("move", {"direction": "forward", "unknown": 1}, "has no parameter"),
    ("move", {"direction": "up"}, "must be one of"),
])
def test_http_semantic_refusals_preserve_result_contract_without_motion(server, name, args, reason):
    async def case():
        async with client_with_rig(server) as (client, rig):
            switches = list(rig.robot.switches)
            response = await client.post(f"/tools/{name}", json=args)
            assert response.status_code == 200
            assert response.json()["success"] is False
            assert reason in response.json()["message"]
            assert await rig.settle()
            assert rig.robot.switches == switches and not harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize("endpoint,body", [
    ("/tools/move", "[1,2]"),
    ("/tools/move", '"forward"'),
    ("/tools/move", '{"direction":'),
    ("/move/velocity", '{"forward_backward_velocity":"invalid"}'),
])
def test_fastapi_rejects_malformed_transport_before_dispatch(server, monkeypatch, endpoint, body):
    async def case():
        async with client_with_rig(server) as (client, rig):
            monkeypatch.setattr(server.gemini_tools, "call", forbidden)
            monkeypatch.setattr(server.gemini_tools, "dispatch", forbidden)
            response = await client.post(endpoint, content=body, headers={"Content-Type": "application/json"})
            assert response.status_code == 422
            assert isinstance(response.json()["detail"], list)
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_direct_browser_velocity_payload_keeps_units_and_ros_signs(server):
    async def case():
        async with client_with_rig(server) as (client, rig):
            response = await client.post("/move/velocity", json={
                "forward_backward_velocity": 0.2, "right_left_velocity": 0.1,
                "turning_velocity": 10.0, "duration": 0.1,
            })
            assert response.status_code == 200 and response.json()["success"] is True
            await rig.settle()
            twists = harness.walk_twists(rig)
            assert twists
            for message in twists:
                assert message.linear.x == pytest.approx(0.2)
                assert message.linear.y == pytest.approx(-0.1)
                assert message.angular.z == pytest.approx(-10.0 * harness.math.pi / 180.0)
    asyncio.run(case())


def test_http_not_initialized_is_503_and_health_does_not_claim_robot_ready(server):
    async def case():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),
                                     base_url="http://in-process.invalid") as client:
            assert (await client.get("/health")).json() == {"status": "running", "tools": False, "runtime": "robot", "hardware_connected": None, "command_epoch": None, "command_epoch_required": True}
            for endpoint in ("/tools", "/queue/status", "/limits"):
                response = await client.get(endpoint)
                assert response.status_code == 503
            response = await client.post("/tools/stop", json={})
            assert response.status_code == 503
    asyncio.run(case())


@pytest.mark.parametrize("field", ["startup_error", "controller_fault"])
def test_http_ready_object_with_fault_refuses_motion_but_stop_remains_available(server, field):
    async def case():
        async with client_with_rig(server) as (client, rig):
            setattr(rig.ts, field, "injected readiness fault")
            response = await client.post("/tools/move", json={"direction": "forward"})
            assert response.status_code == 200
            assert response.json() == {"success": False, "message": "injected readiness fault"}
            stop_generation = rig.ts.stops
            stopped = await client.post("/stop/immediate")
            assert rig.ts.stops == stop_generation + 1
            assert stopped.status_code == 200 and isinstance(stopped.json()["success"], bool)
            assert "message" in stopped.json()
            assert not harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize("gate", ["action_lock", "ready"])
def test_http_stop_bypasses_wait_gate_and_cancels_waiting_motion(server, gate):
    async def case():
        async with client_with_rig(server) as (client, rig):
            if gate == "action_lock":
                await rig.ts.action_lock.acquire()
            else:
                rig.ts.ready.clear()
            pending = asyncio.create_task(client.post("/tools/move", json={"direction": "forward"}))
            try:
                for _ in range(5):
                    await harness._real_sleep(0)
                assert not pending.done()
                stopped = await asyncio.wait_for(client.post("/queue/stop"), timeout=1.0)
                assert stopped.status_code == 200 and stopped.json()["success"] is True
            finally:
                if gate == "action_lock":
                    rig.ts.action_lock.release()
                else:
                    rig.ts.ready.set()
            response = await asyncio.wait_for(pending, timeout=1.0)
            assert response.status_code == 200 and response.json()["success"] is False
            assert "cancelled by a stop" in response.json()["message"]
            assert await rig.settle()
            assert not harness.walk_twists(rig)
    asyncio.run(case())


def test_tool_exception_stays_a_serializable_failure_response(server, monkeypatch):
    async def failure(robot):
        raise RuntimeError("injected tool failure")
    async def case():
        async with client_with_rig(server) as (client, rig):
            monkeypatch.setattr(tool_registry.TOOLS["robot_status"], "run", failure)
            response = await client.post("/tools/robot_status", json={})
            assert response.status_code == 200
            assert response.json() == {"success": False, "message": "robot_status failed on the robot: RuntimeError: injected tool failure"}
            assert not harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize("shutdown_ok", [True, False])
def test_lifespan_uses_explicit_fake_and_shuts_down_without_hardware(server, monkeypatch, shutdown_ok):
    async def case():
        state = {"constructed": 0, "shutdown": 0, "detection_cancelled": False}
        detector_started = asyncio.Event()
        class FakeLifecycleRobot:
            def __init__(self):
                state["constructed"] += 1
            async def shutdown(self):
                state["shutdown"] += 1
                return robot_api.Result(shutdown_ok, "fake shutdown outcome")
        async def fake_detections():
            detector_started.set()
            try:
                await asyncio.Event().wait()
            finally:
                state["detection_cancelled"] = True
        monkeypatch.setattr(server, "Robot", FakeLifecycleRobot)
        monkeypatch.setattr(server.detections, "run", fake_detections)
        monkeypatch.setattr(server, "follower_config", lambda: types.SimpleNamespace(mode="disabled"))
        async with server.app.router.lifespan_context(server.app):
            await detector_started.wait()
            assert isinstance(server.tools, FakeLifecycleRobot)
            assert server.tools.follow_available is False
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),
                                         base_url="http://in-process.invalid") as client:
                assert (await client.get("/health")).json() == {"status": "running", "tools": True, "runtime": "robot", "hardware_connected": None, "command_epoch": None, "command_epoch_required": True}
        assert server.tools is None
        assert state == {"constructed": 1, "shutdown": 1, "detection_cancelled": True}
    asyncio.run(case())


@pytest.mark.parametrize('condition', ['latched', 'fault', 'stale', 'empty_before_status'])
def test_browser_and_gemini_transport_obey_shared_stop_while_stop_stays_available(server, condition):
    async def case():
        async with client_with_rig(server) as (client, rig):
            if condition == 'stale':
                rig.robot.stop_status_enabled = False
                rig.clock.now += .501
            elif condition == 'empty_before_status':
                rig.robot.send('/emergency_stop', harness._Msg())
            else:
                rig.robot.stop_status.update(epoch=2, latched=True, stop_required=True,
                                             fault=condition == 'fault')
                rig.robot.emit_stop_status()
            before = list(rig.robot.switches)
            for endpoint, body in [('/tools/stand_up', {}), ('/move/velocity', {
                    'forward_backward_velocity': .2, 'right_left_velocity': 0.,
                    'turning_velocity': 0., 'duration': .1})]:
                response = await client.post(endpoint, json=body)
                assert response.status_code == 200 and response.json()['success'] is False
            stopped = await client.post('/stop/immediate')
            assert stopped.status_code == 200
            assert isinstance(stopped.json()['success'], bool)
            assert await rig.settle()
            assert rig.robot.switches == before
            assert not harness.walk_twists(rig)
    asyncio.run(case())


@pytest.mark.parametrize("output,returncode,expected", [
    ("Battery:\t0%\t16.49V\n", 0, (0., 16.49)),
    ("Battery:\t100%\t20.50V\n", 0, (100., 20.50)),
    ("Battery:\t50%\t18.00V\n", 1, (None, None)),
    ("Battery:\t101%\t18.00V\n", 0, (None, None)),
    ("malformed output\n", 0, (None, None)),
])
def test_system_battery_uses_one_validated_reading(server, monkeypatch, tmp_path, output, returncode, expected):
    script = tmp_path / "battery.py"
    script.write_text("# Never executed: subprocess is stubbed.\n")
    monkeypatch.setattr(server, "BATTERY_SCRIPT", script)
    calls = []
    def read(args, **kwargs):
        calls.append(args)
        # An independent second read could differ; it is not the same sample.
        return types.SimpleNamespace(returncode=returncode, stdout=output if len(calls) == 1 else "77\n")
    monkeypatch.setattr(server.subprocess, "run", read)
    monkeypatch.setattr(server.psutil, "sensors_temperatures", lambda: {}, raising=False)
    monkeypatch.setattr(server.psutil, "cpu_percent", lambda: 0.)
    monkeypatch.setattr(server.socket, "create_connection", lambda *a, **kw: types.SimpleNamespace(close=lambda: None))
    result = server.system_status()["battery"]
    assert (result["percentage"], result["voltage"]) == expected
    assert calls == [["python3", str(script)]]


def test_detection_task_failure_does_not_skip_robot_shutdown(server, monkeypatch):
    async def case():
        shutdowns = []
        class FakeRobot:
            async def shutdown(self):
                shutdowns.append(True)
                return robot_api.Result(True, "fake shutdown")
        async def broken_detections():
            raise RuntimeError("injected detector transport failure")
        monkeypatch.setattr(server, "Robot", FakeRobot)
        monkeypatch.setattr(server.detections, "run", broken_detections)
        monkeypatch.setattr(server, "follower_config", lambda: types.SimpleNamespace(mode="disabled"))
        try:
            async with server.app.router.lifespan_context(server.app):
                await asyncio.sleep(0)
        finally:
            # Preserve fixture cleanup even when the old implementation fails.
            server.tools = None
        assert shutdowns == [True]
    asyncio.run(case())


@pytest.mark.parametrize("scope", [None, "", "sit_stay_shake"])
def test_fake_lifespan_uses_real_routes_and_realtime_cancel_without_native_access(server, monkeypatch, scope):
    async def case():
        if scope is None:
            monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
        else:
            monkeypatch.setenv("PUPPER_SESSION_SKILLS", scope)
        # Fake mode honors the catalog selector but never consumes a physical
        # deployment manifest, including a stale path left in this environment.
        monkeypatch.setenv("PUPPER_SESSION_MANIFEST", "/nonexistent/fake-only-no-deployment.json")
        monkeypatch.setattr(server, "FAKE_MODE", True)
        # Fixture forbids subprocess, network, camera, native Robot construction
        # and native detections. Fake mode must not call any of those hooks.
        async with server.app.router.lifespan_context(server.app):
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),
                                         base_url="http://in-process.invalid",
                                         event_hooks={"request": [epoch_hook(server)]}) as client:
                health = (await client.get("/health")).json()
                assert health["runtime"] == "fake" and health["hardware_connected"] is False
                assert health["tools"] is True
                assert (await client.get("/tools")).json()["tools"]
                if scope is not None:
                    assert set((await client.get("/animations")).json()) == ({scope} if scope else set())
                system = (await client.get("/system/status")).json()
                assert system["runtime"] == "fake" and system["hardware_connected"] is False
                assert system["battery"]["voltage"] is None
                assert system["services"]["robot"] == "simulated"
                assert (await client.get("/camera/image")).status_code == 503
                assert not (await client.post("/system/volume", json={"volume": 50})).json()["success"]
                accepted = await client.post("/tools/wait", json={"duration": 10.})
                assert accepted.json()["success"] and "queued" in accepted.json()["message"]
                await asyncio.sleep(.03)
                before = (await client.get("/queue/status")).json()
                assert before["running"] and before["runtime"] == "fake"
                assert before["hardware_connected"] is False
                assert (await client.post("/stop/immediate")).json()["success"]
                await asyncio.sleep(.03)
                after = (await client.get("/queue/status")).json()
                assert after["queued"] == 0 and not after["running"]
                assert after["last_outcome"]["success"] is False
                assert after["last_outcome"]["message"] == "cancelled"
                assert not any(abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z)
                               for _, m in harness.ROBOT.twists())
        assert server.tools is None
    asyncio.run(case())


def test_invalid_follower_configuration_does_not_construct_robot(server, monkeypatch):
    async def case():
        constructed = []
        def construct():
            constructed.append(True)
            return types.SimpleNamespace()
        def bad_config():
            raise ValueError("injected invalid follower configuration")
        monkeypatch.setattr(server, "Robot", construct)
        monkeypatch.setattr(server, "follower_config", bad_config)
        try:
            with pytest.raises(ValueError, match="invalid follower configuration"):
                async with server.app.router.lifespan_context(server.app):
                    raise AssertionError("invalid startup unexpectedly reached serving state")
            assert constructed == [], "Robot resources were allocated before configuration validation"
            assert server.tools is None
        finally:
            server.tools = None
    asyncio.run(case())


@pytest.mark.parametrize("scoped", [False, True])
def test_http_limits_reports_current_runtime_envelope(server, monkeypatch, scoped):
    async def case():
        if scoped:
            monkeypatch.setenv("PUPPER_SESSION_SKILLS", "")
        else:
            monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
        async with client_with_rig(server) as (client, rig):
            response = await client.get("/limits")
            assert response.status_code == 200
            limits = response.json()
            if scoped:
                assert limits["vx_max"] == pytest.approx(.49)
                assert limits["vx_forward_max"] == pytest.approx(.49)
                assert limits["vx_backward_max"] == pytest.approx(.30)
                assert limits["vy_max"] == pytest.approx(.49)
            else:
                assert limits["vx_max"] == pytest.approx(1.5)
                assert limits["vx_forward_max"] is None
                assert limits["vx_backward_max"] is None
            assert not rig.robot.switches and not harness.walk_twists(rig)
    asyncio.run(case())
