"""First-party HTTP client ordering with stub transport; never opens a socket."""
import asyncio
import io
import json
from pathlib import Path
import sys
import threading
import urllib.error
from contextlib import asynccontextmanager
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import gemini_chat as gc


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("HTTP epoch test must not use a network")
    monkeypatch.setattr(gc.urllib.request, "urlopen", forbidden)


def backend(epoch="test-runtime:0"):
    b = gc.RobotBackend("http://fake.invalid")
    b._command_epoch = epoch
    return b


def test_setup_binds_epoch_without_changing_tool_contract(monkeypatch):
    b = backend(None)
    monkeypatch.setattr(b, "_get", lambda path: {"prompt": "p", "tools": []}
                        if path == "/tools" else {"command_epoch": "test-runtime:7", "command_epoch_required": True})
    assert b.setup() == ("p", [])
    assert b._command_epoch == "test-runtime:7"


def test_mutation_carries_original_epoch_without_retry(monkeypatch):
    b = backend()
    calls = []
    def post(name, args, epoch):
        calls.append((name, args, epoch))
        if name == "stop":
            return {"success": True, "command_epoch": "test-runtime:1", "command_epoch_required": True}
        return {"success": False, "command_epoch_required": True,
                "command_epoch": "test-runtime:1", "message": "Stale; no command submitted"}
    monkeypatch.setattr(b, "_post", post)
    async def check():
        reply = await b.call("move", {"speed": .2})
        assert reply["success"] is False and not reply.get("outcome_unknown")
        assert calls == [("move", {"speed": .2}, "test-runtime:0")]
        assert not (await b.call("wait", {"duration": 1}))["success"]
        assert len(calls) == 1  # A model replan cannot cross an external Stop.
        await b.call("stop", {})
        assert not (await b.call("wait", {"duration": 1}))["success"]
        assert calls[-1] == ("stop", {}, None)  # Model Stop cannot reauthorize its stale plan.
        assert b._epoch_invalidated
    asyncio.run(check())


@pytest.mark.parametrize("code", [409, 428])
def test_http_epoch_rejection_is_explicit_not_unknown(monkeypatch, code):
    b = backend()
    expected = {"success": False, "command_epoch_required": True,
                "command_epoch": "test-runtime:1", "message": "No command submitted"}
    seen = []
    def urlopen(req, timeout):
        seen.append((req.get_header("X-pupper-command-epoch"), timeout))
        raise urllib.error.HTTPError(req.full_url, code, "rejected", {},
                                     io.BytesIO(json.dumps(expected).encode()))
    monkeypatch.setattr(gc.urllib.request, "urlopen", urlopen)
    assert asyncio.run(b.call("move", {})) == expected
    assert seen == [("test-runtime:0", 30)]


def test_stop_during_epoch_fetch_cannot_reauthorize_old_intent(monkeypatch):
    b = backend(None)
    entered, release = threading.Event(), threading.Event()
    calls = []
    def get(_path):
        entered.set()
        assert release.wait(3)
        return {"command_epoch": "test-runtime:1", "command_epoch_required": True}
    def post(name, _args, epoch):
        calls.append((name, epoch))
        return {"success": True, "command_epoch": "test-runtime:1", "command_epoch_required": True}
    monkeypatch.setattr(b, "_get", get)
    monkeypatch.setattr(b, "_post", post)
    async def check():
        motion = asyncio.create_task(b.call("move", {}))
        try:
            assert await asyncio.to_thread(entered.wait, 2)
            assert (await b.call("stop", {}))["success"]
        finally:
            release.set()
        reply = await motion
        assert reply["success"] is False and "superseded" in reply["message"]
        assert calls == [("stop", None)]
    asyncio.run(check())


def test_old_post_reply_does_not_overwrite_new_stop_epoch(monkeypatch):
    b = backend()
    entered, release = threading.Event(), threading.Event()
    def post(name, _args, epoch):
        if name == "move":
            assert epoch == "test-runtime:0"
            entered.set()
            assert release.wait(3)
            return {"success": True, "command_epoch": "test-runtime:0", "command_epoch_required": True}
        return {"success": True, "command_epoch": "test-runtime:1", "command_epoch_required": True}
    monkeypatch.setattr(b, "_post", post)
    async def check():
        motion = asyncio.create_task(b.call("move", {}))
        try:
            assert await asyncio.to_thread(entered.wait, 2)
            await b.call("stop", {})
        finally:
            release.set()
        late = await motion
        assert late["superseded"] and late["outcome_unknown"]
        assert b._command_epoch == "test-runtime:1"
    asyncio.run(check())


def test_pending_stop_blocks_motion_but_status_remains_available(monkeypatch):
    b = backend()
    entered, release = threading.Event(), threading.Event()
    calls = []
    def post(name, _args, epoch):
        calls.append((name, epoch))
        if name == "stop":
            entered.set()
            assert release.wait(3)
        return {"success": True, "command_epoch": "test-runtime:1", "command_epoch_required": True}
    monkeypatch.setattr(b, "_post", post)
    async def check():
        stop = asyncio.create_task(b.call("stop", {}))
        try:
            assert await asyncio.to_thread(entered.wait, 2)
            assert not (await b.call("move", {}))["success"]
            assert (await b.call("robot_status", {}))["success"]
        finally:
            release.set()
        await stop
        assert calls == [("stop", None), ("robot_status", None)]
    asyncio.run(check())


def test_unknown_stop_is_not_silently_rearmed(monkeypatch):
    b = backend()
    calls = []
    def post(name, _args, _epoch):
        calls.append(name)
        raise TimeoutError("synthetic transport timeout")
    monkeypatch.setattr(b, "_post", post)
    async def check():
        reply = await b.call("stop", {})
        assert reply["outcome_unknown"] and not reply["success"]
        assert not (await b.call("move", {}))["success"]
        assert calls == ["stop"]
    asyncio.run(check())


def test_cancelled_stop_keeps_outcome_unknown(monkeypatch):
    b = backend()
    entered, release = threading.Event(), threading.Event()
    def post(_name, _args, _epoch):
        entered.set()
        assert release.wait(3)
        return {"success": True, "command_epoch": "test-runtime:1", "command_epoch_required": True}
    monkeypatch.setattr(b, "_post", post)
    async def check():
        task = asyncio.create_task(b.call("stop", {}))
        try:
            assert await asyncio.to_thread(entered.wait, 2)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert b._stop_unknown and b._stops_pending == 0
            assert not (await b.call("move", {}))["success"]
        finally:
            release.set()
    asyncio.run(check())


def test_missing_epoch_support_refuses_before_post(monkeypatch):
    b = backend(None)
    monkeypatch.setattr(b, "_get", lambda _path: {"status": "running"})
    reply = asyncio.run(b.call("move", {}))
    assert not reply["success"] and "support is unavailable" in reply["message"]
    assert not reply.get("outcome_unknown")


@pytest.mark.parametrize("marker", [None, False])
def test_epoch_without_required_protocol_marker_cannot_authorize_motion(monkeypatch, marker):
    b = backend(None)
    monkeypatch.setattr(b, "_get", lambda _path: {"command_epoch": "test-runtime:0", "command_epoch_required": marker})
    reply = asyncio.run(b.call("move", {}))
    assert not reply["success"] and "support is unavailable" in reply["message"]


def test_status_metadata_does_not_change_client_epoch(monkeypatch):
    b = backend()
    monkeypatch.setattr(b, "_post", lambda *_args: {"success": True, "command_epoch": "old-runtime:99", "command_epoch_required": True})
    assert asyncio.run(b.call("robot_status", {}))["success"]
    assert b._command_epoch == "test-runtime:0"


@pytest.mark.parametrize("names", [("move", "stop", "robot_status"), ("stop", "stand_up", "wait")])
def test_diagnostic_stop_batch_cannot_start_other_motion(monkeypatch, names):
    from google import genai
    calls, responses = [], []
    async def call(name, args):
        calls.append(name)
        return {"success": True, "message": "test-only receipt"}
    async def receive():
        yield SimpleNamespace(server_content=None, tool_call=SimpleNamespace(function_calls=[
            SimpleNamespace(id=str(i), name=name, args={}) for i, name in enumerate(names)]))
        yield SimpleNamespace(server_content=SimpleNamespace(output_transcription=None, turn_complete=True), tool_call=None)
    async def send_client_content(**_kwargs):
        pass
    async def send_tool_response(**kwargs):
        responses.extend(kwargs["function_responses"])
    @asynccontextmanager
    async def connect(**_kwargs):
        yield SimpleNamespace(receive=receive, send_client_content=send_client_content,
                              send_tool_response=send_tool_response)
    monkeypatch.setattr(genai, "Client", lambda **_kwargs: SimpleNamespace(
        aio=SimpleNamespace(live=SimpleNamespace(connect=connect))))
    monkeypatch.setattr(gc, "api_key", lambda: "synthetic-test-value")
    inputs = iter(("test batch", "quit"))
    monkeypatch.setattr("builtins.input", lambda _prompt: next(inputs))
    fake = SimpleNamespace(name="test-only fake", setup=lambda: ("Test", []), call=call)
    asyncio.run(gc.chat(fake, gc.MODEL, False))
    assert calls == ["stop"] + (["robot_status"] if "robot_status" in names else [])
    assert len(responses) == len(names)
    for reply in responses:
        if reply.name not in ("stop", "robot_status"):
            assert reply.response["result"]["success"] is False
            assert "Not dispatched" in reply.response["result"]["message"]
