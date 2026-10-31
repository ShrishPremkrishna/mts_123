"""Actual diagnostic receive-loop ordering; stub SDK/HTTP, no credentials or sockets."""
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import gemini_chat as gc


def packet(*names):
    return SimpleNamespace(server_content=None, tool_call=SimpleNamespace(function_calls=[
        SimpleNamespace(id=f"call-{i}-{name}", name=name, args={}) for i, name in enumerate(names)]))


def end_turn():
    return SimpleNamespace(server_content=SimpleNamespace(output_transcription=None, turn_complete=True), tool_call=None)


@pytest.fixture(autouse=True)
def no_access(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("Dispatch tests cannot use credentials, network or hardware")
    monkeypatch.setattr(gc.urllib.request, "urlopen", forbidden)
    monkeypatch.setattr(gc, "api_key", lambda: "synthetic-test-value")
    monkeypatch.setattr(gc, "KEY_FILE", SimpleNamespace(exists=forbidden))


async def actual_chat(monkeypatch, backend, script, *, send_error=False, inputs=("test", "quit")):
    from google import genai
    replies = []
    async def send_tool_response(**kwargs):
        if send_error:
            raise ConnectionError("synthetic failed reply transport")
        replies.extend(kwargs["function_responses"])
    async def send_client_content(**_kwargs):
        pass
    async def receive():
        async for msg in script(replies):
            yield msg
    @asynccontextmanager
    async def connect(**_kwargs):
        yield SimpleNamespace(receive=receive, send_client_content=send_client_content,
                              send_tool_response=send_tool_response)
    monkeypatch.setattr(genai, "Client", lambda **_kwargs: SimpleNamespace(
        aio=SimpleNamespace(live=SimpleNamespace(connect=connect))))
    iterator = iter(inputs)
    monkeypatch.setattr("builtins.input", lambda _prompt: next(iterator))
    await asyncio.wait_for(gc.chat(backend, gc.MODEL, False), 2)
    return replies


async def until(predicate):
    async def wait():
        while not predicate():
            await asyncio.sleep(.001)
    await asyncio.wait_for(wait(), .5)


def fake(call):
    return SimpleNamespace(name="test-only fake", setup=lambda: ("Test", []), call=call)


def test_actual_cli_stop_packet_passes_a_slow_http_call_and_drops_its_buffered_action(monkeypatch):
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    b = gc.RobotBackend("http://stub.invalid")
    b._command_epoch = "fake:0"
    monkeypatch.setattr(b, "setup", lambda: ("Test", []))
    calls = []
    def post(name, _args, epoch):
        calls.append((name, epoch))
        if name == "move":
            entered.set()
            assert release.wait(1.5)
            return {"success": True, "message": "old acceptance"}
        assert name == "stop"
        stopped.set()
        return {"success": True, "command_epoch_required": True, "command_epoch": "fake:1"}
    monkeypatch.setattr(b, "_post", post)
    async def script(replies):
        try:
            yield packet("move", "turn")
            await until(entered.is_set)
            yield packet("stop")
            await until(stopped.is_set)  # Must happen before releasing the original HTTP call.
            assert not release.is_set()
            release.set()
            await until(lambda: len(replies) == 3)
            yield end_turn()
        finally:
            release.set()
    replies = asyncio.run(actual_chat(monkeypatch, b, script))
    assert calls == [("move", "fake:0"), ("stop", None)]
    assert next(r for r in replies if r.name == "move").response["result"]["superseded"]
    assert "Not dispatched" in next(r for r in replies if r.name == "turn").response["result"]["message"]


def test_non_stop_batches_preserve_dispatch_order_and_quit_cancels_queued_work(monkeypatch):
    async def check():
        entered, release = asyncio.Event(), asyncio.Event()
        calls = []
        async def call(name, _args):
            calls.append(name)
            if name == "move":
                entered.set()
                await release.wait()
            return {"success": True, "message": "accepted"}
        async def script(replies):
            yield packet("move")
            await entered.wait()
            yield packet("turn")
            await asyncio.sleep(.01)
            assert calls == ["move"]
            release.set()
            await until(lambda: len(replies) == 2)
            yield end_turn()
        await actual_chat(monkeypatch, fake(call), script)
        assert calls == ["move", "turn", "stop"]  # Cleanup is a Stop, never deferred motion.
    asyncio.run(check())


def test_pending_stop_rejects_later_motion_but_keeps_status_available(monkeypatch):
    async def check():
        entered, release = asyncio.Event(), asyncio.Event()
        calls = []
        async def call(name, _args):
            calls.append(name)
            if name == "stop":
                entered.set()
                await release.wait()
            return {"success": True, "message": "receipt"}
        async def script(replies):
            yield packet("stop")
            await entered.wait()
            yield packet("move", "robot_status")
            await until(lambda: "robot_status" in calls)
            assert calls == ["stop", "robot_status"]
            release.set()
            await until(lambda: len(replies) == 3)
            yield end_turn()
        replies = await actual_chat(monkeypatch, fake(call), script)
        assert next(r for r in replies if r.name == "move").response["result"]["success"] is False
    asyncio.run(check())


def test_unknown_stop_blocks_motion_in_later_user_turn_and_cleanup_reports_retry(monkeypatch, capsys):
    async def check():
        calls, turn = [], 0
        async def call(name, _args):
            calls.append(name)
            return {"success": name != "stop", "outcome_unknown": name == "stop", "message": "synthetic"}
        async def script(replies):
            nonlocal turn
            turn += 1
            yield packet("stop") if turn == 1 else packet("move", "robot_status")
            await until(lambda: len(replies) == (1 if turn == 1 else 3))
            yield end_turn()
        await actual_chat(monkeypatch, fake(call), script, inputs=("stop", "move", "quit"))
        assert calls == ["stop", "robot_status", "stop"]
    asyncio.run(check())
    assert "connection cleanup Stop" in capsys.readouterr().out


def test_confirmed_stop_allows_a_new_explicit_user_turn(monkeypatch):
    async def check():
        calls, turn = [], 0
        async def call(name, _args):
            calls.append(name)
            return {"success": True, "message": "receipt"}
        async def script(replies):
            nonlocal turn
            turn += 1
            yield packet("stop") if turn == 1 else packet("move")
            await until(lambda: len(replies) == turn)
            yield end_turn()
        await actual_chat(monkeypatch, fake(call), script, inputs=("stop", "move", "quit"))
        assert calls == ["stop", "move", "stop"]
    asyncio.run(check())


def test_receive_failure_cancels_pending_dispatch_and_sends_no_late_sdk_reply(monkeypatch):
    async def check():
        entered = asyncio.Event()
        calls, cancelled = [], []
        async def call(name, _args):
            calls.append(name)
            if name == "move":
                entered.set()
                try:
                    await asyncio.Event().wait()
                except asyncio.CancelledError:
                    cancelled.append(name)
                    raise
            return {"success": True, "message": "receipt"}
        async def script(replies):
            yield packet("move")
            await entered.wait()
            yield packet("turn")
            raise ConnectionError("synthetic receive failure")
        with pytest.raises(ConnectionError, match="receive failure"):
            await actual_chat(monkeypatch, fake(call), script)
        assert calls == ["move", "stop"] and cancelled == ["move"]
        assert not [t for t in asyncio.all_tasks() if t is not asyncio.current_task() and not t.done()]
    asyncio.run(check())


def test_tool_reply_transport_failure_interrupts_stalled_receive_and_stops_work(monkeypatch):
    async def check():
        calls = []
        async def call(name, _args):
            calls.append(name)
            return {"success": True, "message": "receipt"}
        async def script(_replies):
            yield packet("move")
            await asyncio.Event().wait()
        with pytest.raises(ConnectionError, match="failed reply transport"):
            await actual_chat(monkeypatch, fake(call), script, send_error=True)
        assert calls == ["move", "stop"]
    asyncio.run(check())


def test_drain_failure_keeps_pending_siblings_owned_for_shutdown():
    async def check():
        entered, cancelled = asyncio.Event(), asyncio.Event()
        async def call(name, _args):
            if name == "move":
                entered.set()
                try:
                    await asyncio.Event().wait()
                except asyncio.CancelledError:
                    cancelled.set()
                    raise
            return {"success": True, "message": "receipt"}
        async def failed_response(**_kwargs):
            raise ConnectionError("synthetic Stop reply failure during drain")
        dispatcher = gc._ToolDispatcher(fake(call), SimpleNamespace(send_tool_response=failed_response), SimpleNamespace)
        dispatcher.submit(packet("move").tool_call.function_calls)
        await entered.wait()
        dispatcher.submit(packet("stop").tool_call.function_calls)
        try:
            with pytest.raises(ConnectionError, match="during drain"):
                await dispatcher.drain()
            assert len(dispatcher.tasks) == 1
        finally:
            await dispatcher.close()
        assert cancelled.is_set()
        assert dispatcher.tasks == []
        assert not [t for t in asyncio.all_tasks() if t is not asyncio.current_task() and not t.done()]
    asyncio.run(check())
