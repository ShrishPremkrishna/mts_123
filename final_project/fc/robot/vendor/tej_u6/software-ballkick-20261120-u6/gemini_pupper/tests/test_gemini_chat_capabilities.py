"""The offline diagnostic respects the selected follower capability."""
import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import gemini_chat


@pytest.mark.parametrize("realtime", [False, True])
def test_disabled_follower_is_absent_and_refused_without_fake_service_calls(monkeypatch, realtime):
    monkeypatch.setenv("PUPPER_FOLLOWER_MODE", "disabled")
    async def check():
        backend = gemini_chat.DeskBackend(realtime=realtime)
        try:
            _, declarations = backend.setup()
            await backend.rig.ts.ready.wait()
            assert backend.rig.ts.follow_available is False
            assert "follow" not in {d["name"] for d in declarations}
            reply = await backend.call("follow", {"enabled": True})
            assert reply["success"] is False
            assert backend.rig.robot.follow_calls == []
        finally:
            await backend.aclose()
    asyncio.run(check())


def test_invalid_follower_mode_fails_before_fake_runtime_allocation(monkeypatch):
    monkeypatch.setenv("PUPPER_FOLLOWER_MODE", "invented-mode")
    async def check():
        backend = gemini_chat.DeskBackend(realtime=True)
        with pytest.raises(ValueError, match="PUPPER_FOLLOWER_MODE"):
            backend.setup()
        assert not hasattr(backend, "rig")
        await backend.aclose()
    asyncio.run(check())
