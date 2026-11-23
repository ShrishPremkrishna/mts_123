"""Live setup configuration without credentials, sockets, ROS or hardware."""
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import gemini_chat


@pytest.mark.parametrize("model,override,expected", [
    ("gemini-3.1-flash-live-preview", None, "HIGH"),
    ("models/gemini-3.1-flash-live-preview", None, "HIGH"),
    ("gemini-2.5-flash-native-audio-preview-12-2025", None, None),
    ("gemini-3.1-flash-tts-preview", None, None),
    ("gemini-3.1-flash-live-preview", "MINIMAL", "MINIMAL"),
    ("gemini-3.1-flash-live-preview", "HIGH", "HIGH"),
    ("gemini-3.1-flash-live-preview", "medium", "MEDIUM"),
])
def test_actual_sdk_setup_has_reproducible_thinking_configuration(monkeypatch, capsys,
                                                                 model, override, expected):
    from google import genai

    seen = []

    @asynccontextmanager
    async def connect(**setup):
        seen.append(setup)
        yield SimpleNamespace()

    def forbidden(*args, **kwargs):
        raise AssertionError("Configuration test must not access credentials or network")

    monkeypatch.setattr(gemini_chat.urllib.request, "urlopen", forbidden)
    monkeypatch.setattr(gemini_chat, "KEY_FILE", SimpleNamespace(exists=forbidden))
    monkeypatch.setattr(gemini_chat, "api_key", lambda: "synthetic-test-value")
    monkeypatch.setattr(genai, "Client", lambda **_kwargs: SimpleNamespace(
        aio=SimpleNamespace(live=SimpleNamespace(connect=connect))))
    monkeypatch.setattr("builtins.input", lambda _prompt: "quit")
    backend = SimpleNamespace(name="setup-only fake", setup=lambda: ("Test prompt", []))
    # The original three-argument call remains valid and selects AUTO.
    kwargs = {} if override is None else {"thinking_level": override}
    asyncio.run(gemini_chat.chat(backend, model, False, **kwargs))
    assert len(seen) == 1 and seen[0]["model"] == model
    config = seen[0]["config"]
    if expected is None:
        assert config.thinking_config is None
        assert "thinking_config" not in config.model_dump(exclude_none=True)
    else:
        assert config.thinking_config.thinking_level.value == expected
    assert f"Thinking level: {expected or 'model default (not overridden)'}" in capsys.readouterr().out


def test_cli_explicit_thinking_level_reaches_chat_without_opening_a_connection(monkeypatch):
    seen = []

    async def capture(backend, model, full, *, thinking_level):
        seen.append((backend, model, full, thinking_level))

    monkeypatch.setattr(gemini_chat, "chat", capture)
    monkeypatch.setattr(sys, "argv", ["gemini_chat.py", "--realtime", "--thinking-level", "minimal"])
    gemini_chat.main()
    assert len(seen) == 1
    backend, model, full, level = seen[0]
    assert isinstance(backend, gemini_chat.DeskBackend) and backend.realtime
    assert model == gemini_chat.MODEL and full is False and level == "MINIMAL"


def test_cli_rejects_unknown_thinking_level_before_any_connection(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Invalid configuration must fail before creating a backend")

    monkeypatch.setattr(gemini_chat, "DeskBackend", forbidden)
    monkeypatch.setattr(sys, "argv", ["gemini_chat.py", "--thinking-level", "unlimited"])
    with pytest.raises(SystemExit) as result:
        gemini_chat.main()
    assert result.value.code == 2
