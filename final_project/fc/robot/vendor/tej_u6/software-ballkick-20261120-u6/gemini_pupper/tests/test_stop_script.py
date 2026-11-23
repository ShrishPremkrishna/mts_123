"""Run shutdown shell with mocked network/process commands; never contact hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "robot" / "stop.sh"
EPOCH = "1" * 32 + ":4"


def run_shutdown(tmp_path, mode):
    commands = tmp_path / "bin"
    commands.mkdir()
    log = tmp_path / "calls.jsonl"
    mock = "#!" + sys.executable + "\n" + r'''import json, os, sys
from pathlib import Path
name = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["MOCK_LOG"], "a") as out:
    out.write(json.dumps({"command": name, "args": args}) + "\n")
if name != "curl":
    sys.exit(1 if name in ("pkill", "pgrep") else 0)
mode = os.environ["MOCK_MODE"]
url = args[-1]
epoch = "1" * 32 + ":4"
if url.endswith("/stop/immediate"):
    if mode == "stop_transport": sys.exit(28)
    print(json.dumps({"success": mode != "stop_semantic", "command_epoch": epoch}))
elif url.endswith("/health"):
    if mode == "health_transport": sys.exit(7)
    if mode == "health_invalid": print("not JSON")
    else: print(json.dumps({"command_epoch_required": mode != "unknown_protocol", **({} if mode == "missing_epoch" else {"command_epoch": epoch})}))
elif url.endswith("/walking/deactivate"):
    header = "X-Pupper-Command-Epoch: " + epoch
    http_error = header not in args or mode == "stale_epoch"
    # Reproduce real curl: HTTP rejection is exit 0 unless fail mode is set.
    if http_error and any(a in ("-f", "--fail", "--fail-with-body") or (a.startswith("-") and not a.startswith("--") and "f" in a[1:]) for a in args):
        sys.exit(22)
    if mode == "deactivate_transport": sys.exit(28)
    print(json.dumps({"success": not http_error and mode != "deactivate_semantic", "message": "PRIVATE_RESPONSE_MARKER"}))
else:
    raise SystemExit("unexpected mock URL")
'''
    for name in ("curl", "pkill", "pgrep", "ss", "sleep", "sudo"):
        path = commands / name
        path.write_text(mock)
        path.chmod(0o700)
    env = {**os.environ, "PATH": str(commands) + os.pathsep + os.environ["PATH"],
           "MOCK_LOG": str(log), "MOCK_MODE": mode}
    result = subprocess.run(["/bin/bash", str(SCRIPT)], env=env, text=True,
                            capture_output=True, timeout=10)
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert "PRIVATE_RESPONSE_MARKER" not in result.stdout + result.stderr
    # Existing process teardown is preserved even if graceful HTTP shutdown fails.
    assert any(c["command"] == "pkill" for c in calls)
    assert any(c["command"] == "sudo" and c["args"] == ["-n", "systemctl", "start", "pupper-gui"] for c in calls)
    return result, [c["args"] for c in calls if c["command"] == "curl"]


def test_shutdown_sends_current_epoch_after_stop_and_reports_only_acceptance(tmp_path):
    result, calls = run_shutdown(tmp_path, "ok")
    assert result.returncode == 0
    assert [c[-1] for c in calls] == ["localhost:8000/stop/immediate", "localhost:8000/health", "localhost:8000/walking/deactivate"]
    assert "X-Pupper-Command-Epoch: " + EPOCH in calls[2]
    assert "accepted" in result.stdout and "not verified" in result.stdout


@pytest.mark.parametrize("mode", ["missing_epoch", "stale_epoch", "health_invalid", "health_transport", "unknown_protocol",
                                  "stop_transport", "stop_semantic", "deactivate_transport", "deactivate_semantic"])
def test_shutdown_failures_never_masquerade_as_success_or_retry(tmp_path, mode):
    result, calls = run_shutdown(tmp_path, mode)
    assert result.returncode != 0
    assert "not confirmed" in result.stderr
    assert result.stdout.strip() != "stopped"
    mutations = [c for c in calls if c[-1].endswith("/walking/deactivate")]
    assert len(mutations) <= 1
    if mode in ("missing_epoch", "health_invalid", "health_transport", "unknown_protocol"):
        assert mutations == []
