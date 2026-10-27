"""Exercise the subprocess lifecycle without model weights or a GPU."""

from pathlib import Path
import sys
import textwrap

import pytest

from mjlab.tasks.tracking.config.pupper import unimate as U


def fake_worker(tmp_path: Path, monkeypatch, source: str):
  python = tmp_path / ".venv/bin/python"
  python.parent.mkdir(parents=True)
  python.symlink_to(sys.executable)
  (tmp_path / "worker.py").write_text(textwrap.dedent(source))
  monkeypatch.setattr(U, "HERE", tmp_path)
  return tmp_path


def test_worker_context_closes_process_and_descriptors(tmp_path, monkeypatch):
  directory = fake_worker(tmp_path, monkeypatch, '''
    import json, sys
    print(json.dumps({"ok": True, "ready": True}), flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        print(json.dumps({"ok": True, "received": request["cmd"]}), flush=True)
  ''')
  with U.UniMate(directory, timeout_s=5) as worker:
    assert worker.call(cmd="hello")["received"] == "hello"
    process = worker.proc
  assert process.poll() is not None
  assert process.stdin.closed and process.stdout.closed
  assert worker._log_handle.closed
  worker.close()  # repeated cleanup is harmless


def test_worker_timeout_terminates_instead_of_hanging(tmp_path, monkeypatch):
  directory = fake_worker(tmp_path, monkeypatch, '''
    import json, sys, time
    print(json.dumps({"ok": True, "ready": True}), flush=True)
    for line in sys.stdin:
        if json.loads(line)["cmd"] == "register":
            print(json.dumps({"ok": True}), flush=True)
        else:
            time.sleep(30)
  ''')
  worker = U.UniMate(directory, timeout_s=5)
  worker.timeout_s = 0.1
  with pytest.raises(TimeoutError, match="did not respond"):
    worker.call(cmd="hang")
  assert worker.proc.poll() is not None
  assert worker._log_handle.closed


def test_bad_startup_response_is_cleaned_up(tmp_path, monkeypatch):
  directory = fake_worker(tmp_path, monkeypatch, '''
    import time
    print("not valid JSON", flush=True)
    time.sleep(30)
  ''')
  created = []
  popen = U.subprocess.Popen

  def capture(*args, **kwargs):
    process = popen(*args, **kwargs)
    created.append(process)
    return process

  monkeypatch.setattr(U.subprocess, "Popen", capture)
  with pytest.raises(RuntimeError, match="Invalid worker response"):
    U.UniMate(directory, timeout_s=5)
  assert created[0].poll() is not None
