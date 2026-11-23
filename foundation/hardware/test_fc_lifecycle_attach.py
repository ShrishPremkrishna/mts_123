"""Deferred perception attachment refusals in the real lifecycle module. Nothing is spawned."""
import json
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "gemini_pupper/robot"))
import session_processes as life

SESSION = "a" * 32


@pytest.fixture
def run(tmp_path, monkeypatch):
    boot = tmp_path / "proc/sys/kernel/random/boot_id"
    boot.parent.mkdir(parents=True)
    boot.write_text("boot-one\n")
    monkeypatch.setattr(life, "PROC", tmp_path / "proc")
    directory = tmp_path / "run"
    directory.mkdir(mode=0o700)
    spawned = []
    monkeypatch.setattr(life.subprocess, "Popen", lambda *a, **kw: spawned.append(a) or pytest.fail("spawned"))

    def write(**changes):
        receipt = dict(schema=life.SCHEMA, uid=os.getuid(), root=str(life.HERE), boot_id="boot-one",
                       session_id=SESSION, status="running", ros="created", kiosk=False, gui_stopped=False,
                       processes={"manager": {"pid": 1}}, pending={}, events=[], perception_deferred=True)
        receipt.update(changes)
        path = directory / "owned_session.json"
        path.write_text(json.dumps(receipt))
        path.chmod(0o600)
    write()
    return directory, write, spawned


@pytest.mark.parametrize("role, changes", [
    ("recorder", {}),                                   # only camera and detector may attach late
    ("launch", {}),
    ("camera", {"status": "starting"}),                 # Home has not completed
    ("camera", {"status": "stopping"}),                 # Stop/shutdown already began
    ("camera", {"perception_deferred": False}),         # camera was started with Home or already attached
    ("camera", {"perception_deferred": None}),
    ("detector", {"ros": "borrowed"}),                  # never attach to a ROS graph this session does not own
])
def test_attachment_requires_a_running_deferred_session_and_perception_role(run, role, changes):
    directory, write, spawned = run
    write(**changes)
    with pytest.raises(ValueError, match="No deferred camera attachment"):
        life.spawn(directory, role, "camera_node", ["ros2", "run"], attach_perception=True)
    assert not spawned


def test_attachment_never_restarts_an_owned_or_pending_role(run):
    directory, write, spawned = run
    for changes in ({"processes": {"manager": {"pid": 1}, "camera": {"pid": 2}}},
                    {"pending": {"camera": {"gate": {"pid": 3}}}}):
        write(**changes)
        with pytest.raises(ValueError, match="automatic restart is forbidden"):
            life.spawn(directory, "camera", "camera_node", ["ros2", "run"], attach_perception=True)
    assert not spawned


def test_ordinary_spawn_still_belongs_only_to_startup(run):
    directory, write, spawned = run
    with pytest.raises(ValueError, match="startup canceled or already finished"):
        life.spawn(directory, "camera", "camera_node", ["ros2", "run"])
    assert not spawned


def test_foreign_boot_receipt_cannot_attach(run):
    directory, write, spawned = run
    write(boot_id="boot-zero")
    with pytest.raises(ValueError, match="foreign ownership receipt"):
        life.spawn(directory, "camera", "camera_node", ["ros2", "run"], attach_perception=True)
    assert not spawned
