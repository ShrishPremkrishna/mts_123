"""Operator-only soccer kick, using the exported policy, fake ROS and in-process HTTP."""

import asyncio
from contextlib import asynccontextmanager
import hashlib
import json

import httpx
import pytest

from test_server_transport import harness, robot_api, server
import operator_kick
from tricks import TRICK_DIR, WALK_CONTROLLER, controller_params, load_tricks


@asynccontextmanager
async def selected_rig(monkeypatch, selection="leg_kick"):
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", selection)
    catalog = load_tricks()
    for name in ("time", "asyncio", "load_tricks"):
        monkeypatch.setattr(robot_api, name, getattr(robot_api, name))
    rig = harness.Rig(tricks=lambda: catalog)
    try:
        assert await rig.settle()
        yield rig
    finally:
        rig.close()
        await asyncio.sleep(0)


def test_candidate_assets_and_default_exclusion(monkeypatch):
    folder = TRICK_DIR / "operator" / "leg_kick"
    manifest = json.loads((folder / "artifact_manifest.json").read_text())
    for name, digest in manifest["files"].items():
        assert hashlib.sha256((folder / name).read_bytes()).hexdigest() == digest
    assert manifest["policy_run"] is None
    assert manifest["hardware_validated"] is manifest["ta_approval_recorded"] is False
    monkeypatch.delenv("PUPPER_SESSION_SKILLS", raising=False)
    assert "leg_kick" not in load_tricks()
    for selection in ("", "sit_stay_shake"):
        monkeypatch.setenv("PUPPER_SESSION_SKILLS", selection)
        assert "leg_kick" not in load_tricks()
    monkeypatch.setenv("PUPPER_SESSION_SKILLS", "leg_kick")
    trick = load_tricks()["leg_kick"]
    assert trick.operator_only
    params = controller_params(trick, {})
    assert (params["init_duration"], params["fade_in_duration"]) == (0.5, 1.0)


@pytest.mark.parametrize("side,frame", [("right", 180), ("left", 380)])
def test_http_entry_and_walker_restoration(server, monkeypatch, side, frame):
    async def case():
        async with selected_rig(monkeypatch, "leg_kick,secret_dance") as rig:
            server.tools = rig.ts
            try:
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app),
                                             base_url="http://in-process.invalid") as client:
                    headers = {"X-Pupper-Command-Epoch": rig.ts.command_epoch}
                    tools = (await client.get("/tools")).text
                    for name in ("kick_in_place", "leg_kick", "secret_dance"):
                        assert name not in tools
                    for path, body in [("/tools/kick_in_place", {"side": side}),
                                       ("/tools/play_trick", {"name": "leg_kick", "count": 1, "count_unit": "routine_playback"})]:
                        refused = await client.post(path, headers=headers, json=body)
                        assert not refused.json()["success"]
                    path = "/operator/kick_in_place"
                    assert (await client.post(path, json={"side": side})).status_code == 428
                    assert (await client.post(path, json={"side": "auto"}, headers=headers)).status_code == 422
                    assert (await client.post(path, json={"side": side, "speed": 1}, headers=headers)).status_code == 422
                    assert not rig.robot.activations("trick_leg_kick")
                    accepted = (await client.post(path, headers=headers, json={"side": side})).json()
                    assert accepted["success"] and await rig.settle()
                    final = rig.ts.last_outcome
                    assert final["job_id"] == accepted["details"]["submission"]["job_id"]
                    assert final["success"] and f"kick_{side} at frame " in final["message"]
                    actual_frame = int(final["message"].split("frame ")[1].split(".")[0])
                    assert frame <= actual_frame < frame + 21
                    assert "Walker restoration confirmed" in final["message"]
                    assert "contact unmeasured" in final["message"]
                    assert [act for _, _, act in rig.robot.switches] == [["trick_leg_kick"], [WALK_CONTROLLER]]
                    assert rig.robot.controllers[WALK_CONTROLLER] == "active"
                    await client.post("/tools/stop", json={})
                    assert (await client.post(path, headers=headers, json={"side": side})).status_code == 409
            finally:
                server.tools = None
    asyncio.run(case())


@pytest.mark.parametrize("bad", ["stand_idle", "wrong_foot", "early_frame", "out_of_range", "wrong_activation",
                                 "wrong_request", "queued", "holding"])
def test_receipt_anomalies_fail_and_restore_walker(monkeypatch, bad):
    async def case():
        async with selected_rig(monkeypatch) as rig:
            original = rig.robot.send
            def interfere(topic, msg):
                if topic == "/trick_leg_kick/clip_status":
                    fields = msg.data.split()
                    index, value = {"stand_idle": (0, "stand_idle"), "wrong_foot": (0, "kick_left"),
                                    "early_frame": (3, "179"), "out_of_range": (3, "201"),
                                    "wrong_activation": (7, "f" * 32), "wrong_request": (9, "e" * 32),
                                    "queued": (5, "1"), "holding": (1, "holding")}[bad]
                    fields[index] = value
                    msg = harness._DataMsg(" ".join(fields))
                original(topic, msg)
            monkeypatch.setattr(rig.robot, "send", interfere)
            assert (await operator_kick.kick_in_place(rig.ts, "right")).ok
            assert await rig.settle()
            assert not rig.ts.last_outcome["success"]
            assert "did not confirm kick_right" in rig.ts.last_outcome["message"]
            assert "Walker restoration confirmed" in rig.ts.last_outcome["message"]
            assert rig.robot.controllers[WALK_CONTROLLER] == "active"
    asyncio.run(case())


@pytest.mark.parametrize("interrupt", ["stop", "fall", "fault", "shared_block", "external_owner", "walker_refused"])
def test_interruption_after_valid_receipt_never_reports_walker_restored(monkeypatch, interrupt):
    async def case():
        async with selected_rig(monkeypatch) as rig:
            original = operator_kick.confirm_kick_progress
            async def after_playback(robot, trick, clip):
                progress = await original(robot, trick, clip)
                assert progress.ok
                if interrupt == "stop":
                    robot.stops += 1
                elif interrupt == "fall":
                    robot.fallen = trick.controller
                elif interrupt == "fault":
                    robot.controller_fault = "injected controller fault"
                elif interrupt == "shared_block":
                    monkeypatch.setattr(robot, "motion_block_reason", lambda: "shared stop latched")
                elif interrupt == "external_owner":
                    rig.robot.controllers[trick.controller] = "inactive"
                    rig.robot.controllers["neural_controller_default"] = "active"
                elif interrupt == "walker_refused":
                    switch = robot.switch_to
                    async def refuse(controller, *args, **kwargs):
                        if controller == WALK_CONTROLLER:
                            return robot_api.Result(False, "walker refused")
                        return await switch(controller, *args, **kwargs)
                    monkeypatch.setattr(robot, "switch_to", refuse)
                return progress
            monkeypatch.setattr(operator_kick, "confirm_kick_progress", after_playback)
            assert (await operator_kick.kick_in_place(rig.ts, "left")).ok
            assert await rig.settle()
            final = rig.ts.last_outcome
            assert not final["success"] and "confirmed kick_left" in final["message"]
            assert "Walker restoration unconfirmed" in final["message"]
            assert [act for _, _, act in rig.robot.switches if act] == [["trick_leg_kick"]]
            if interrupt == "walker_refused":
                assert rig.robot.controllers["trick_leg_kick"] == "inactive"
            elif interrupt == "external_owner":
                assert rig.robot.controllers["neural_controller_default"] == "active"
    asyncio.run(case())


def test_without_selection_or_active_walker_cannot_start_kick(monkeypatch):
    async def case():
        async with selected_rig(monkeypatch, "") as rig:
            assert not (await operator_kick.kick_in_place(rig.ts, "right")).ok
            assert not rig.robot.switches
        async with selected_rig(monkeypatch) as rig:
            rig.robot.controllers[WALK_CONTROLLER] = "inactive"
            assert (await operator_kick.kick_in_place(rig.ts, "right")).ok
            assert await rig.settle()
            assert not rig.ts.last_outcome["success"]
            assert not rig.robot.switches
    asyncio.run(case())


def test_stop_during_playback_cancels_and_releases_without_walker_reactivation(monkeypatch):
    async def case():
        async with selected_rig(monkeypatch) as rig:
            playing = asyncio.Event()
            async def await_receipt(robot, trick, clip):
                playing.set()
                await asyncio.Future()
            monkeypatch.setattr(operator_kick, "confirm_kick_progress", await_receipt)
            accepted = await operator_kick.kick_in_place(rig.ts, "left")
            assert accepted.ok
            await asyncio.wait_for(playing.wait(), timeout=2)
            assert rig.robot.controllers["trick_leg_kick"] == "active"
            result = await rig.ts.immediate_stop(relax_playing_trick=True)
            assert result.ok and await rig.settle()
            assert not rig.ts.last_outcome["success"]
            assert result.details["running_job"]["job_id"] == accepted.details["submission"]["job_id"]
            assert not rig.robot.activations(WALK_CONTROLLER)
            assert rig.robot.controllers["trick_leg_kick"] == "inactive"
    asyncio.run(case())


def test_walker_switch_reply_without_active_inventory_is_not_restoration(monkeypatch):
    async def case():
        async with selected_rig(monkeypatch) as rig:
            switch = rig.ts.switch_to
            async def lose_walker(controller, *args, **kwargs):
                result = await switch(controller, *args, **kwargs)
                if controller == WALK_CONTROLLER and kwargs.get("expected_active") == "trick_leg_kick":
                    rig.robot.controllers[WALK_CONTROLLER] = "inactive"
                return result
            monkeypatch.setattr(rig.ts, "switch_to", lose_walker)
            assert (await operator_kick.kick_in_place(rig.ts, "left")).ok and await rig.settle()
            assert not rig.ts.last_outcome["success"]
            assert "Could not verify the walker" in rig.ts.last_outcome["message"]
    asyncio.run(case())
