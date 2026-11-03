"""Exact manual and model quantity entrypoints through in-process ASGI only."""
import asyncio
import pytest
from test_server_transport import server, client_with_rig


def test_manual_animation_explicitly_submits_one_routine(server):
    async def case():
        async with client_with_rig(server) as (client, rig):
            response = await client.post("/animation", json={"animation_name": "bow"})
            assert response.status_code == 200 and response.json()["success"]
            assert await rig.settle()
            assert len(rig.robot.activations(rig.ts.tricks["bow"].controller)) == 1
    asyncio.run(case())


@pytest.mark.parametrize("args", [
    {"name": "bow"},
    {"name": "bow", "count": 10, "count_unit": "physical_repetition"},
    {"name": "bow", "count": 2, "count_unit": "routine_playback"},
])
def test_model_quantity_cannot_be_downgraded_by_http_endpoint(server, args):
    async def case():
        async with client_with_rig(server) as (client, rig):
            switches = list(rig.robot.switches)
            rig.ts.following = True
            response = await client.post("/tools/play_trick", json=args)
            assert response.status_code == 200 and not response.json()["success"]
            assert rig.ts.following and rig.robot.switches == switches
            assert rig.ts.status()["queued"] == 0
    asyncio.run(case())


@pytest.mark.parametrize("extra", [{"count": 10}, {"count_unit": "physical_repetition"}, {"repetitions": 10}])
def test_single_routine_manual_endpoint_does_not_silently_ignore_quantity(server, extra):
    async def case():
        async with client_with_rig(server) as (client, rig):
            switches = list(rig.robot.switches)
            response = await client.post("/animation", json={"animation_name": "bow", **extra})
            assert response.status_code == 422
            assert rig.robot.switches == switches and rig.ts.status()["queued"] == 0
    asyncio.run(case())


def test_model_swim_pair_is_one_http_submission_with_two_actual_playbacks(server, monkeypatch):
    from test_server_transport import harness
    from tricks import load_tricks
    swim = load_tricks()["swim"]
    monkeypatch.setattr(harness, "fixture_tricks", lambda: {"swim": swim})
    async def case():
        async with client_with_rig(server) as (client, rig):
            response = await client.post("/tools/play_trick", json={
                "name": "swim", "count": 2, "count_unit": "routine_playback"})
            result = response.json()
            assert response.status_code == 200 and result["success"], result
            assert result["details"]["submission"]["state"] == "queued"
            assert "accepted, not completed" in result["message"]
            assert rig.ts._job_sequence == 1
            assert await rig.settle()
            assert len(rig.robot.activations(swim.controller)) == 2
            assert rig.robot.active() == {swim.controller}
            assert rig.ts.last_outcome["job_id"] == result["details"]["submission"]["job_id"]
            assert rig.ts.last_outcome["success"]
    asyncio.run(case())
