"""Explicit routine quantity at the external tool boundary; fake ROS only."""
import asyncio
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot'))
import test_tools as harness
import tool_registry


def run_case(case):
    async def run():
        rig = harness.Rig()
        try:
            assert await rig.settle()
            await case(rig)
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(run())


def test_model_schema_requires_quantity_and_unit():
    async def case(rig):
        p = next(d for d in tool_registry.declarations(rig.ts) if d['name'] == 'play_trick')['parameters']
        assert set(p['required']) == {'name', 'count', 'count_unit'}
        assert p['properties']['count']['type'] == 'INTEGER'
        assert p['properties']['count_unit']['enum'] == ['routine_playback', 'physical_repetition']
    run_case(case)


@pytest.mark.parametrize('extra', [
    {}, {'count': 1}, {'count_unit': 'routine_playback'},
    {'count': True, 'count_unit': 'routine_playback'},
    {'count': 1.5, 'count_unit': 'routine_playback'},
    {'count': 0, 'count_unit': 'routine_playback'},
    {'count': -1, 'count_unit': 'routine_playback'},
    {'count': 10**1000, 'count_unit': 'routine_playback'},
    {'count': 11, 'count_unit': 'routine_playback'},
    {'count': 1, 'count_unit': 'invented'},
])
def test_invalid_or_missing_quantity_cannot_change_queue_or_follow(extra):
    async def case(rig):
        rig.ts.following = True
        switches = list(rig.robot.switches)
        status = rig.ts.status()
        result = await tool_registry.call(rig.ts, 'play_trick', {'name': 'bow', **extra})
        assert not result.ok
        assert rig.ts.following is True
        assert rig.robot.switches == switches
        current = rig.ts.status()
        assert (current['queued'], current['running'], current['state']) == (status['queued'], status['running'], status['state'])
    run_case(case)


@pytest.mark.parametrize('unit,count,code', [
    ('physical_repetition', 1, 'physical_count_unsupported'),
    ('physical_repetition', 10, 'physical_count_unsupported'),
    ('routine_playback', 2, 'playback_count_unsupported'),
    ('routine_playback', 10, 'playback_count_unsupported'),
])
def test_unsupported_quantity_refuses_before_prepare_or_queue(unit, count, code):
    async def case(rig):
        rig.ts.following = True
        switches = list(rig.robot.switches)
        result = await tool_registry.call(rig.ts, 'play_trick', {'name': 'bow', 'count': count, 'count_unit': unit})
        assert not result.ok and result.details['refusal'] == {'schema': 'pupper_trick_quantity_refusal_v1', 'code': code}
        assert rig.ts.following and rig.robot.switches == switches
        assert rig.ts.status()['queued'] == 0
    run_case(case)


def test_explicit_once_retains_existing_one_playback_behavior():
    async def case(rig):
        result = await tool_registry.call(rig.ts, 'play_trick', {'name': 'bow', 'count': 1, 'count_unit': 'routine_playback'})
        assert result.ok, result.message
        assert result.details['submission'] == {'schema': 'pupper_submission_v1', 'state': 'queued'}
        assert await rig.settle()
        assert len(rig.robot.activations(rig.ts.tricks['bow'].controller)) == 1
        assert rig.ts.last_outcome['success']
    run_case(case)


@pytest.mark.parametrize("code,expected", [
    ("physical_count_unsupported", "Exact physical repetition counts are not supported."),
    ("playback_count_unsupported", "Only one routine playback is supported per request."),
    ("invalid_quantity", "Trick quantity must be an integer from 1 to 10 with a supported unit."),
])
def test_quantity_feedback_uses_closed_facts_with_unknown_precedence(code, expected):
    from operator_feedback import receipt_feedback
    result = {"success": False, "message": "ARBITRARY_MODEL_OR_SERVER_TEXT", "details": {"refusal": {"schema": "pupper_trick_quantity_refusal_v1", "code": code}}}
    assert receipt_feedback("play_trick", result) == expected
    assert receipt_feedback("play_trick", {**result, "outcome_unknown": True}) == "I could not confirm that request."
    assert receipt_feedback("play_trick", {**result, "superseded": True}) == "That request was superseded."
    assert receipt_feedback("play_trick", {**result, "success": True}) == "Trick request accepted."
    for bad in ({}, "injected_code", None):
        malformed = {**result, "details": {"refusal": {"schema": "pupper_trick_quantity_refusal_v1", "code": bad}}}
        assert receipt_feedback("play_trick", malformed) == "That request failed. Check the controls for details."
