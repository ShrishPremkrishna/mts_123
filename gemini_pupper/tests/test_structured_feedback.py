"""Real producer facts and mirrored closed operator wording; no robot/network."""
import asyncio
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import test_tools as harness
from operator_feedback import receipt_feedback
from robot_api import LIMITS
from part2_walk import check_move, _check_move_result, walk
CASES = json.loads((Path(__file__).parent / "fixtures/structured-feedback.json").read_text())

@pytest.mark.parametrize("case", CASES, ids=lambda c:c["title"])
def test_shared_receipts(case):
    text = receipt_feedback(case["name"], case["result"])
    assert text == case["expected"]
    assert len(text) <= 240
    assert "INJECTED" not in text

@pytest.mark.parametrize("args,code", [((.1,0,0,1),"minimum_speed"),((9,0,0,1),"speed_limit"),((.4,0,0,21),"duration_limit"),((.4,0,0,20),"distance_limit"),((float('nan'),0,0,1),"invalid_values")])
def test_check_move_keeps_string_contract_and_typed_reason(args,code):
    result = _check_move_result(*args, LIMITS)
    assert result.ok is False
    assert check_move(*args) == result.message
    assert result.details["refusal"]["code"] == code
    assert "rejected:" in receipt_feedback("move",result.as_dict())


def test_actual_walk_refuses_before_queue_and_speaks_useful_minimum():
    async def run():
        rig = harness.Rig()
        try:
            await rig.settle()
            result = await walk(rig.ts,.1,0,0,.5)
            assert not result.ok and rig.ts.commands.qsize() == 0
            assert result.details["refusal"]["code"] == "minimum_speed"
            assert "speed below minimum" in receipt_feedback("move", result.as_dict())
        finally:
            rig.close()
            for _ in range(5): await asyncio.sleep(0)
    asyncio.run(run())


def test_enormous_json_integer_is_unusable_status_not_formatter_crash():
    result = {"success": True, "details": {"running": False, "queued": 10 ** 1000}}
    assert receipt_feedback("robot_status", result) == "Status request accepted."
