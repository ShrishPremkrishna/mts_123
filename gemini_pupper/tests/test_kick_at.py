"""robot/kick_at.py: the turn-and-walk plan and the one kick_at tool call, without a robot."""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import kick_at as ka  # noqa: E402
from ball_seek import PAW_Y  # noqa: E402


def after(x, y, turn_deg, walk_m, turn_gain=1.0, walk_gain=1.0):
    """Ball position relative to the robot after the turn and the walk."""
    th = math.radians(turn_deg) * turn_gain
    x1, y1 = math.cos(th) * x + math.sin(th) * y, -math.sin(th) * x + math.cos(th) * y
    return x1 - walk_m * walk_gain, y1


@pytest.mark.parametrize("x,y", [(0.6, 0.0), (0.5, 0.2), (0.9, -0.4), (0.3, 0.05), (0.1, 0.5),
                                 (-0.5, 0.2), (-0.6, -0.1), (1.5, 0.8), (0.27, -0.03)])
@pytest.mark.parametrize("side", ["right", "left"])
def test_plan_puts_the_ball_at_the_chest_on_the_paw_line(x, y, side):
    turn, walk = ka.plan(x, y, side)
    bx, by = after(x, y, turn, walk)
    assert abs(bx - ka.TARGET_X) < 1e-9 and abs(by - PAW_Y[side]) < 1e-9, (bx, by)


def test_auto_side_follows_the_ball():
    assert ka.choose_side(0.2, "auto") == "left" and ka.choose_side(-0.2, "auto") == "right"
    assert ka.choose_side(0.2, "right") == "right"


class Client:
    def __init__(self):
        self.calls = []

    def kick_at(self, x, y, side, dry_run, gain, delay):
        self.calls.append((x, y, side, dry_run, gain, delay))
        return f"kick_{side} played"


def test_run_makes_one_kick_at_call_with_the_chosen_side():
    """The turn, the walk and the kick are one job of the API (tests/test_kick_ball_tool.py
    checks that job); the command only chooses the side and passes the walker model."""
    c = Client()
    result = ka.run(c, 0.7, 0.15, "auto", log=lambda text: None)
    assert c.calls == [(0.7, 0.15, "left", False, ka.FWD_GAIN, ka.DEAD_S)] and "kick_left" in result
    ka.run(c, 0.7, -0.15, "auto", dry_run=True, log=lambda text: None, gain=1.2, delay=0.1)
    assert c.calls[-1] == (0.7, -0.15, "right", True, 1.2, 0.1)


def test_run_refuses_before_any_call():
    c = Client()
    with pytest.raises(ValueError):
        ka.run(c, 5.0, 0.0, "right", log=lambda text: None)
    assert c.calls == []


@pytest.mark.parametrize("x,y", [(5.0, 0.0), (0.0, 0.03)])
def test_far_or_under_the_robot_is_refused(x, y):
    with pytest.raises(ValueError):
        ka.plan(x, y, "right")


def test_walker_errors_still_land_in_the_kick_window():
    """Turn gain 0.9-1.1 and walk gain 0.9-1.1 (true travel / modelled travel), balls 0.3-1.0 m
    away within 30 deg: the ball ends 3 cm toward the body to 4 cm short of contact (0.258 m),
    and within 2.5 cm of the paw line. A ball pushed by the chest counts as at the chest.
    2026-11-19: 96 % at 0-15 deg, 91 % at 15-30, 69 % at 30-45, 51 % at 45-60 (no camera in
    the loop: a turn error moves the ball sideways; wide turns need fresh coordinates)."""
    rng = random.Random(1)
    bad = []
    for _ in range(500):
        r, b = rng.uniform(0.3, 1.0), math.radians(rng.uniform(-30, 30))
        x, y = r * math.cos(b), r * math.sin(b)
        side = ka.choose_side(y, "auto")
        turn, walk = ka.plan(x, y, side)
        bx, by = after(x, y, turn, walk, rng.uniform(0.9, 1.1), rng.uniform(0.9, 1.1))
        bx = max(bx, ka.TARGET_X)  # the chest pushes the ball
        if not (0.258 - 0.03 <= bx <= 0.258 + 0.04 and abs(by - PAW_Y[side]) <= 0.025):
            bad.append((round(x, 2), round(y, 2), round(bx, 3), round(by - PAW_Y[side], 3)))
    assert len(bad) <= 40, bad[:10]  # 92 %
