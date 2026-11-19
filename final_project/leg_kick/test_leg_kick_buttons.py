"""Offline checks of the leg kick trigger logic (no ROS): python3 -m pytest -q"""

import leg_kick_buttons as lkb


def run(logic, t_end, joints=lambda t: 0.0, statuses=lambda t: "stand_idle", dt=0.004, t0=0.0):
  """Feed clip status at 50 Hz, joint samples at 250 Hz and ticks at 50 Hz."""
  actions, t = [], t0
  while t < t_end:
    if round(t / dt) % 5 == 0:
      clip = statuses(t)
      if clip:
        actions += [(round(t, 2), a) for a in logic.status(t, clip)]
      actions += [(round(t, 2), a) for a in logic.tick(t)]
    actions += [(round(t, 2), a) for a in logic.joints(t, joints(t))]
    t += dt
  return actions


def kick_clip(t):
  return "kick_right" if 5.6 <= t < 8.6 else "stand_idle"


def test_kick_after_wait_then_exit_after_clip():
  acts = [a for _, a in run(lkb.KickLogic(), 10.0, statuses=kick_clip)]
  assert acts == [("kick", "right"), ("exit", "kick done")]


def test_exit_time_is_half_a_second_after_the_clip():
  acts = run(lkb.KickLogic(), 10.0, statuses=kick_clip)
  assert 9.08 <= acts[-1][0] <= 9.16


def test_fast_swing_inside_the_kick_does_not_trip_the_watchdog():
  acts = run(lkb.KickLogic(), 10.0, statuses=kick_clip, joints=lambda t: 15.0 if 6.8 < t < 7.2 else 0.1)
  assert [a for _, a in acts] == [("kick", "right"), ("exit", "kick done")]


def test_shake_before_the_kick_exits_and_cancels_the_kick():
  acts = run(lkb.KickLogic(), 8.0, joints=lambda t: 2.5 if t > 2.0 else 0.05)
  assert [a[0] for _, a in acts] == ["exit"]
  assert 2.1 <= acts[0][0] <= 2.3  # the window mean passes 1.5 rad/s after about 0.18 s


def test_quiet_robot_never_trips():
  acts = run(lkb.KickLogic(), 4.0, joints=lambda t: 0.4)
  assert acts == []


def test_side_buttons():
  logic = lkb.KickLogic()
  logic.button(4)
  assert logic.side == "left"
  logic.button(7)
  assert logic.side == "right"
  logic.button(0)
  assert logic.side == "right"


def test_each_activation_after_a_gap_kicks_again():
  logic = lkb.KickLogic()
  run(logic, 10.0, statuses=kick_clip)
  second = run(logic, 22.0, t0=12.0, statuses=lambda t: kick_clip(t - 12.0))
  assert [a for _, a in second] == [("kick", "right"), ("exit", "kick done")]
