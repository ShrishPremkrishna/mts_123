"""Local Ball Eye docking evidence: admission, expiry and the paw contact contract.

All observations are synthetic. These tests prove timing/identity/selection
behaviour of the consumer, not physical Ball Eye accuracy.
"""
import math
import random

import pytest

from final_project.pupper_fc import paw_approach
from final_project.pupper_fc.local_ball import (
    DockingProfile, LocalBallObservation, LocalDocking, body_contact_window, live_profile, local_kick_ready,
    overhead_body_offset, synthetic_profile)
from final_project.pupper_fc.model import Ball, Field, Pose

PROFILE_ID, CAL_ID = "a" * 64, "b" * 64
FORWARD = paw_approach.COMBINED_PAW_FORWARD_M["left"]
LEFT = paw_approach.PAW_LEFT_M["left"]
ORIGIN = Pose(0., 0., 0.)


def obs(t, x=FORWARD, y=LEFT, *, sigma=.002, method="eye_fit", radius=.11, profile=PROFILE_ID, cal=CAL_ID,
        status="synthetic", measured=False, seq=0):
    return LocalBallObservation(t, x, y, sigma, method, profile, cal, radius, status, "cam", seq, measured, measured)


def row(t, state="measured", **fields):
    o = obs(t, **fields)
    candidate = dict(method=o.method, x=o.x, y=o.y, sigma_xy_m=o.sigma_xy_m)
    return dict(state=state, selected=0 if state == "measured" else None,
                candidates=[candidate] if state == "measured" else [], captured_at_mono_ns=round(t * 1e9),
                producer_instance="cam", sequence=0,
                identity=dict(profile_id=o.profile_id, calibration_id=o.calibration_id, ball_radius_m=o.ball_radius_m,
                              status=o.profile_status, radius_measured=False, mount_measured=False))


def docking(**overrides):
    dock = LocalDocking(synthetic_profile(.11, **overrides))
    dock.note_command(0., 0., 0., 0.)  # the robot is commanded still from t = 0
    return dock


def confirmed(dock, t0=1.0, **fields):
    assert dock.observe(obs(t0, **fields), t0 + .02)[0]
    assert dock.observe(obs(t0 + .1, **fields), t0 + .12) == (True, "confirmed")
    return t0 + .1


def agree(dock, t, robot=ORIGIN, ball=None):
    ball = ball or Ball(robot.x + FORWARD, robot.y + LEFT)
    dock.note_overhead(robot, t, ball, t)
    return dock.evidence(t + .03, robot, t)


@pytest.mark.parametrize("captured", [float("nan"), float("inf"), None, "1.0"])
def test_missing_or_nonfinite_capture_is_refused_not_repaired(captured):
    dock = docking()
    accepted, reason = dock.observe(obs(captured), 1.0)
    assert not accepted and "not finite" in reason and dock.track.highwater is None


def test_future_and_stale_captures_are_refused_without_poisoning_order():
    dock = docking()
    assert dock.observe(obs(1.5), 1.0) == (False, "capture time is in the future")
    assert dock.observe(obs(0.70), 1.0) == (False, "capture is stale")
    # Neither refused capture advanced the order, so a valid earlier capture still enters.
    assert dock.observe(obs(0.95), 1.0)[0]


def test_duplicate_and_reordered_captures_cannot_become_new_evidence():
    dock = docking()
    t = confirmed(dock)
    assert dock.observe(obs(t), t + .05) == (False, "duplicate or reordered capture")
    assert dock.observe(obs(t - .05), t + .05) == (False, "duplicate or reordered capture")
    assert dock.track.current(t + .05, dock.ledger).captured_at == t


def test_outlier_never_seeds_but_suspends_older_evidence_until_consistent_fix():
    dock = docking()
    t = confirmed(dock)
    assert dock.observe(obs(t + .1, x=FORWARD + .3), t + .12) == (False, "outlier against the current track")
    # The newer contradiction suspends the older fix; it does not replace it.
    assert dock.track.current(t + .12, dock.ledger) is None and dock.track.fixes[-1].x == FORWARD
    assert dock.observe(obs(t + .2, x=FORWARD + .003), t + .22) == (True, "confirmed")
    assert dock.track.current(t + .22, dock.ledger).x == FORWARD + .003
    # A persistent jump (the ball really moved) only replaces the track after it
    # lapses, and then needs fresh confirmation; the refused fixes are not reused.
    for k in (.1, .2, .29):
        assert not dock.observe(obs(t + .2 + k, x=FORWARD + .3), t + .22 + k)[0]
    assert dock.observe(obs(t + .55, x=FORWARD + .3), t + .57) == (True, "tentative")
    assert dock.track.current(t + .57, dock.ledger) is None
    assert dock.observe(obs(t + .65, x=FORWARD + .301), t + .67) == (True, "confirmed")


def test_contradicted_tentative_seed_is_dropped_with_the_contradiction():
    dock = docking()
    assert dock.observe(obs(1.0), 1.01)[0]
    assert not dock.observe(obs(1.1, x=FORWARD + .4), 1.11)[0]
    assert not dock.track.fixes and dock.track.reason == "outlier against the current track"
    assert dock.observe(obs(1.2, x=FORWARD + .4), 1.21) == (True, "tentative")


def test_newer_capture_without_a_ball_ends_the_track_and_older_rows_stay_out():
    dock = docking()
    t = confirmed(dock)
    assert agree(dock, t).status == "local"
    assert dock.observe_row(row(t + .1, state="searching"), t + .12) == (True, "a newer capture measured no ball")
    assert dock.evidence(t + .13, ORIGIN, t + .1).status == "blocked"
    assert not dock.observe_row(row(t + .05), t + .13)[0]
    assert dock.observe_row(row(t + .2, state="conflict"), t + .21)[0]
    assert dock.observe_row(row(t + .3), t + .31) == (True, "tentative")


def test_box_methods_and_imprecise_fixes_cannot_dock():
    dock = docking()
    assert dock.observe(obs(1.0, method="eye_box"), 1.01) == (False, "method eye_box is not allowed for docking")
    assert dock.observe(obs(1.1, sigma=.05), 1.11) == (False, "image uncertainty exceeds the docking limit")


def test_commanded_motion_expires_local_body_offsets():
    dock = docking()
    t = confirmed(dock)
    assert agree(dock, t).status == "local"
    dock.note_command(t + .04, .1, 0., 0.)  # 0.1 m/s for 0.15 s: 15 mm of commanded travel
    dock.note_command(t + .19, 0., 0., 0.)
    assert dock.track.current(t + .2, dock.ledger) is None
    assert dock.track.reason == "body moved since the local capture" and dock.track.agreed is None
    evidence = dock.evidence(t + .2, ORIGIN, t + .19)
    assert evidence.status == "blocked" and "confirmed" in evidence.reason


def test_turning_between_captures_restarts_the_track():
    dock = docking()
    assert dock.observe(obs(1.0), 1.01)[0]
    dock.note_command(1.02, 0., 0., .5)
    dock.note_command(1.08, 0., 0., 0.)
    assert dock.observe(obs(1.1), 1.11) == (True, "tentative")


def test_unknown_motion_clears_history_and_refuses_captures_from_before_it():
    dock = LocalDocking(synthetic_profile(.11))
    assert dock.observe(obs(1.0), 1.01)[0]
    assert dock.observe(obs(1.1), 1.11) == (True, "tentative")  # no command history covers the gap
    dock = docking()
    t = confirmed(dock)
    dock.unknown_motion(t + .2, "kick clip moves the body")  # rows captured mid-clip arrive late
    assert dock.observe(obs(t + .15), t + .25) == (False, "capture precedes the last reset")
    assert not dock.ledger.segments
    assert dock.observe(obs(t + .3), t + .31) == (True, "tentative")
    assert dock.observe(obs(t + .4), t + .41) == (True, "tentative")  # still no motion history: moved
    dock.note_command(1.6, 0., 0., float("nan"))
    assert not dock.ledger.segments


def test_overhead_motion_step_or_slow_drift_since_capture_clears_the_track():
    dock = docking()
    t = confirmed(dock)
    assert agree(dock, t).status == "local"
    dock.note_overhead(Pose(.05, 0., 0.), t + .05)
    assert not dock.track.fixes and dock.track.reason == "overhead observed body motion"
    assert dock.observe(obs(t + .04), t + .06) == (False, "capture precedes the last reset")
    dock = docking()
    t = confirmed(dock)
    assert agree(dock, t).status == "local"
    for k, x in ((1, .025), (2, .05)):  # 2.5 cm per frame: under the step budget each time
        dock.note_overhead(Pose(x, 0., 0.), t + .05 * k)
    evidence = dock.evidence(t + .11, Pose(.05, 0., 0.), t + .1)
    assert evidence.status == "blocked" and evidence.reason == "overhead pose moved since the local capture"


def test_overhead_seeing_the_ball_move_after_the_local_capture_blocks():
    dock = docking()
    t = confirmed(dock)
    assert agree(dock, t).status == "local"
    dock.note_overhead(ORIGIN, t + .05, Ball(FORWARD + .12, LEFT), t + .05)
    evidence = dock.evidence(t + .06, ORIGIN, t + .05)
    assert evidence.status == "blocked" and "ball move" in evidence.reason


def test_recalibration_radius_and_camera_status_are_identity_checks():
    dock = docking()
    t = confirmed(dock)
    assert dock.observe(obs(t + .1, cal="c" * 64), t + .12) == (False, "camera profile changed within the track")
    assert not dock.track.fixes
    for k, radius in enumerate((.091, .06)):  # donor and simulator radii are not the selected 0.11 m
        assert dock.observe(obs(t + .2 + .01 * k, radius=radius), t + .25) == (
            False, "ball radius differs from the selected field radius")
    assert docking().observe(obs(1.0, status="estimated"), 1.01) == (
        False, "synthetic docking accepts only synthetic camera rows")
    pinned = docking(profile_id=PROFILE_ID, calibration_id=CAL_ID)
    assert not pinned.observe(obs(1.0, profile="d" * 64), 1.01)[0]
    qualified = DockingProfile("measured", "live", .11, PROFILE_ID, measured_error_m=.005,
                               radius_measured=True, mount_measured=True)
    assert qualified.problem() is None
    live = LocalDocking(qualified)
    assert live.observe(obs(1.0, status="estimated"), 1.01) == (False, "live docking requires a measured camera profile")
    assert live.observe(obs(1.1, status="measured", measured=True), 1.11)[0]


def test_live_profile_is_off_until_radius_mount_and_error_are_measured():
    field = Field(ball_radius=.11)
    profile = live_profile(field, profile_id=PROFILE_ID)
    assert profile.ball_radius_m == field.ball_radius
    assert "measured ball radius" in profile.problem() and "independent local error" in profile.problem()
    dock = LocalDocking(profile)
    dock.note_command(0., 0., 0., 0.)
    evidence = dock.evidence(1.03, ORIGIN, 1.0)
    assert evidence.status == "overhead" and evidence.reason.startswith("local docking unqualified")
    assert not evidence.synthetic
    too_coarse = DockingProfile("measured", "live", .11, PROFILE_ID, measured_error_m=.012,
                                radius_measured=True, mount_measured=True)
    assert "half the lateral contact window" in too_coarse.problem()
    assert LocalDocking(None).evidence(1., ORIGIN, 1.).reason == "local docking is not configured"


def test_local_ball_never_renews_a_stale_overhead_pose():
    dock = docking()
    t = confirmed(dock)
    assert agree(dock, t).status == "local"
    stale = dock.evidence(t + .03, ORIGIN, t - .2)
    assert stale.status == "blocked" and stale.reason == "overhead pose is stale; local ball cannot renew global pose"
    evidence = dock.evidence(t + .03, ORIGIN, t - .1)
    assert evidence.status == "local" and evidence.expires_at == pytest.approx(t - .1 + paw_approach.MAX_CAPTURE_GAP_S)
    assert evidence.synthetic and evidence.profile == "synthetic"


def test_disagreement_blocks_and_resets_while_hidden_ball_keeps_earned_agreement():
    dock = docking()
    t = confirmed(dock)
    evidence = agree(dock, t, ball=Ball(FORWARD + .25, LEFT))
    assert evidence.status == "blocked" and evidence.agreement_m == pytest.approx(.25)
    assert not dock.track.fixes
    dock = docking()
    t = confirmed(dock)
    assert dock.evidence(t + .02, ORIGIN, t).reason == "local track has not agreed with an overhead ball"
    assert agree(dock, t).status == "local"
    # The overhead ball can be hidden under the nose during docking; the same
    # still track keeps its earned agreement, a new track would not.
    assert dock.observe(obs(t + .1), t + .12)[0]
    dock.note_overhead(ORIGIN, t + .11)
    assert dock.evidence(t + .13, ORIGIN, t + .11).status == "local"


def test_local_kick_readiness_needs_steady_fixes_and_a_forecast_that_stays_on_the_paw():
    dock = docking()
    t = confirmed(dock)
    assert not dock.track.steady() and dock.track.velocity() is None
    assert dock.observe(obs(t + .1, x=FORWARD + .002), t + .11)[0]
    evidence = agree(dock, t + .1)
    assert evidence.steady and local_kick_ready(evidence, t + .13, 0., "left", forward_m=FORWARD)
    assert not local_kick_ready(evidence, t + .13, .2, "left", forward_m=FORWARD)  # heading still from overhead
    rolling = docking()
    for k in range(4):  # 2.5 mm per 0.1 s: steady by the 8 mm test, yet off the paw within the wind-up
        assert rolling.observe(obs(1. + .1 * k, x=FORWARD + .0025 * k), 1.01 + .1 * k)[0]
    evidence = agree(rolling, 1.3)
    assert evidence.status == "local" and evidence.steady
    assert evidence.velocity[0] > 0 and not local_kick_ready(evidence, 1.33, 0., "left", forward_m=FORWARD)


def test_body_window_matches_overhead_paw_window_for_the_same_offset():
    rng = random.Random(7)
    for _ in range(500):
        robot = Pose(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-math.pi, math.pi))
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        fwd, lat = FORWARD + rng.uniform(-.04, .04), LEFT + rng.uniform(-.03, .03)
        ball = Ball(robot.x + c * fwd - s * lat, robot.y + s * fwd + c * lat)
        heading = rng.uniform(-.1, .1)
        for side in ("left", "right"):
            for scale in (1., .55):
                expected = paw_approach.contact_window(robot, ball, heading, side, scale=scale, forward_m=FORWARD)
                assert body_contact_window(*overhead_body_offset(robot, ball), heading, side,
                                           forward_m=FORWARD, scale=scale) == expected


def test_radius_scaled_overhead_error_is_what_agreed_local_evidence_corrects():
    """Synthetic consumer case: the overhead ball sits 4.5 cm short (the range
    error a radius mismatch causes); the robot docks on it. An agreeing local
    measurement of the true offset shows the paw is outside the window."""
    robot = Pose(-FORWARD - .045, -LEFT, 0.)
    true_ball, overhead_ball = Ball(0., 0.), Ball(-.045, 0.)
    assert paw_approach.contact_window(robot, overhead_ball, 0., "left", forward_m=FORWARD)
    assert not paw_approach.contact_window(robot, true_ball, 0., "left", forward_m=FORWARD)
    dock = docking()
    truth = overhead_body_offset(robot, true_ball)
    t = confirmed(dock, x=truth[0], y=truth[1])
    evidence = agree(dock, t, robot=robot, ball=overhead_ball)
    assert evidence.status == "local" and evidence.agreement_m == pytest.approx(.045)
    assert not body_contact_window(evidence.forward, evidence.lateral, 0., "left", forward_m=FORWARD)
