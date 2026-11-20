"""The offline handoff has no transport and never touches an actuator."""

from dataclasses import replace
import pytest

from final_project.pupper_fc.kick_handoff import KickHandoff
from final_project.pupper_fc.model import Command, Decision


def tick(handoff, now, phase="kick_ready", *, epoch="e1", attempt_id="shot", side="right",
         permitted=True, vx=0.0, ttl=0.1):
    command = Command(vx=vx, expires_at=now + ttl, robot_id="p1")
    return handoff.step(Decision(phase, phase, command), now, epoch=epoch,
                        attempt_id=attempt_id, side=side, permitted=permitted)


@pytest.mark.parametrize("side", ["left", "right"])
def test_dispatch_once_and_hold_walk_until_ordered_correlated_receipts(side):
    handoff = KickHandoff("p1")
    first = tick(handoff, 1.0, side=side)
    job = first.request
    assert job.side == side and job.start_before == 1.1 and job.robot_id == "p1"
    assert first.decision.phase == "kick_in_flight"
    assert tick(handoff, 1.04, side=side).request is None
    assert tick(handoff, 1.08, "approach", side=side, vx=0.2).decision.command.vx == 0
    assert not handoff.receipt(replace(job, attempt_id="old"), "clip_confirmed", clip=f"kick_{side}")
    assert not handoff.receipt(replace(job, robot_id="p2"), "clip_confirmed", clip=f"kick_{side}")
    assert not handoff.receipt(job, "clip_confirmed", clip="stand_idle")
    assert handoff.receipt(job, "clip_confirmed", clip=f"kick_{side}")
    assert not handoff.receipt(job, "walker_restored", walker_active=False)
    assert handoff.receipt(job, "walker_restored", walker_active=True)
    assert not handoff.receipt(job, "walker_restored", walker_active=True)
    settled = tick(handoff, 1.10, side=side)
    assert settled.request is None and settled.decision.phase == "kick_complete"
    assert tick(handoff, 2.04, side=side, attempt_id="next").request != job


@pytest.mark.parametrize("phase", ["stopped", "tracking_lost", "obstacle_hold", "external_hold"])
def test_veto_retires_fired_attempt_and_requires_restoration_even_after_new_epoch(phase):
    handoff = KickHandoff("p1")
    job = tick(handoff, 1).request
    veto = tick(handoff, 1.01, phase, permitted=False, epoch=None, attempt_id=None)
    assert veto.cancel == job and veto.decision.phase == phase
    assert tick(handoff, 1.02, phase, permitted=False).cancel is None
    next_move = tick(handoff, 1.03, "move", epoch="e2", vx=0.2, permitted=False)
    assert next_move.decision.command.vx == 0
    assert not handoff.receipt(job, "clip_confirmed", clip="kick_right")
    assert handoff.receipt(job, "walker_restored", walker_active=True)
    assert tick(handoff, 1.04).decision.phase == "kick_cancelled"
    assert tick(handoff, 1.05, "move", epoch="e2", vx=0.2, permitted=False).decision.command.vx == 0.2
    assert tick(handoff, 1.06, epoch="e2").request != job


def test_timeout_requests_cancellation_without_releasing_the_legs():
    handoff = KickHandoff("p1", job_timeout_s=1)
    job = tick(handoff, 1, ttl=1.5).request
    timeout = tick(handoff, 2)
    assert timeout.cancel == job and timeout.decision.phase == "kick_cancelling"
    assert tick(handoff, 3, "move", attempt_id="new", vx=0.2).decision.command.vx == 0
    assert handoff.receipt(job, "walker_restored", walker_active=True)
    assert tick(handoff, 4).decision.phase == "kick_failed"


def test_failed_backend_request_requires_confirmed_walker_restoration():
    handoff = KickHandoff("p1")
    job = tick(handoff, 1).request
    assert handoff.receipt(job, "failed")
    assert tick(handoff, 1.1).cancel == job
    assert handoff.receipt(job, "walker_restored", walker_active=True)
    assert tick(handoff, 1.2).decision.phase == "kick_failed"


@pytest.mark.parametrize("kwargs", [{"permitted":False}, {"side": None}, {"attempt_id": None},
                                     {"vx": 0.1}, {"ttl": 0.0}, {"epoch": ""}])
def test_cannot_dispatch_from_an_unqualified_decision(kwargs):
    refused = tick(KickHandoff("p1"), 1, **kwargs)
    assert refused.request is None and refused.decision.phase == "kick_hold"
    assert refused.decision.command.vx == 0


def test_acquisition_requires_authority_past_the_actual_next_runner_tick():
    from final_project.behaviors.fc_executor_bridge import TICK_S

    handoff = KickHandoff("p1")
    for i in range(5):
        now = 1.0 + i * TICK_S
        # With only 19 ms on a 50 Hz output boundary, the next 20 ms check
        # would revoke the active job even if a new camera frame arrived.
        remaining = TICK_S if i % 2 else TICK_S - .001
        step = tick(handoff, now, ttl=remaining)
        assert step.request is None and step.cancel is None and handoff.active is None
        assert step.decision.phase == "kick_hold"
        assert step.decision.command.expires_at == now + remaining
    # Same epoch, same attempt, independently renewed authority. The gate
    # doesn't consume the shot. The asynchronous switch keeps its original deadline.
    now += TICK_S
    job = tick(handoff, now, ttl=TICK_S + .005).request
    assert job is not None and job.start_before == now + TICK_S + .005
    assert tick(handoff, now + TICK_S, ttl=.09).cancel is None


def test_executor_defers_late_settled_frames_then_dispatches_same_attack_on_timely_capture():
    from final_project.pupper_fc.integration import Executor
    from final_project.pupper_fc.tests.test_integration import arm, frame, intent

    robot_id = "pupper-1"
    ex, handoff = Executor(kick_side="left"), KickHandoff(robot_id)
    assert arm(ex)["accepted"]
    staged = {robot_id: dict(x=-.258, y=-.040, yaw=0.)}
    for seq in range(1, 13):
        stamp = seq / 27
        age = .135 if seq < 12 else .120
        now = stamp + age
        assert ex.handle(intent(now, seq))["accepted"]
        assert ex.handle(frame(now, seq, captured_at=stamp, robots=staged))["accepted"]
        step = ex.kick_step(handoff, now)
        if seq < 12:
            assert step.request is None and step.cancel is None and handoff.active is None
            if seq >= 8:
                assert step.decision.phase == "kick_hold"  # steady, 15 ms remains to switch
        else:
            assert step.request is handoff.active and handoff.active is not None
            assert step.request.issued_at == now
            assert step.request.start_before == pytest.approx(stamp + .15)


@pytest.mark.parametrize("restored_before_tick", [False, True])
def test_later_renewal_cannot_bridge_an_unobserved_gap(restored_before_tick):
    handoff = KickHandoff("p1")
    job = tick(handoff, 0.0, ttl=0.07).request
    assert handoff.receipt(job, "clip_confirmed", clip="kick_right")
    assert tick(handoff, 0.04, ttl=0.08).cancel is None
    if restored_before_tick:
        assert handoff.receipt(job, "walker_restored", walker_active=True)
    # Old evidence expired at .12. A fresh event arrived while the loop stalled.
    after_gap = tick(handoff, 0.13, ttl=0.15)
    if restored_before_tick:
        assert after_gap.cancel is None and after_gap.decision.phase == "kick_cancelled"
    else:
        assert after_gap.cancel == job and after_gap.decision.phase == "kick_cancelling"
        assert handoff.receipt(job, "walker_restored", walker_active=True)
    assert tick(handoff, 0.14).decision.phase == "kick_cancelled"

def test_terminal_restoration_before_clip_retires_unverified_job_without_blocking_walker():
    handoff = KickHandoff("p1")
    job = tick(handoff, 0).request
    assert handoff.receipt(job, "walker_restored", walker_active=True)
    assert not handoff.receipt(job, "clip_confirmed", clip="kick_right")
    terminal = tick(handoff, 0.01)
    assert terminal.decision.phase == "kick_failed" and terminal.request is None
    assert tick(handoff, 0.02, "move", vx=0.1, permitted=False).decision.command.vx == 0.1
    new_job = tick(handoff, 0.03, epoch="e2").request
    assert new_job is not None and not handoff.receipt(job, "clip_confirmed", clip="kick_right")
    assert not handoff.receipt(job, "walker_restored", walker_active=True)
    assert handoff.active == new_job


def test_repaired_track_cannot_erase_a_veto_observed_between_output_ticks():
    from final_project.pupper_fc.integration import Executor
    from final_project.pupper_fc.tests.test_integration import arm, frame, intent

    robot_id = "pupper-1"
    executor = Executor(kick_side="left")
    handoff = KickHandoff(robot_id)
    arm(executor)
    staged = {robot_id: dict(x=-0.258, y=-0.040, yaw=0)}
    executor.handle(intent(now=0, seq=1))
    job = None
    for seq in range(1, 8):
        now = (seq - 1) * .04
        assert executor.handle(frame(now, seq, robots=staged))["accepted"]
        step = executor.kick_step(handoff, now)
        if step.request is not None:
            job = step.request
    assert job is not None
    assert executor.handle(frame(.251, 8, robots=staged, ball=None))["accepted"]
    assert executor.handle(frame(.26, 9, robots=staged))["accepted"]
    step = executor.kick_step(handoff, .27)
    assert step.cancel == job and step.request is None
    assert step.decision.command.vx == step.decision.command.vy == step.decision.command.wz == 0
