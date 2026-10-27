"""Behavioral regressions for the repaired classical control and perception tools."""
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for relative in ('pd_control_lab', 'ik_heuristic_walking_lab', 'pupper-gait-tuner',
                 'ik_heuristic_walking_lab/extension', 'follow_me_pupper/lab', 'follow_me_pupper/robot'):
    sys.path.insert(0, str(ROOT / relative))
from control_math import sine_target, pd_torque, SampleDelay
from kinematics import LEG_FK, LEG_LIMITS, inverse_kinematics
from gait import GaitParams, LEGS, fk_batch, foot_position
from tuned_params_to_keyframes import keyframes_for_leg, nominal_offset
from lab_checks import FakeDetection, FakeTime, load_follow_me
import target_lock
import boxes

follow_me = load_follow_me()


def test_sine_velocity_is_derivative():
    for t in np.linspace(-2, 8, 30):
        q, v = sine_target(t, amplitude=.35, frequency=.7, center=.2)
        dq = (sine_target(t+1e-6, .35, .7, .2)[0]-sine_target(t-1e-6, .35, .7, .2)[0])/2e-6
        assert v == pytest.approx(dq, abs=1e-7)


def test_pd_sign_bounds_and_invalid_sensor():
    assert pd_torque(0, 0, 1, 0, 7, .5, 3) == 3
    assert pd_torque(0, 2, 0, 0, 7, .5, 3) == -1
    assert pd_torque(0, 0, 0, 0, 7, .5, 3) == 0
    assert pd_torque(float('nan'), 0, 0, 0, 7, .5, 3) == 0


def test_delay_zero_warmup_and_reset():
    assert SampleDelay(0, 200).push((1, 2)) == (1, 2)
    delay = SampleDelay(.01, 200)
    assert delay.push('first') is None
    assert delay.push('second') is None
    assert delay.push('third') == 'first'
    assert delay.push('fourth') == 'second'
    delay.clear()
    assert delay.push('new') is None
    with pytest.raises(ValueError):
        SampleDelay(-1, 200)


@pytest.mark.parametrize('i', range(4))
def test_scalar_batch_fk_agree_and_ik_respects_limits(i):
    rng = np.random.default_rng(100+i)
    sign = -1 if i in (1, 3) else 1
    for _ in range(12):
        q = np.array([rng.uniform(-.2, .2), sign*rng.uniform(.1, .5), -sign*rng.uniform(.1, .6)])
        target = LEG_FK[i](q)
        np.testing.assert_allclose(target, fk_batch(q[None], LEGS[i])[0], atol=2e-6)
        solved = inverse_kinematics(LEG_FK[i], target, max_iterations=150, tolerance=1e-4,
                                    strict=True, joint_limits=LEG_LIMITS[i])
        assert np.linalg.norm(LEG_FK[i](solved)-target) <= 1e-4
        assert np.all(solved >= LEG_LIMITS[i][:, 0])
        assert np.all(solved <= LEG_LIMITS[i][:, 1])


def test_unreachable_ik_rejected():
    with pytest.raises(ValueError, match='did not converge'):
        inverse_kinematics(LEG_FK[0], [2, 2, 2], max_iterations=100,
                           strict=True, joint_limits=LEG_LIMITS[0])
    with pytest.raises(ValueError):
        inverse_kinematics(LEG_FK[0], [float('nan'), 0, 0])


def test_cycloid_swing_touches_down_with_zero_velocity():
    p = GaitParams(swing_profile='cycloid')
    eps = 1e-6
    landing = foot_position(1-eps, 'front_r', p)
    previous = foot_position(1-2*eps, 'front_r', p)
    assert np.linalg.norm((landing-previous)/eps) < 1e-4
    for phase in [p.duty, (1+p.duty)/2, 1-1e-12]:
        np.testing.assert_allclose(foot_position(phase, 'front_r', p),
                                  foot_position(phase, 'front_r', GaitParams(swing_profile='sine')), atol=1e-10)


def test_keyframes_preserve_leg_phase_and_offset():
    p = GaitParams(pattern='trot')
    for leg in LEGS:
        keyframes = keyframes_for_leg(leg.name, p)
        assert keyframes.shape == (6, 3)
        assert np.isfinite(keyframes).all()
        # Diagonal pair shares foot-relative phase; other diagonal differs.
    np.testing.assert_allclose(keyframes_for_leg('front_r', p), keyframes_for_leg('back_l', p))
    assert not np.allclose(keyframes_for_leg('front_r', p), keyframes_for_leg('front_l', p))
    assert np.min(keyframes_for_leg('front_r', p)[:, 2]) == -p.body_height


def test_missing_color_cannot_reidentify_without_position():
    lock = target_lock.Lock()
    lock.color = np.array([200, 40, 40])
    unknown = FakeDetection(200, 100, 350, 400, class_id=0)
    assert target_lock.pick_target_by_color([unknown], lock) is None
    assert target_lock.pick_target_by_color([], lock) is None
    assert target_lock.pick_target([], lock) is None
    for text in ['nan,0,0', '300,0,0', 'inf,2,3', '-1,0,0']:
        unknown.id = text
        assert boxes.detection_color(unknown) is None


def test_stale_camera_cancels_detour_and_publishes_stop(monkeypatch):
    class Twist:
        def __init__(self):
            self.angular = SimpleNamespace(z=0.)
            self.linear = SimpleNamespace(x=0.)
    monkeypatch.setattr(follow_me, 'Twist', Twist)
    sent = []
    node = SimpleNamespace(tracking_enabled=True, state=follow_me.State.AVOID_PASS,
                           last_frame_time=FakeTime(10), obstacle_pos=.1,
                           get_clock=lambda: SimpleNamespace(now=lambda: FakeTime(12)),
                           command_publisher=SimpleNamespace(publish=sent.append))
    follow_me.StateMachineNode.timer_callback(node)
    assert len(sent) == 1
    assert sent[0].linear.x == 0 and sent[0].angular.z == 0
    assert node.state == follow_me.State.SEARCH
    assert node.obstacle_pos is None
