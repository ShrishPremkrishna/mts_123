"""Geometry and honest scoring for camera-free destination trial preparation."""
import math

import pytest

from final_project.behaviors.tools.pose_trial import plan, score
from final_project.behaviors.tools.movement_trial import session_request
from final_project.pupper_fc.integration import Executor
from final_project.pupper_fc.model import Pose, Field


@pytest.mark.parametrize("start,target", [
    (Pose(0,0,0),Pose(.3,0,0)), (Pose(0,0,0),Pose(0,.3,0)),
    (Pose(0,0,0),Pose(-.3,0,math.pi)),
    (Pose(-.2,.1,1.2),Pose(.1,-.1,-3.0)),
    (Pose(0,0,3.1),Pose(0,0,-3.1)),
])
def test_plan_reuses_existing_requests_and_reaches_target_only_in_ideal_math(start,target):
    row = plan(start,target)
    x,y,yaw=start.x,start.y,start.yaw
    for step in row['steps']:
        session_request(step,'walk')
        if 'turn_angle_rad' in step:
            yaw+=step['turn_angle_rad']
        else:
            distance=step['speed']*step['duration']*row['distance_scale']
            x+=distance*math.cos(yaw);y+=distance*math.sin(yaw)
    assert (x,y)==pytest.approx((target.x,target.y))
    assert math.atan2(math.sin(yaw-target.yaw),math.cos(yaw-target.yaw))==pytest.approx(0)
    assert row['physical_arrival']=='unmeasured'
    assert row['hardware_executed'] is False


def test_same_target_contract_is_accepted_with_real_feedback_path():
    row=plan(Pose(0,0,0),Pose(.2,.1,.3))
    core=Executor({'pupper-1':Field()})
    assert core.handle(dict(event='arm',now=10,clock='test',epoch='trial'))['accepted']
    assert core.handle(dict(event='frame',now=10,clock='test',epoch='trial',seq=1,
                            captured_at=10,robots={'pupper-1':dict(x=0,y=0,yaw=0)},
                            ball=None,goals={}))['accepted']
    assert core.handle(dict(event='intent',now=10,clock='test',epoch='trial',seq=1,
                            robot_id='pupper-1',intent_id='one',issued_at=10,
                            valid_until=10.2,**row['tracked_action']))['accepted']
    assert core.tick(10.01)['pupper-1'].phase=='move'


@pytest.mark.parametrize('kwargs',[
    {'speed':.31},{'speed':float('nan')},{'distance_scale':0},{'calibration_note':''},
])
def test_invalid_calibration_and_limits_rejected(kwargs):
    with pytest.raises(ValueError):plan(Pose(0,0,0),Pose(.3,0,0),**kwargs)


def test_large_target_is_not_automatically_split_or_extended():
    with pytest.raises(ValueError,match='three-second'):plan(Pose(0,0,0),Pose(1.,0,0))
    with pytest.raises(ValueError,match='margin'):plan(Pose(0,0,0),Pose(1.8,0,0))


def test_precise_but_inaccurate_and_failed_attempts_remain_visible():
    rows=[dict(trial_id=str(i),outcome='completed',x=.2,y=.1,yaw=0) for i in range(3)]
    rows.append(dict(trial_id='failed',outcome='interrupted',x=None,y=None,yaw=None))
    result=score(Pose(.4,.1,0),rows)
    assert result['repeatability_rms_m']==pytest.approx(0)
    assert result['accuracy_rms_m']==pytest.approx(.2)
    assert result['attempts']==4 and result['unmeasured']==1


def test_wrap_and_duplicate_trial_ids():
    row=dict(trial_id='one',outcome='completed',x=0,y=0,yaw=-math.pi+.01)
    assert abs(score(Pose(0,0,math.pi-.01),[row])['trials'][0]['heading_error_deg'])<2
    with pytest.raises(ValueError):score(Pose(0,0,0),[row,row])


def test_position_and_heading_use_independent_measurements():
    rows = [
        dict(trial_id="tape", outcome="completed", x=.3, y=.4, yaw=None),
        dict(trial_id="both", outcome="interrupted", x=.6, y=.8, yaw=math.radians(179)),
        dict(trial_id="heading", outcome="completed", x=None, y=.9, yaw=math.radians(-179)),
        dict(trial_id="missing", outcome="refused"),
    ]
    result = score(Pose(0, 0, math.pi), rows)
    assert (result["attempts"], result["measured"], result["unmeasured"]) == (4, 2, 2)
    assert result["heading_measured"] == 2
    assert result["accuracy_rms_m"] == pytest.approx(math.sqrt((.5**2 + 1.0**2) / 2))
    assert result["repeatability_rms_m"] == pytest.approx(.25)
    assert result["mean_endpoint"] == pytest.approx(dict(x=.45, y=.6, yaw=math.pi))
    tape, both, heading, missing = result["trials"]
    assert tape["position_error_m"] == pytest.approx(.5) and not tape["heading_measured"]
    assert "heading_error_deg" not in tape
    assert both["outcome"] == "interrupted" and both["heading_error_deg"] == pytest.approx(-1)
    assert heading["heading_error_deg"] == pytest.approx(1) and not heading["measured"]
    assert "position_error_m" not in heading
    assert missing["outcome"] == "refused" and not missing["measured"] and not missing["heading_measured"]


def test_a_single_position_can_measure_accuracy_only():
    result = score(Pose(.2, 0, 0), [dict(trial_id="only", outcome="completed", x=.22, y=0)])
    assert result["accuracy_rms_m"] == pytest.approx(.02)
    assert result["repeatability_rms_m"] is None
    assert result["mean_endpoint"] == dict(x=.22, y=0, yaw=None)


def test_heading_only_and_unpaired_coordinates_do_not_create_an_endpoint():
    rows = [dict(trial_id="one", outcome="completed", x=.2, yaw=1.0),
            dict(trial_id="two", outcome="interrupted", y=.1)]
    result = score(Pose(.2, 0, 0), rows)
    assert result["measured"] == 0 and result["unmeasured"] == 2
    assert result["heading_measured"] == 1
    assert result["accuracy_rms_m"] is None and result["repeatability_rms_m"] is None
    assert result["mean_endpoint"] == dict(x=None, y=None, yaw=1.0)


def test_opposing_headings_have_no_mean_direction():
    rows = [dict(trial_id="east", outcome="completed", yaw=0),
            dict(trial_id="west", outcome="completed", yaw=math.pi)]
    result = score(Pose(0, 0, 0), rows)
    assert result["heading_measured"] == 2
    assert result["mean_endpoint"]["yaw"] is None


@pytest.mark.parametrize("field,value", [("x", math.nan), ("y", math.inf), ("yaw", True)])
def test_invalid_values_are_rejected_even_when_other_fields_are_missing(field, value):
    row = {"trial_id": "partial", "outcome": "interrupted", field: value}
    with pytest.raises(ValueError, match=field + " must be finite"):
        score(Pose(0, 0, 0), [row])
