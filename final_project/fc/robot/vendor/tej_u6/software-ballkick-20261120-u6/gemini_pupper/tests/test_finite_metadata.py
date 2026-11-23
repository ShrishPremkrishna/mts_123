"""Reject API timing/end-frame metadata that contradicts native motion playback."""
from pathlib import Path
import json
import sys
import pytest

ROBOT=Path(__file__).resolve().parents[1]/'robot'
sys.path.insert(0,str(ROBOT))
from tricks import load_tricks


@pytest.fixture
def export(tmp_path):
    source=ROBOT.parent/'tricks/sneeze/policy.json'
    data=json.loads(source.read_text())
    folder=tmp_path/'tricks/candidate';folder.mkdir(parents=True)
    (folder/'trick.yaml').write_text('type: policy\npolicy: policy.json\nstart_pose: standing\nend_pose: standing\n')
    def save():
        (folder/'policy.json').write_text(json.dumps(data))
    save()
    return folder,data,save


@pytest.mark.parametrize('bad',[None,True,False,119.0,'119',0,1,-1,118,120])
def test_metadata_frame_count_must_be_exact_integer_matching_actual_motion(export,bad):
    folder,data,save=export
    data['trick']['frames']=bad;save()
    with pytest.raises(ValueError,match='invalid trick playback metadata'):
        load_tricks(folder.parent)


def test_missing_frame_count_is_a_clear_pre_activation_refusal(export):
    folder,data,save=export;del data['trick']['frames'];save()
    with pytest.raises(ValueError,match='frames must be an integer'):
        load_tricks(folder.parent)


@pytest.mark.parametrize('fault',['missing_motion','missing_positions','dict_rows','ragged','scalar_row','nonfinite','string','boolean'])
def test_malformed_motion_cannot_supply_a_trusted_endpoint(export,fault):
    folder,data,save=export
    if fault=='missing_motion':del data['motion']
    elif fault=='missing_positions':del data['motion']['joint_pos']
    elif fault=='dict_rows':data['motion']['joint_pos']={str(i):r for i,r in enumerate(data['motion']['joint_pos'])}
    elif fault=='ragged':data['motion']['joint_pos'][0].pop()
    elif fault=='scalar_row':data['motion']['joint_pos'][0]=0
    else:data['motion']['joint_pos'][0][0]={'nonfinite':float('nan'),'string':'0','boolean':False}[fault]
    save()
    with pytest.raises(ValueError,match='invalid trick playback metadata'):
        load_tricks(folder.parent)


@pytest.mark.parametrize('field,bad',[('fps',0),('fps',True),('fps','50'),('fps',float('inf')),('fps',25),
 ('duration_s',False),('duration_s','2.38'),('duration_s',float('nan')),('duration_s',.01),('duration_s',100.)])
def test_timer_metadata_cannot_understate_or_overstate_native_playback(export,field,bad):
    folder,data,save=export;data['trick'][field]=bad;save()
    with pytest.raises(ValueError,match='invalid trick playback metadata'):
        load_tricks(folder.parent)


def test_actual_export_identity_and_all_supplied_catalog_endpoints_are_preserved(export):
    folder,data,save=export;before=(folder/'policy.json').read_bytes()
    trick=load_tricks(folder.parent)['candidate']
    assert trick.frame_count==len(data['motion']['joint_pos'])==119
    assert trick.duration==trick.frame_count/trick.fps
    assert (folder/'policy.json').read_bytes()==before
    catalog=load_tricks()
    assert catalog and all(t.frame_count>=2 for t in catalog.values() if t.kind=='policy')
