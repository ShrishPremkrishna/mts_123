import importlib.util
from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'foundation/scripts'))
import colab_spider_torso as runner


def test_matched_plan_changes_only_reward_weight(monkeypatch,tmp_path):
    monkeypatch.setattr(runner,'verify_source',lambda *a:None)
    checkpoint=tmp_path/'model_6497.pt';checkpoint.touch()
    monkeypatch.setattr(runner.original,'sha',lambda p:runner.CHECKPOINT_SHA if Path(p)==checkpoint else 'd'*64)
    control,common=runner.plan(ROOT,checkpoint,tmp_path,'control','a'*40)
    torso,other=runner.plan(ROOT,checkpoint,tmp_path,'torso','a'*40)
    assert common==other and control['torso_support_weight']==0 and torso['torso_support_weight']==-2
    normalize=lambda command:[str(v).replace('/control/','/ARM/').replace('/torso/','/ARM/') for v in command]
    a,b=normalize(control['training_command']),normalize(torso['training_command']);i=a.index('--torso-support-weight')+1
    assert a[:i]==b[:i] and a[i+1:]==b[i+1:]
    assert a[a.index('--iterations')+1]=='1000' and a[a.index('--num-envs')+1]=='4096'
    probe,_=runner.plan(ROOT,checkpoint,tmp_path,'probe','a'*40)
    assert probe['training_command'] is None and probe['training_iterations']==0


def test_wrong_input_refused(monkeypatch,tmp_path):
    monkeypatch.setattr(runner,'verify_source',lambda *a:None)
    monkeypatch.setattr(runner.original,'sha',lambda p:'0'*64)
    with pytest.raises(ValueError,match='exact'):runner.plan(ROOT,tmp_path/'wrong',tmp_path,'control','a'*40)
