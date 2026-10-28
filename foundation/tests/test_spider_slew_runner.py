import importlib.util
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]

@pytest.fixture
def runner(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'foundation/scripts'))
    spec=importlib.util.spec_from_file_location('slew_runner',ROOT/'foundation/scripts/colab_spider_slew.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_slew_arms_change_only_reward_width_and_output(runner, monkeypatch, tmp_path):
    monkeypatch.setattr(runner,'verify_source',lambda *args:None)
    monkeypatch.setattr(runner.original,'sha',lambda _:runner.original.CHECKPOINT_SHA)
    monkeypatch.setattr(runner.original.subprocess,'check_output',lambda *args,**kwargs:'pin\n')
    a,ca=runner.plan(ROOT,tmp_path/'model',tmp_path,'baseline','pin')
    b,cb=runner.plan(ROOT,tmp_path/'model',tmp_path,'precision','pin')
    pre,cp=runner.plan(ROOT,tmp_path/'model',tmp_path,'prefiltered','pin')
    assert ca==cb==cp
    assert ca[-2:]==['--target-slew-rad-s','15.0']
    first,second=a['training_command'],b['training_command']
    differences=[i for i,(x,y) in enumerate(zip(first,second,strict=True)) if x!=y]
    assert differences==[first.index('--output')+1,first.index('--linear-tracking-std')+1]
    assert pre['training_command'] is None and pre['training_iterations']==0
    assert pre['evaluation_seeds']==[101,202,303]
    assert first[first.index('--iterations')+1]=='1000'
    assert first[first.index('--num-envs')+1]=='4096'
    assert first[first.index('--seed')+1]=='42'
    assert all(not p['automatic_promotion'] for p in (a,b,pre))


def test_slew_driver_refuses_missing_or_different_physics(runner):
    valid=dict(contract='pupper_position_target_slew_v1',max_rate_rad_s=15.,max_step_s=.02,driver_limits_sha256='abc')
    runner.check_contract(valid,'abc')
    for block in (None,{**valid,'max_rate_rad_s':14},{**valid,'max_step_s':.021},{**valid,'driver_limits_sha256':'other'}):
        with pytest.raises(RuntimeError,match='physical contract'):runner.check_contract(block,'abc')
