"""A contract change must never silently reuse a completed legacy stage."""
import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('colab_optional_resume',Path(__file__).parents[1]/'scripts/colab_optional.py')
batch=importlib.util.module_from_spec(spec);spec.loader.exec_module(batch)


def baseline(tmp_path,task='Mjlab-MixedGaits-Flat-Pupper-v3',status='training_finished_not_evaluated'):
    checkpoint=tmp_path/'model_2999.pt';checkpoint.write_bytes(b'preserved baseline checkpoint')
    (tmp_path/'provenance.json').write_text(json.dumps(dict(task=task,status=status)))
    return checkpoint


def test_finished_exact_task_required_before_warm_start(tmp_path):
    checkpoint=baseline(tmp_path,task='Mjlab-StableGait-Flat-Pupper-v3')
    with pytest.raises(ValueError,match='exact MixedGaits'):batch.baseline_configuration(checkpoint,1500)
    checkpoint=baseline(tmp_path,status='training_started')
    with pytest.raises(ValueError,match='finish'):batch.baseline_configuration(checkpoint,1500)
    checkpoint=baseline(tmp_path)
    config=batch.baseline_configuration(checkpoint,1500)
    assert config['checkpoint_sha256']==batch.file_sha256(checkpoint)
    assert config['iterations']==1500


def test_resume_requires_same_clip_checkpoint_bytes_and_iteration_budget(tmp_path):
    checkpoint=baseline(tmp_path)
    config=batch.baseline_configuration(checkpoint,1500)
    previous=dict(deployment_target_clamp=True,baseline_configuration=config)
    batch.validate_resume_contract(previous,True,config)
    with pytest.raises(ValueError,match='fresh'):batch.validate_resume_contract(previous,False,config)
    with pytest.raises(ValueError,match='COM randomization'):
        batch.validate_resume_contract(previous,True,config,compose_com=True)
    with pytest.raises(ValueError,match='iterations'):
        batch.validate_resume_contract(previous,True,batch.baseline_configuration(checkpoint,3000))
    checkpoint.write_bytes(b'replaced checkpoint with the same filename')
    with pytest.raises(ValueError,match='checkpoint/hash'):
        batch.validate_resume_contract(previous,True,batch.baseline_configuration(checkpoint,1500))


def test_old_batch_and_marker_cannot_be_relabelled_as_new_contract(tmp_path):
    default=batch.baseline_configuration(None,3000)
    batch.validate_resume_contract({},False,default)
    with pytest.raises(ValueError,match='fresh'):batch.validate_resume_contract({},True,default)
    marker=tmp_path/'mixed-baseline.done.json'
    marker.write_text(json.dumps(dict(command=['train','--iterations','3000'],returncode=0)))
    assert batch.completed_stage(marker,['train','--iterations',3000])['returncode']==0
    with pytest.raises(ValueError,match='fresh'):
        batch.completed_stage(marker,['train','--iterations',3000,'--deployment-target-clamp'])
