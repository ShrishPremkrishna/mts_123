"""Corrected contract, truthful process exits and recoverable artifact handling."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import venv

import pytest

SCRIPT=Path(__file__).parents[1]/'scripts/colab_optional_corrected.py'
spec=importlib.util.spec_from_file_location('optional_corrected_test',SCRIPT)
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)

@pytest.fixture
def args(tmp_path):
    repo=tmp_path/'repo';(repo/'pupper-mjlab/src/mjlab').mkdir(parents=True)
    (repo/'pupper-mjlab/src/mjlab/identity.py').write_text('value=1\n')
    (repo/'pupper-mjlab/configs/optional2').mkdir(parents=True)
    (repo/'pupper-mjlab/configs/optional2/mystery_experiment.json').write_text('{}')
    (repo/'foundation/configs').mkdir(parents=True)
    source=SCRIPT.parents[1]/'configs/class-driver-joint-limits.json'
    (repo/'foundation/configs/class-driver-joint-limits.json').write_bytes(source.read_bytes())
    folder=tmp_path/'initial';(folder/'params').mkdir(parents=True)
    cp=folder/'model_4498.pt';cp.write_bytes(b'exact checkpoint bytes')
    for name in ('policy.json','params/env.yaml','params/agent.yaml'):(folder/name).write_text('{}')
    (folder/'provenance.json').write_text(json.dumps({'task':runner.FLAT,'status':'training_finished_not_evaluated','profile':'baseline','reference_dir':None,'rewards':{'track_linear_velocity':10.,'track_yaw_velocity':7.}}))
    return SimpleNamespace(repo=repo,python=Path(sys.executable),out=tmp_path/'out',checkpoint=cp,
         profile='baseline',reference_dir=None,driver_limits=None,selection_file=None,
         phase='flat',iterations=None,min_free_gib=8.,device='cpu',smoke=False)


def corrected_selection(args):
    path=args.checkpoint.parent/'provenance.json';p=json.loads(path.read_text())
    p.update(deployment_target_clamp=True,compose_com_randomization=True,locomotion_torso_contact=True,
             driver_target_contract={'source':{'sha256':runner.shared.sha(args.repo/'foundation/configs/class-driver-joint-limits.json')}})
    path.write_text(json.dumps(p));args.selection_file=args.repo/'selection.json'
    args.selection_file.write_text(json.dumps({'checkpoint_sha256':runner.shared.sha(args.checkpoint),'profile':args.profile,'references':{}}))


def test_flat_plan_preserves_lineage_and_all_named_corrections(args):
    plan=runner.make_plan(args)
    assert plan['iterations']==1000 and plan['training_envs']==4096
    assert plan['flags']==['--deployment-target-clamp','--compose-com-randomization','--locomotion-torso-contact']
    assert plan['inputs']['checkpoint']['sha256']==runner.shared.sha(args.checkpoint)
    assert not plan['automatic_promotion'] and not plan['approved_for_hardware']


def test_recorded_reward_values_are_passed_explicitly_not_regenerated(args,monkeypatch):
    path=args.checkpoint.parent/'provenance.json';p=json.loads(path.read_text());p['rewards']['track_linear_velocity']=12.5;path.write_text(json.dumps(p))
    plan=runner.make_plan(args);worker=runner.Worker(plan,args.out)
    captured=[];monkeypatch.setattr(worker,'run',lambda name,kind,argv:captured.append(argv))
    worker.train('train',args.checkpoint,runner.FLAT,1000,None)
    weights=Path(captured[0][captured[0].index('--weights')+1])
    assert json.loads(weights.read_text())==p['rewards']==plan['lineage_rewards']

@pytest.mark.parametrize('phase,iterations',[('bumpy',1500),('mystery',500),('validate',1)])
def test_downstream_requires_frozen_selection_and_corrected_training(args,phase,iterations):
    args.phase=phase
    with pytest.raises(ValueError,match='Freeze'):runner.make_plan(args)
    corrected_selection(args);plan=runner.make_plan(args)
    assert plan['iterations']==iterations and 'selection' in plan['inputs']
    p=args.checkpoint.parent/'provenance.json';v=json.loads(p.read_text());v['compose_com_randomization']=False;p.write_text(json.dumps(v))
    with pytest.raises(ValueError,match='complete corrected'):runner.make_plan(args)

@pytest.mark.parametrize('fault',['unfinished','wrong_task','profile','references'])
def test_unfinished_or_mismatched_lineage_is_refused(args,fault):
    path=args.checkpoint.parent/'provenance.json';p=json.loads(path.read_text())
    if fault=='unfinished':p['status']='running'
    if fault=='wrong_task':p['task']=runner.BUMPY
    if fault=='profile':p['profile']='fast'
    if fault=='references':p.update(reference_dir='/content/refs',references={'captured_reach_gaits.npz':'a'*64})
    path.write_text(json.dumps(p))
    with pytest.raises(ValueError):runner.make_plan(args)


def test_smoke_is_explicit_and_interpreter_symlink_is_preserved(args,tmp_path):
    env=tmp_path/'venv';venv.EnvBuilder(with_pip=False,symlinks=True).create(env)
    args.python=env/'bin/python';args.smoke=True
    plan=runner.make_plan(args)
    assert plan['smoke'] and plan['iterations']==2 and plan['seconds']==.2
    assert plan['python']==str(args.python) and args.python.is_symlink()


def test_real_supervisor_records_actual_child_exit_and_refuses_overwrite(tmp_path):
    spec=tmp_path/'job.json'
    runner.shared.save(spec,{'argv':[sys.executable,'-c','import sys; print("child output"); sys.exit(7)'],'cwd':str(tmp_path),'environment':{}})
    result=runner.supervised.launch(spec,sys.executable)
    assert result['returncode']==7 and result['exit_observed_by']=='direct parent supervisor'
    assert result['job_sha256']==runner.shared.sha(spec)
    assert (tmp_path/'stdout.log').read_text().strip()=='child output'
    assert json.loads((tmp_path/'child.json').read_text())['argv'][-1].endswith('sys.exit(7)')
    again=subprocess.run([sys.executable,str(SCRIPT.with_name('supervised_job.py')),str(spec)],capture_output=True,text=True)
    assert again.returncode and 'overwrite' in again.stderr
    assert json.loads((tmp_path/'exit.json').read_text())['returncode']==7


def test_real_supervisor_completion_is_available_without_live_scheduler(tmp_path):
    spec=tmp_path/'job.json';runner.shared.save(spec,{'argv':[sys.executable,'-c','print("done")'],'cwd':str(tmp_path)})
    proc=subprocess.Popen([sys.executable,str(SCRIPT.with_name('supervised_job.py')),str(spec)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
    proc.communicate(timeout=10)
    assert proc.returncode==0 and json.loads((tmp_path/'exit.json').read_text())['returncode']==0
    assert not runner.attempt_live(tmp_path)


def test_worker_refuses_live_attempt_without_relaunch(args,monkeypatch):
    plan=runner.make_plan(args);worker=runner.Worker(plan,args.out)
    folder=args.out/'jobs/x/attempt-001';folder.mkdir(parents=True)
    worker.status['stages']['x']=[{'state':'running','directory':str(folder)}];worker.save()
    monkeypatch.setattr(runner,'attempt_live',lambda _:True)
    monkeypatch.setattr(runner.supervised,'launch',lambda *a:pytest.fail('must not duplicate a live job'))
    with pytest.raises(ValueError,match='live supervisor'):worker.run('x','evaluate',['unused'])


def test_orphan_before_child_record_is_detected_by_owned_group(args,monkeypatch):
    plan=runner.make_plan(args);worker=runner.Worker(plan,args.out,retry_incomplete=True)
    folder=args.out/'jobs/x/attempt-001';folder.mkdir(parents=True)
    runner.shared.save(folder/'supervisor.json',{'identity':{'pid':9999,'start_ticks':'1'},'process_group':9999})
    worker.status['stages']['x']=[{'state':'running','directory':str(folder)}];worker.save()
    monkeypatch.setattr(runner.supervised,'alive',lambda _:False)
    monkeypatch.setattr(runner.supervised,'group_alive',lambda group:group==9999)
    monkeypatch.setattr(runner.supervised,'launch',lambda *a:pytest.fail('unrecorded live child must not be duplicated'))
    assert not (folder/'child.json').exists()
    with pytest.raises(ValueError,match='live supervisor'):worker.run('x','evaluate',['unused'])


def test_exclusive_lock_refuses_two_schedulers_resuming_same_root(args):
    plan=runner.make_plan(args);first=runner.Worker(plan,args.out)
    with pytest.raises(BlockingIOError):runner.Worker(plan,args.out,resume=True)
    first.close();second=runner.Worker(plan,args.out,resume=True);second.close()


def test_worker_recovers_actual_exit_without_inventing_returncode(args,monkeypatch):
    plan=runner.make_plan(args);worker=runner.Worker(plan,args.out)
    folder=args.out/'jobs/x/attempt-001';folder.mkdir(parents=True);(folder/'output').mkdir()
    runner.shared.save(folder/'job.json',{'argv':['original']})
    runner.shared.save(folder/'exit.json',{'returncode':0,'job_sha256':runner.shared.sha(folder/'job.json'),'exit_observed_by':'direct parent supervisor'})
    worker.status['stages']['x']=[{'state':'running','directory':str(folder),'argv':['original']}];worker.save()
    monkeypatch.setattr(runner,'verify_output',lambda *a:{'actual_artifact':'exact-hash'})
    monkeypatch.setattr(runner.supervised,'launch',lambda *a:pytest.fail('completed child must not rerun'))
    assert worker.run('x','evaluate',['original'])==folder/'output'
    item=worker.status['stages']['x'][0]
    assert item['state']=='completed' and item['exit']['returncode']==0 and item['recovered_from_supervisor']


def test_unknown_stopped_attempt_requires_explicit_retry_and_new_directory(args,monkeypatch):
    plan=runner.make_plan(args);worker=runner.Worker(plan,args.out)
    folder=args.out/'jobs/x/attempt-001';folder.mkdir(parents=True)
    (folder/'partial').write_text('preserved')
    worker.status['stages']['x']=[{'state':'running','directory':str(folder),'argv':[]}];worker.save()
    with pytest.raises(ValueError,match='retry-incomplete'):worker.run('x','evaluate',['unused'])
    assert (folder/'partial').read_text()=='preserved'


def test_resume_refuses_source_input_and_contract_changes(args):
    plan=runner.make_plan(args);runner.Worker(plan,args.out)
    args.checkpoint.write_bytes(b'different weights')
    with pytest.raises(ValueError,match='identical'):runner.Worker(runner.make_plan(args),args.out,resume=True)


def test_captured_bundle_preserves_reach_bytes_and_rejects_mutation(args,tmp_path):
    worker=runner.Worker(runner.make_plan(args),args.out)
    capture=tmp_path/'capture';capture.mkdir();(capture/'captured_jump_gait.npz').write_bytes(b'jump');(capture/'jump_report.json').write_text('{}')
    refs=tmp_path/'refs';refs.mkdir();(refs/'captured_reach_gaits.npz').write_bytes(b'reach');(refs/'reference_manifest.json').write_text('{}')
    combined=worker.combined_jump_references(capture,refs)
    assert (combined/'captured_reach_gaits.npz').read_bytes()==b'reach'
    (combined/'captured_jump_gait.npz').write_bytes(b'changed')
    with pytest.raises(ValueError,match='bundle changed'):worker.combined_jump_references(capture,refs)
