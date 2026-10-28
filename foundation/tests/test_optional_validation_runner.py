"""Completed-stage identity, matched comparisons and failure preservation."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import venv

import numpy as np
import pytest

PATH = Path(__file__).parents[1] / 'scripts/colab_optional_validation.py'
spec = importlib.util.spec_from_file_location('optional_validation', PATH)
runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)


@pytest.fixture
def args(tmp_path):
    repo = tmp_path / 'repo'
    package = repo / 'pupper-mjlab/src/mjlab'
    (package / 'tasks/pupper_gait/mdp').mkdir(parents=True)
    (package / 'tasks/pupper_gait/mdp/gait_reference.py').write_text('# fixed stock references\n')
    (package / 'scripts').mkdir()
    (package / 'scripts/pupper_locomotion.py').write_text('DEFAULT_SEQUENCE = ' + repr([[i*.01,0,0] for i in range(16)]))
    refs = tmp_path / 'refs'
    refs.mkdir()
    (refs / 'captured_reach_gaits.npz').write_bytes(b'exact captured references')
    (refs / 'reference_manifest.json').write_text(json.dumps({'reference_sha256': runner.shared.sha(refs / 'captured_reach_gaits.npz'), 'frequency_mult': 2.}))
    hashes = {p.name: runner.shared.sha(p) for p in refs.iterdir()}
    checkpoints = {}
    provenance = {}
    for label, name, task, rh in [('baseline','mixed-baseline',runner.FLAT,{}),
                                  ('selected','bootstrap-1',runner.FLAT,hashes),
                                  ('bumpy','mixed-bumpy',runner.BUMPY,hashes)]:
        folder = tmp_path / name
        (folder / 'params').mkdir(parents=True)
        checkpoint = folder / 'model_4498.pt'; checkpoint.write_bytes(label.encode())
        checkpoints[label] = checkpoint
        for fn in ('policy.json','params/env.yaml','params/agent.yaml'):
            (folder / fn).write_text('{}')
        p = {'command':'train','out':f'/content/results/{name}','task':task,
             'status':'training_finished_not_evaluated','deployment_target_clamp':True,
             'profile':'baseline' if label=='baseline' else 'fast',
             'source_sha256':'a'*64, 'reference_dir':'/content/refs' if rh else None,
             'references':rh}
        if label == 'bumpy':
            p.update(checkpoint='/content/results/bootstrap-1/model_4498.pt',
                     checkpoint_sha256=runner.shared.sha(checkpoints['selected']))
        provenance[label] = p
        (folder / 'provenance.json').write_text(json.dumps(p))
    selection = {'smoke':False,'deployment_target_clamp':True,'stages':{
        label:{'status':'complete','returncode':0,'command':['-m','mjlab.scripts.pupper_locomotion','train','--out',p['out']]}
        for label,p in provenance.items()},'selected_flat':{
            'checkpoint':'/content/results/bootstrap-1/model_4498.pt','reference_dir':'/content/refs','profile':'fast'}}
    selection_file = tmp_path / 'batch_status.json'; selection_file.write_text(json.dumps(selection))
    return SimpleNamespace(repo=repo,python=Path(sys.executable),selection_file=selection_file,
        selected_checkpoint=checkpoints['selected'],baseline_checkpoint=checkpoints['baseline'],
        bumpy_checkpoint=checkpoints['bumpy'],selected_reference_dir=refs,scope='all',device='cuda:0')


def fake_runtime(plan, log):
    log.write_text('mocked preflight\n'); return {'fixture':True}


def evaluator(plan, *, fail=None, randomization_fault=None, protocol_fault=False):
    def execute(argv, repo, log):
        def arg(flag):return argv[argv.index(flag)+1]
        folder = Path(arg('--out'));folder.mkdir();log.write_text('retained evaluation log\n')
        name = folder.parent.name
        if name == fail:
            (folder / 'partial').write_text('partial retained');return 1
        job = next(j for j in plan['jobs'] if j['name']==name)
        p = {**runner.CONTRACT,'seed':job['seed'],'task':job['task'],
             'source_sha256':plan['source_sha256'],'checkpoint_sha256':job['checkpoint_sha256'],
             'references':job['references'], 'zero_policy':False, 'reset_each_command':False,
             'no_heading_hold':False, 'allow_out_of_range':False}
        if protocol_fault:p['compose_com_randomization']=True
        rows=[{'command':c,'failures':0,'timeouts':0,'valid_samples':32,'mean_xy_error_m_s':.1} for c in job['commands']]
        (folder / 'metrics.json').write_text(json.dumps({'segments':rows,'provenance':p}))
        (folder / 'provenance.json').write_text(json.dumps(p))
        (folder / 'env.yaml').write_text(f"task: {job['task']}\nseed: {job['seed']}\n")
        np.savez(folder / 'randomization_snapshot.npz',mass=[job['seed'] + (name==randomization_fault)])
        np.savez(folder / 'trace.npz',sample=[1])
        return 0
    return execute


def test_exact_seeds_tasks_references_and_legacy_contract_are_frozen(args):
    plan=runner.make_plan(args)
    assert len(plan['jobs'])==8 and plan['workload']['vector_control_steps']==64000
    assert [j['seed'] for j in plan['jobs']]==[404,404,202,202,303,303,404,404]
    for j in plan['jobs']:
        argv=j['argv_without_out']
        assert '--randomized' in argv and '--deployment-target-clamp' in argv
        assert '--compose-com-randomization' not in argv and '--locomotion-torso-contact' not in argv
        assert '--geometry-audit' not in argv and 'train' not in argv
        if j['group']=='bumpy':assert j['references'] and j['task']==runner.BUMPY
    assert not plan['automatic_promotion']


def test_flat_scope_needs_no_bumpy_artifacts(args):
    args.scope='flat';args.bumpy_checkpoint=None
    plan=runner.make_plan(args)
    assert len(plan['jobs'])==2 and 'bumpy' not in plan['inputs']


@pytest.mark.parametrize('fault',['selection_pending','unfinished','wrong_final_name','wrong_task','composed_com','driver_limits','reference_bytes','bumpy_warmstart'])
def test_unfinished_mismatched_or_changed_selection_is_refused(args,fault):
    if fault=='selection_pending':
        s=json.loads(args.selection_file.read_text());s.pop('selected_flat');args.selection_file.write_text(json.dumps(s))
    elif fault=='wrong_final_name':
        s=json.loads(args.selection_file.read_text());s['selected_flat']['checkpoint']=s['selected_flat']['checkpoint'].replace('4498','1');args.selection_file.write_text(json.dumps(s))
    elif fault=='reference_bytes':(args.selected_reference_dir / 'captured_reach_gaits.npz').write_bytes(b'changed')
    else:
        path=(args.bumpy_checkpoint if fault=='bumpy_warmstart' else args.selected_checkpoint).parent/'provenance.json'
        p=json.loads(path.read_text())
        if fault=='unfinished':p['status']='training_started'
        if fault=='wrong_task':p['task']=runner.BUMPY
        if fault=='composed_com':p['compose_com_randomization']=True
        if fault=='driver_limits':p['driver_limits']='local-class-driver-limits.json'
        if fault=='bumpy_warmstart':p['checkpoint_sha256']='0'*64
        path.write_text(json.dumps(p))
    with pytest.raises(ValueError):runner.make_plan(args)


def test_same_flat_policy_uses_one_job_with_explicit_evidence_alias(args):
    args.scope='flat';args.selected_checkpoint=args.baseline_checkpoint;args.selected_reference_dir=None
    s=json.loads(args.selection_file.read_text());s['selected_flat']={'checkpoint':'/content/results/mixed-baseline/model_4498.pt','reference_dir':None,'profile':'baseline'}
    args.selection_file.write_text(json.dumps(s))
    plan=runner.make_plan(args)
    assert len(plan['jobs'])==1 and plan['aliases']=={'flat-selected-s404':'flat-baseline-s404'}
    out=args.repo/'out'
    assert runner.run_plan(plan,out,execute=evaluator(plan),check_runtime=fake_runtime)==0
    summary=json.loads((out/'summary.json').read_text())
    assert summary['comparisons'][0]['same_evidence_not_independent_trial']
    assert not summary['automatic_promotion']


def test_interpreter_symlink_is_preserved(args,tmp_path):
    env=tmp_path/'venv';venv.EnvBuilder(with_pip=False,symlinks=True).create(env)
    args.python=env/'bin/python'
    plan=runner.make_plan(args)
    assert args.python.is_symlink() and plan['python']==str(args.python)
    assert all(j['argv_without_out'][0]==str(args.python) for j in plan['jobs'])


def test_realized_randomization_mismatch_withholds_comparison(args):
    plan=runner.make_plan(args);out=args.repo/'out'
    assert runner.run_plan(plan,out,execute=evaluator(plan,randomization_fault='bumpy-finetuned-s303'),check_runtime=fake_runtime)==1
    s=json.loads((out/'summary.json').read_text())
    mismatch=next(c for c in s['comparisons'] if c['group']=='bumpy' and c['seed']==303)
    assert not mismatch['realized_randomization_equal'] and 'left_metrics' not in mismatch
    assert s['errors'] and not s['automatic_promotion']


def test_retry_preserves_failure_and_skips_verified_completed_jobs(args):
    plan=runner.make_plan(args);out=args.repo/'out'
    assert runner.run_plan(plan,out,execute=evaluator(plan,fail='bumpy-finetuned-s202'),check_runtime=fake_runtime)==1
    partial=out/'jobs/bumpy-finetuned-s202/attempt-001/partial'
    calls=[];success=evaluator(plan)
    def retry(*a):calls.append(a);return success(*a)
    assert runner.run_plan(plan,out,resume=True,execute=retry,check_runtime=fake_runtime)==0
    assert len(calls)==1 and partial.read_text()=='partial retained'
    assert (out/'jobs/bumpy-finetuned-s202/attempt-002/metrics.json').exists()
    args.selection_file.write_text(args.selection_file.read_text()+'\n')
    with pytest.raises(ValueError,match='bytes changed'):
        runner.run_plan(plan,out,resume=True,execute=retry,check_runtime=fake_runtime)


def test_protocol_mismatch_cannot_be_called_completed(args):
    plan=runner.make_plan(args);out=args.repo/'out'
    assert runner.run_plan(plan,out,execute=evaluator(plan,protocol_fault=True),check_runtime=fake_runtime)==1
    status=json.loads((out/'status.json').read_text())
    assert all(items[-1]['state']=='failed' for items in status['jobs'].values())


@pytest.mark.parametrize('fault',['running','tampered_output'])
def test_resume_refuses_unknown_worker_or_changed_output_before_preflight(args,fault):
    args.scope='flat';plan=runner.make_plan(args);out=args.repo/'out'
    runner.run_plan(plan,out,execute=evaluator(plan),check_runtime=fake_runtime)
    status=json.loads((out/'status.json').read_text())
    if fault=='running':
        status['jobs']['flat-baseline-s404'][-1]['state']='running'
        (out/'status.json').write_text(json.dumps(status))
    else:
        (out/'jobs/flat-baseline-s404/attempt-001/metrics.json').write_text('{}')
    with pytest.raises(ValueError,match='still be running|output changed'):
        runner.run_plan(plan,out,resume=True,execute=lambda *a:pytest.fail('must not run'),
                        check_runtime=lambda *a:pytest.fail('must refuse before preflight'))
