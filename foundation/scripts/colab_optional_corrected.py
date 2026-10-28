#!/usr/bin/env python3
"""Resumable corrected Optional2 experiments; each invocation owns one lineage.

Run flat baseline and fast arms in SEPARATE output roots after measuring device
memory. This worker does not allocate runtimes, update Git, stop other jobs or
install policies. Every train/evaluate uses controller+named driver limits,
composed COM and torso contact. Exact references and original reward profile are
frozen. Bumpy and Mystery require a separate frozen selection JSON containing
checkpoint_sha256, profile, and references (filename-to-SHA256 mapping).

Flat uses selection seed 101 only. Held-out evaluation is a separate invocation
(--phase validate) after the candidate choice is frozen. Bumpy compares before/
after on identical seeds 202/303/404; Mystery requires real flight capture before
its conditional second training stage. --smoke is explicitly infrastructure only.
"""
from __future__ import annotations
import argparse
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys


def load(name, filename):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(filename))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
shared=load('optional_corrected_shared','colab_gait_validation.py')
supervised=load('optional_corrected_supervised','supervised_job.py')
FLAT='Mjlab-MixedGaits-Flat-Pupper-v3';BUMPY='Mjlab-MixedGaits-Bumpy-Pupper-v3';MYSTERY='Mjlab-Mystery-Flat-Pupper-v3'
FLAGS=['--deployment-target-clamp','--compose-com-randomization','--locomotion-torso-contact']


def track(inputs,label,path):
    path=Path(path).expanduser().resolve()
    if not path.is_file():raise ValueError(f'Missing {label}: {path}')
    inputs[label]={'path':str(path),'sha256':shared.sha(path)};return path

def reference_hashes(directory):
    if directory is None:return {}
    return {p.name:shared.sha(p) for p in sorted(Path(directory).iterdir()) if p.is_file() and p.suffix in ('.npz','.json')}

def checkpoint(directory):
    paths=[p for p in directory.glob('model_*.pt') if p.stem.split('_')[-1].isdigit()]
    if not paths:raise ValueError(f'No final checkpoint in {directory}')
    return max(paths,key=lambda p:int(p.stem.split('_')[-1]))

def make_plan(args):
    repo=args.repo.expanduser().resolve();inputs={}
    python=Path(os.path.abspath((args.python or repo/'pupper-mjlab/.venv/bin/python').expanduser()))
    if not python.is_file():raise ValueError('Missing exact worker Python environment')
    initial=track(inputs,'checkpoint',args.checkpoint)
    provenance_path=track(inputs,'checkpoint_provenance',initial.parent/'provenance.json')
    provenance=json.loads(provenance_path.read_text())
    if provenance.get('task')!=FLAT or provenance.get('status')!='training_finished_not_evaluated':
        raise ValueError('Require completed MixedGaits-Flat input checkpoint with source provenance')
    if provenance.get('profile')!=args.profile:raise ValueError('Retain the input lineage reward profile')
    rewards=provenance.get('rewards')
    if (not isinstance(rewards,dict) or not rewards or any(not isinstance(k,str) or isinstance(v,bool)
            or not isinstance(v,(float,int)) or not math.isfinite(v) for k,v in rewards.items())):
        raise ValueError('Input provenance must record the exact finite reward weights')
    for name in ('policy.json','params/env.yaml','params/agent.yaml'):track(inputs,'checkpoint_'+name,initial.parent/name)
    refs=args.reference_dir.expanduser().resolve() if args.reference_dir else None
    hashes=reference_hashes(refs)
    if hashes!=provenance.get('references',{}) or bool(refs)!=bool(provenance.get('reference_dir')):
        raise ValueError('Exact original reference files must match checkpoint training provenance')
    if refs:
        if not {'captured_reach_gaits.npz','reference_manifest.json'}<=hashes.keys():raise ValueError('Missing reach reference capture or manifest')
        manifest=json.loads((refs/'reference_manifest.json').read_text())
        if manifest.get('reference_sha256')!=hashes['captured_reach_gaits.npz']:raise ValueError('Reference manifest/capture hash mismatch')
        for name in hashes:track(inputs,'reference_'+name,refs/name)
    limits=track(inputs,'driver_limits',args.driver_limits or repo/'foundation/configs/class-driver-joint-limits.json')
    data=json.loads(limits.read_text())
    if data.get('contract')!='local_class_driver_position_limits_v1' or len(data.get('joint_limits',{}))!=12:
        raise ValueError('Require the reviewed named local class-driver bounds contract')
    if args.phase in ('bumpy','mystery','validate'):
        if args.selection_file is None:raise ValueError('Freeze the candidate selection before downstream/held-out runs')
        selected=json.loads(track(inputs,'selection',args.selection_file).read_text())
        if (selected.get('checkpoint_sha256')!=shared.sha(initial) or selected.get('profile')!=args.profile or selected.get('references')!=hashes):
            raise ValueError('Input does not match frozen selection identity')
        if (not all(provenance.get(k) for k in ('deployment_target_clamp','compose_com_randomization','locomotion_torso_contact'))
                or provenance.get('driver_target_contract',{}).get('source',{}).get('sha256')!=shared.sha(limits)):
            raise ValueError('Downstream selection must come from the complete corrected physics contract')
    if args.iterations is not None and args.iterations<1:raise ValueError('Iterations must be positive')
    if args.min_free_gib<0 or not math.isfinite(args.min_free_gib):raise ValueError('Invalid memory threshold')
    mystery=track(inputs,'mystery_weights',repo/'pupper-mjlab/configs/optional2/mystery_experiment.json')
    for name in ('colab_optional_corrected.py','supervised_job.py','colab_gait_validation.py'):
        track(inputs,'script_'+name,Path(__file__).with_name(name))
    training_iterations=args.iterations or {'flat':1000,'bumpy':1500,'mystery':500,'validate':1}[args.phase]
    return {'version':1,'phase':args.phase,'repo':str(repo),'python':str(python),'device':args.device,
            'inputs':inputs,'source_sha256':shared.source_identity(repo),'checkpoint':str(initial),
            'profile':args.profile,'references':hashes,'reference_dir':str(refs) if refs else None,
            'lineage_rewards':rewards,
            'driver_limits':str(limits),'flags':FLAGS,'iterations':2 if args.smoke else training_iterations,
            'training_envs':2 if args.smoke else 4096,'evaluation_envs':2 if args.smoke else 32,
            'seconds':.2 if args.smoke else 10.,'warmup':0. if args.smoke else 2.,
            'training_seed':42,'smoke':args.smoke,'min_free_gib':args.min_free_gib,
            'automatic_promotion':False,'approved_for_hardware':False}


def verify_inputs(plan):
    shared.verify_inputs(plan)

def runtime_preflight(plan,out):
    report=shared.runtime_preflight(plan,out/'runtime_preflight.log')
    probe='import json,torch,sys; d=torch.device(sys.argv[1]); print(json.dumps(dict(zip(("free_bytes","total_bytes"),torch.cuda.mem_get_info(d)))) if d.type=="cuda" else json.dumps({"device":"cpu"}))'
    result=subprocess.run([plan['python'],'-B','-c',probe,plan['device']],capture_output=True,text=True,timeout=60,check=True)
    memory=json.loads(result.stdout.strip().splitlines()[-1]);report['memory_before_worker']=memory
    if memory.get('free_bytes',float('inf'))<plan['min_free_gib']*2**30:
        raise RuntimeError('Insufficient measured free GPU memory; no stage launched')
    shared.save(out/'runtime.json',report);return report

def artifact_hashes(directory,kind):
    if kind=='evaluate':
        result=shared.artifact_hashes(directory)
        if (directory/'torso_geometry_audit.json').is_file():result['torso_geometry_audit.json']=shared.sha(directory/'torso_geometry_audit.json')
        return result
    if kind=='train':
        names=['provenance.json','policy.json','params/env.yaml','params/agent.yaml',str(checkpoint(directory).relative_to(directory))]
        names += [str(p.relative_to(directory)) for p in sorted(directory.glob('*.onnx'))]
        if not any(n.endswith('.onnx') for n in names):raise ValueError('Missing final ONNX export')
        if (directory/'randomization_snapshot.npz').is_file():names.append('randomization_snapshot.npz')
    else:names=['captured_jump_gait.npz','jump_report.json']
    return {name:shared.sha(directory/name) for name in names}


def verify_output(plan,argv,directory,kind):
    if kind=='capture':return artifact_hashes(directory,kind)
    p=json.loads((directory/'provenance.json').read_text())
    def arg(key):return argv[argv.index(key)+1]
    checks={'task':arg('--task'),'source_sha256':plan['source_sha256'],
            'checkpoint_sha256':shared.sha(Path(arg('--checkpoint'))),'deployment_target_clamp':True,
            'compose_com_randomization':True,'locomotion_torso_contact':True}
    if any(p.get(k)!=v for k,v in checks.items()):raise ValueError('Output source/input/physics contract differs from frozen plan')
    if p.get('driver_target_contract',{}).get('source',{}).get('sha256')!=plan['inputs']['driver_limits']['sha256']:
        raise ValueError('Output driver-limit bytes differ from frozen plan')
    refs=reference_hashes(arg('--reference-dir') if '--reference-dir' in argv else None)
    if p.get('references',{})!=refs:raise ValueError('Output references differ from stage inputs')
    if kind=='train':
        if p.get('status')!='training_finished_not_evaluated' or p.get('iterations')!=int(arg('--iterations')):
            raise ValueError('Training did not finish the requested iterations')
        expected_weights=json.loads(Path(plan['inputs']['mystery_weights']['path']).read_text()) if p['task']==MYSTERY else plan['lineage_rewards']
        if p.get('profile')!=plan['profile'] or p.get('rewards')!=expected_weights:
            raise ValueError('Training changed the frozen reward profile/weights')
    else:
        metrics=json.loads((directory/'metrics.json').read_text())
        if metrics.get('provenance')!=p or len(metrics.get('segments',[]))!=16:raise ValueError('Evaluation provenance or command sequence incomplete')
        for key,flag in (('seed','--seed'),('num_envs','--num-envs'),('seconds','--seconds'),('warmup','--warmup')):
            if p.get(key)!=float(arg(flag)):raise ValueError('Evaluation settings changed')
        if not p.get('randomized'):raise ValueError('Evaluation must use the same training randomization configuration')
    return artifact_hashes(directory,kind)


def attempt_live(directory):
    for filename,key in (('launcher.json','supervisor_identity'),('supervisor.json','identity'),('child.json','identity')):
        path=directory/filename
        if path.is_file():
            record=json.loads(path.read_text())
            if supervised.alive(record.get(key)):return True
            if record.get('process_group') and supervised.group_alive(record['process_group']):return True
    return False


class Worker:
    def __init__(self,plan,out,resume=False,retry_incomplete=False):
        self.plan,self.out,self.retry=plan,out.resolve(),retry_incomplete
        if not resume:self.out.mkdir(parents=True,exist_ok=False)
        self.lock=(self.out/'.scheduler.lock').open('a')
        try:
            fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if resume:
                if json.loads((self.out/'plan.json').read_text())!=plan:raise ValueError('Resume requires identical inputs/source/contract')
                self.status=json.loads((self.out/'status.json').read_text())
            else:
                shared.save(self.out/'plan.json',plan)
                self.status={'state':'prepared','stages':{},'created_utc':shared.utc_now()}
            weights=self.out/'lineage-weights.json';expected=json.dumps(plan['lineage_rewards'],indent=2,allow_nan=False)+'\n'
            if weights.exists() and weights.read_text()!=expected:raise ValueError('Frozen lineage reward file changed')
            if not weights.exists():weights.write_text(expected)
        except BaseException:self.close();raise
        self.save()
    def close(self):
        if getattr(self,'lock',None) is not None:self.lock.close();self.lock=None
    def __del__(self):self.close()
    def save(self):
        self.status['updated_utc']=shared.utc_now();shared.save(self.out/'status.json',self.status)
    def run(self,name,kind,argv):
        verify_inputs(self.plan);attempts=self.status['stages'].setdefault(name,[])
        if kind=='train':
            requested=json.loads(Path(argv[argv.index('--weights')+1]).read_text())
            task=argv[argv.index('--task')+1]
            expected=json.loads(Path(self.plan['inputs']['mystery_weights']['path']).read_text()) if task==MYSTERY else self.plan['lineage_rewards']
            if requested!=expected:raise ValueError('Stage weights differ from frozen plan')
        if attempts:
            prior=attempts[-1];folder=Path(prior['directory']);output=folder/'output'
            if prior['state']=='completed':
                if artifact_hashes(output,kind)!=prior['artifact_sha256']:raise ValueError('Completed artifacts changed; preserve and use a new root')
                return output
            if attempt_live(folder):raise ValueError('Previous attempt still owns a live supervisor/child; do not duplicate it')
            exit_path=folder/'exit.json'
            if exit_path.is_file() and json.loads(exit_path.read_text()).get('returncode')==0:
                if json.loads(exit_path.read_text())['job_sha256']!=shared.sha(folder/'job.json'):raise ValueError('Recovered job bytes changed')
                prior['artifact_sha256']=verify_output(self.plan,prior['argv'],output,kind)
                prior.update(state='completed',exit=json.loads(exit_path.read_text()),recovered_from_supervisor=True);self.save();return output
            if prior['state']=='running' and not self.retry:raise ValueError('Unknown prior attempt needs explicit --retry-incomplete after confirming both processes stopped')
        folder=self.out/'jobs'/name/f'attempt-{len(attempts)+1:03d}';folder.mkdir(parents=True)
        output=folder/'output';command=[*argv,str(output)] if kind=='capture' else [*argv,'--out',str(output)]
        item={'state':'running','kind':kind,'directory':str(folder),'argv':command,'started_utc':shared.utc_now()};attempts.append(item);self.save()
        environment={'PYTHONUNBUFFERED':'1','WARP_CACHE_PATH':str(self.out/'cache/warp'),
                     'MPLCONFIGDIR':str(self.out/'cache/matplotlib'),'XDG_CACHE_HOME':str(self.out/'cache/general')}
        spec={'argv':command,'cwd':self.plan['repo'],'environment':environment}
        shared.save(folder/'job.json',spec)
        try:
            result=supervised.launch(folder/'job.json',self.plan['python']);item['exit']=result
            if result['returncode']!=0:raise RuntimeError(f'{name} exited {result["returncode"]}; see {folder}/stdout.log')
            verify_inputs(self.plan);item['artifact_sha256']=verify_output(self.plan,command,output,kind);item['state']='completed'
        except BaseException as exc:
            item.update(state='failed' if (folder/'exit.json').exists() else 'running',error=f'{type(exc).__name__}: {exc}');raise
        finally:self.save()
        return output

    def evaluate(self,name,checkpoint_path,task,seed,refs):
        p=self.plan;argv=[p['python'],'-B','-m','mjlab.scripts.pupper_locomotion','evaluate','--task',task,
            '--checkpoint',str(checkpoint_path),'--device',p['device'],'--seed',str(seed),'--num-envs',str(p['evaluation_envs']),
            '--seconds',str(p['seconds']),'--warmup',str(p['warmup']),'--randomized',*FLAGS,'--driver-limits',p['driver_limits']]
        if refs:argv+=['--reference-dir',str(refs)]
        if task!=BUMPY:argv+=['--geometry-audit']
        return self.run(name,'evaluate',argv)
    def train(self,name,initial,task,iterations,refs,weights=None):
        p=self.plan;argv=[p['python'],'-B','-m','mjlab.scripts.pupper_locomotion','train','--task',task,
            '--checkpoint',str(initial),'--device',p['device'],'--seed',str(p['training_seed']),
            '--num-envs',str(p['training_envs']),'--iterations',str(iterations),'--profile',p['profile'],
            '--logger','tensorboard',*FLAGS,'--driver-limits',p['driver_limits']]
        if refs:argv+=['--reference-dir',str(refs)]
        argv+=['--weights',str(weights or self.out/'lineage-weights.json')]
        return self.run(name,'train',argv)
    def compare(self,name,left,right):
        import numpy as np
        a=json.loads((left/'metrics.json').read_text());b=json.loads((right/'metrics.json').read_text())
        with np.load(left/'randomization_snapshot.npz',allow_pickle=False) as x,np.load(right/'randomization_snapshot.npz',allow_pickle=False) as y:
            equal=set(x.files)==set(y.files) and all(np.array_equal(x[k],y[k]) for k in x.files)
        if not equal or shared.sha(left/'env.yaml')!=shared.sha(right/'env.yaml'):
            raise ValueError('Matched comparison refused: realized randomization or environment differs')
        report={'left':str(left),'right':str(right),'realized_randomization_equal':True,'environment_equal':True,
                'left_segments':a['segments'],'right_segments':b['segments'],'automatic_promotion':False}
        shared.save(self.out/f'{name}.json',report)

    def combined_jump_references(self,capture,original):
        source_files={p.name:p for p in capture.iterdir() if p.suffix in ('.npz','.json')}
        if original:
            source_files.update({name:original/name for name in ('captured_reach_gaits.npz','reference_manifest.json')})
        expected={name:shared.sha(path) for name,path in source_files.items()}
        target=self.out/'captured-reference-bundle'
        if target.exists():
            if reference_hashes(target)!=expected:raise ValueError('Captured reference bundle changed; preserve and use a new root')
        else:
            target.mkdir()
            for name,path in source_files.items():shutil.copy2(path,target/name)
        self.status['captured_reference_bundle']={'directory':str(target),'references':expected};self.save()
        return target


def missing_jump_audit_fields(trace_path):
    """Distinguish unaudited telemetry from measured absence of a viable jump."""
    import numpy as np
    required={'robot_ground_contact_count','robot_com_height_m','robot_com_vertical_velocity_m_s',
              'audited_foot_contact','contact_audit_metadata_json'}
    with np.load(trace_path,allow_pickle=False) as trace:
        return sorted(required-set(trace.files))


def execute(plan,out,resume=False,retry_incomplete=False):
    worker=Worker(plan,out,resume,retry_incomplete)
    initial=Path(plan['checkpoint']);refs=Path(plan['reference_dir']) if plan['reference_dir'] else None
    phase=plan['phase'];task={'flat':FLAT,'bumpy':BUMPY,'mystery':MYSTERY,'validate':FLAT}[phase]
    try:
        verify_inputs(plan);runtime_preflight(plan,worker.out)
        worker.status['state']='running';worker.save()
        if phase=='validate':
            for seed in (202,303,404):worker.evaluate(f'heldout-s{seed}',initial,task,seed,refs)
        else:
            seeds=(202,303,404) if phase=='bumpy' else (101,)
            before={seed:worker.evaluate(f'pre-s{seed}',initial,task,seed,refs) for seed in seeds}
            weights=plan['inputs']['mystery_weights']['path'] if phase=='mystery' else None
            trained=worker.train('train',initial,task,plan['iterations'],refs,weights);final=checkpoint(trained)
            for seed in seeds:
                after=worker.evaluate(f'post-s{seed}',final,task,seed,refs);worker.compare(f'paired-s{seed}',before[seed],after)
            if phase=='mystery':
                trace=after/'trace.npz'
                missing=missing_jump_audit_fields(trace)
                if missing:
                    worker.status['capture_audit_required']={
                        'trace':str(trace),'trace_sha256':shared.sha(trace),'evaluation_directory':str(after),
                        'missing_fields':missing,
                        'reason':'Ordinary rollout telemetry lacks full-body COM/contact audit. This does not show that the policy cannot jump.',
                        'next_step':'Audit unchanged source poses with foundation/scripts/audit_jump_capture_contacts.py, run the strict bootstrap jump gate, review loader timing, then use a separate bounded continuation with exact audited references.'}
                    worker.status['candidate']={'checkpoint':str(final),'checkpoint_sha256':shared.sha(final),
                        'task':task,'profile':plan['profile'],'references':reference_hashes(refs),
                        'reference_dir':str(refs) if refs else None,'requires_review':True,'hardware_validated':False}
                    worker.status['state']='finished_requires_capture_audit';worker.save();return 0
                if worker.status.get('jump_capture_outcome'):
                    worker.status['state']='finished_with_capability_gap';worker.save();return 0
                capture_argv=[plan['python'],'-B','-m','mjlab.tasks.pupper_gait.bootstrap','jump',str(after/'trace.npz'),'--auto']
                try:
                    capture=worker.run('capture','capture',capture_argv)
                except RuntimeError:
                    attempt=worker.status['stages']['capture'][-1]
                    log=Path(attempt['directory'])/'stdout.log'
                    if 'No clean jump with loading' not in log.read_text():raise
                    worker.status['jump_capture_outcome']='Complete audited telemetry contained no capture satisfying the physical/model-bound gate; no captured continuation was run.'
                    worker.status['state']='finished_with_capability_gap';worker.save();return 0
                refs=worker.combined_jump_references(capture,refs)
                trained=worker.train('captured-train',final,task,2 if plan['smoke'] else 1000,refs,weights)
                final=checkpoint(trained)
                for seed in (101,202,303,404):worker.evaluate(f'captured-post-s{seed}',final,task,seed,refs)
            worker.status['candidate']={'checkpoint':str(final),'checkpoint_sha256':shared.sha(final),
                'task':task,'profile':plan['profile'],'references':reference_hashes(refs),'reference_dir':str(refs) if refs else None,
                'requires_review':True,'hardware_validated':False}
        worker.status['state']='finished_requires_review'
        worker.save();return 0
    except BaseException as exc:
        worker.status.update(state='failed_or_interrupted',error=f'{type(exc).__name__}: {exc}');worker.save();raise
    finally:worker.close()


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--python',type=Path)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--reference-dir',type=Path);p.add_argument('--driver-limits',type=Path);p.add_argument('--selection-file',type=Path)
    p.add_argument('--profile',choices=('baseline','fast'),required=True);p.add_argument('--phase',choices=('flat','bumpy','mystery','validate'),default='flat')
    p.add_argument('--iterations',type=int);p.add_argument('--device',default='cuda:0');p.add_argument('--min-free-gib',type=float,default=8.)
    p.add_argument('--smoke',action='store_true');p.add_argument('--plan-only',action='store_true')
    p.add_argument('--resume',action='store_true');p.add_argument('--retry-incomplete',action='store_true')
    args=p.parse_args()
    if args.retry_incomplete and not args.resume:p.error('--retry-incomplete requires --resume')
    plan=make_plan(args)
    if args.plan_only:print(json.dumps(plan,indent=2));return 0
    return execute(plan,args.out,args.resume,args.retry_incomplete)

if __name__=='__main__':raise SystemExit(main())
