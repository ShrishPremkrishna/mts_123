#!/usr/bin/env python3
"""Second-G4 batch: Optional1 supervised work and measured Optional2 experiments.

Use uploaded --dataset ZIP/directory. This script never connects to robot hardware.
Every subprocess has its own log. Check batch_status.json while a detached worker runs.
"""
from __future__ import annotations
import argparse
import hashlib
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--out',type=Path,default=Path('/content/pupper-optional-results'))
    p.add_argument('--dataset',type=Path,help='Uploaded tactile_data.zip or extracted tactile_data directory')
    p.add_argument('--dataset-drive-id',default='1-Jcldkcl7cbHorDAKQsJM5b5Xl2mlVP-',help='Public course ZIP, used only when --dataset is absent')
    p.add_argument('--mode',choices=['all','supervised','locomotion'],default='all')
    p.add_argument('--skip-implicit',action='store_true',help='Use when the local full implicit run already completed')
    p.add_argument('--smoke',action='store_true',help='Explicitly limited infrastructure tests, never final results')
    p.add_argument('--bootstrap-rounds',type=int,default=2)
    p.add_argument('--device',default='cuda:0')
    p.add_argument('--wandb',action=argparse.BooleanOptionalAction,default=True)
    args=p.parse_args()
    if not 0<=args.bootstrap_rounds<=3: p.error('bootstrap-rounds must be between0and3')
    args.repo=args.repo.resolve();args.out=args.out.resolve();args.out.mkdir(parents=True,exist_ok=True)
    os.chdir(args.repo)
    sys.path.insert(0,str(args.repo))
    env=os.environ.copy();env.pop('PUPPER_REFERENCE_DIR',None)
    env.setdefault('MPLCONFIGDIR',str(args.out/'cache/matplotlib'))
    env.setdefault('WARP_CACHE_PATH',str(args.out/'cache/warp'))
    env.setdefault('TORCH_HOME',str(args.out/'cache/torch'))
    source_paths=sorted(p for base in ('implicit_compliance','pupper-mjlab/src','pupper-mjlab/configs','foundation/scripts') for p in (args.repo/base).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.py','.json','.yaml','.yml','.xml','.urdf','.toml','.sh','.csv','.npz'))
    source_files={str(p.relative_to(args.repo)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    source_hash=hashlib.sha256(json.dumps(source_files,sort_keys=True).encode()).hexdigest()
    (args.out/'source_hashes.json').write_text(json.dumps(source_files,indent=2)+'\n')
    revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    state={'started':datetime.now(timezone.utc).isoformat(),'smoke':args.smoke,'stages':{},'status':'running','git_revision':revision,'source_sha256':source_hash,'seed':123,'locomotion_training_seed':42,'evaluation_seeds':[101,202,303]}
    status_file=args.out/'batch_status.json'
    def save():
        tmp=status_file.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(status_file)
    def run(name,argv):
        marker=args.out/f'{name}.done.json'
        if marker.exists():
            state['stages'][name]=json.loads(marker.read_text());save();return
        info={'status':'running','command':[str(a) for a in argv],'started':datetime.now(timezone.utc).isoformat()}
        state['stages'][name]=info;save();print(f'START {name}',flush=True)
        with (args.out/f'{name}.log').open('a') as log:
            result=subprocess.run([sys.executable,*map(str,argv)],env=env,stdout=log,stderr=subprocess.STDOUT)
        info.update(status='complete' if result.returncode==0 else 'failed',returncode=result.returncode,finished=datetime.now(timezone.utc).isoformat())
        save()
        if result.returncode: raise RuntimeError(f'{name} failed; see {args.out/name}.log')
        marker.write_text(json.dumps(info,indent=2)+'\n');print(f'FINISH {name}',flush=True)
    def fresh(stem):
        # Preserve incomplete attempts rather than overwriting their evidence.
        base=args.out/stem
        done=args.out/f'{stem}.done.json'
        if done.exists():
            argv=json.loads(done.read_text())['command']
            if '--out' in argv:return Path(argv[argv.index('--out')+1])
            return base
        if not base.exists():return base
        for i in range(2,1000):
            candidate=args.out/f'{stem}-attempt{i}'
            if not candidate.exists(): return candidate
        raise RuntimeError('Too many attempts')
    def checkpoint(directory):
        paths=list(Path(directory).glob('model_*.pt'))
        if not paths: raise FileNotFoundError(f'No model checkpoints in {directory}')
        # Resume numbering belongs to the runner; never assume model_4499.pt.
        return max(paths,key=lambda x:int(x.stem.split('_')[-1]))
    def publish(name,directory):
        if not args.wandb:return
        import wandb
        with wandb.init(project='pupper-foundation',name=f'optional-{name}',job_type='capability-evidence',config={'smoke':args.smoke},reinit=True) as wrun:
            artifact=wandb.Artifact(f'pupper-{name}',type='capability-evidence',metadata={'smoke':args.smoke,'hardware_validated':False})
            directory=Path(directory)
            for path in directory.rglob('*'):
                if path.is_file() and path.name!='last.pt' and 'cache' not in path.parts:
                    artifact.add_file(str(path),name=str(path.relative_to(directory)))
            wrun.log_artifact(artifact).wait()
            state.setdefault('artifacts',{})[name]=f'{wrun.entity}/{wrun.project}/{artifact.name}'
            save()
    save()
    if args.mode in ('all','supervised'):
        try:
            if not args.dataset:
                # Read-only download of the public course dataset. No user Drive
                # credentials or browser cookies are exported into this worker.
                try:import gdown
                except ImportError:
                    subprocess.run([sys.executable,'-m','pip','install','gdown'],check=True)
                    import gdown
                args.dataset=args.out/'tactile_data.zip'
                if not args.dataset.exists():
                    result=gdown.download(id=args.dataset_drive_id,output=str(args.dataset),quiet=False)
                    if not result:raise RuntimeError('Public dataset download failed; pass an uploaded --dataset ZIP instead')
            dataset=args.dataset.resolve()
            if dataset.suffix.lower()=='.zip':
                from implicit_compliance.data import extract_dataset
                target=args.out/'data'
                dataset=target/'tactile_data'
                if not dataset.exists(): dataset=extract_dataset(args.dataset,target)
            prep=args.out/'prepared'
            run('prepare',['-m','implicit_compliance.data',dataset,prep,'--image-contact-source','annotation'])
            manifest=prep/'manifest.json'
            for kind in (['explicit'] if args.skip_implicit else ['implicit','explicit']):
                target=fresh(kind)
                command=['-m','implicit_compliance.train','train','--manifest',manifest,'--out',target,'--kind',kind,'--epochs',2 if args.smoke else 50 if kind=='implicit' else 10,'--batch-size',64 if kind=='explicit' else 512,'--device',args.device]
                if args.smoke:command+=['--max-samples',128]
                run(kind,command)
                run(f'{kind}-test',['-m','implicit_compliance.train','evaluate','--manifest',manifest,'--checkpoint',target/'best.pt','--out',target/'test.json','--device',args.device,'--batch-size',64])
                if kind=='implicit':run('implicit-export',['-m','implicit_compliance.train','export','--checkpoint',target/'best.pt','--out',target/'deploy'])
                publish(kind,target)
        except Exception as exc:
            state['supervised_error']=str(exc);save();traceback.print_exc()
    if args.mode in ('all','locomotion'):
        try:
            common=['-m','mjlab.scripts.pupper_locomotion']
            num_envs=32 if args.smoke else 4096
            def train(name,iterations,profile='baseline',previous=None,reference=None,task='Mjlab-MixedGaits-Flat-Pupper-v3',weights=None):
                target=fresh(name)
                cmd=common+['train','--out',target,'--task',task,'--iterations',2 if args.smoke else iterations,'--num-envs',num_envs,'--device',args.device,'--logger','wandb' if args.wandb else 'tensorboard','--profile',profile]
                if previous:cmd+=['--checkpoint',previous]
                if reference:cmd+=['--reference-dir',reference]
                if weights:cmd+=['--weights',weights]
                run(name,cmd);return checkpoint(target),target
            def evaluate(name,ckpt,reference=None,task='Mjlab-MixedGaits-Flat-Pupper-v3',seed=101,randomized=False):
                target=fresh(name)
                cmd=common+['evaluate','--checkpoint',ckpt,'--out',target,'--task',task,'--seed',seed,'--num-envs',2 if args.smoke else 32,'--seconds',1 if args.smoke else 10,'--warmup',.1 if args.smoke else 2,'--device',args.device]
                if reference:cmd+=['--reference-dir',reference]
                if randomized:cmd+=['--randomized']
                run(name,cmd);return target
            base,base_dir=train('mixed-baseline',3000)
            baseline_eval=evaluate('mixed-baseline-eval',base)
            best,reference,profile,best_eval=base,None,'baseline',baseline_eval
            authored=args.repo/'pupper-mjlab/configs/optional2/extended_trot'
            candidate,candidate_dir=train('mixed-authored',1500,previous=base,reference=authored)
            candidate_eval=evaluate('mixed-authored-eval',candidate,authored)
            from mjlab.tasks.pupper_gait.bootstrap import compare_evaluations
            def compare(name,old,new):
                report=compare_evaluations(json.loads((old/'metrics.json').read_text()),json.loads((new/'metrics.json').read_text()))
                (args.out/f'{name}.json').write_text(json.dumps(report,indent=2)+'\n')
                state.setdefault('comparisons',{})[name]=report;save();return report['promotable_for_this_seed']
            if compare('authored-vs-stock',best_eval,candidate_eval):best,reference,best_eval=candidate,authored,candidate_eval
            # A distinct reward ablation follows the reference-only comparison.
            fast,fast_dir=train('mixed-fast-rewards',1500,profile='fast',previous=candidate,reference=authored)
            fast_eval=evaluate('mixed-fast-rewards-eval',fast,authored)
            if compare('fast-rewards-vs-best',best_eval,fast_eval):best,reference,profile,best_eval=fast,authored,'fast',fast_eval
            failures=0
            if not args.smoke:
                for round_id in range(1,args.bootstrap_rounds+1):
                    ref=args.out/f'bootstrap-reference-{round_id}'
                    frequency=2 if reference else 1
                    if reference:frequency=json.loads((reference/'reference_manifest.json').read_text())['frequency_mult']
                    try:
                        run(f'fold-{round_id}',['-m','mjlab.tasks.pupper_gait.bootstrap','fold',best_eval/'trace.npz',ref,'--frequency-mult',frequency])
                    except Exception as exc:
                        state['bootstrap_capture_gap']=str(exc);save();break
                    ckpt,round_dir=train(f'bootstrap-{round_id}',1500,profile,previous=best,reference=ref)
                    ev=evaluate(f'bootstrap-{round_id}-eval',ckpt,ref)
                    if compare(f'bootstrap-{round_id}-vs-best',best_eval,ev):best,reference,best_eval=ckpt,ref,ev;failures=0
                    else:failures+=1
                    if failures>=2:break
            state['selected_flat']={'checkpoint':str(best),'reference_dir':str(reference) if reference else None,'profile':profile};save()
            heldout=[]
            for seed in (202,303):
                old=evaluate(f'baseline-flat-seed{seed}',base,seed=seed,randomized=True)
                new=evaluate(f'best-flat-seed{seed}',best,reference,seed=seed,randomized=True)
                heldout.append(compare(f'heldout-seed{seed}-vs-baseline',old,new))
            state['selected_flat']['heldout_promotion_passed']=all(heldout) if best!=base else False
            state['selected_flat']['promotion_scope']='simulation only; hardware validation not performed'
            save()
            bumpy,bumpy_dir=train('mixed-bumpy',1500,profile,previous=best,reference=reference,task='Mjlab-MixedGaits-Bumpy-Pupper-v3')
            evaluate('mixed-bumpy-eval',bumpy,reference,task='Mjlab-MixedGaits-Bumpy-Pupper-v3',randomized=True)
            weights=args.repo/'pupper-mjlab/configs/optional2/mystery_experiment.json'
            mystery,mystery_dir=train('mystery-pilot',500,previous=best,reference=reference,task='Mjlab-Mystery-Flat-Pupper-v3',weights=weights)
            mystery_eval=evaluate('mystery-pilot-eval',mystery,reference,task='Mjlab-Mystery-Flat-Pupper-v3')
            jump_ref=args.out/'jump-reference'
            # A missing real flight is an explicit scientific outcome, not a failed batch.
            try:
                run('capture-jump',['-m','mjlab.tasks.pupper_gait.bootstrap','jump',mystery_eval/'trace.npz',jump_ref,'--auto'])
                if reference:
                    for filename in ('captured_reach_gaits.npz','reference_manifest.json'):shutil.copy2(reference/filename,jump_ref/filename)
                jump,jump_dir=train('mystery-captured',1000,previous=mystery,reference=jump_ref,task='Mjlab-Mystery-Flat-Pupper-v3',weights=weights)
                evaluate('mystery-captured-eval',jump,jump_ref,task='Mjlab-Mystery-Flat-Pupper-v3')
                publish('mystery-captured',jump_dir)
            except Exception as exc:
                state['jump_capture_outcome']=str(exc);save()
            for name,directory in [('best-flat',best.parent),('mixed-bumpy',bumpy_dir),('mystery-pilot',mystery_dir)]:publish(name,directory)
        except Exception as exc:
            state['locomotion_error']=str(exc);save();traceback.print_exc()
    state['status']='complete_with_gaps' if any(k.endswith('_error') for k in state) or 'jump_capture_outcome' in state or 'bootstrap_capture_gap' in state else 'finished'
    state['finished']=datetime.now(timezone.utc).isoformat();save()
    print(json.dumps(state,indent=2),flush=True)

if __name__=='__main__':main()
