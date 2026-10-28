#!/usr/bin/env python3
"""Record a child's actual exit independently of its workflow scheduler.

The supervisor owns only the single command in its immutable job.json. It never
signals unrelated processes. A stopped/killed scheduler does not erase exit.json.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def now(): return datetime.now(timezone.utc).isoformat()

def save(path, value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');tmp.replace(path)

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def identity(pid):
    result={'pid':int(pid)}
    try:
        stat=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
        result.update(state=stat[0],start_ticks=stat[19])
    except (OSError,IndexError):
        try:os.kill(pid,0)
        except ProcessLookupError:return None
        except PermissionError:pass
        result.update(state='unknown',start_ticks=None)
    return result

def alive(record):
    if not record:return False
    current=identity(record['pid'])
    return current is not None and current['state']!='Z' and (
        record.get('start_ticks') is None or current.get('start_ticks')==record['start_ticks'])

def group_alive(group):
    """Detect a child even if its supervisor died before recording child.json."""
    if Path('/proc').is_dir():
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields=path.read_text().rsplit(')',1)[1].split()
                if fields[0]!='Z' and int(fields[2])==group and int(fields[3])==group:return True
            except (OSError,ValueError,IndexError):continue
        return False
    try:os.killpg(group,0)
    except ProcessLookupError:return False
    except PermissionError:return True
    return True

def supervise(spec_path):
    spec_path=spec_path.resolve();spec=json.loads(spec_path.read_text());folder=spec_path.parent
    if (folder/'exit.json').exists() or (folder/'supervisor.json').exists():
        raise ValueError('Refuse to overwrite an existing supervisor attempt')
    if os.getsid(0)!=os.getpid():raise ValueError('Supervisor requires its own isolated session')
    save(folder/'supervisor.json',{'identity':identity(os.getpid()),'process_group':os.getpgrp(),
                                  'started_utc':now(),'job_sha256':sha(spec_path)})
    env=os.environ.copy();env.pop('PUPPER_REFERENCE_DIR',None);env.update(spec.get('environment',{}))
    start=time.monotonic();child=None
    try:
        with (folder/'stdout.log').open('x') as stream:
            child=subprocess.Popen(spec['argv'],cwd=spec['cwd'],env=env,stdout=stream,stderr=subprocess.STDOUT)
            save(folder/'child.json',{'identity':identity(child.pid),'started_utc':now(),'argv':spec['argv'],'job_sha256':sha(spec_path)})
            returncode=child.wait()
        save(folder/'exit.json',{'returncode':returncode,'finished_utc':now(),'elapsed_seconds':time.monotonic()-start,
                                'job_sha256':sha(spec_path),'child_pid':child.pid,'exit_observed_by':'direct parent supervisor'})
        return 0
    except BaseException as exc:
        # A supervisor interruption is not evidence of the child's exit. Preserve
        # that distinction; a resumed scheduler must check both process identities.
        save(folder/'supervisor_error.json',{'error':f'{type(exc).__name__}: {exc}','utc':now(),
                                           'child_identity':identity(child.pid) if child else None})
        raise

def launch(spec_path, python):
    folder=spec_path.parent
    with (folder/'supervisor.log').open('x') as log:
        process=subprocess.Popen([str(python),'-B',str(Path(__file__).resolve()),str(spec_path)],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    save(folder/'launcher.json',{'supervisor_identity':identity(process.pid),'process_group':process.pid,'utc':now()})
    process.wait()
    if not (folder/'exit.json').exists():
        raise RuntimeError('Supervisor exited without an observed child exit; inspect preserved identities/logs')
    result=json.loads((folder/'exit.json').read_text())
    if result['job_sha256']!=sha(spec_path):raise ValueError('Job specification changed')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('spec',type=Path)
    raise SystemExit(supervise(parser.parse_args().spec))
