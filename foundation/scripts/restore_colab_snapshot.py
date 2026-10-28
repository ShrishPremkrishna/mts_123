"""Materialize a verified periodic snapshot into a new directory; never starts jobs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


def restore(archive, output):
    index=json.loads((archive/'index.json').read_text())
    plan=[]
    for name,item in index['files'].items():
        relative=Path(name);digest=item['sha256']
        if relative.is_absolute() or '..' in relative.parts or not re.fullmatch('[0-9a-f]{64}',digest):
            raise ValueError('Invalid snapshot path or identity')
        source=archive/'objects'/digest
        if hashlib.sha256(source.read_bytes()).hexdigest()!=digest or source.stat().st_size!=item['bytes']:
            raise ValueError(f'Snapshot verification failed: {name}')
        plan.append((source,output/relative))
    output.mkdir(parents=True,exist_ok=False)
    for source,destination in plan:
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,destination)
    (output/'recovery-snapshot.json').write_text(json.dumps(index,indent=2)+'\n')
    return len(plan)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();print(json.dumps({'restored_files':restore(args.archive,args.output)}))
