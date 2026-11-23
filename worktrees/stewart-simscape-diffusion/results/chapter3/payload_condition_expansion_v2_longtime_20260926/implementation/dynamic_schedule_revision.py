"""Record a scheduler-only revision after stopping this task's workers."""
import hashlib
import json
from pathlib import Path
import time

root=Path(__file__).resolve().parents[2]
out=root/'results/chapter3/payload_condition_expansion_v2_longtime_20260926'
identity=json.loads((out/'identity.json').read_text(encoding='utf-8-sig'))
code=root/'analysis/payload_expansion/pgExpandDataset.m'
new_hash=hashlib.sha256(code.read_bytes()).hexdigest()
(out/'dynamic_scheduler_revision.json').write_text(json.dumps(dict(previousIdentity=identity,
    updatedUnix=time.time(),newImplementationSha256=new_hash,
    reason='Atomic whole-group claims replace static assignment; same tasks, constraints, warm-start order, solver budget and completed checkpoints',
    completedNewChecks=len(list((out/'checks').glob('expand_*.json')))),indent=2),encoding='utf-8')
identity['implementationSha256']=new_hash
(out/'identity.json').write_text(json.dumps(identity),encoding='utf-8')
print('Dynamic group scheduling recorded; physical gate and completed results unchanged.')
