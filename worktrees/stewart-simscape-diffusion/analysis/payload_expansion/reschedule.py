"""Record the explicit sequential-to-three-worker scheduling revision."""
import hashlib
import json
from pathlib import Path
import time

root=Path(__file__).resolve().parents[2]
out=root/'results/chapter3/payload_condition_expansion_v1_20260926'
identity=json.loads((out/'identity.json').read_text(encoding='utf-8-sig'))
new_hash=hashlib.sha256((root/'analysis/payload_expansion/pgExpandDataset.m').read_bytes()).hexdigest()
revision=out/'scheduler_revision.json'
if not revision.exists():
    revision.write_text(json.dumps(dict(previousIdentity=identity,updatedUnix=time.time(),
        newImplementationSha256=new_hash,reason='Three disjoint condition shards; same physical gate, solver and sampling; retain completed checkpoints',
        previousProgress=json.loads((out/'progress.json').read_text(encoding='utf-8-sig'))),indent=2),encoding='utf-8')
identity['implementationSha256']=new_hash
(out/'identity.json').write_text(json.dumps(identity),encoding='utf-8')
print('Scheduler identity revised; completed checkpoints retained.')
