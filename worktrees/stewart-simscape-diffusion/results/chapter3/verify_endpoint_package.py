"""Verify an extracted delivery without any original archives or planner."""
import collections,gzip,hashlib,json,pathlib
P=pathlib.Path(__file__).resolve().parent
manifest=json.loads((P/'MANIFEST.json').read_text(encoding='utf8'))
for item in manifest['files']:
 p=P/item['path'];assert p.is_file(),str(p)
 assert p.stat().st_size==item['bytes'],str(p)
 assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'],str(p)
with gzip.open(P/'data/filtered_calls.jsonl.gz','rt',encoding='utf8') as f:rs=[json.loads(l) for l in f]
counts=collections.Counter((r['method'],r['B']) for r in rs)
assert len(rs)==7884 and len(counts)==18 and set(counts.values())=={438}
assert len({r['taskId'] for r in rs})==146 and len({r['baseSceneId'] for r in rs})==20
print(json.dumps({'verifiedFiles':len(manifest['files']),'calls':len(rs),'measuredCells':len(counts),'passed':True}))
