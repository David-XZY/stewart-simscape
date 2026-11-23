"""Read-only audit of durable repair checkpoints; partial runs stay partial."""
from pathlib import Path
from collections import Counter
import json, hashlib
import numpy as np
from scipy.io import loadmat
root=Path(__file__).resolve().parent
identity=json.loads((root.parent/'runtime_source_identity.json').read_text('utf-8'))
label='repair_'+identity['diffusion_source_sha256'][:8]+'_002_nonideal_force_320ms_original'
attempts=list((root/(label+'.mat.journal')).glob('attempt_*'))
assert len(attempts)==1, 'Audit attempts separately; do not merge retries.'
rows=[]
def get(x,k):
    return x[k] if isinstance(x,dict) else getattr(x,k)
for f in sorted(attempts[0].glob('chunk_*.mat')):
    data=loadmat(f,simplify_cells=True); r=data['newRecords']; p=data['newPredictions']['prediction']
    row=dict(time=float(r['time']),chosen=int(p['chosen']),fallback=bool(r['diagnostic']['usedFallback']),controller_seconds=float(r['controller_seconds']),raw_sha256=hashlib.sha256(f.read_bytes()).hexdigest())
    checks=np.asarray(p['checks'],dtype=object).reshape(-1)
    row['total_candidates']=len(checks)
    row['accepted_candidates']=sum(bool(get(c,'accepted')) for c in checks)
    for name in ['denoise_attempts','final_attempts']:
        solves=np.asarray(p[name],dtype=object).reshape(-1)
        row[name]=dict(calls=len(solves),non_success=sum(not bool(get(s,'solver_success')) for s in solves),statuses=dict(Counter(str(get(s,'status')) for s in solves)))
    rows.append(row)
report=root/(label+'_report.json'); complete=report.exists()
value=dict(diffusion_source_sha256=identity['diffusion_source_sha256'],partial=not complete,actual_episode_metrics_verified=complete,control_calls=len(rows),fallbacks=sum(r['fallback'] for r in rows),controller_median_seconds=float(np.median([r['controller_seconds'] for r in rows])),rows=rows)
value['total_candidates']=sum(r['total_candidates'] for r in rows)
value['accepted_candidates']=sum(r['accepted_candidates'] for r in rows)
for name in ['denoise_attempts','final_attempts']:
    value[name]=dict(calls=sum(r[name]['calls'] for r in rows),non_success=sum(r[name]['non_success'] for r in rows))
if complete:
    value['completed_report']=json.loads(report.read_text('utf-8'))
(root/'repair_nonideal_progress_snapshot.json').write_text(json.dumps(value,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in value.items() if k!='rows'},indent=2))
