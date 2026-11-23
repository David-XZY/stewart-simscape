"""Frozen, train/validation-only 474-label experiment contract."""
from pathlib import Path
import hashlib
import json
import os
import sys

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'chapter3/payload_generalization'))
NAME='payload_retrain474_v1'
OUT=ROOT/'results/chapter3'/NAME
SOURCE=ROOT/'results/chapter3/payload_condition_expansion_v2_longtime_20260926'
SEEDS=[20260919,20260920,20260921]
SAMPLE_SEEDS=[20261001,20261002,20261003]
LEARNED=['ddpm_basic','ddpm_signature','regression','regression_signature']
METHODS=LEARNED+['retrieval_signature','ihsid']
DATA_SHA='c6251d84be3a3d78a5072b8903baa20c3d9cbe204f8397a33a5d3a1f79854cf1'

def read(p): return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def atomic(p,v):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    temp=p.with_suffix(p.suffix+'.tmp')
    temp.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    os.replace(temp,p)

def spec_from_task(t):
    return dict(taskId=t['taskId'],baseSceneId=t['baseSceneId'],split='validation',
        taskClass=t['taskClass'],family=t['family'],generalizationType=t['generalizationType'],
        payloadRegionId=t.get('payloadRegionId',''),q0=t['q0'],qGoal=t['qGoal'],parameters=t['parameters'],
        shape=t['payload']['shape'],dimensions=t['payload']['dimensions'],mass=t['payload']['mass'])

def physical_identity(s):
    return json.dumps([s['family'],s['q0'],s['qGoal'],s['parameters'][:3],s['shape'],s['dimensions'],s['mass']],sort_keys=True)

def plateau(history,terminal):
    if terminal:return True,'all_success_or_search_exhausted'
    if len(history)>=4 and history[-1]['allowance']>=160 and all(h.get('expanded',True) for h in history[-3:]) and len({h['successes'] for h in history[-4:]})==1:
        return True,'three_doublings_without_new_success'
    return False,'continue'

def score_panels(checks,metadata):
    if len(checks)!=len(metadata):raise ValueError('Incomplete physical validation')
    rates={}
    for panel in ['legacy','longtime']:
        values=[bool(c['passed']) for c,m in zip(checks,metadata) if m['panel']==panel]
        if not values:raise ValueError('Missing validation panel')
        rates[panel]=sum(values)/len(values)
    return .5*(rates['legacy']+rates['longtime']),rates
