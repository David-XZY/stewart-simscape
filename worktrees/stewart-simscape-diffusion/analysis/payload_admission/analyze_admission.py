"""Measure within-condition diversity and replay relaxed admission policies."""
from __future__ import annotations
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

METRICS = ['rawModeRms','fittedModeRms','positionRmsMm','positionMaxMm',
           'rotationRmsDeg','rotationMaxDeg','velocityRmsMmS',
           'angularVelocityRmsDegS','accelerationRmsMmS2',
           'angularAccelerationRmsDegS2','legVelocityRmsMmS',
           'legAccelerationRmsMmS2','forceRmsN','forceMaxN']

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')

def quantiles(values):
    a=np.asarray(list(values),float)
    if not len(a): return None
    return {k:float(v) for k,v in zip(['min','median','p90','p95','p99','max'],np.quantile(a,[0,.5,.9,.95,.99,1]))}

def select(group, distances, threshold, cap, order='source'):
    """Strict > threshold; farthest-first starts at minimum teacher objective."""
    if order=='source':
        ordered=sorted(group,key=lambda r:int(r['labelId'].split('_')[-1])); kept=[]
        for row in ordered:
            if len(kept)>=cap: break
            if all(distances[frozenset((row['labelId'],other['labelId']))]>threshold for other in kept):
                kept.append(row)
        return kept
    remaining=sorted(group,key=lambda r:(r['objective'],r['labelId']))
    kept=[remaining.pop(0)]
    while remaining and len(kept)<cap:
        ranked=[(min(distances[frozenset((r['labelId'],k['labelId']))] for k in kept),r) for r in remaining]
        distance,row=max(ranked,key=lambda x:(x[0],-x[1]['objective'],x[1]['labelId']))
        if distance<=threshold: break
        kept.append(row); remaining.remove(row)
    return kept

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folder',type=Path)
    args=ap.parse_args(); folder=args.folder
    payload=read(folder/'candidates.json'); candidates=payload['candidates']
    with (folder/'pairs.csv').open(encoding='utf-8-sig',newline='') as f:
        pairs=list(csv.DictReader(f))
    for p in pairs:
        for k in METRICS:p[k]=float(p[k])
    by_task=collections.defaultdict(list)
    for c in candidates:
        assert c['split'] in ['train','validation'] and c['physicalRecheckPassed']
        assert np.isfinite(c['controls']).all()
        by_task[c['taskId']].append(c)
    matrix={m:{frozenset((p['labelA'],p['labelB'])):p[m] for p in pairs} for m in METRICS}
    stats={}
    for split in ['train','validation']:
        ps=[p for p in pairs if p['split']==split]
        maximum=collections.defaultdict(lambda:collections.defaultdict(float))
        for p in ps:
            for m in METRICS:maximum[p['taskId']][m]=max(maximum[p['taskId']][m],p[m])
        stats[split]={'candidates':sum(c['split']==split for c in candidates),
                      'conditions':sum(g[0]['split']==split for g in by_task.values()),
                      'pairs':len(ps),'pairWeighted':{m:quantiles(p[m] for p in ps) for m in METRICS},
                      'conditionMaximum':{m:quantiles(maximum[t][m] for t,g in by_task.items() if g[0]['split']==split) for m in METRICS}}
    sweep=[]
    for metric in ['rawModeRms','fittedModeRms']:
        for threshold in [.15,.1,.05,.02,.01,.005,.002,.001,.0005,.0002,.0001,1e-5,1e-6,1e-7,1e-8,0.]:
            for cap,order in [(4,'source'),(8,'quality_farthest')]:
                selected=[r for g in by_task.values() for r in select(g,matrix[metric],threshold,cap,order)]
                sweep.append(dict(metric=metric,threshold=threshold,cap=cap,order=order,
                                  train=sum(r['split']=='train' for r in selected),
                                  validation=sum(r['split']=='validation' for r in selected)))
    summary={'scope':'Train and validation candidates only; no formal test use.',
             'identity':payload['identity'],'statistics':stats,'sweep':sweep,
             'notes':['Pair metrics compare fitted training targets at 513 common time points.',
                      'Mode metrics use 33 progress-aligned points, following the original feature rule.',
                      'All candidates passed the unchanged complete physical recheck.',
                      'More labels do not imply more independent conditions or improved trained performance.']}
    write(folder/'difference_summary.json',summary)
    with (folder/'threshold_sweep.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(sweep[0]));w.writeheader();w.writerows(sweep)
    best=sorted([p for p in pairs if p['split']=='train'],key=lambda p:p['positionRmsMm'],reverse=True)[:10]
    write(folder/'largest_differences.json',best)
    print(json.dumps({'statistics':stats,'legacy_raw_sweep':[r for r in sweep if r['metric']=='rawModeRms' and r['cap']==4],
                      'largestPositionPair':best[0]},ensure_ascii=True))

if __name__=='__main__':main()
