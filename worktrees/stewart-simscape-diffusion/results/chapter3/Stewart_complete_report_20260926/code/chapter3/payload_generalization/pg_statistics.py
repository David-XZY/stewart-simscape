"""Prespecified paired scene-cluster inference with globally shared seed blocks."""
import numpy as np

def paired_bootstrap(records,method,baseline,group='G2',budget=2,replicates=2000,seed=20260919):
    rows=[r for r in records if r['method'] in [method,baseline] and r['B']==budget and r['generalizationType']==group]
    values={}; scenes={}
    for r in rows:
        key=(r['taskId'],r['repeat'],r['method'])
        if key in values: raise ValueError('Duplicate actual call in paired table')
        values[key]=float(r['finalQualified']); scenes.setdefault(r['baseSceneId'],set()).add(r['taskId'])
    blocks=sorted({r['repeat'] for r in rows}); clusters=sorted(scenes)
    if not clusters or not blocks: raise ValueError('No matched comparison')
    for tasks in scenes.values():
        for task in tasks:
            for block in blocks:
                for m in [method,baseline]:
                    if (task,block,m) not in values: raise ValueError('Missing method or repeat; no silent complete-case filtering')
    def effect(cluster_ids,block_ids):
        delta=[]
        for scene in cluster_ids:
            for task in sorted(scenes[scene]):
                delta.append(np.mean([values[(task,b,method)]-values[(task,b,baseline)] for b in block_ids]))
        return float(np.mean(delta))
    rng=np.random.default_rng(seed); draws=[]
    for _ in range(replicates):
        selected_clusters=rng.choice(clusters,len(clusters),replace=True)
        selected_blocks=rng.choice(blocks,len(blocks),replace=True)  # same seed draw for every scene
        draws.append(effect(selected_clusters,selected_blocks))
    return dict(difference=effect(clusters,blocks),ci95=np.quantile(draws,[.025,.975]).tolist(),sceneClusters=len(clusters),seedBlocks=len(blocks),
                tasks=len({r['taskId'] for r in rows}),replicates=replicates,limitation='Only three trained-seed blocks are planned; repeated calls are not independent tasks.')

def quality_caps(rows):
    out={}
    for shape in ['cylinder','box']:
        selected=[r for r in rows if r['shape']==shape and r['split']=='validation' and r['passed']]
        if not selected: raise ValueError('Missing qualified validation references for '+shape)
        counts={}
        for r in selected: counts[r['taskId']]=counts.get(r['taskId'],0)+1
        def p95(field):
            order=sorted(selected,key=lambda r:r[field]); values=np.array([r[field] for r in order]); weights=np.array([1/counts[r['taskId']] for r in order]); weights/=weights.sum()
            return float(np.interp(.95,np.cumsum(weights),values))*1.5
        out[shape]=dict(position=p95('positionDetour'),rotation=p95('rotationDetour'),conditions=len(counts))
    return out

def allow_m34(basic,signature,physloss):
    ds=signature-basic; dp=physloss-basic
    return ds>=-.02-1e-12 and dp>=-.02-1e-12 and max(ds,dp)>=.02-1e-12
