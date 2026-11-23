"""Analyze only completed new-protocol calls; all budgets remain independent."""
import collections, gzip, itertools, json
import numpy as np
from prepare import OUT,read,write,sha,METHODS,BUDGETS
METHODS=[m for m in METHODS if m!="ihsid"]

METRICS=['positionLengthM','rotationLengthRad','legTotalVariation','positionAccelRms','angularAccelRms','legAccelRms','positionJerkRms','angularJerkRms','legJerkRms','forceRmsN','forceRateRms','absoluteMechanicalWorkJ']
def boot(rows,values):
    scenes=sorted({r['baseSceneId'] for r in rows});seeds=sorted({r['repeat'] for r in rows})
    sums=np.zeros((len(scenes),len(seeds)));counts=sums.copy()
    for r,v in zip(rows,values):
        if not np.isfinite(v):continue
        i=scenes.index(r['baseSceneId']);j=seeds.index(r['repeat']);sums[i,j]+=v;counts[i,j]+=1
    rng=np.random.default_rng(20260928);si=rng.integers(len(scenes),size=(2000,len(scenes)));sj=rng.integers(len(seeds),size=(2000,len(seeds)))
    num=sums[si[:,:,None],sj[:,None,:]].sum((1,2));den=counts[si[:,:,None],sj[:,None,:]].sum((1,2));draws=num[den>0]/den[den>0]
    return dict(mean=float(sums.sum()/counts.sum()),ci95=np.quantile(draws,[.025,.975]).tolist(),n=len(rows),tasks=len({r['taskId'] for r in rows}),scenes=len(scenes))

def summarize():
    p=read(OUT/'protocol_four.json');assert read(OUT/'formal/audit.json')['passed'];data=OUT/'analysis_four/data'
    data.mkdir(parents=True,exist_ok=True);rows=[read(OUT/'formal/calls'/j['runId']/'result.json') for j in p['jobs']]
    with gzip.open(data/'all_calls.jsonl.gz','wt',encoding='utf-8') as f:
        for r in rows:f.write(json.dumps(r,ensure_ascii=False)+'\n')
    summary=[];strata=[];timing=[];outcomes=[];matched=[];pairs=[];effects=[]
    stages={'input_and_endpoint':'输入/编码','outer_input':'输入/编码','encoding_and_retrieval':'输入/编码',
      'generation':'生成','generation_other':'生成','physical':'检查','quality':'检查','acceptance_other':'检查',
      'repair':'精修','repair_exception':'精修','solver':'NLP','solver_initialization':'NLP','solver_build':'NLP','solver_other':'NLP','ihsid_including_internal_acceptance':'NLP'}
    categories=['直接成功','公共精修成功','超时失败','复查拒绝','其他失败']
    groups={(m,b):[r for r in rows if r['method']==m and r['B']==b] for m in METHODS for b in BUDGETS}
    for (m,b),rs in groups.items():
        assert len(rs)==438
        summary.append(dict(method=m,B=b,status='measured',qualified=sum(r['finalQualified'] for r in rs),**boot(rs,[r['finalQualified'] for r in rs])))
        for field,extract in [('generalizationType',lambda r:r['generalizationType']),('shape',lambda r:r['payloadShape']),('family',lambda r:r['job']['spec']['family'])]:
            for value in sorted({extract(r) for r in rs}):
                selected=[r for r in rs if extract(r)==value]
                strata.append(dict(method=m,B=b,stratum=field,value=value,**boot(selected,[r['finalQualified'] for r in selected])))
        totals=collections.Counter({k:0. for k in ['输入/编码','生成','排序','检查','重新计时','精修','NLP','其他']});counts=collections.Counter()
        for r in rs:
            totals['其他']+=r['otherSeconds']
            for e in r['events']:totals[stages.get(e['stage'],'其他')]+=e['seconds']
            if r['finalQualified']:cat={'direct':'直接成功','repair':'公共精修成功','ihsid':'独立IHSID成功'}[r['returnStage']]
            elif not r['returnedWithinBudget']:cat='超时失败'
            elif r['onlinePassed']:cat='复查拒绝'
            else:cat='其他失败'
            counts[cat]+=1
        timing.append(dict(method=m,B=b,n=len(rs),stageMeanSeconds={k:v/len(rs) for k,v in totals.items()},totalMeanSeconds=float(np.mean([r['actualReturnTime'] for r in rs])),returnTimes=[r['actualReturnTime'] for r in rs]))
        outcomes.extend(dict(method=m,B=b,outcome=cat,calls=counts[cat]) for cat in categories)
    for b in BUDGETS:
        maps={m:{(r['taskId'],r['repeat']):r for r in groups[m,b]} for m in METHODS}
        common=set.intersection(*[{k for k,r in mp.items() if r['finalQualified']} for mp in maps.values()])
        for k in sorted(common):
            for m in METHODS:
                r=maps[m][k];quality=dict(r['rawMetrics'])
                quality['legTotalVariation']=float(np.sum(quality['legTotalVariation']))
                matched.append(dict(method=m,B=b,taskId=k[0],repeat=k[1],baseSceneId=r['baseSceneId'],quality=quality))
        for a,c in itertools.combinations(METHODS,2):
            rr=groups[a,b];diff=[int(r['finalQualified'])-int(maps[c][r['taskId'],r['repeat']]['finalQualified']) for r in rr]
            effects.append(dict(method=a,baseline=c,B=b,**boot(rr,diff)))
            shared=[k for k in maps[a] if maps[a][k]['finalQualified'] and maps[c][k]['finalQualified']]
            if not shared:continue
            for metric in METRICS:
                rr=[maps[a][k] for k in shared]
                values=[float(np.sum(maps[a][k]['rawMetrics'][metric]))-float(np.sum(maps[c][k]['rawMetrics'][metric])) for k in shared]
                pairs.append(dict(method=a,baseline=c,B=b,metric=metric,**boot(rr,values)))
    for name,value in [('success_summary',summary),('strata',strata),('timing',timing),('outcomes',outcomes),('matched_observations_all_budgets',matched),('matched_observations',[r for r in matched if r['B']==10]),('paired_quality',pairs),('paired_success',effects),('excluded_tasks',p['exclusions'])]:write(data/(name+'.json'),value)
    write(OUT/'analysis_four/provenance.json',dict(configHash=p['configHash'],callCount=len(rows),joint10sCalls=len([r for r in matched if r['B']==10])//4,files={f.name:sha(f) for f in data.iterdir()}))
if __name__=='__main__':summarize()
