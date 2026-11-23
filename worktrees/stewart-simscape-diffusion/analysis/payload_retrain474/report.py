"""Uniform exclusions, paired scene bootstrap, and standalone result report."""
import collections
import csv
import html
import itertools
import numpy as np
from common import *

def aggregate(rows,excluded):
    table=[]
    for budget in [2,5,10,'unlimited']:
        for method in METHODS:
            rr=[r for r in rows if r['budget']==budget and r['method']==method and r['taskId'] not in excluded]
            if not rr:continue
            times=[r['cumulativeSeconds'] if budget=='unlimited' else r['elapsed'] for r in rr]
            stage_times=collections.Counter()
            failures=collections.Counter()
            for r in rr:
                for e in r.get('events',[]):stage_times[e['stage']]+=e['seconds']
                if not r['finalQualified']:
                    reasons=[a.get('reason') for a in r.get('attempts',[]) if a.get('reason') not in (None,'passed','timeout')]
                    failures[reasons[-1] if reasons else r['reason']]+=1
            table.append(dict(method=method,budget=budget,calls=len(rr),conditions=len({r['taskId'] for r in rr}),
                successes=sum(r['finalQualified'] for r in rr),successRate=sum(r['finalQualified'] for r in rr)/len(rr),
                stages=dict(collections.Counter(r['returnStage'] for r in rr if r['finalQualified'])),
                meanStageSeconds={k:v/len(rr) for k,v in stage_times.items()},failureReasons=dict(failures),
                timeMedian=float(np.median(times)),timeP95=float(np.quantile(times,.95))))
    return table

def paired_intervals(rows,excluded):
    # Equal condition weights within scene; paired scene resampling preserves
    # duration/repeat dependence. Training and sampling seeds are averaged.
    result=[];rng=np.random.default_rng(20260928)
    for budget in [2,5,10,'unlimited']:
        rr=[r for r in rows if r['budget']==budget and r['taskId'] not in excluded]
        values=collections.defaultdict(list);scene_of={}
        for r in rr:values[(r['method'],r['taskId'])].append(int(r['finalQualified']));scene_of[r['taskId']]=r['baseSceneId']
        scenes=sorted(set(scene_of.values()));counts=np.array([sum(v==s for v in scene_of.values()) for s in scenes])
        sums={m:np.array([sum(np.mean(values[(m,t)]) for t,s in scene_of.items() if s==scene) for scene in scenes]) for m in METHODS}
        indices=rng.integers(len(scenes),size=(2000,len(scenes)))
        for a,b in itertools.combinations(METHODS,2):
            diff=sums[a]-sums[b];samples=diff[indices].sum(axis=1)/counts[indices].sum(axis=1)
            lo,hi=np.quantile(samples,[.025,.975]);result.append(dict(budget=budget,methodA=a,methodB=b,difference=float(diff.sum()/counts.sum()),low=float(lo),high=float(hi),bootstrap=2000,cluster='baseSceneId',seedUncertainty='training_and_sampling_seed_variation_reported_separately'))
    return result

def finish(v,models,matlab):
    rows=[];latest=[]
    for B in [2,5,10]:
        for p in (OUT/'calls'/f'finite_{B}').glob('*/result.json'):rows.append(dict(read(p),budget=B))
    for method in METHODS:
        s=read(OUT/'unlimited'/(method+'.json'));assert s['status']=='completed'
        for r in s['units'].values():rows.append(dict(r,budget='unlimited'));latest.append(r)
    assert len(rows)==40320
    success_ids={r['taskId'] for r in latest if r['finalQualified']}
    failed=[r for r in v['tasks'] if r['taskId'] not in success_ids]
    folder=OUT/'diagnosis';folder.mkdir(exist_ok=True)
    cases=[]
    for r in failed:
        paths=sorted({x['statePath'] for x in latest if x['taskId']==r['taskId']})
        cases.append(dict(spec=spec_from_task(r['task']),statePaths=paths))
    request=OUT/'diagnosis_request.json';atomic(request,dict(output=str(folder),cases=cases))
    if cases:matlab("p474Diagnose('"+str(request).replace('\\','/')+"');",'diagnosis')
    diagnoses=[read(folder/(r['taskId']+'.json')) for r in failed]
    excluded={d['taskId'] for d in diagnoses if d['exclude']}
    # Exclusion can never remove a witnessed feasible trajectory, at any budget.
    assert not excluded&{r['taskId'] for r in rows if r['finalQualified']}
    atomic(OUT/'exclusions.json',dict(taskIds=sorted(excluded),evidence=[d for d in diagnoses if d['exclude']],rule='necessary_constraint_evidence_only'))
    full=aggregate(rows,set());filtered=aggregate(rows,excluded)
    groups=[]
    for field in ['T','family','taskClass','shape','panel','trainingSeed']:
        for value in sorted({r[field] for r in rows},key=str):groups.append(dict(field=field,value=value,table=aggregate([r for r in rows if r[field]==value],excluded)))
    intervals=paired_intervals(rows,excluded)
    seed_summary=[]
    for method in LEARNED:
        for budget in [2,5,10,'unlimited']:
            rates=[]
            for seed in SEEDS:
                rr=[r for r in rows if r['method']==method and r['budget']==budget and r['trainingSeed']==seed and r['taskId'] not in excluded]
                rates.append(sum(r['finalQualified'] for r in rr)/len(rr))
            seed_summary.append(dict(method=method,budget=budget,seeds=SEEDS,rates=rates,mean=float(np.mean(rates)),standardDeviation=float(np.std(rates,ddof=1))))
    result=dict(status='completed',scope='training_disjoint_validation_comparison_not_test',raw=full,filtered=filtered,excludedConditions=len(excluded),commonFailures=len(failed),diagnoses=diagnoses,groups=groups,seedSummary=seed_summary,pairedIntervals=intervals)
    atomic(OUT/'summary.json',result)
    fields=['taskId','baseSceneId','method','budget','trainingSeed','samplingSeed','finalQualified','returnStage','elapsed','cumulativeSeconds','reason','T','family','taskClass','shape']
    with (OUT/'calls.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    labels=['基础扩散+修复','物理签名扩散+修复','基础回归+修复','物理签名回归+修复','增强检索+修复','IHSID']
    def table(data):
        lookup={(r['method'],r['budget']):r for r in data};body=''
        for m,label in zip(METHODS,labels):
            body+='<tr><th>'+label+'</th>'+''.join(f'<td>{lookup[m,b]["successRate"]:.2%}</td>' for b in [2,5,10,'unlimited'])+'</tr>'
        return '<table><tr><th>方法</th><th>2 s</th><th>5 s</th><th>10 s</th><th>不限时平台</th></tr>'+body+'</table>'
    stage=''.join('<tr>'+''.join('<td>'+html.escape(str(r[k]))+'</td>' for k in ['method','budget','successes','stages','timeMedian','timeP95'])+'</tr>' for r in filtered)
    failure=''.join('<tr><td>'+html.escape(d['taskId'])+'</td><td>'+html.escape(d['classification'])+'</td><td>'+str(d['exclude'])+'</td><td>'+html.escape(d['reason'])+'</td></tr>' for d in diagnoses)
    document='<!doctype html><meta charset="utf-8"><title>474轨迹六方法比较</title><style>body{font:16px system-ui;max-width:1250px;margin:40px auto;color:#172433}table{border-collapse:collapse;margin:24px 0;width:100%}th,td{border:1px solid #d6dce3;padding:10px;text-align:left}th{background:#edf3f8}h1,h2{color:#15446b}</style><h1>474条训练轨迹：六方法限时比较</h1><p>物理验收；允许局部修复；无IHSID回退。此为验证实验，不是正式测试集成绩。</p><h2>全部工况</h2>'+table(full)+'<h2>统一排除后</h2>'+table(filtered)+f'<p>排除 {len(excluded)} 个工况；共同失败 {len(failed)} 个。完整证据见 diagnosis 和 exclusions.json。</p><h2>阶段贡献与时间</h2><table><tr><th>方法</th><th>预算</th><th>成功次数</th><th>阶段</th><th>时间中位数/s</th><th>P95/s</th></tr>'+stage+'</table><h2>共同失败诊断</h2><table><tr><th>工况</th><th>分类</th><th>排除</th><th>原因</th></tr>'+failure+'</table><p>逐调用记录：calls.csv；分组和场景聚类置信区间：summary.json。</p>'
    (OUT/'report.html').write_text(document,encoding='utf-8')
