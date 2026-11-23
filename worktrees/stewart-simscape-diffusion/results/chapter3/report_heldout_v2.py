import pathlib,json,collections,hashlib,html,zipfile,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parent
SRC=ROOT/'payload_heldout_complete_v2';OUT=ROOT/'payload_heldout_report_v2';OUT.mkdir(exist_ok=True)
def read(f):return json.loads(f.read_text(encoding='utf-8'))
def write(n,d):(OUT/n).write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
allrows={};audits={};tables=[]
for run,expected in [('main',16128),('controls',3456),('quality',2016)]:
 rows=[read(f) for f in (SRC/('payload_heldout_'+run+'_v2')/'calls').glob('*/result.json')];assert len(rows)==expected
 protocol=read(SRC/'payload_test_orchestration_v2'/(run+'_frozen.json'));jobs={j['runId']:j for j in protocol['jobs']};assert len(jobs)==expected
 for r in rows:
  assert r['runId'] in jobs and r['configHash']==protocol['configHash']
  assert r['job']==jobs[r['runId']]
  assert not r['finalQualified'] or (r['returnedWithinBudget'] and r['independentAuditPassed'] and (not r.get('fineAuditSelected') or r['fineAuditPassed']))
 closure=max(abs(r['actualReturnTime']-r['otherSeconds']-sum(e['seconds'] for e in r['events'])) for r in rows);assert closure<1e-6
 audits[run]=dict(calls=len(rows),tasks=len({r['taskId'] for r in rows}),sceneClusters=len({r['baseSceneId'] for r in rows}),finalQualified=sum(r['finalQualified'] for r in rows),postAuditRejected=sum(r['onlinePassed'] and not r['finalQualified'] for r in rows),fineChecks=sum(bool(r.get('fineAuditSelected')) for r in rows),maxTimingClosureError=closure)
 allrows[run]=rows
 groups=collections.defaultdict(list)
 for r in rows:groups[(r['method'],r['B'],r['generalizationType'])].append(r)
 for (method,B,group),rr in sorted(groups.items()):
  stages=collections.Counter(r['returnStage'] for r in rr if r['finalQualified']);events=collections.defaultdict(float)
  for r in rr:
   for e in r['events']:events[e['stage']]+=e['seconds']/len(rr)
   events['other']+=r['otherSeconds']/len(rr)
  tables.append(dict(run=run,method=method,B=B,group=group,calls=len(rr),qualified=sum(r['finalQualified'] for r in rr),rate=np.mean([r['finalQualified'] for r in rr]),returnSecondsMean=np.mean([r['actualReturnTime'] for r in rr]),returnSecondsMedian=np.median([r['actualReturnTime'] for r in rr]),overBudget=sum(not r['returnedWithinBudget'] for r in rr),stages=dict(stages),failureReasons=dict(collections.Counter(r['reason'] for r in rr if not r['finalQualified'])),meanExclusiveSeconds=dict(events)))
# Every scenario shares the same resampled seed block; paired outcomes, no success-only filtering.
contrasts=[];rows=allrows['main'];pairs=[('ddpm_basic','retrieval'),('ddpm_basic','regression'),('ddpm_signature','ddpm_basic'),('ddpm_physloss','ddpm_basic'),('ddpm_guided','ddpm_basic')]
for group in ['G0','G1','G2','G3']:
 for B in [.5,1,2,5]:
  for method,baseline in pairs:
   rr=[r for r in rows if r['generalizationType']==group and r['B']==B and r['method'] in [method,baseline]]
   lookup={(r['taskId'],r['repeat'],r['method']):int(r['finalQualified']) for r in rr};assert len(lookup)==len(rr)
   scenes=sorted({r['baseSceneId'] for r in rr});blocks=sorted({r['repeat'] for r in rr})
   mat=np.array([[np.mean([lookup[(t,b,method)]-lookup[(t,b,baseline)] for t in sorted({r['taskId'] for r in rr if r['baseSceneId']==s})]) for b in blocks] for s in scenes])
   rng=np.random.default_rng(20260919);si=rng.integers(len(scenes),size=(2000,len(scenes)));bi=rng.integers(len(blocks),size=(2000,len(blocks)));draws=mat[si[:,:,None],bi[:,None,:]].mean(axis=(1,2))
   contrasts.append(dict(group=group,B=B,method=method,baseline=baseline,difference=float(mat.mean()),ci95=np.quantile(draws,[.025,.975]).tolist(),clusters=len(scenes),seedBlocks=len(blocks),bootstrap=2000))
write('audit.json',audits);write('method_group_budget.json',tables);write('paired_contrasts.json',contrasts)
mainmethods=['retrieval','regression','ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided','ihsid']
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,g in zip(axes.flat,['G0','G1','G2','G3']):
 for m in mainmethods:
  tt=sorted([t for t in tables if t['run']=='main' and t['group']==g and t['method']==m],key=lambda t:t['B']);ax.plot([t['B'] for t in tt],[100*t['rate'] for t in tt],marker='o',label=m)
 ax.set(title=g,xlabel='Planning budget (s)',ylabel='Qualified return (%)',ylim=(0,100))
axes[0,0].legend(fontsize=7);fig.savefig(OUT/'success_by_budget.png',dpi=160);plt.close(fig)
primary=[c for c in contrasts if c['group']=='G2' and c['B']==2]
fig,ax=plt.subplots(figsize=(10,4),layout='constrained');y=np.arange(len(primary));x=np.array([100*c['difference'] for c in primary]);lo=np.array([100*c['ci95'][0] for c in primary]);hi=np.array([100*c['ci95'][1] for c in primary]);ax.errorbar(x,y,xerr=[x-lo,hi-x],fmt='o');ax.set_yticks(y,[c['method']+' − '+c['baseline'] for c in primary]);ax.axvline(0,color='gray',ls='--');ax.set_xlabel('Paired difference (percentage points), G2 / 2 s');fig.savefig(OUT/'primary_contrasts.png',dpi=160);plt.close(fig)
# Quality raw metrics and pairwise comparisons only on jointly successful tasks.
quality=[];index={(r['taskId'],r['repeat'],r['B'],r['method']):r for r in rows}
for method in ['ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided']:
 for B in [2,5]:
  pairsq=[]
  for r in rows:
   if r['method']!=method or r['B']!=B or not r['finalQualified']:continue
   z=index[(r['taskId'],r['repeat'],B,'retrieval')]
   if z['finalQualified']:pairsq.append((r['rawMetrics'],z['rawMetrics']))
  fields=set.intersection(*[set(a)&set(b) for a,b in pairsq]) if pairsq else set()
  deltas={}
  for k in fields:
   if all(isinstance(a[k],(float,int)) and isinstance(b[k],(float,int)) for a,b in pairsq):
    v=[a[k]-b[k] for a,b in pairsq if np.isfinite(a[k]) and np.isfinite(b[k])]
    if v:deltas[k]=dict(n=len(v),medianDifference=float(np.median(v)),meanDifference=float(np.mean(v)))
  quality.append(dict(method=method,baseline='retrieval',B=B,jointlySuccessfulCalls=len(pairsq),metrics=deltas))
write('joint_success_quality.json',quality)
tr=''.join(f"<tr><td>{c['method']} − {c['baseline']}</td><td>{100*c['difference']:.2f}</td><td>[{100*c['ci95'][0]:.2f}, {100*c['ci95'][1]:.2f}]</td></tr>" for c in primary)
report=f"""<!doctype html><meta charset='utf-8'><title>Stewart载荷泛化：正式测试结果</title><style>body{{font:16px/1.85 system-ui;max-width:1100px;margin:30px auto;padding:0 24px;color:#203344}}img{{width:100%}}td,th{{padding:8px;border-bottom:1px solid #ddd}}pre{{white-space:pre-wrap}}</style><h1>Stewart载荷泛化：正式在线测试结果</h1><p>192条件、24基础场景、3种子区组。版本v2，空候选处理修复后全方法重跑；旧v1六条结果不混入。</p><h2>完整性审计</h2><pre>{html.escape(json.dumps(audits,ensure_ascii=False,indent=2))}</pre><h2>分组预算返回率</h2><img src='success_by_budget.png'><p>G0：新任务／已见载荷原型；G1：域内新载荷；G2：未见组合；G3：质量或长度轻度外推。端点非法和全部失败均保留。总预算包含检查，过期返回均不成功。</p><h2>预设主要比较：G2、2秒</h2><table><tr><th>比较</th><th>差/百分点</th><th>95%区间</th></tr>{tr}</table><img src='primary_contrasts.png'><p>按基础场景成簇bootstrap 2000次，训练种子为跨场景共享区组；只有3训练种子，区间存在局限。其余分组属适用范围分析，不依据正差挑选主结论。</p><h2>结果解释边界</h2><p>综合流程的成功包含精修和回退，不能全部归于网络直接生成。成功来源、失败原因、计时分解见method_group_budget.json；双方均成功轨迹的原始质量配对见joint_success_quality.json。机械功不是电能。独立5ms及预定2.5ms检查不是连续安全证明。</p><h2>仍在补充的内容</h2><p>测试参考解与历史全库诊断另行运行。本报告不宣称全部历史路径不可用，也不把优化失败当不可行证明；尚未完成的诊断保持unknown。当前不包含数据量重训实验、完整机制案例曲线或全库排除证据。</p></html>"""
(OUT/'implementation_report.html').write_text(report,encoding='utf-8')
print(json.dumps(dict(audit=audits,primary=primary),ensure_ascii=False))

# Additional outcome-source and cost panels, always including failures in timing.
fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained');bottom=np.zeros(len(mainmethods))
for stage in ['direct','retimed','repair','ihsid_fallback','ihsid']:
 y=[]
 for m in mainmethods:
  rr=[r for r in rows if r['method']==m and r['B']==2];y.append(100*sum(r['finalQualified'] and r['returnStage']==stage for r in rr)/len(rr))
 axes[0].bar(mainmethods,y,bottom=bottom,label=stage);bottom+=np.array(y)
axes[0].set_ylabel('Qualified returns / all calls (%)');axes[0].set_ylim(0,100);axes[0].legend(fontsize=7)
bottom=np.zeros(len(mainmethods))
for category,stages in [('generation',['generation','generation_other','encoding_and_retrieval','input_and_endpoint','outer_input','coarse_ranking','selection']),('guidance',['guidance']),('retiming',['retiming']),('checks',['physical','quality','acceptance_other']),('repair',['repair']),('solver',['solver','solver_initialization','solver_build','solver_other','ihsid_including_internal_acceptance'])]:
 y=[]
 for m in mainmethods:
  rr=[r for r in rows if r['method']==m and r['B']==2];y.append(np.mean([sum(e['seconds'] for e in r['events'] if e['stage'] in stages) for r in rr]))
 axes[1].bar(mainmethods,y,bottom=bottom,label=category);bottom+=np.array(y)
other=[]
for i,m in enumerate(mainmethods):
 rr=[r for r in rows if r['method']==m and r['B']==2];other.append(np.mean([r['actualReturnTime'] for r in rr])-bottom[i])
axes[1].bar(mainmethods,other,bottom=bottom,label='other');axes[1].set_ylabel('Mean real call time (s), failures included');axes[1].legend(fontsize=7)
for ax in axes:ax.tick_params(axis='x',rotation=35)
fig.savefig(OUT/'sources_and_timing.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,metric in zip(axes.flat,['positionLengthM','positionJerkRms','forceRmsN','absoluteMechanicalWorkJ']):
 vals=[[r['rawMetrics'][metric] for r in rows if r['method']==m and r['B']==2 and r['finalQualified']] for m in mainmethods[:-1]]
 ax.boxplot(vals,tick_labels=mainmethods[:-1],showfliers=False);ax.set_title(metric);ax.tick_params(axis='x',rotation=35)
fig.savefig(OUT/'successful_output_quality.png',dpi=160);plt.close(fig)
page=OUT/'implementation_report.html';s=page.read_text(encoding='utf-8');s=s.replace('<h2>结果解释边界</h2>',"<h2>成功来源与实际调用耗时</h2><img src='sources_and_timing.png'><h2>成功输出的原始质量分布</h2><img src='successful_output_quality.png'><p>此图的成功任务集合不同，仅描述分布；比较质量优劣应使用共同成功任务的配对结果，不能由此图均值直接判优。机械功不代表电能。</p><h2>结果解释边界</h2>")
s=s.replace('<h2>仍在补充的内容</h2>', '<h2>当前结论</h2><p>预设G2、2秒主比较中，基础扩散相对检索为−1.39个百分点，95%区间[−9.03,6.94]；没有支持基础扩散优于强检索的证据。物理签名相对基础扩散为+6.25个百分点，区间[−2.08,16.67]，存在正向点估计但证据仍不确定。物理损失和引导在该比较中没有增加返回率。应保留这些结论，不以其他有利分层替换主比较。</p><h2>仍在补充的内容</h2>')
page.write_text(s,encoding='utf-8')
