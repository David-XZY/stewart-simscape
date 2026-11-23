"""Offline analysis only; never alters tasks or numerical results."""
import json, collections, html, zipfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
SRC=ROOT/'results/chapter3/payload_validation_complete_v2'
OUT=ROOT/'results/chapter3/payload_validation_delivery_v2'
OUT.mkdir(parents=True,exist_ok=True)
rows=[json.loads(f.read_text(encoding='utf-8')) for f in (SRC/'payload_enhanced_validation_v2/calls').glob('*/result.json')]
def write(name,data): (OUT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
def summary(rr):
 strata=collections.defaultdict(list)
 for r in rr:
  s=r['job']['spec'];strata[(s['family'],s['taskClass'],s['shape'])].append(r['finalQualified'])
 return dict(calls=len(rr),passed=sum(r['finalQualified'] for r in rr),rawRate=np.mean([r['finalQualified'] for r in rr]),balancedRate=np.mean([np.mean(v) for v in strata.values()]),stages=dict(collections.Counter(r['returnStage'] for r in rr if r['finalQualified'])),failures=dict(collections.Counter(r['reason'] for r in rr if not r['finalQualified'])))
methods=sorted({r['method'] for r in rows});budgets=[2,5]
table=[dict(method=m,B=b,**summary([r for r in rows if r['method']==m and r['B']==b])) for m in methods for b in budgets]
contrasts=[]
for b in budgets:
 for a,z in [('ddpm_basic','retrieval'),('ddpm_basic','regression'),('ddpm_signature','ddpm_basic'),('ddpm_physloss','ddpm_basic'),('ddpm_guided','ddpm_basic')]:
  rr=[r for r in rows if r['B']==b and r['method'] in [a,z]]
  scenes=sorted({r['baseSceneId'] for r in rr});seeds=sorted({r['trainingSeed'] for r in rr})
  vals={(r['taskId'],r['trainingSeed'],r['method']):int(r['finalQualified']) for r in rr}
  matrix=np.array([[np.mean([vals[(t,s,a)]-vals[(t,s,z)] for t in sorted({r['taskId'] for r in rr if r['baseSceneId']==c})]) for s in seeds] for c in scenes])
  rng=np.random.default_rng(20260919);ci=rng.integers(len(scenes),size=(2000,len(scenes)));si=rng.integers(len(seeds),size=(2000,len(seeds)))
  draws=matrix[ci[:,:,None],si[:,None,:]].mean(axis=(1,2))
  contrasts.append(dict(method=a,baseline=z,B=b,difference=float(matrix.mean()),ci95=np.quantile(draws,[.025,.975]).tolist(),sceneClusters=len(scenes),seedBlocks=len(seeds),estimand='task equal; scene clustered and globally shared seed blocks'))
regions=[]
for field in ['shape','mass','family','taskClass']:
 for value in sorted({str(r['job']['spec'][field]) for r in rows}):
  for m in ['retrieval','ddpm_basic','regression']:
   rr=[r for r in rows if r['B']==2 and r['method']==m and str(r['job']['spec'][field])==value]
   regions.append(dict(field=field,value=value,method=m,**summary(rr)))
audit=dict(calls=len(rows),uniqueTasks=len({r['taskId'] for r in rows}),baseScenes=len({r['baseSceneId'] for r in rows}),onlinePassed=sum(r['onlinePassed'] for r in rows),fineChecks=sum(bool(r.get('fineAuditSelected')) for r in rows),postAuditFailures=sum(r['onlinePassed'] and not r['finalQualified'] for r in rows),invalidSuccesses=sum(r['finalQualified'] and (not r['returnedWithinBudget'] or not r['independentAuditPassed']) for r in rows),maxTimingClosureError=max(abs(r['actualReturnTime']-r['otherSeconds']-sum(e['seconds'] for e in r['events'])) for r in rows))
write('summary.json',dict(scope='development_validation_only',table=table,contrasts=contrasts,regions=regions,audit=audit))
shown=['ihsid','retrieval','regression','ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided']
fig,ax=plt.subplots(figsize=(10,4),layout='constrained')
for b in budgets:ax.plot(shown,[100*next(t['balancedRate'] for t in table if t['method']==m and t['B']==b) for m in shown],marker='o',label=f'{b} s')
ax.set_ylabel('Stratum-equal qualified return (%)');ax.tick_params(axis='x',rotation=20);ax.legend();ax.set_ylim(0,100);fig.savefig(OUT/'budget.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,4),layout='constrained');bottom=np.zeros(len(shown))
for stage in ['direct','repair','ihsid','ihsid_fallback']:
 y=np.array([100*next(t['stages'].get(stage,0)/t['calls'] for t in table if t['method']==m and t['B']==2) for m in shown]);ax.bar(shown,y,bottom=bottom,label=stage);bottom+=y
ax.set_ylabel('Task-equal returned calls (%)');ax.tick_params(axis='x',rotation=20);ax.legend();fig.savefig(OUT/'stages.png',dpi=160);plt.close(fig)
trs=''.join(f"<tr><td>{c['method']} − {c['baseline']}</td><td>{c['B']}</td><td>{100*c['difference']:.1f}</td><td>[{100*c['ci95'][0]:.1f}, {100*c['ci95'][1]:.1f}]</td></tr>" for c in contrasts)
doc=f"""<!doctype html><meta charset='utf-8'><title>载荷研究：开发验证完整分析</title><style>body{{max-width:1100px;margin:32px auto;font:16px/1.8 system-ui;color:#203344}}img{{width:100%}}td,th{{padding:8px;border-bottom:1px solid #ddd}}pre{{white-space:pre-wrap}}</style><h1>载荷研究：开发验证完整分析</h1><p>64条件、8基础场景、3共享种子区组。正式测试尚未开展。全部失败保留；重复调用不增加独立样本数。</p><h2>同预算返回能力</h2><img src='budget.png'><p>此图按环境、任务类别和形状分层等权；下方置信区间按任务等权，两者估计对象不同，不能直接混用百分点。</p><h2>配对差与95%区间</h2><p>2000次场景成簇bootstrap；每次全场景共享抽取训练种子。仅8场景、3种子，区间证据有限。</p><table><tr><th>比较</th><th>预算/s</th><th>差/百分点</th><th>95%区间</th></tr>{trs}</table><h2>成功来自哪里</h2><img src='stages.png'><p>direct是候选流程在公共精修前通过，不等于单个网络样本；引导方法的direct包含物理引导。基础扩散2秒140次成功中36次来自精修。</p><h2>验收与计时</h2><pre>{html.escape(json.dumps(audit,ensure_ascii=False,indent=2))}</pre><h2>解释边界</h2><p>基础扩散完整流程的点估计优于检索和回归，但单候选直接生成不优于回归；物理增强没有稳定提高总预算返回率。正式研究保留这些负结果，不据此调整测试域。IHSID短预算失败不能推断充分预算不可行。</p><p>载荷形状、质量、环境与任务类别的分区结果和失败原因完整保存在<a href='summary.json'>summary.json</a>。开发尺寸与质量仅少量档位，尚不能确定连续载荷能力边界。</p></html>"""
(OUT/'implementation_report.html').write_text(doc,encoding='utf-8')
print(json.dumps(contrasts,ensure_ascii=False))
