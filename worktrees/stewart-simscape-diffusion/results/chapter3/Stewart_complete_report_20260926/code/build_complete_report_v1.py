"""Read-only analysis of archived experiments; never changes experimental code."""
import pathlib,json,zipfile,gzip,collections,hashlib,html,shutil,datetime,re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=pathlib.Path(__file__).resolve().parent
ROOT=P.parents[1]
O=P/'Stewart_complete_report_20260926'
O.mkdir(exist_ok=True)
for d in ['data','figures','evidence','code']:(O/d).mkdir(exist_ok=True)
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write(name,obj):
 (O/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def archive(name,items):
 with gzip.open(O/'data'/name,'wt',encoding='utf-8') as f:
  for x in items:f.write(json.dumps(x,ensure_ascii=False,separators=(',',':'))+'\n')
methods=['ihsid','retrieval','regression','ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided']
labels=dict(zip(methods,['IHSID','增强检索','普通回归','基础扩散','物理签名扩散','物理损失扩散','物理引导扩散']))
rows=[];audits={};refs=[]
for zname in ['payload_heldout_complete_v2.zip','payload_budget10_failure_inputs.zip']:
 with zipfile.ZipFile(P/zname) as z:
  for name in z.namelist():
   if '/calls/' in name and name.endswith('/result.json'):
    r=json.loads(z.read(name));r['_source']=name
    r['_scope']='main' if '_main_' in name else ('controls' if '_controls_' in name else 'quality')
    rows.append(r)
   elif '/attempt_' in name and name.endswith('.json'):refs.append(json.loads(z.read(name)))
assert len(rows)==30096 and len(refs)==768
for group in ['main','controls','quality']:
 rr=[r for r in rows if r['_scope']==group]
 assert len({r['runId'] for r in rr})==len(rr)
 for r in rr:
  assert r['codeHash']=='65b808d13012949e1fd67f0344ed18e73206cccf2c3642d239eb28bc2f8fd24d'
  assert not r['finalQualified'] or (r['returnedWithinBudget'] and r['independentAuditPassed'] and (not r.get('fineAuditSelected') or r['fineAuditPassed']))
 closure=max(abs(r['actualReturnTime']-r['otherSeconds']-sum(e['seconds'] for e in r['events'])) for r in rr)
 assert closure<1e-6
 audits[group]={'calls':len(rr),'qualified':sum(r['finalQualified'] for r in rr),'onlinePassAuditReject':sum(r['onlinePassed'] and not r['finalQualified'] for r in rr),'fineChecks':sum(bool(r.get('fineAuditSelected')) for r in rr),'maxTimingClosureError':closure}
 archive(group+'_all_calls.jsonl.gz',rr)
archive('test_reference_attempts.jsonl.gz',refs)
main=[r for r in rows if r['_scope']=='main']; ten=[r for r in main if r['B']==10]
endpoint={r['taskId']:r['reason'] for r in refs if not r['endpointPassed']}
refok={r['taskId'] for r in refs if r['passed']}
tables=[]
for scope in ['main','controls','quality']:
 rs=[r for r in rows if r['_scope']==scope]
 for m in sorted({r['method'] for r in rs}):
  for b in sorted({r['B'] for r in rs if r['method']==m}):
   for g in ['ALL','G0','G1','G2','G3']:
    rr=[r for r in rs if r['method']==m and r['B']==b and (g=='ALL' or r['generalizationType']==g)]
    if not rr:continue
    ok=[r for r in rr if r['finalQualified']];valid=[r for r in rr if r['taskId'] not in endpoint]
    events=collections.defaultdict(float)
    for r in rr:
     for e in r['events']:events[e['stage']]+=e['seconds']/len(rr)
     events['other']+=r['otherSeconds']/len(rr)
    causes=collections.Counter()
    for r in rr:
     if not r['finalQualified']:causes.update(set(a.get('reason','') for a in r['attempts'] if a.get('reason')!='passed'))
    tables.append(dict(scope=scope,method=m,B=b,group=g,n=len(rr),qualified=len(ok),rate=len(ok)/len(rr),endpointEligible=len(valid),rateEligible=len(ok)/len(valid) if valid else None,
      boundaryRejected=sum(r['reason']=='boundary_rejected' for r in rr),timeout=sum(r['reason']=='timeout' for r in rr),postAuditRejected=sum(r['onlinePassed'] and not r['finalQualified'] for r in rr),
      returnMean=float(np.mean([r['actualReturnTime'] for r in rr])),returnP50=float(np.median([r['actualReturnTime'] for r in rr])),returnP95=float(np.quantile([r['actualReturnTime'] for r in rr],.95)),
      successReturnP50=float(np.median([r['actualReturnTime'] for r in ok])) if ok else None,stageCounts=dict(collections.Counter(r['returnStage'] for r in ok)),meanExclusiveSeconds=dict(events),
      candidateMean=float(np.mean([r['candidateGenerated'] for r in valid])) if valid else None,failedCallsObservedAttemptCausesOverlap=dict(causes)))
def stat(m,b,g='ALL',scope='main'):return next(t for t in tables if (t['method'],t['B'],t['group'],t['scope'])==(m,b,g,scope))
contrasts=[]
pairs=[('ddpm_basic','retrieval'),('ddpm_basic','regression'),('ddpm_signature','ddpm_basic'),('ddpm_physloss','ddpm_basic'),('ddpm_guided','ddpm_basic'),('ddpm_signature','retrieval')]
for b in [.5,1,2,5,10]:
 for group in ['ALL','G0','G1','G2','G3']:
  rr=[r for r in main if r['B']==b and (group=='ALL' or r['generalizationType']==group)]
  lookup={(r['taskId'],r['repeat'],r['method']):int(r['finalQualified']) for r in rr}
  scenes=sorted({r['baseSceneId'] for r in rr});bs=sorted({r['repeat'] for r in rr})
  tasks={s:sorted({r['taskId'] for r in rr if r['baseSceneId']==s}) for s in scenes}
  for m,base in pairs:
   matrix=np.array([[np.mean([lookup[t,k,m]-lookup[t,k,base] for t in tasks[s]]) for k in bs] for s in scenes])
   rng=np.random.default_rng(20260919);si=rng.integers(len(scenes),size=(2000,len(scenes)));bi=rng.integers(len(bs),size=(2000,len(bs)))
   draws=matrix[si[:,:,None],bi[:,None,:]].mean(axis=(1,2))
   contrasts.append(dict(group=group,B=b,method=m,baseline=base,difference=float(matrix.mean()),ci95=np.quantile(draws,[.025,.975]).tolist(),clusters=len(scenes),seedBlocks=len(bs),replicates=2000))
quality=[];index={(r['taskId'],r['repeat'],r['B'],r['method']):r for r in main}
for b in [2,5,10]:
 for m in methods:
  if m=='retrieval':continue
  paired=[]
  for r in main:
   if r['B']!=b or r['method']!=m or not r['finalQualified']:continue
   r2=index[r['taskId'],r['repeat'],b,'retrieval']
   if r2['finalQualified']:paired.append((r['rawMetrics'],r2['rawMetrics']))
  metrics={}
  if paired:
   for k in set.intersection(*[set(x)&set(y) for x,y in paired]):
    if all(isinstance(x[k],(float,int)) and isinstance(y[k],(float,int)) for x,y in paired):
     vals=[(x[k],y[k]) for x,y in paired if np.isfinite(x[k]) and np.isfinite(y[k])]
     if vals:
      v=np.array(vals);metrics[k]={'n':len(v),'methodMedian':float(np.median(v[:,0])),'retrievalMedian':float(np.median(v[:,1])),'pairedMedianDifference':float(np.median(v[:,0]-v[:,1])),'pairedMeanDifference':float(np.mean(v[:,0]-v[:,1]))}
  quality.append(dict(method=m,B=b,jointCalls=len(paired),metrics=metrics))
tasktable=[]
for tid in sorted({r['taskId'] for r in main}):
 r=next(r for r in main if r['taskId']==tid);success={}
 for scope in ['main','controls']:
  ss=[q for q in rows if q['taskId']==tid and q['B']==10 and q['_scope']==scope]
  success[scope]=dict(collections.Counter(q['method'] for q in ss if q['finalQualified']))
 tasktable.append({'task':r['job']['spec'],'endpointReason':endpoint.get(tid),'hasStrictReference':tid in refok,'success10':success})
write('data/summary.json',tables);write('data/paired_intervals.json',contrasts);write('data/joint_success_quality.json',quality);write('data/tasks_192.json',tasktable);write('data/integrity.json',audits)
# Full original failure audit, source-level declarations, and current dataset revision evidence.
for d,names in {
 'payload_budget10_failure_audit_v1':['failure_audit.json','failure_audit.html'],
 'payload_admission_v2_20260926':['audit_summary.json','difference_summary.json','selection.json'],
 'payload_condition_expansion_v2_longtime_20260926':['dataset_report.json','physical_acceptance_report.json','time_scaling_report.json'],
 'payload_retrain474_v1':['status.json','execution_snapshot.json','preflight.json','integration.json','diagnostic_tests.json','training_resume_tests.json'],
 'payload_audit_v1':['matlab_audit.json','torch_audit.json','python_tests.json'],
 'boundary_delivery_v1':['implementation_status.json'],
 'boundary_payload_pilot_v1':['summary.json'],
 'boundary_payload_reuse_v1':['summary.json'],
 'chapter3_v3/report':['summary.json'],
}.items():
 for name in names:
  f=P/d/name
  if f.exists():shutil.copy2(f,O/'evidence'/(d.replace('/','_')+'__'+name))
for name in ['dataset_report.json','dataset.json','library.json']:
 f=P/'payload_heldout_complete_v2/payload_formal_dataset_v1'/name
 if f.exists():
  if name=='dataset_report.json':shutil.copy2(f,O/'evidence'/('formal_'+name))
  else:
   with gzip.open(O/'data'/('formal_'+name+'.gz'),'wb') as w:w.write(f.read_bytes())
for name in ['main_frozen.json','controls_frozen.json','quality_frozen.json']:
 f=P/'payload_heldout_complete_v2/payload_test_orchestration_v2'/name
 with gzip.open(O/'data'/name.replace('.json','.json.gz'),'wb') as w:w.write(f.read_bytes())
for directory in ['chapter3/payload_generalization','chapter3/boundary','diffusion','opt_minimal/core','chapter3']:
 for f in (ROOT/directory).glob('*'):
  if f.is_file() and f.suffix in ['.m','.py','.json']:
   target=O/'code'/directory/f.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,target)
# Earlier study numeric logs are retained as a separate version, never pooled with payload.
old=list((P/'chapter3_v3/jobs').glob('*/online_summary.json'))
archive('historical_chapter3_v3_calls.jsonl.gz',({'_source':str(f.relative_to(P)),**read(f)} for f in old))
plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','SimHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':10})
fig,ax=plt.subplots(figsize=(10,4.7),layout='constrained')
for m in methods:
 bs=[.5,1,2,5,10];ax.plot(bs,[100*stat(m,b)['rate'] for b in bs],'-o',label=labels[m])
ax.set(xlabel='在线总预算 B / 秒（包含检查）',ylabel='最终物理合格返回率 / %',ylim=(0,100),xticks=bs);ax.legend(ncol=3,fontsize=9);ax.grid(alpha=.2)
fig.savefig(O/'figures/budget.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
for ax,b in zip(axs,[2,10]):
 cs=[c for c in contrasts if c['B']==b and c['group']=='G2'][:5];yy=np.arange(len(cs));xx=np.array([100*c['difference'] for c in cs]);ll=np.array([100*c['ci95'][0] for c in cs]);hh=np.array([100*c['ci95'][1] for c in cs]);ax.errorbar(xx,yy,xerr=[xx-ll,hh-xx],fmt='o');ax.set_yticks(yy,[labels[c['method']]+'−'+labels[c['baseline']] for c in cs],fontsize=8);ax.axvline(0,color='gray',ls='--');ax.set(title=f'G2，B={b}秒'+('（预设主比较）' if b==2 else '（追加分析）'),xlabel='成功率差 / 百分点，95%区间')
fig.savefig(O/'figures/intervals.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(12,4.6),layout='constrained')
stages=['direct','retimed','repair','ihsid_fallback','ihsid'];bottom=np.zeros(len(methods))
for stage,label in zip(stages,['直接/引导输出','重新计时','公共精修','IHSID回退','独立IHSID']):
 vals=[100*stat(m,10)['stageCounts'].get(stage,0)/576 for m in methods];axs[0].bar([labels[m] for m in methods],vals,bottom=bottom,label=label);bottom+=np.array(vals)
axs[0].set(ylabel='占全部调用的比例 / %',ylim=(0,100),title='10秒成功来自哪个阶段');axs[0].legend(fontsize=8)
bottom=np.zeros(len(methods))
for key,label in [('boundaryRejected','端点拒绝'),('timeout','最终超时'),('postAuditRejected','离线复查拒绝')]:
 vals=[stat(m,10)[key] for m in methods];axs[1].bar([labels[m] for m in methods],vals,bottom=bottom,label=label);bottom+=np.array(vals)
axs[1].set(ylabel='失败调用数 / 每方法576次',title='10秒失败的互斥分类');axs[1].legend(fontsize=8)
for ax in axs:ax.tick_params(axis='x',rotation=35)
fig.savefig(O/'figures/stages_failures.png',dpi=170);plt.close(fig)
write('data/key_findings.json',{'main10':[stat(m,10) for m in methods],'G2B10':[c for c in contrasts if c['B']==10 and c['group']=='G2'],'ALLB10':[c for c in contrasts if c['B']==10 and c['group']=='ALL'],'historicalCalls':len(old)})
print(json.dumps({'audit':audits,'G2B10':[c for c in contrasts if c['B']==10 and c['group']=='G2'],'ten':[{'m':m,'mean':stat(m,10)['returnMean'],'stages':stat(m,10)['stageCounts']} for m in methods]},ensure_ascii=False))
