"""Frozen endpoint-pass supplemental analysis. No planner/training mutations."""
import argparse,collections,gzip,hashlib,html,itertools,json,pathlib,shutil,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

P=pathlib.Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--source',type=pathlib.Path,default=P/'Stewart_complete_report_20260926/data');parser.add_argument('--output',type=pathlib.Path,default=P/'payload_endpoint_pass_figures_v1_20260927');parser.add_argument('--supplement',type=pathlib.Path);parser.add_argument('--snapshot',type=pathlib.Path,help='Replot from a delivered v1 package without the larger source archive')
args=parser.parse_args();SRC=args.source;OUT=args.output
OUT.mkdir(exist_ok=True)
for d in ['figures','data']:(OUT/d).mkdir(exist_ok=True)
M=['ddpm_basic','regression','ddpm_signature','regression_signature','ihsid'];BS=[1,2,5,10]
LABEL=dict(zip(M,['基础扩散','普通回归','物理签名扩散','物理签名回归','IHSID']))
SHORT=dict(zip(M,['扩散','回归','签名扩散','签名回归','IHSID']))
COL=dict(zip(M,['#2876b9','#df8428','#138774','#a75c9f','#55626c']))
plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','SimHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':10,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write(name,x):(OUT/name).write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def jsonl(p):
 with gzip.open(p,'rt',encoding='utf-8') as f:
  for line in f:yield json.loads(line)
if args.snapshot:
 assert OUT.resolve()!=args.snapshot.resolve(),'Keep the delivered snapshot unchanged; choose a new output.'
 tasks=[{'task':t,'endpointReason':None} for t in read(args.snapshot/'data/included_tasks.json')]+[{'task':t['spec'],'endpointReason':t['reason']} for t in read(args.snapshot/'data/excluded_tasks.json')]
else:tasks=read(SRC/'tasks_192.json')
specs={t['task']['taskId']:t['task'] for t in tasks}
excluded={t['task']['taskId']:t['endpointReason'] for t in tasks if t['endpointReason']}
valid=set(specs)-set(excluded)
assert len(excluded)==46 and len(valid)==146
assert collections.Counter(excluded.values())=={'endpoint_leg':32,'endpoint_clearance_not_certified':14}
rows=[];removed=collections.Counter();sources={}
for s in ([] if args.snapshot else ['main','controls']):
 f=SRC/(s+'_all_calls.jsonl.gz');sources[f.name]=sha(f)
 for r in jsonl(f):
  if r['method'] not in M or r['B'] not in BS:continue
  assert (r['method']=='regression_signature')==(s=='controls')
  if r['taskId'] in excluded:
   assert r['reason']=='boundary_rejected';removed[(r['method'],r['B'])]+=1;continue
  assert r['taskId'] in valid and r['reason']!='boundary_rejected';rows.append(r)
if args.snapshot:
 rows=list(jsonl(args.snapshot/'data/filtered_calls.jsonl.gz'))
 sources=read(args.snapshot/'analysis_config.json')['sourceSha256']
 for r in rows:removed[r['method'],r['B']]=138
assert len(rows)==7884
identity={'codeHash':rows[0]['codeHash'],'libraryHash':rows[0]['libraryHash']}
if args.supplement:
 assert OUT.name!='payload_endpoint_pass_figures_v1_20260927','Supplement must use a new output directory.'
 proof=read(args.supplement/'compatibility.json')
 assert all(proof[k]==v for k,v in identity.items())
 assert proof['originalHardwareRuntimeThreadCacheIdentityVerified'] is True
 supplements=list(jsonl(args.supplement/'calls.jsonl.gz'));assert len(supplements)==876
 baseline={(r['taskId'],r['repeat']):r for r in rows if r['method']=='regression_signature' and r['B']==2}
 for r in supplements:
  assert r['method']=='regression_signature' and r['B'] in [1,5] and r['taskId'] in valid
  old=baseline[r['taskId'],r['repeat']]
  for k in ['codeHash','libraryHash','taskHash','trainingSeed','samplingSeed','returnPolicy']:assert r[k]==old[k],k
  assert r['job']['modelKey']==old['job']['modelKey'] and r['job']['spec']==old['job']['spec']
 rows.extend(supplements);sources['supplement_calls']=sha(args.supplement/'calls.jsonl.gz')
 assert len({r['runId'] for r in rows})==len(rows)
if not args.snapshot:sources['tasks_192.json']=sha(SRC/'tasks_192.json')
for r in rows:
 assert r['codeHash']==identity['codeHash'] and r['returnPolicy']=='physical'
 assert not r['finalQualified'] or (r['returnedWithinBudget'] and r['independentAuditPassed'] and (not r.get('fineAuditSelected') or r['fineAuditPassed']))
 assert abs(r['actualReturnTime']-r['otherSeconds']-sum(e['seconds'] for e in r['events']))<1e-6
 ids=[e['id'] for e in r['candidateEvents']];assert len(ids)==len(set(ids))
 assert r['candidateGenerated']<=len(ids)
 if r['method'] in ['regression','regression_signature']:assert r['candidateGenerated']<=1
groups={(m,b):[r for r in rows if r['method']==m and r['B']==b] for m in M for b in BS}
for (m,b),rs in groups.items():
 if not rs:assert m=='regression_signature' and b in [1,5];continue
 assert len(rs)==438 and {r['taskId'] for r in rs}==valid
 assert len({(r['taskId'],r['repeat']) for r in rs})==438
 if (m,b) in removed:assert removed[m,b]==138
with gzip.open(OUT/'data/filtered_calls.jsonl.gz','wt',encoding='utf-8') as f:
 for r in rows:f.write(json.dumps(r,ensure_ascii=False,separators=(',',':'))+'\n')
write('data/excluded_tasks.json',[{'taskId':t,'reason':why,'spec':specs[t]} for t,why in sorted(excluded.items())])
write('data/included_tasks.json',[specs[t] for t in sorted(valid)])
scenes=sorted({specs[t]['baseSceneId'] for t in valid});assert len(scenes)==20
def boot(rs,value):
 """Ratio of resampled sums, preserving unequal cluster sizes and shared seeds."""
 ss=sorted({r['baseSceneId'] for r in rs});ks=sorted({r['repeat'] for r in rs});tot=np.zeros((len(ss),len(ks)));cnt=tot.copy()
 si={s:i for i,s in enumerate(ss)};ki={k:i for i,k in enumerate(ks)}
 for r in rs:
  v=value(r)
  if v is None or not np.isfinite(v):continue
  i,j=si[r['baseSceneId']],ki[r['repeat']];tot[i,j]+=v;cnt[i,j]+=1
 rng=np.random.default_rng(20260927);ii=rng.integers(len(ss),size=(2000,len(ss)));jj=rng.integers(len(ks),size=(2000,len(ks)))
 den=cnt[ii[:,:,None],jj[:,None,:]].sum((1,2));num=tot[ii[:,:,None],jj[:,None,:]].sum((1,2));ds=num[den>0]/den[den>0]
 return {'mean':float(tot.sum()/cnt.sum()),'ci95':np.quantile(ds,[.025,.975]).tolist(),'n':int(cnt.sum()),'tasks':len({r['taskId'] for r in rs}),'scenes':len(ss),'seedBlocks':len(ks),'bootstrap':2000,'validDraws':len(ds)}
summary=[]
for m in M:
 for b in BS:
  rs=groups[m,b]
  if not rs:summary.append({'method':m,'B':b,'status':'unmeasured','n':0});continue
  z=boot(rs,lambda r:int(r['finalQualified']));z.update(method=m,B=b,status='measured',qualified=sum(r['finalQualified'] for r in rs),returnMean=float(np.mean([r['actualReturnTime'] for r in rs])),returnP50=float(np.median([r['actualReturnTime'] for r in rs])),returnP95=float(np.quantile([r['actualReturnTime'] for r in rs],.95)));summary.append(z)
def stat(m,b):return next(s for s in summary if s['method']==m and s['B']==b)
pairs=[('ddpm_basic','regression'),('ddpm_signature','regression_signature'),('ddpm_signature','ddpm_basic'),('regression_signature','regression')]
contrasts=[]
for a,b in pairs:
 for budget in BS:
  aa=groups[a,budget];bb=groups[b,budget]
  if not aa or not bb:contrasts.append({'method':a,'baseline':b,'B':budget,'status':'unmeasured'});continue
  idx={(r['taskId'],r['repeat']):r for r in bb}
  z=boot(aa,lambda r:int(r['finalQualified'])-int(idx[r['taskId'],r['repeat']]['finalQualified']));z.update(method=a,baseline=b,B=budget,status='measured');contrasts.append(z)
def save(fig,name):
 # Reserve physical header space: normalized offsets overlap on short figures.
 fig.canvas.draw();positions=[ax.get_position().frozen() for ax in fig.axes];fig.set_layout_engine(None)
 w,h=fig.get_size_inches();extra=1.35 if fig.legends else .9;fig.set_size_inches(w,h+extra)
 for ax,pos in zip(fig.axes,positions):ax.set_position([pos.x0,pos.y0*h/(h+extra),pos.width,pos.height*h/(h+extra)])
 for lg in fig.legends:lg.set_bbox_to_anchor((.5,(h+.45)/(h+extra)),transform=fig.transFigure)
 s,sub=fig._report_title;fig.text(.5,(h+extra-.08)/(h+extra),s,fontsize=18,fontweight='bold',ha='center',va='top');fig.text(.5,(h+extra-.48)/(h+extra),sub,fontsize=10,color='#536775',ha='center',va='top')
 fig.savefig(OUT/'figures'/(name+'.png'),dpi=260,bbox_inches='tight');fig.savefig(OUT/'figures'/(name+'.svg'),bbox_inches='tight');plt.close(fig)
def methodlegend(fig,y=1.01):fig.legend(handles=[Patch(color=COL[m],label=LABEL[m]) for m in M],loc='upper center',bbox_to_anchor=(.5,y),ncol=5,frameon=False,fontsize=10)
def title(fig,s,sub):fig._report_title=(s,sub)
def grid(ax):ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
def missing(ax,x,y):ax.text(x,y,'未测',ha='center',va='bottom',color='#8d969d',fontsize=9)

# 1. Rates and paired effects: NaN leaves genuinely missing budget links broken.
fig,axs=plt.subplots(1,2,figsize=(14,5),layout='constrained')
for m in M:
 yy=[100*stat(m,b)['mean'] if groups[m,b] else np.nan for b in BS];lo=[100*stat(m,b)['ci95'][0] if groups[m,b] else np.nan for b in BS];hi=[100*stat(m,b)['ci95'][1] if groups[m,b] else np.nan for b in BS]
 axs[0].plot(BS,yy,'o-',color=COL[m],label=LABEL[m],lw=2);axs[0].fill_between(BS,lo,hi,color=COL[m],alpha=.1)
 for x,y,l,h in zip(BS,yy,lo,hi):
  if np.isfinite(y):axs[0].vlines(x,l,h,color=COL[m],alpha=.7,lw=1)
axs[0].set(xlabel='在线总预算 B / s',ylabel='最终物理合格率 / %',xticks=BS,ylim=(-2,103),title='146任务 × 3区组；每个完整数据点 n=438');grid(axs[0])
missingBudgets=[b for b in BS if not groups['regression_signature',b]]
if missingBudgets:axs[0].text(.03,.12,'签名回归：'+ '、'.join(f'{b} s' for b in missingBudgets)+'未测',transform=axs[0].transAxes,color=COL['regression_signature'])
for i,(a,b) in enumerate(pairs):
 cc=[next(c for c in contrasts if (c['method'],c['baseline'],c['B'])==(a,b,t)) for t in BS];yy=np.array([c['mean']*100 if c['status']=='measured' else np.nan for c in cc]);lo=np.array([c['ci95'][0]*100 if c['status']=='measured' else np.nan for c in cc]);hi=np.array([c['ci95'][1]*100 if c['status']=='measured' else np.nan for c in cc]);xx=np.arange(4)+(i-1.5)*.12
 axs[1].errorbar(xx,yy,yerr=[yy-lo,hi-yy],fmt='o',capsize=3,label=SHORT[a]+' − '+SHORT[b])
axs[1].axhline(0,color='#666',ls='--');axs[1].set(xticks=np.arange(4),xticklabels=BS,xlabel='在线总预算 B / s',ylabel='配对成功率差 / 百分点',title='2000次场景成簇重抽样；95%区间');axs[1].legend(fontsize=8,loc='upper left');grid(axs[1]);methodlegend(fig);title(fig,'图1  预算与合格返回率','端点通过子集补充分析；区间跨0时不认定优势');save(fig,'01_success_budget')

# 2. Generalization, shape, and scene-family strata.
strata=[];fig,axs=plt.subplots(2,2,figsize=(14,9),layout='constrained')
for ax,b,key,values,names in [(axs[0,0],2,'generalizationType',['G0','G1','G2','G3'],['G0 已见载荷新任务','G1 域内新载荷','G2 未见组合','G3 轻度外推']), (axs[0,1],10,'generalizationType',['G0','G1','G2','G3'],['G0','G1','G2','G3']), (axs[1,0],10,'shape',['cylinder','box'],['圆柱','箱体']), (axs[1,1],10,'family',['open','dual_fixture'],['开放环境','双工装'])]:
 ticks=[]
 for v,name in zip(values,names):
  n=sum(specs[t][key]==v for t in valid);ticks.append(f'{name}\n{n}任务 / {n*3}次')
 for i,m in enumerate(M):
  ys=[];los=[];his=[]
  for v in values:
   rr=[r for r in groups[m,b] if specs[r['taskId']][key]==v];z=boot(rr,lambda r:int(r['finalQualified']));z.update(method=m,B=b,stratum=key,value=v,qualified=sum(r['finalQualified'] for r in rr));strata.append(z);ys.append(z['mean']*100);los.append(z['ci95'][0]*100);his.append(z['ci95'][1]*100)
  xx=np.arange(len(values))+(i-2)*.15;ax.bar(xx,ys,.14,color=COL[m]);ax.errorbar(xx,ys,yerr=[np.array(ys)-los,np.array(his)-ys],fmt='none',ecolor='#4f5960',lw=.7,capsize=1.5)
 ax.set(xticks=np.arange(len(values)),xticklabels=ticks,ylim=(0,103),ylabel='合格率 / %',title=f'B={b} s · '+{'generalizationType':'泛化类型','shape':'载荷形状','family':'环境类型'}[key]);grid(ax)
methodlegend(fig);title(fig,'图2  泛化与载荷适用范围','每组保留全部端点通过任务；组间分母不同，不按胜负挑选任务');save(fig,'02_generalization')

# 3. Exclusive event mapping, including rare composite exception timers.
stageNames=['输入/编码','生成','排序','检查','重新计时','精修','NLP','其他']
mapping={'input_and_endpoint':0,'encoding_and_retrieval':0,'outer_input':0,'generation':1,'generation_other':1,'coarse_ranking':2,'selection':2,'physical':3,'quality':3,'acceptance_other':3,'retiming':4,'repair':5,'solver_initialization':6,'solver_build':6,'solver':6,'solver_other':6,'ihsid_including_internal_acceptance':6}
def times(r):
 x=np.zeros(8);x[-1]=r['otherSeconds']
 for e in r['events']:
  if e['stage']=='guidance':assert e['seconds']==0;continue
  assert e['stage'] in mapping,e['stage'];x[mapping[e['stage']]]+=e['seconds']
 assert abs(x.sum()-r['actualReturnTime'])<1e-6;return x
timeTable=[];fig,axs=plt.subplots(4,2,figsize=(14,15),layout='constrained');sc=plt.cm.tab20c(np.linspace(0,1,8))
for j,b in enumerate(BS):
 ax=axs[j,0]
 for i,m in enumerate(M):
  rr=groups[m,b]
  if not rr:missing(ax,i,.03);continue
  v=np.array([r['actualReturnTime'] for r in rr]);box=ax.boxplot([v],positions=[i],widths=.5,patch_artist=True,showfliers=True,flierprops={'markersize':2,'alpha':.2});box['boxes'][0].set(facecolor=COL[m],alpha=.65)
  t=np.mean([times(r) for r in rr],axis=0);timeTable.append({'method':m,'B':b,'n':len(rr),'stageMeanSeconds':dict(zip(stageNames,t.tolist())),'totalMeanSeconds':float(v.mean()),'P50':float(np.median(v)),'P95':float(np.quantile(v,.95)),'maxSeconds':float(v.max())})
 ax.axhline(b,color='#c44139',ls='--',label=f'预算 {b}s');ax.set(xticks=range(5),xticklabels=[SHORT[m] for m in M],ylabel='实际返回时间 / s',title=f'B={b}s · 全438次分布，含失败');grid(ax)
 ax=axs[j,1];bottom=np.zeros(5)
 for k,name in enumerate(stageNames):
  vals=[next(t for t in timeTable if t['method']==m and t['B']==b)['stageMeanSeconds'][name] if groups[m,b] else 0 for m in M];ax.bar(range(5),vals,bottom=bottom,color=sc[k],label=name);bottom+=vals
 for i,m in enumerate(M):
  if not groups[m,b]:missing(ax,i,0)
 ax.axhline(b,color='#c44139',ls='--',lw=.8);ax.set(xticks=range(5),xticklabels=[SHORT[m] for m in M],ylabel='平均耗时 / s',title=f'B={b}s · 互斥阶段平均时间');grid(ax)
axs[0,1].legend(ncol=4,fontsize=8,loc='upper left');title(fig,'图3  真实耗时与时间分解','不截断超时值；迭代回调并非硬实时中断；少量求解异常计时保留为NLP复合时间');save(fig,'03_time_decomposition')

# 4. Mutually exclusive outcomes and overlapping observed candidate gates.
outcomeNames=['直接成功','公共精修成功','IHSID回退成功','独立IHSID成功','超时失败','复查拒绝','其他失败'];oc=['#1b9e77','#70bc95','#4a8bc5','#64737d','#de8f35','#c64e56','#b3b6bb']
def outcome(r):
 if r['finalQualified']:
  assert r['returnStage'] in ['direct','repair','ihsid_fallback','ihsid'];return {'direct':0,'repair':1,'ihsid_fallback':2,'ihsid':3}[r['returnStage']]
 if r['onlinePassed']:return 5
 if r['reason']=='timeout':return 4
 return 6
outcomes=[];causeTable=[];causes=[('clearance_not_certified','间隙未获证书'),('leg_kinematics','腿运动违规'),('force','力超限'),('wall_budget','求解预算终止')]
fig,axs=plt.subplots(2,2,figsize=(14,10),layout='constrained')
for j,b in enumerate([2,10]):
 ax=axs[0,j];bottom=np.zeros(5)
 for k,name in enumerate(outcomeNames):
  vals=[]
  for m in M:
   rr=groups[m,b];v=sum(outcome(r)==k for r in rr);vals.append(v);outcomes.append({'method':m,'B':b,'outcome':name,'calls':v,'denominator':438})
  ax.bar(range(5),vals,bottom=bottom,color=oc[k],label=name)
  for i,v in enumerate(vals):
   if v>=15:ax.text(i,bottom[i]+v/2,str(v),ha='center',va='center',fontsize=9,color='white' if k in [0,3,5] else '#25323a')
  bottom+=vals
 assert all(bottom==438);ax.set(xticks=range(5),xticklabels=[SHORT[m] for m in M],ylim=(0,445),ylabel='调用数 / 每方法438次',title=f'B={b}s · 互斥结果分类')
 ax=axs[1,j];mat=[]
 for m in M:
  rr=[r for r in groups[m,b] if not r['finalQualified']];cc=[]
  for key,name in causes:
   n=sum(any((key in a.get('reason','') if key=='wall_budget' else a.get('reason')==key) for a in r['attempts']) for r in rr);cc.append(n);causeTable.append({'method':m,'B':b,'observedReason':key,'failedCallsWithReason':n,'totalFailedCalls':len(rr),'overlap':True})
  mat.append(cc)
 ax.imshow(mat,cmap='YlOrRd',vmin=0,vmax=438,aspect='auto');ax.set(yticks=range(5),yticklabels=[SHORT[m] for m in M],xticks=range(4),xticklabels=[c[1] for c in causes],title=f'B={b}s · 失败调用中观察到的原因（可重叠）')
 for i,row in enumerate(mat):
  for k,n in enumerate(row):ax.text(k,i,str(n),ha='center',va='center',color='white' if n>250 else '#20313e')
fig.legend(handles=[Patch(color=c,label=n) for c,n in zip(oc,outcomeNames)],loc='upper center',bbox_to_anchor=(.5,1.005),ncol=4,fontsize=9,frameon=False);title(fig,'图4  成功来源与失败去向','违规原因记录的是早拒绝门；间隙未获证书不等于已证明碰撞');save(fig,'04_outcomes_failures')

# 5/6. Common matched successes, never unrelated success populations.
idx={m:{(r['taskId'],r['repeat']):r for r in groups[m,10]} for m in M}
joint=sorted(set.intersection(*[{k for k,r in idx[m].items() if r['finalQualified']} for m in M]));assert len(joint)==248
jointMeta={'matchedCallsPerMethod':len(joint),'tasks':len({t for t,k in joint}),'scenes':len({specs[t]['baseSceneId'] for t,k in joint}),'B':10,'keys':joint}
QM=[('positionLengthM','位置路径长度','m',1),('rotationLengthRad','旋转路径长度','rad',1),('legTotalVariation','六腿总行程','m',1),('positionAccelRms','平台线加速度 RMS','m/s²',1),('angularAccelRms','平台角加速度 RMS','rad/s²',1),('legAccelRms','腿加速度 RMS','m/s²',1),('positionJerkRms','平台线 jerk RMS','m/s³',1),('angularJerkRms','平台角 jerk RMS','rad/s³',1),('legJerkRms','腿 jerk RMS','m/s³',1),('forceRmsN','驱动力 RMS','N',1),('forceRateRms','力变化率 RMS','N/s',1),('absoluteMechanicalWorkJ','绝对机械功','J',1)]
def metric(r,k):
 if k=='legTotalVariation':return sum(r['rawMetrics'][k])
 return r['rawMetrics'][k]
def boxscatter(ax,values,title,unit,threshold=None):
 rng=np.random.default_rng(7)
 for i,(m,v) in enumerate(zip(M,values)):
  v=np.asarray(v,float);v=v[np.isfinite(v)]
  if not len(v):missing(ax,i,0);continue
  box=ax.boxplot([v],positions=[i],widths=.5,patch_artist=True,showfliers=False,medianprops={'color':'#172b3b','lw':1.3});box['boxes'][0].set(facecolor=COL[m],alpha=.35)
  ax.scatter(i+rng.uniform(-.16,.16,len(v)),v,s=5,color=COL[m],alpha=.25,linewidths=0,rasterized=False)
 if threshold is not None:ax.axhline(threshold,color='#c7443d',ls='--',lw=1)
 ax.set(xticks=range(5),xticklabels=[SHORT[m] for m in M],title=title,ylabel=unit);ax.tick_params(axis='x',labelsize=8,rotation=20);grid(ax)
qualityRows=[];fig,axs=plt.subplots(3,4,figsize=(16,11),layout='constrained')
for ax,(k,name,u,scale) in zip(axs.flat,QM):
 vals=[[metric(idx[m][key],k)*scale for key in joint] for m in M];boxscatter(ax,vals,name,u)
 for m,v in zip(M,vals):qualityRows.append({'method':m,'metric':k,'unit':u,'n':len(v),'median':float(np.median(v)),'q25':float(np.quantile(v,.25)),'q75':float(np.quantile(v,.75)),'mean':float(np.mean(v))})
title(fig,'图5  共同成功轨迹的品质',f'B=10s；五方法共同成功：248组匹配调用 / {jointMeta["tasks"]}任务 / {jointMeta["scenes"]}场景；机械功不是电能');save(fig,'05_quality_matched')
SM=[('minClearance','载荷间隙余量','mm',1000,0),('minimumLegLengthMargin','腿长最小余量','mm',1000,0),('maxSpeedUtilization','腿速最大利用率','%',100,100),('maxAccelerationUtilization','腿加速度最大利用率','%',100,100),('maxForceUtilization','驱动力最大利用率','%',100,100),('minSigma','归一化最小奇异值','',1,.1),('maxCondition','最大条件数','',1,20)]
def safety(r,k):
 if k in ['minClearance','minSigma','maxCondition']:
  v=r['report']['physical'].get(k)
  if v is None:return None
  return v-.01 if k=='minClearance' else v
 return r['rawMetrics'][k]
safetyRows=[];fig,axs=plt.subplots(2,4,figsize=(16,8),layout='constrained')
for ax,(k,name,u,scale,threshold) in zip(axs.flat,SM):
 vals=[]
 for m in M:
  vv=[safety(idx[m][key],k) for key in joint];v=[x*scale for x in vv if x is not None and np.isfinite(x)];vals.append(v);safetyRows.append({'method':m,'metric':k,'unit':u,'n':len(v),'notApplicable':sum(x is None for x in vv),'median':float(np.median(v)) if v else None,'min':float(min(v)) if v else None,'max':float(max(v)) if v else None})
 boxscatter(ax,vals,name,u,threshold)
 if k=='minClearance':ax.set_title(name+f'（适用 n={len(vals[0])}）')
axs[1,3].axis('off');axs[1,3].text(0,.9,'判读方式\n\n余量：越大越宽裕\n利用率：越接近100%越紧\n最小奇异值：不得低于0.1\n条件数：不得超过20\n\n无障碍间隙记“不适用”，不填大数。\n散点是匹配调用，不是独立场景。\n数值稠密检查不是连续安全证明。',va='top',fontsize=11)
title(fig,'图6  约束余量与稳健程度','与图5使用完全相同的248组匹配调用；红虚线为硬阈值；保留全部散点');save(fig,'06_safety_margins')

# Pairwise quality effects on each pair's own shared successes, all 10 pairs.
pairQuality=[]
for a,b in itertools.combinations(M,2):
 for budget in BS:
  aa=groups[a,budget];bb=groups[b,budget]
  if not aa or not bb:continue
  lookup={(r['taskId'],r['repeat']):r for r in bb};match=[r for r in aa if r['finalQualified'] and lookup[r['taskId'],r['repeat']]['finalQualified']]
  if not match:continue
  for k,name,u,scale in QM:
   fn=lambda r:scale*(metric(r,k)-metric(lookup[r['taskId'],r['repeat']],k))
   z=boot(match,fn);z.update(method=a,baseline=b,B=budget,metric=k,unit=u,pairedMedianDifference=float(np.median([fn(r) for r in match])));pairQuality.append(z)

# 7. Generated unique candidates vs outcome, and real candidate-phase time.
candidateRows=[];fig,axs=plt.subplots(4,2,figsize=(14,15),layout='constrained');cc=['#168475','#d58a2e','#ba5561'];statusNames=['直接成功','精修/回退成功','失败'];lm=M[:-1]
for j,b in enumerate(BS):
 ax=axs[j,0]
 for i,m in enumerate(lm):
  rr=groups[m,b]
  if not rr:missing(ax,i,0);continue
  for status in range(3):
   subset=[r for r in rr if (0 if r['finalQualified'] and r['returnStage']=='direct' else 1 if r['finalQualified'] else 2)==status]
   hist=collections.Counter(r['candidateGenerated'] for r in subset)
   for n,count in hist.items():ax.scatter(i+(status-1)*.2,n,s=12+count*.5,c=cc[status],alpha=.7,edgecolors='white',lw=.4)
   candidateRows.append({'method':m,'B':b,'outcome':statusNames[status],'calls':len(subset),'generatedHistogram':dict(sorted(hist.items()))})
 ax.set_yscale('symlog',linthresh=1,base=2,linscale=.8)
 ax.set(xticks=range(4),xticklabels=[SHORT[m] for m in lm],yticks=[0,1,4,8,16,32,64],yticklabels=[0,1,4,8,16,32,64],ylim=(-.15,80),ylabel='唯一候选数（非线性刻度）',title=f'B={b}s · 气泡面积反映调用数量');grid(ax)
 ax=axs[j,1]
 for i,m in enumerate(lm):
  rr=groups[m,b]
  if not rr:missing(ax,i,0);continue
  v=[sum(e['seconds'] for e in r['events'] if e['stage'] in ['input_and_endpoint','encoding_and_retrieval','outer_input','generation','generation_other','coarse_ranking'])+sum(e['seconds'] for e in r['events'][:next((k for k,e in enumerate(r['events']) if e['stage'] in ['selection','solver_initialization','ihsid_including_internal_acceptance']),len(r['events']))] if e['stage'] in ['physical','quality','acceptance_other']) for r in rr]
  box=ax.boxplot([v],positions=[i],widths=.5,patch_artist=True,showfliers=True,flierprops={'markersize':2,'alpha':.25});box['boxes'][0].set(facecolor=COL[m],alpha=.6)
  for row in candidateRows:
   if row['method']==m and row['B']==b:row['candidatePhaseMeanSeconds']=float(np.mean(v));row['candidatePhaseP50Seconds']=float(np.median(v));row['candidatePhaseDefinition']='input+encoding+generation+coarse ranking+checks before selection; excludes later repair/fallback'
 ax.axhline(b*.5,color='#89949a',ls='--',lw=.8);ax.set(xticks=range(4),xticklabels=[SHORT[m] for m in lm],ylabel='候选阶段实际耗时 / s',title=f'B={b}s · 虚线为50%阶段预算');grid(ax)
fig.legend(handles=[Patch(color=c,label=s) for c,s in zip(cc,statusNames)],loc='upper center',bbox_to_anchor=(.5,1.005),ncol=3,frameon=False)
title(fig,'图7  候选数量与生成效率','只统计完成生成的唯一候选；不把重复检查当候选；不同结果间的关联不等于候选数的因果效应');save(fig,'07_candidates')

# Complete plot data: raw matched observations remain accessible in filtered calls.
for name,obj in [('success_summary',summary),('paired_success',contrasts),('strata',strata),('timing',timeTable),('outcomes',outcomes),('failure_causes',causeTable),('quality_summary',qualityRows),('safety_summary',safetyRows),('pairwise_quality',pairQuality),('candidate_summary',candidateRows),('common_success',jointMeta)]:write('data/'+name+'.json',obj)
jointData=[]
for m in M:
 for key in joint:
  r=idx[m][key];jointData.append({'method':m,'taskId':key[0],'repeat':key[1],'quality':{k:metric(r,k) for k,_,_,_ in QM},'safety':{k:safety(r,k) for k,_,_,_,_ in SM}})
write('data/matched_observations.json',jointData)
config={'version':'endpoint_pass_five_methods_v1','methods':M,'labels':LABEL,'colors':COL,'budgets':BS,'successField':'finalQualified','bootstrapReplicates':2000,'bootstrapSeed':20260927,'cluster':'baseSceneId','seedBlock':'shared_across_scenes','weighting':'ratio_of_resampled_sums_task_equal','independentTasks':146,'sceneClusters':20,'excludedTasks':46,'futureSupplementCalls':876,'missingCells':[{'method':m,'B':b} for m,b in groups if not groups[m,b]],'sourceIdentity':identity,'sourceSha256':sources,'experimentsLaunched':False}
config['version']=OUT.name
config['futureSupplementCalls']=438*len(missingBudgets)
if args.snapshot:config['snapshotCallsSha256']=sha(args.snapshot/'data/filtered_calls.jsonl.gz')
write('analysis_config.json',config)
integrity={'passed':True,'retainedCalls':len(rows),'groups':len([x for x in groups.values() if x]),'callsPerMeasuredGroup':438,'commonSuccessCalls':len(joint),'commonSuccessTasks':jointMeta['tasks'],'commonSuccessScenes':jointMeta['scenes'],'maxTimingClosureError':max(abs(times(r).sum()-r['actualReturnTime']) for r in rows),'removedPerOriginalMeasuredGroup':138,'postAuditRejects':sum(r['onlinePassed'] and not r['finalQualified'] for r in rows),'compositeSolverTimers':sum(e['stage']=='ihsid_including_internal_acceptance' for r in rows for e in r['events']),'rawMatchedKeysVerified':True}
write('data/integrity.json',integrity)
print(json.dumps({'integrity':integrity,'success':summary,'paired10':[c for c in contrasts if c['B']==10]},ensure_ascii=False))
shutil.copy2(__file__,OUT/pathlib.Path(__file__).name)
