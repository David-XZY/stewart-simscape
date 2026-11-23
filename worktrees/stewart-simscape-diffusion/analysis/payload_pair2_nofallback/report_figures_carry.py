"""Presentation-only redraw of the frozen endpoint-pass analysis; no experiments."""
import json,pathlib,hashlib,shutil,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from prepare import OUT as CAMPAIGN
P=CAMPAIGN
SRC=CAMPAIGN/'analysis_carry/data'
OUT=CAMPAIGN/'analysis_carry/figures';OUT.mkdir(parents=True,exist_ok=True)
M=['ddpm_basic','ddpm_signature','regression','regression_signature','ihsid']
N=['扩散','物理签名扩散','回归','物理签名回归','IHSID']
SHORT=['扩散','物理签名\n扩散','回归','物理签名\n回归','IHSID']
C=['#2476B5','#189C8B','#ED9A32','#CF5047','#171717']
plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','SimHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':12,'axes.titlesize':15,'axes.labelsize':13,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
used={}
def read(name):
 f=SRC/(name+'.json');used[f.name]=hashlib.sha256(f.read_bytes()).hexdigest();return json.loads(f.read_text(encoding='utf8'))
def clean(ax):
 ax.grid(axis='y',alpha=.17);ax.set_axisbelow(True)
def legend(fig,y=.925):
 fig.legend(handles=[Patch(facecolor=c,label=n) for c,n in zip(C,N)],loc='upper center',bbox_to_anchor=(.5,y),ncol=5,frameon=False,fontsize=12)
def save(fig,name):
 fig.savefig(OUT/(name+'.png'),dpi=300,bbox_inches='tight',pad_inches=.18)
 fig.savefig(OUT/(name+'.svg'),bbox_inches='tight',pad_inches=.18)
 plt.close(fig)
rates=read('success_summary');strata=read('strata');timing=read('timing');outcomes=read('outcomes');matched=read('matched_observations')

# Keep measured points and the original uncertainty bars, remove all bands/text.
fig,ax=plt.subplots(figsize=(11.8,7));fig.subplots_adjust(top=.79,bottom=.13,left=.1,right=.97)
for m,n,c in zip(M,N,C):
 rs=[next(r for r in rates if r['method']==m and r['B']==b) for b in [1,2,5,10,20]]
 y=np.array([100*r['mean'] if r['status'] in ['measured','budget_coverage'] else np.nan for r in rs])
 ax.plot([1,2,5,10,20],y,'o-',color=c,lw=2.4,ms=7)
 for b,r in zip([1,2,5,10,20],rs):
  if r['status'] in ['measured','budget_coverage']:ax.vlines(b,*[100*x for x in r['ci95']],color=c,lw=1,alpha=.7)
ax.set(xlabel='规划时间预算 / s',ylabel='合格轨迹返回率 / %',xticks=[1,2,5,10,20],ylim=(-2,104));clean(ax)
fig.suptitle('不同规划时间预算下的轨迹合格率',fontsize=21,y=.985);legend(fig);save(fig,'01_规划预算与轨迹合格率')

fig,ax=plt.subplots(figsize=(12.8,7.2));fig.subplots_adjust(top=.79,bottom=.16,left=.09,right=.98)
g=['G0','G1','G2','G3'];labs=['G0\n已见载荷的新任务','G1\n训练范围内的新载荷','G2\n未见参数组合','G3\n训练范围外的轻度外推']
for i,(m,c) in enumerate(zip(M,C)):
 rs=[next(r for r in strata if r['B']==10 and r['method']==m and r['stratum']=='generalizationType' and r['value']==v) for v in g]
 y=np.array([100*r['mean'] for r in rs]);lo=np.array([100*r['ci95'][0] for r in rs]);hi=np.array([100*r['ci95'][1] for r in rs]);x=np.arange(4)+(i-2)*.16
 ax.bar(x,y,.145,color=c);ax.errorbar(x,y,yerr=[y-lo,hi-y],fmt='none',ecolor='#525252',capsize=2,lw=.85)
ax.set(xticks=range(4),xticklabels=labs,ylabel='合格轨迹返回率 / %',ylim=(0,105));clean(ax)
fig.suptitle('10秒规划预算下不同泛化任务的轨迹合格率',fontsize=21,y=.985);legend(fig);save(fig,'02_10秒泛化任务合格率')

# Horizontal stage bars; the table preserves readable values for tiny segments.
stageKeys=['输入/编码','生成','排序','检查','精修','NLP','其他']
stageLabels=['输入与编码','候选生成','候选排序','轨迹检查','轨迹精修','IHSID建模与求解','其他开销']
stageColors=['#ACCAE0','#5599C9','#A494BD','#42AA9C','#E6A44D','#292929','#D8D8D8']
for b,num in [(2,'03'),(10,'04')]:
 fig=plt.figure(figsize=(14,8.1));gs=fig.add_gridspec(2,1,height_ratios=[3.4,1.5],left=.155,right=.965,bottom=.06,top=.79,hspace=.25)
 ax=fig.add_subplot(gs[0]);tabax=fig.add_subplot(gs[1]);tabax.axis('off')
 matrix=[];totals=[]
 for m in M:
  r=next(r for r in timing if r['method']==m and r['B']==b);assert r['n']==438 and r['stageMeanSeconds']['重新计时']==0
  matrix.append([r['stageMeanSeconds'][k] for k in stageKeys]);totals.append(r['totalMeanSeconds'])
 matrix=np.array(matrix);assert np.allclose(matrix.sum(axis=1),totals,atol=1e-12)
 left=np.zeros(5)
 for j,(n,c) in enumerate(zip(stageLabels,stageColors)):
  ax.barh(np.arange(5),matrix[:,j],left=left,height=.60,color=c,label=n)
  for i,v in enumerate(matrix[:,j]):
   if v>max(totals)*.065:ax.text(left[i]+v/2,i,f'{v:.3f}',ha='center',va='center',color='white' if j==5 else '#122736',fontsize=11)
  left+=matrix[:,j]
 for i,t in enumerate(totals):ax.text(t+max(totals)*.018,i,f'总计 {t:.3f} s',va='center',fontsize=11,fontweight='bold',color=C[i])
 ax.set(yticks=np.arange(5),yticklabels=N,xlabel='平均耗时 / s',xlim=(0,max(totals)*1.24));ax.invert_yaxis();ax.grid(axis='x',alpha=.17);ax.set_axisbelow(True)
 for t,c in zip(ax.get_yticklabels(),C):t.set_color(c);t.set_fontweight('bold')
 cell=[[f'{v:.4f}' for v in row]+[f'{t:.4f}'] for row,t in zip(matrix,totals)]
 table=tabax.table(cellText=cell,rowLabels=N,colLabels=['输入与\n编码','候选\n生成','候选\n排序','轨迹\n检查','轨迹\n精修','IHSID建模\n与求解','其他\n开销','总耗时'],cellLoc='center',rowLoc='center',bbox=[0,0,1,1]);table.auto_set_font_size(False);table.set_fontsize(10.2)
 for (r,c),cell in table.get_celld().items():
  cell.set_edgecolor('#D8E0E5');cell.set_linewidth(.65)
  if r==0:cell.set_facecolor('#EAF0F3');cell.set_text_props(weight='bold')
  elif c==-1:cell.set_text_props(color=C[r-1],weight='bold');cell.set_facecolor('#FAFBFC')
  elif c==7:cell.set_facecolor('#F2F4F6');cell.set_text_props(weight='bold')
 fig.suptitle(f'{b}秒规划预算下各方法的平均耗时构成',fontsize=21,y=.985)
 fig.text(.56,.925,'全部438次调用均计入平均值（包含失败）；各阶段耗时不重复计算，表中单位为秒',ha='center',fontsize=12,color='#415461')
 fig.legend(handles=[Patch(color=c,label=n) for c,n in zip(stageColors,stageLabels)],loc='upper center',bbox_to_anchor=(.55,.895),ncol=4,frameon=False,fontsize=11)
 save(fig,f'{num}_{b}秒平均耗时分解')

categories=['直接成功','公共精修成功','IHSID回退成功','独立IHSID成功','超时失败','复查拒绝','其他失败']
oc=['#189C8B','#88C8B1','#609BCC','#171717','#E8A447','#CA5362','#CACACA']
fig,ax=plt.subplots(figsize=(12.8,8));fig.subplots_adjust(top=.77,bottom=.12,left=.09,right=.98);bottom=np.zeros(5)
for name,c in zip(categories,oc):
 vals=[next((r['calls'] for r in outcomes if r['B']==10 and r['method']==m and r['outcome']==name),0) for m in M]
 ax.bar(range(5),vals,.65,bottom=bottom,color=c,label=name)
 for i,v in enumerate(vals):
  if v>=15:ax.text(i,bottom[i]+v/2,str(v),ha='center',va='center',fontsize=13,color='white' if name in ['直接成功','独立IHSID成功','复查拒绝'] else '#22323D')
  elif v:ax.annotate(str(v),(i+.325,bottom[i]+v/2),xytext=(8,0),textcoords='offset points',ha='left',va='center',fontsize=10,color=c,fontweight='bold')
 bottom+=vals
assert np.all(bottom==438)
ax.set(xticks=range(5),xticklabels=N,ylabel='调用次数',ylim=(0,460));clean(ax)
for t,c in zip(ax.get_xticklabels(),C):t.set_color(c);t.set_fontweight('bold')
fig.suptitle('10秒规划预算下各方法的成功与失败分布',fontsize=21,y=.985)
fig.legend(handles=[Patch(color=c,label=n) for c,n in zip(oc,categories)],loc='upper center',bbox_to_anchor=(.53,.925),ncol=4,frameon=False,fontsize=12)
save(fig,'05_10秒成功与失败分布')

QM=[('positionLengthM','位置路径长度','m'),('rotationLengthRad','旋转路径长度','rad'),('legTotalVariation','六腿累计行程','m'),('positionAccelRms','平台线加速度 RMS','m/s²'),('angularAccelRms','平台角加速度 RMS','rad/s²'),('legAccelRms','腿加速度 RMS','m/s²'),('positionJerkRms','平台线 jerk RMS','m/s³'),('angularJerkRms','平台角 jerk RMS','rad/s³'),('legJerkRms','腿 jerk RMS','m/s³'),('forceRmsN','驱动力 RMS','N'),('forceRateRms','力变化率 RMS','N/s'),('absoluteMechanicalWorkJ','绝对机械功','J')]
fig,axs=plt.subplots(3,4,figsize=(19,13.6));fig.subplots_adjust(top=.90,bottom=.06,left=.055,right=.985,hspace=.44,wspace=.30)
for ax,(k,name,unit) in zip(axs.flat,QM):
 rng=np.random.default_rng(7)
 for i,(m,c) in enumerate(zip(M,C)):
  v=np.array([r['quality'][k] for r in matched if r['method']==m]);assert len(v)==len(matched)//5 and np.isfinite(v).all()
  bx=ax.boxplot([v],positions=[i],widths=.57,patch_artist=True,showfliers=False,medianprops={'color':'#222','linewidth':1.4},whiskerprops={'color':'#4D4D4D'},capprops={'color':'#4D4D4D'})
  bx['boxes'][0].set(facecolor=c,alpha=.32,edgecolor='#4D4D4D')
  ax.scatter(i+rng.uniform(-.19,.19,len(v)),v,s=8,color=c,alpha=.32,linewidths=0)
 ax.set(title=name,ylabel=unit,xticks=range(5),xticklabels=SHORT);ax.tick_params(axis='x',labelsize=10);ax.tick_params(axis='y',labelsize=10);clean(ax)
fig.suptitle('10秒规划预算下不同方法的轨迹质量对比',fontsize=24,y=.986);legend(fig,y=.955);save(fig,'06_轨迹质量指标对比')


meta={'scope':'original booklet presentation; new measurements and explicitly recorded source reuse in separate data tables',
      'methodOrder':M,'methodLabels':N,'methodColors':C,'qualityMatchedCalls':len(matched)//5,
      'qualityTasks':len({r['taskId'] for r in matched}),'qualityScenes':len({r['baseSceneId'] for r in matched}),
      'dataSha256':used,'plotSource':'results/chapter3/redraw_selected_five_methods.py'}
(OUT.parent/'figure_provenance.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT.parent/'index.html').write_text('<meta charset="utf-8"><title>五方法结果图</title>'+''.join('<img style="width:100%;max-width:1400px" src="figures/'+p.name+'">' for p in sorted(OUT.glob('*.png'))),encoding='utf-8')
import sys,subprocess
if '--pdf' in sys.argv:
 marker='C:/Users/David_X/.cache/codex-runtimes/codex-primary-runtime/plugins/openai-primary-runtime/plugins/pdf/skills/pdf/container_tools/mark_artifact_operation_started.mjs'
 subprocess.run(['node',marker,'--operation-kind','create','--expected-output-count','1','--output-format','pdf'],check=True)
 import fitz
 doc=fitz.open()
 for p in sorted(OUT.glob('*.png')):
  page=doc.new_page(width=1190.55,height=841.89);page.insert_image(fitz.Rect(22,22,1168.55,819.89),filename=str(p),keep_proportion=True)
 doc.save(OUT.parent/'五方法预算成功沿用图册.pdf');doc.close()
