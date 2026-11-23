"""Presentation-only: six PNG/SVG figures and optional local, one-page-per-figure PDF."""
import argparse, json, subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from prepare import OUT,read,write,sha,METHODS,BUDGETS
METHODS=[m for m in METHODS if m!="ihsid"]

N=['扩散','物理签名扩散','回归','物理签名回归']
C=['#2476B5','#189C8B','#ED9A32','#CF5047']
QM=[('positionLengthM','位置路径长度','m'),('rotationLengthRad','旋转路径长度','rad'),('legTotalVariation','六腿累计行程','m'),('positionAccelRms','平台线加速度 RMS','m/s²'),('angularAccelRms','平台角加速度 RMS','rad/s²'),('legAccelRms','腿加速度 RMS','m/s²'),('positionJerkRms','平台线 jerk RMS','m/s³'),('angularJerkRms','平台角 jerk RMS','rad/s³'),('legJerkRms','腿 jerk RMS','m/s³'),('forceRmsN','驱动力 RMS','N'),('forceRateRms','力变化率 RMS','N/s'),('absoluteMechanicalWorkJ','绝对机械功','J')]

def render(pdf=False):
    data=OUT/'analysis_four/data';out=OUT/'analysis_four/figures';out.mkdir(parents=True,exist_ok=True)
    rates=read(data/'success_summary.json');strata=read(data/'strata.json');timing=read(data/'timing.json');outcomes=read(data/'outcomes.json');matched=read(data/'matched_observations.json')
    assert len(rates)==20 and all(r['n']==438 for r in rates)
    plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','SimHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':12,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
    images=[]
    def save(fig,name):
        for ext in ['png','svg']:fig.savefig(out/(name+'.'+ext),dpi=300,bbox_inches='tight',pad_inches=.18)
        images.append(out/(name+'.png'));plt.close(fig)
    def legend(fig,y=.92):fig.legend(handles=[Patch(facecolor=c,label=n) for c,n in zip(C,N)],loc='upper center',bbox_to_anchor=(.5,y),ncol=4,frameon=False)
    def grid(ax):ax.grid(axis='y',alpha=.17);ax.set_axisbelow(True)
    fig,ax=plt.subplots(figsize=(12,7));fig.subplots_adjust(top=.79,bottom=.13)
    for m,c in zip(METHODS,C):
        rs=[next(r for r in rates if r['method']==m and r['B']==b) for b in BUDGETS]
        ax.plot(BUDGETS,[100*r['mean'] for r in rs],'o-',color=c,lw=2.4)
        for b,r in zip(BUDGETS,rs):ax.vlines(b,*[100*x for x in r['ci95']],color=c,alpha=.7,lw=1)
    ax.set(xlabel='规划时间预算 / s',ylabel='合格轨迹返回率 / %',xticks=BUDGETS,ylim=(-2,104));grid(ax)
    fig.suptitle('不同规划时间预算下的轨迹合格率',fontsize=21,y=.985);legend(fig);save(fig,'01_规划预算与轨迹合格率')
    fig,ax=plt.subplots(figsize=(13,7.2));fig.subplots_adjust(top=.79,bottom=.16)
    for i,(m,c) in enumerate(zip(METHODS,C)):
        rs=[next(r for r in strata if r['method']==m and r['B']==10 and r['stratum']=='generalizationType' and r['value']==g) for g in ['G0','G1','G2','G3']]
        y=np.array([100*r['mean'] for r in rs]);x=np.arange(4)+(i-1.5)*.19;ax.bar(x,y,.175,color=c)
        for xx,r in zip(x,rs):ax.vlines(xx,*[100*v for v in r['ci95']],color='#444',lw=.8)
    ax.set(xticks=range(4),xticklabels=['G0\n已见载荷的新任务','G1\n训练范围内的新载荷','G2\n未见参数组合','G3\n训练范围外的轻度外推'],ylabel='合格轨迹返回率 / %',ylim=(0,105));grid(ax)
    fig.suptitle('10秒规划预算下不同泛化任务的轨迹合格率',fontsize=21,y=.985);legend(fig);save(fig,'02_10秒泛化任务合格率')
    keys=['输入/编码','生成','检查','精修','其他'];labels=['输入与编码','候选生成','轨迹检查','轨迹精修','其他开销'];colors=['#ACCAE0','#5599C9','#42AA9C','#E6A44D','#D8D8D8']
    for B,num in [(2,'03'),(10,'04')]:
        fig=plt.figure(figsize=(14,8.1));gs=fig.add_gridspec(2,1,height_ratios=[3.4,1.5],left=.155,right=.965,bottom=.06,top=.79,hspace=.25);ax=fig.add_subplot(gs[0]);tab=fig.add_subplot(gs[1]);tab.axis('off')
        rs=[next(r for r in timing if r['method']==m and r['B']==B) for m in METHODS]
        matrix=np.array([[r['stageMeanSeconds'][k] for k in keys] for r in rs]);totals=np.array([r['totalMeanSeconds'] for r in rs]);assert np.allclose(matrix.sum(1),totals)
        assert all(r['stageMeanSeconds']['NLP']==0 for r in rs);left=np.zeros(4)
        for j,(label,c) in enumerate(zip(labels,colors)):
            ax.barh(range(4),matrix[:,j],left=left,height=.6,color=c,label=label)
            for i,v in enumerate(matrix[:,j]):
                if v>max(totals)*.065:ax.text(left[i]+v/2,i,f'{v:.3f}',ha='center',va='center',color='white' if j==4 else '#122736',fontsize=11)
            left+=matrix[:,j]
        for i,t in enumerate(totals):ax.text(t+max(totals)*.018,i,f'总计 {t:.3f} s',va='center',color=C[i],fontsize=11)
        ax.set(yticks=range(4),yticklabels=N,xlabel='平均耗时 / s',xlim=(0,max(totals)*1.24));ax.invert_yaxis();ax.grid(axis='x',alpha=.17)
        for tick,color in zip(ax.get_yticklabels(),C):tick.set_color(color);tick.set_fontweight('bold')
        table=tab.table(cellText=[[f'{v:.4f}' for v in row]+[f'{total:.4f}'] for row,total in zip(matrix,totals)],rowLabels=N,colLabels=labels+['总耗时'],cellLoc='center',bbox=[0,0,1,1]);table.auto_set_font_size(False);table.set_fontsize(10)
        for (i,j),cell in table.get_celld().items():
            cell.set_edgecolor('#D8E0E5');cell.set_linewidth(.65)
            if i==0:cell.set_facecolor('#EAF0F3')
        fig.suptitle(f'{B}秒规划预算下各方法的平均耗时构成',fontsize=21,y=.985)
        fig.text(.56,.925,'全部438次调用均计入平均值（包含失败）；各阶段耗时不重复计算，表中单位为秒',ha='center',fontsize=12)
        fig.legend(handles=[Patch(color=c,label=n) for c,n in zip(colors,labels)],loc='upper center',bbox_to_anchor=(.55,.89),ncol=3,frameon=False)
        save(fig,f'{num}_{B}秒平均耗时分解')
    cats=['直接成功','公共精修成功','超时失败','复查拒绝','其他失败'];colors=['#189C8B','#88C8B1','#E8A447','#CA5362','#CACACA']
    fig,ax=plt.subplots(figsize=(13,8));fig.subplots_adjust(top=.77,bottom=.12);bottom=np.zeros(4)
    for cat,c in zip(cats,colors):
        vals=[next(r['calls'] for r in outcomes if r['method']==m and r['B']==10 and r['outcome']==cat) for m in METHODS]
        ax.bar(range(4),vals,.65,bottom=bottom,color=c)
        for i,v in enumerate(vals):
            if v>=15:ax.text(i,bottom[i]+v/2,str(v),ha='center',va='center',color='white' if cat in ['直接成功','复查拒绝'] else '#22323D')
            elif v:ax.annotate(str(v),(i+.325,bottom[i]+v/2),xytext=(8,0),textcoords='offset points',fontsize=10)
        bottom+=vals
    assert np.all(bottom==438);ax.set(xticks=range(4),xticklabels=N,ylabel='调用次数',ylim=(0,460));grid(ax)
    for tick,color in zip(ax.get_xticklabels(),C):tick.set_color(color);tick.set_fontweight('bold')
    fig.suptitle('10秒规划预算下各方法的成功与失败分布',fontsize=21,y=.985)
    fig.legend(handles=[Patch(color=c,label=n) for c,n in zip(colors,cats)],loc='upper center',bbox_to_anchor=(.5,.925),ncol=3,frameon=False);save(fig,'05_10秒成功与失败分布')
    fig,axes=plt.subplots(3,4,figsize=(19,13.6));fig.subplots_adjust(top=.90,bottom=.06,left=.055,right=.985,hspace=.44,wspace=.3)
    for ax,(key,label,unit) in zip(axes.flat,QM):
        rng=np.random.default_rng(7)
        for i,(m,c) in enumerate(zip(METHODS,C)):
            v=np.array([r['quality'][key] for r in matched if r['method']==m]);assert np.isfinite(v).all()
            if len(v):
                box=ax.boxplot([v],positions=[i],widths=.57,patch_artist=True,showfliers=False,medianprops={'color':'#222'});box['boxes'][0].set(facecolor=c,alpha=.32)
                ax.scatter(i+rng.uniform(-.19,.19,len(v)),v,s=8,color=c,alpha=.32,linewidths=0)
        if not matched:ax.text(.5,.5,'四方法无共同合格调用',ha='center',transform=ax.transAxes)
        ax.set(title=label,ylabel=unit,xticks=range(4),xticklabels=['扩散','物理签名\n扩散','回归','物理签名\n回归']);ax.tick_params(labelsize=10);grid(ax)
    fig.suptitle('10秒规划预算下不同方法的轨迹质量对比',fontsize=24,y=.986);legend(fig,.955);save(fig,'06_轨迹质量指标对比')
    write(OUT/'analysis_four/figure_provenance.json',dict(configHash=read(OUT/'protocol_four.json')['configHash'],renderHash=sha(__file__),methodOrder=METHODS,colors=C,qualityMatchedCalls=len(matched)//4,qualityTasks=len({r['taskId'] for r in matched}),qualityScenes=len({r['baseSceneId'] for r in matched}),figures={p.name:sha(p) for p in images}))
    (OUT/'analysis_four/index.html').write_text('<meta charset="utf-8"><title>四方法无回退实验</title><h1>四方法无回退、双候选即时检查实验</h1><p>146个端点通过任务、20个基础场景，每方法每预算438次调用。所有结果来自本轮重新运行。</p>'+''.join(f'<img style="width:100%;max-width:1400px" src="figures/{p.name}">' for p in images),encoding='utf-8')
    if pdf:
        marker=Path('C:/Users/David_X/.cache/codex-runtimes/codex-primary-runtime/plugins/openai-primary-runtime/plugins/pdf/skills/pdf/container_tools/mark_artifact_operation_started.mjs')
        subprocess.run(['node',str(marker),'--operation-kind','create','--expected-output-count','1','--output-format','pdf'],check=True)
        import fitz
        doc=fitz.open()
        for p in images:
            page=doc.new_page(width=1190.55,height=841.89);page.insert_image(fitz.Rect(22,22,1168.55,819.89),filename=str(p),keep_proportion=True)
        doc.save(OUT/'analysis_four/四方法无回退重测图册.pdf');doc.close()
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--pdf',action='store_true');args=parser.parse_args();render(args.pdf)
