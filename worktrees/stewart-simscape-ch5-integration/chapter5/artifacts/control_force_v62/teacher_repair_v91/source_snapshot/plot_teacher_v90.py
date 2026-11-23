"""Export paired counts/timing and dense safety traces from saved raw runs."""
from pathlib import Path
import argparse
import json
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'chapter5/artifacts/control_force_v62'
OUT=BASE/'teacher_repair_v90'


def main(version='v90'):
    global OUT
    OUT=BASE/('teacher_repair_'+version)
    report=json.loads((OUT/'comparison.json').read_text(encoding='utf-8'))
    assert report['completed'], 'All fixed full runs must have result states.'
    rows=report['rows'];x=np.arange(len(rows));width=.35
    labels=[r['task']+'\n'+r['mode'] for r in rows]
    fig,axes=plt.subplots(1,3,figsize=(14,4.4),layout='constrained')
    for ax,before,after,title in [
        (axes[0],'before_ihsid','after_ihsid','IHSID calls (including prechecks)'),
        (axes[1],'before_solver','after_solver','Actual NLP starts (including retreat)'),
        (axes[2],'before_wall_seconds','observed_reference_wall_seconds','Observed wall time [s]; version in report')]:
        for offset,key,color,label in [(-width/2,before,'#9aa6b2','V89'),(width/2,after,'#2376b7',version.upper())]:
            bars=ax.bar(x+offset,[r[key] for r in rows],width,color=color,label=label)
            ax.bar_label(bars,fmt='%.0f',padding=3,fontsize=9)
        ax.set_xticks(x,labels,fontsize=9);ax.set_title(title,fontsize=11);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        ax.margins(y=.16)
    axes[0].legend();fig.savefig(OUT/'calls_and_timing.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    for ax,row in zip(axes,rows):
        name=row['task']+'_'+row['mode'].replace('-','_')+'.mat'
        for rev,color in [('v89','#929eaa'),(version,'#1670af')]:
            source=BASE/('teacher_repair_'+rev)/name if rev=='v89' else Path(row['source_full_run'])
            audit=loadmat(source,variable_names=['audit'],simplify_cells=True)['audit']
            ax.plot(np.ravel(audit['time']),np.ravel(audit['clearance'])*1000,color=color,label=rev.upper(),lw=1.4)
        ax.axhline(15,color='#bd3939',ls='--',label='Required clearance: 15 mm')
        ax.set_ylabel('Clearance [mm]');ax.set_title(row['task']+' / '+row['mode'],fontsize=11);ax.grid(alpha=.2)
    axes[0].legend(ncol=3,fontsize=9);axes[-1].set_xlabel('Simulation time [s]')
    fig.savefig(OUT/'dense_clearance.png',dpi=170);plt.close(fig)
    page=OUT/'index.html';text=page.read_text(encoding='utf-8')
    if 'calls_and_timing.png' not in text:
        text+='<p><img src="calls_and_timing.png" style="width:100%" alt="调用次数和墙钟耗时对比"></p>'
        text+='<p><img src="dense_clearance.png" style="width:100%" alt="完整运行的实体净空，安全距离 15 mm"></p>'
        page.write_text(text,encoding='utf-8')
    print('Exported calls_and_timing.png and dense_clearance.png')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--version',choices=['v90','v91'],default='v90')
    main(parser.parse_args().version)
