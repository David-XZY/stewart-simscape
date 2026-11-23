"""Label the source version on every wall-time bar in the final standalone plot."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parents[3]/'chapter5/artifacts/control_force_v62/teacher_repair_v92'
rows=json.loads((OUT/'comparison.json').read_text(encoding='utf-8'))['rows']
x=np.arange(3);w=.35;labels=[r['task']+'\n'+r['mode'] for r in rows]
fig,axes=plt.subplots(1,3,figsize=(14,4.8),layout='constrained')
for ax,before,after,title in [
    (axes[0],'before_ihsid','after_ihsid','IHSID calls (including prechecks)'),
    (axes[1],'before_solver','after_solver','Actual NLP starts'),
    (axes[2],'before_wall_seconds','observed_reference_wall_seconds','Observed wall time [s]')]:
    for offset,key,color,label in [(-w/2,before,'#9aa6b2','V89'),(w/2,after,'#2376b7','Validated scheduling')]:
        bars=ax.bar(x+offset,[r[key] for r in rows],w,color=color,label=label)
        values=[f"{r[key]:.0f}" for r in rows]
        if ax==axes[2] and key==after:
            values=[f"{r[key]:.0f}\n({r['timing_version'].upper()}{'*' if r['timing_version']=='v90' else ''})" for r in rows]
        ax.bar_label(bars,labels=values,padding=3,fontsize=9)
    ax.set_xticks(x,labels,fontsize=9);ax.set_title(title,fontsize=11);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True);ax.margins(y=.22)
axes[0].legend(fontsize=9)
fig.supxlabel('* Orbit wall times are full V90 runs verified equivalent to V92; crossing is a new V92 run. Historical matched-task timings.',fontsize=10)
fig.savefig(OUT/'calls_and_timing.png',dpi=170);plt.close(fig)
file=OUT/'index.html';text=file.read_text(encoding='utf-8').replace('完整运行状态：','验证条件状态：')
file.write_text(text,encoding='utf-8')
print('Final plot distinguishes V90 source timings from the new V92 simulation.')
