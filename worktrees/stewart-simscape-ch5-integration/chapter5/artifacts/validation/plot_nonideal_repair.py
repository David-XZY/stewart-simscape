"""Plot actual Simscape audits for the matched completed nonideal prefix."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ART = Path(__file__).resolve().parent.parent
identity=json.loads((ART/'runtime_source_identity.json').read_text('utf-8'))
prefix=identity['diffusion_source_sha256'][:8]
names=[('Intermediate v004','repair_0f0f9173_002_nonideal_force_320ms_original'),('Repaired v006',f'repair_{prefix}_002_nonideal_force_320ms_original')]
report=json.loads((ART/'validation'/(names[1][1]+'_report.json')).read_text('utf-8'))
assert report['passed'] and report['all_constraints_valid']
out=ART/'repair_report';out.mkdir(exist_ok=True)
config=json.loads((ART.parent/'config.json').read_text('utf-8'))
colors=['#d97706','#2563eb']
fig,axes=plt.subplots(2,2,figsize=(10,6.5),constrained_layout=True)
rows=[];summary=[]
for (label,name),color in zip(names,colors):
    path=ART/'validation'/(name+'.mat')
    data=loadmat(path,variable_names=['audit','records'],simplify_cells=True);a=data['audit']
    t=np.asarray(a['time']);err=np.asarray(a['position_error'])*1000
    model=np.max(np.abs(a['leg_acceleration_model_actual_force']),axis=1)
    measured=np.max(np.abs(a['leg_acceleration_measured']),axis=1)
    clearance=np.asarray(a['clearance'])*1000
    records=np.asarray(data['records'],dtype=object).reshape(-1)
    ct=np.asarray([r['time'] for r in records]);fallback=np.asarray([r['diagnostic']['usedFallback'] for r in records],dtype=int)
    axes[0,0].plot(t,err,label=label,color=color)
    axes[0,1].plot(t,model,label=label,color=color)
    axes[1,0].plot(t,clearance,label=label,color=color)
    axes[1,1].step(ct,fallback,where='post',label=label,color=color,linewidth=2)
    checksum=hashlib.sha256(path.read_bytes()).hexdigest()
    summary.append(dict(stage=label,raw_file=str(path),raw_sha256=checksum,model_acceleration_max=float(model.max()),measured_acceleration_max=float(measured.max()),final_position_error_mm=float(err[-1]),minimum_clearance_mm=float(clearance.min()),fallback_count=int(fallback.sum())))
    rows.extend(dict(stage=label,time_s=float(t[i]),position_error_mm=float(err[i]),max_abs_model_leg_acceleration_m_s2=float(model[i]),max_abs_measured_leg_acceleration_m_s2=float(measured[i]),clearance_mm=float(clearance[i])) for i in range(len(t)))
axes[0,0].set_ylabel('Position error (mm)')
axes[0,1].set_ylabel('Max. absolute leg acceleration (m/s²)')
axes[0,1].axhline(1.2,color='#b91c1c',linestyle='--',label='Hard limit: 1.2 m/s²')
axes[1,0].set_ylabel('Minimum physical clearance (mm)')
axes[1,0].axhline(config['safe_distance']*1000,color='#b91c1c',linestyle='--',label='Safety distance')
axes[1,1].set_ylabel('Safety QP fallback');axes[1,1].set_yticks([0,1]);axes[1,1].set_ylim(-.1,1.15)
for ax in axes.flat:
    ax.set_xlabel('Simulation time (s)');ax.set_xlim(0,.32);ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('Completed nonideal Simscape prefix, seed 5101 (not a formal result)')
fig.savefig(out/'nonideal_prefix_before_after.png',dpi=180)
fig.savefig(out/'nonideal_prefix_before_after.svg')
plt.close(fig)
with (out/'nonideal_actual_timeseries.csv').open('w',newline='',encoding='utf-8-sig') as handle:
    writer=csv.DictWriter(handle,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
(out/'nonideal_actual_comparison.json').write_text(json.dumps(dict(diffusion_source_sha256=identity['diffusion_source_sha256'],prefix_only=True,summary=summary),indent=2),'utf-8')
print(json.dumps(summary,indent=2))
