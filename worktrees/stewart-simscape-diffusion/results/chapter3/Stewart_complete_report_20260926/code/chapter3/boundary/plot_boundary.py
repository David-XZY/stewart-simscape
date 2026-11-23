"""Observed-result plots only; absent experiment families are explicitly omitted."""
from collections import defaultdict
import random
from run_boundary import write,read,resolve_artifact


def paired_interval(rows,a,b,budget,replicates=2000):
    values=defaultdict(lambda:defaultdict(dict)); clusters={}
    for r in rows:
        if r['B']==budget and r['method'] in (a,b):
            task=r['taskId']; values[task][r['method']][r['job']['repeat']]=float(r['passed']); clusters[task]=r['job'].get('macroCellId',r['job']['groupId'])
    differences=defaultdict(list)
    for task,m in values.items():
        if a in m and b in m:
            paired={k:m[a][k]-m[b][k] for k in set(m[a])&set(m[b])}
            if paired: differences[clusters[task]].append(paired)
    groups=list(differences)
    if not groups: return None
    rng=random.Random(20260919); samples=[]; seeds=sorted({k for group in differences.values() for row in group for k in row})
    for _ in range(replicates):
        draw=rng.choices(seeds,k=len(seeds)); selected=[]
        for g in rng.choices(groups,k=len(groups)):
            for row in differences[g]:
                v=[row[k] for k in draw if k in row]
                if v: selected.append(sum(v)/len(v))
        if selected: samples.append(sum(selected)/len(selected))
    samples.sort(); all_values=[sum(v.values())/len(v) for g in groups for v in differences[g]]
    return dict(difference=sum(all_values)/len(all_values),lower=samples[int(.025*len(samples))],upper=samples[min(len(samples)-1,int(.975*len(samples)))],clusters=len(groups),tasks=len(all_values),seedBlocks=len(seeds),resampling='paired scenario clusters and global seed blocks; few-seed uncertainty remains limited')


def plot_report(rows,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    splits=sorted({r['job']['split'] for r in rows}); comparisons=[]
    for split in splits:
        subset=[r for r in rows if r['job']['split']==split]; methods=sorted({r['method'] for r in subset})
        fig,ax=plt.subplots(figsize=(6.5,4))
        for method in methods:
            rates=[]; budgets=sorted({r['B'] for r in subset if r['method']==method})
            for B in budgets:
                calls=[r for r in subset if r['method']==method and r['B']==B]; rates.append(100*sum(r['passed'] for r in calls)/len(calls))
            ax.plot(budgets,rates,'o-',label=method,linewidth=1.4,markersize=4)
        ax.set(xlabel='Planning budget B (s)',ylabel='Physically qualified returns (%)',ylim=(-2,102),title=split); ax.legend(); ax.grid(alpha=.2); fig.tight_layout()
        for ext in ['png','svg']: fig.savefig(out/f'time_success_{split}.{ext}',dpi=180)
        plt.close(fig)
        for baseline in ['M1','M2_H','M3_L','M3_N']:
            interval=paired_interval(subset,'M4',baseline,2)
            if interval: comparisons.append(dict(split=split,baseline=baseline,**interval))
        # Boundary points use majority of the three actual calls, not best seed.
        paired=defaultdict(dict); taskmeta={}
        for r in subset:
            if r['B']==2 and r['method'] in ['M4','M2_H']:
                paired[r['taskId']].setdefault(r['method'],[]).append(r['passed']); taskmeta[r['taskId']]=r['job']
        colors=['#777777','#4575b4','#d95f02','#4d9221']; labels=['neither','retrieval only','diffusion only','both']
        fig,ax=plt.subplots(figsize=(5.5,4)); used=set()
        for task,p in paired.items():
            if set(p)!= {'M4','M2_H'}: continue
            cat=2*int(sum(p['M4'])/len(p['M4'])>=.5)+int(sum(p['M2_H'])/len(p['M2_H'])>=.5)
            params=taskmeta[task]['parameters']; ax.scatter(params[0]*1000,params[1]*1000,c=colors[cat],label=labels[cat] if cat not in used else None); used.add(cat)
        ax.set(xlabel='Fixture A offset (mm)',ylabel='Fixture B offset (mm)',title=split+'; B=2 s; mixed T and yaw (overview)')
        if used: ax.legend()
        fig.tight_layout(); fig.savefig(out/f'boundary_overview_{split}.png',dpi=180); plt.close(fig)
    write(out/'paired_B2.json',comparisons)
    produced=['time_success','boundary_overview']
    # Representative: deterministic smallest taskId in each observed outcome class.
    candidates=sorted((r for r in rows if r.get('passed') and r.get('trajectoryRef')),key=lambda r:(r['taskId'],r['method']))
    chosen={}
    for r in candidates:
        chosen.setdefault(r['method'],r)
    for method,r in chosen.items():
        from pathlib import Path
        datafile=resolve_artifact(r['trajectoryRef']).parent/'plot_data.json'
        if not datafile.exists(): continue
        d=read(datafile); t=d['t']; q=d['q']
        fig,axes=plt.subplots(2,3,figsize=(12,6.3))
        for leg in range(6):
            axes[0,0].plot(t,d['legLength'][leg],linewidth=1)
            axes[0,1].plot(t,d['legSpeed'][leg],linewidth=1)
            axes[0,2].plot(t,d['force'][leg],linewidth=1)
        axes[0,0].set(ylabel='Leg length (m)'); axes[0,1].set(ylabel='Leg speed (m/s)'); axes[0,2].set(ylabel='Force (N)')
        for obstacle in range(len(d['frontGapLower'])):
            axes[1,0].plot(t,[1000*v for v in d['frontGapLower'][obstacle]],label=f'front vs {obstacle+1}')
            axes[1,0].plot(t,[1000*v for v in d['rearGapLower'][obstacle]],'--',label=f'rear vs {obstacle+1}')
        axes[1,0].axhline(10,color='black',linewidth=.8); axes[1,0].set(ylabel='Separation certificate (mm)'); axes[1,0].legend(fontsize=7)
        axes[1,1].plot([v*1000 for v in d['center'][1]],[v*180/3.141592653589793 for v in q[5]]); axes[1,1].set(xlabel='Payload center y (mm)',ylabel='Platform yaw (deg)')
        axes[1,2].bar(range(1,7),d['metrics']['legTotalVariation']); axes[1,2].set(xlabel='Leg',ylabel='Sampled leg travel (m)')
        for ax in [*axes[0],axes[1,0]]: ax.set_xlabel('Time (s)'); ax.grid(alpha=.2)
        fig.suptitle(f'{r["taskId"]}; {method}; {r["returnStage"]}'); fig.tight_layout(); fig.savefig(out/f'mechanism_{method}.png',dpi=180); plt.close(fig)
        produced.append('trajectory_mechanism_panels')
    sizes=sorted({r['job']['libraryConditions'] for r in rows if 'libraryConditions' in r['job']})
    if len(sizes)>=3:
        fig,ax=plt.subplots(figsize=(6,4))
        for method in sorted({r['method'] for r in rows}):
            x=[]; y=[]
            for size in sizes:
                calls=[r for r in rows if r['method']==method and r['B']==2 and r['job'].get('libraryConditions')==size]
                if calls: x.append(size); y.append(100*sum(r['passed'] for r in calls)/len(calls))
            if x: ax.plot(x,y,'o-',label=method)
        ax.set(xlabel='Training conditions',ylabel='Qualified returns (%)',title='B = 2 s'); ax.legend(); fig.tight_layout(); fig.savefig(out/'library_size.png',dpi=180); plt.close(fig); produced.append('nested_library_sizes')
    candidate_files=list((out.parent/'candidate_diagnostics').glob('*.json'))
    if candidate_files:
        grouped=defaultdict(list)
        for file in candidate_files:
            d=read(file); candidates=d['candidates']; candidates=[candidates] if isinstance(candidates,dict) else candidates
            valid=[c for c in candidates if c['passed']]; centers=[]
            for candidate in valid:
                f=candidate['modeFeature']
                if not any(sum((a-b)**2 for a,b in zip(f,c))/len(f)<.01 for c in centers): centers.append(f)
            grouped[(d['job']['method'],d['generated'])].append((len(valid)/max(1,len(candidates)),len(centers)))
        fig,axes=plt.subplots(1,2,figsize=(9,3.8))
        for method in sorted({k[0] for k in grouped}):
            ks=sorted(k[1] for k in grouped if k[0]==method)
            for i,ax in enumerate(axes): ax.plot(ks,[sum(v[i] for v in grouped[(method,k)])/len(grouped[(method,k)]) for k in ks],'o-',label=method)
        axes[0].set(ylabel='Valid candidate fraction'); axes[1].set(ylabel='Distinct valid motion proxies')
        for ax in axes: ax.set_xlabel('Unique generated candidates'); ax.legend()
        fig.tight_layout(); fig.savefig(out/'candidate_modes.png',dpi=180); plt.close(fig); produced.append('candidate_mode_coverage')
    write(out/'figure_coverage.json',dict(produced=sorted(set(produced)),note='Figures are emitted only when the corresponding observed-result data exist; motion proxies are not topology.'))
