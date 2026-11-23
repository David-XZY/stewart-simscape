"""Generate traceable chapter-three materials from measured records (including failures)."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

METHODS = ['geo_ihsid', 'retrieve_ihsid', 'diff_ihsid', 'geo_fast', 'retrieve_fast', 'diff_fast', 'previous_ihsid']
LABELS = ['Geometry + IHSID', 'Retrieval + IHSID', 'Diffusion + IHSID', 'Geometry cascade', 'Retrieval cascade', 'Diffusion cascade', 'Previous + IHSID']
TIMES = ['inputPreparation','boundary', 'generation', 'screening', 'physical', 'quality', 'repair', 'solverInitialization','solverBuild', 'solver', 'other']
CORE_CATEGORIES = {'id', 'goal_position', 'goal_attitude', 'layout', 'joint'}


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def flat(data, prefix=''):
    result = {}
    for key, value in data.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flat(value, name+'.'))
        elif isinstance(value, (list, tuple)):
            if all(isinstance(x, (int, float, type(None))) for x in value):
                result.update({f'{name}.{i}': x for i, x in enumerate(value)})
        else:
            result[name] = value
    return result


def csv_file(path, rows):
    fields = sorted({key for row in rows for key in row})
    with path.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def stats(rows, field):
    values = [r[field] for r in rows if r.get(field) is not None and np.isfinite(r[field])]
    if not values:
        return {'measured': 0, 'median': None, 'mean': None, 'p90': None}
    return {'measured': len(values), 'median': float(np.median(values)), 'mean': float(np.mean(values)), 'p90': float(np.percentile(values,90))}


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('run', type=Path)
    args = parser.parse_args(); root = args.run.resolve(); report = root/'report'; report.mkdir(exist_ok=True)
    jobs = read(root/'jobs.json'); standard = read(root/'standard.json'); manifest = read(root/'manifest.json')
    frozen=read(root/'frozen.json') if (root/'frozen.json').exists() else {}
    rows = []; integrity = []; failures = []; timing_rows = []; profiles = []
    for job in jobs:
        path = root/'jobs'/job['jobId']/'result.json'
        item=manifest['cases'][job['caseIndex']-1] if job.get('caseIndex') and manifest['cases'] else {}
        context={key:item.get(key) for key in ['family','category','groupId','rootGroupId','sequence','step','parameter','value']}
        context['planningBoundaryPassed']=item.get('task',{}).get('endpointPassed')
        row = dict(context,**read(path)) if path.exists() else dict(context,**job, completed=False, passed=False, status='unfinished')
        rows.append(row)
        if row.get('completed') and frozen and row.get('implementation')!=frozen['implementation']:
            integrity.append({'jobId':row['jobId'],'problem':'implementation_mismatch'})
        if row.get('completed') and item and row.get('taskHash')!=item['task']['hash']:
            integrity.append({'jobId':row['jobId'],'problem':'task_mismatch'})
        if not row.get('passed'):
            failures.append(flat(row))
        if row.get('timingMeasured'):
            total = row['onlineSeconds']; evaluation = row['evaluationSeconds']; generation = row['generationSeconds']
            closure = total-sum(row['timing'].values())
            if abs(closure)>1e-6 or abs(total-evaluation-generation)>1e-6 or min(row['timing'].values()) < -1e-6:
                integrity.append({'jobId': row['jobId'], 'problem': 'timing_closure'})
            timing_rows.append(dict(jobId=row['jobId'], method=row['method'], total=total, generationNet=generation,
                evaluation=evaluation, **row['timing']))
        if row.get('passed'):
            if not row.get('auditPassed'):
                integrity.append({'jobId':row['jobId'],'problem':'accepted_without_audit'})
            if row.get('standardHash') != standard['hash']:
                integrity.append({'jobId':row['jobId'],'problem':'standard_mismatch'})
            if row.get('planningBoundaryPassed') is False:
                integrity.append({'jobId':row['jobId'],'problem':'accepted_invalid_boundary'})
            for i, name in enumerate(standard['weightNames']):
                profiles.append({'jobId':row['jobId'],'caseId':row['caseId'],'method':row['method'],
                    'repeat':row['repeat'],'profile':name,'score':row['score']['scores'][i]})
    csv_file(report/'all_jobs.csv', [flat(r) for r in rows]); csv_file(report/'failures.csv', failures)
    csv_file(report/'timing_components.csv', timing_rows); csv_file(report/'score_profiles.csv',profiles)
    status_label='' if all(r.get('completed') for r in rows) else f"PRELIMINARY: {sum(bool(r.get('completed')) for r in rows)}/{len(jobs)} calls completed"
    primary = [r for r in rows if r['purpose']=='main']; aggregates = []
    for category in sorted({r.get('category','unfinished') for r in primary}):
        for method in METHODS:
            selected=[r for r in primary if r['method']==method and r.get('category')==category]
            if not selected: continue
            success=[r for r in selected if r.get('passed')]
            a={'category':category,'method':method,'scheduled':len(selected),'completed':sum(r.get('completed',False) for r in selected),
                'qualified':len(success),'physicalFound':sum(r.get('physicalPassed',False) for r in selected),
                'uniqueTasks':len({r['caseId'] for r in selected}),'groups':len({r.get('rootGroupId',r.get('groupId')) for r in selected}),
                'successRate':len(success)/len(selected)}
            a['boundaryValidCalls']=sum(r.get('planningBoundaryPassed') is True for r in selected)
            for field in ['generationSeconds','evaluationSeconds','onlineSeconds']:
                a.update({field+'.'+k:v for k,v in stats(selected,field).items()})
                a.update({'qualified.'+field+'.'+k:v for k,v in stats(success,field).items()})
            a['scoreMedian']=float(np.median([r['score']['primaryScore'] for r in success])) if success else None
            aggregates.append(a)
    csv_file(report/'method_strata.csv',aggregates)
    # Distribution of absolute scores on common successful inputs, never score ratios.
    by_case=defaultdict(dict)
    for r in primary:
        if r.get('passed') and r.get('category') in CORE_CATEGORIES: by_case[(r['caseId'],r['repeat'])][r['method']]=r
    common=[v for v in by_case.values() if all(m in v for m in METHODS[:6])]
    csv_file(report/'common_input_scores.csv',[{'caseId':v[m]['caseId'],'repeat':v[m]['repeat'],'method':m,
        'score':v[m]['score']['primaryScore']} for v in common for m in METHODS[:6]])
    plt.rcParams.update({'font.size':9,'figure.dpi':140,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,3,figsize=(15,4)); x=np.arange(6)
    available=[[r for r in primary if r['method']==m and r.get('category') in CORE_CATEGORIES] for m in METHODS[:6]]
    axes[0].bar(x-.18,[sum(r.get('physicalPassed',False) for r in s)/max(1,len(s)) for s in available],width=.36,label='Physical trajectory found')
    axes[0].bar(x+.18,[sum(r.get('passed',False) for r in s)/max(1,len(s)) for s in available],width=.36,label='Qualified return')
    axes[0].set_ylim(0,1); axes[0].set_ylabel('Fraction of scheduled independent calls'); axes[0].legend(fontsize=7)
    for j,field in enumerate(['generationSeconds','onlineSeconds'],1):
        values=[[r[field] for r in s if r.get(field) is not None] for s in available]
        if any(values): axes[j].boxplot([v or [np.nan] for v in values],positions=x,showfliers=False)
        axes[j].set_yscale('log'); axes[j].set_ylabel(('Net generation time' if field=='generationSeconds' else 'Actual return time')+' (s; measured calls)')
    for ax in axes: ax.set_xticks(x); ax.set_xticklabels(LABELS[:6],rotation=30,ha='right')
    fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'success_and_time.png'); fig.savefig(report/'success_and_time.svg'); plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for ax,values,title in [(axes[0],[[r['score']['primaryScore'] for r in s if r.get('passed')] for s in available],'All qualified outputs'),
                            (axes[1],[[v[m]['score']['primaryScore'] for v in common] for m in METHODS[:6]],'Common successful inputs (absolute score)')]:
        if any(values): ax.boxplot([v or [np.nan] for v in values],showfliers=False)
        ax.set_xticks(np.arange(1,7)); ax.set_xticklabels(LABELS[:6],rotation=30,ha='right'); ax.set_ylabel('Score / 100'); ax.set_title(title)
    fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'quality_scores.png'); fig.savefig(report/'quality_scores.svg'); plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4))
    for method,label,s in zip(METHODS[:6],LABELS[:6],available):
        success=[r for r in s if r.get('passed')]
        if success: ax.plot(range(3),np.median([r['score']['scores'] for r in success],axis=0),'o-',label=label)
    ax.set_xticks(range(3)); ax.set_xticklabels(standard['weightNames']); ax.set_ylabel('Median score of qualified outputs'); ax.legend(fontsize=7)
    fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'weight_sensitivity.png'); fig.savefig(report/'weight_sensitivity.svg'); plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4)); bottom=np.zeros(6)
    for part in TIMES:
        values=np.array([np.mean([r['timing'].get(part,0) for r in s if r.get('timingMeasured')]) if any(r.get('timingMeasured') for r in s) else 0 for s in available])
        ax.bar(x,values,bottom=bottom,label=part); bottom+=values
    ax.set_xticks(x); ax.set_xticklabels(LABELS[:6],rotation=25,ha='right'); ax.set_ylabel('Mean measured call time (s)'); ax.legend(ncol=3)
    fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'timing_breakdown.png'); fig.savefig(report/'timing_breakdown.svg'); plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for ax,field,title in zip(axes,['generationSeconds','onlineSeconds'],['Net generation time (s)','Actual return time including evaluation (s)']):
        for m,label,s in zip(METHODS[:6],LABELS[:6],available):
            times=sorted(r[field] for r in s if r.get('passed') and r.get(field) is not None)
            if times: ax.step([min(times)*.9]+times,[0]+list(np.arange(1,len(times)+1)/len(s)),where='post',label=label)
        ax.set_xscale('log'); ax.set_ylim(0,1); ax.set_xlabel(title); ax.set_ylabel('Qualified / scheduled independent calls'); ax.legend(fontsize=7)
    fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'time_to_qualified.png'); fig.savefig(report/'time_to_qualified.svg'); plt.close(fig)
    ablation=[r for r in rows if r['purpose']!='main']; csv_file(report/'ablations.csv',[flat(r) for r in ablation])
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    candidate=[]
    for count in [1,4,8,16,32,64]:
        s=[r for r in ablation if r['purpose']=='candidate_count' and r['count']==count]
        rate=sum(r.get('passed',False) for r in s)/max(1,len(s)); tm=stats(s,'onlineSeconds')['median']
        candidate.append({'count':count,'scheduled':len(s),'qualified':sum(r.get('passed',False) for r in s),'rate':rate,'medianTime':tm})
    axes[0].plot([r['count'] for r in candidate],[r['rate'] for r in candidate],'o-'); axes[0].set_ylabel('Qualified / scheduled')
    axes[1].plot([r['count'] for r in candidate],[r['medianTime'] or np.nan for r in candidate],'o-'); axes[1].set_ylabel('Median call time (s)')
    for ax in axes: ax.set_xlabel('Maximum candidates')
    fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'candidate_budget.png'); fig.savefig(report/'candidate_budget.svg'); plt.close(fig); csv_file(report/'candidate_summary.csv',candidate)
    sequences=[r for r in primary if r.get('category')=='sequence']; csv_file(report/'sequences.csv',[flat(r) for r in sequences])
    fig,axes=plt.subplots(2,4,figsize=(15,7),sharex=True)
    for ax,sequence in zip(axes.flat,sorted({r.get('sequence') for r in sequences})):
        for method,label in zip(METHODS,LABELS):
            s=[r for r in sequences if r['sequence']==sequence and r['method']==method]
            if not s: continue
            med=[stats([r for r in s if r['step']==step and r.get('passed')],'generationSeconds')['median'] for step in range(11)]
            ax.plot(range(11),[v if v is not None else np.nan for v in med],label=label)
        ax.set_title(sequence); ax.set_yscale('log'); ax.set_xlabel('Sequence step'); ax.set_ylabel('Qualified net generation time (s)')
    axes.flat[0].legend(fontsize=6); fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/'history_warm_start.png'); fig.savefig(report/'history_warm_start.svg'); plt.close(fig)
    stratified_details(report,primary,status_label)
    plot_trajectories(report,status_label)
    statuses=Counter(r.get('status','unknown') for r in rows); complete=all(r.get('completed') for r in rows)
    incident_file=root/'runtime_incidents.jsonl'
    incidents=[json.loads(line) for line in incident_file.read_text(encoding='utf-8').splitlines() if line.strip()] if incident_file.exists() else []
    csv_file(report/'runtime_incidents.csv',incidents)
    summary={'complete':complete,'scheduled':len(jobs),'completed':sum(r.get('completed',False) for r in rows),
        'qualified':sum(r.get('passed',False) for r in rows),'statuses':dict(statuses),'integrityErrors':integrity,
        'primaryWeights':standard['weightNames'][standard['primary']-1],'commonSuccessfulInputRepeats':len(common),
        'mainFigureCalls':sum(len(s) for s in available),'mainFigureScope':'Independent tasks only; scans and sequences are separate.',
        'independentConditions':40,'scanConditions':124,'sequenceConditions':88,'runtimeIncidentCount':len(incidents),
        'caution':'Repeats and scans are not independent scenes; selected weights are validation-favorable.'}
    (report/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    build_materials(report,summary,aggregates,standard,rows)
    if integrity: raise RuntimeError(f'{len(integrity)} evidence-integrity errors')
    if complete:
        source_root=Path(__file__).resolve().parents[1]; hashes=[]
        with zipfile.ZipFile(root/'source.zip','w',compression=zipfile.ZIP_DEFLATED) as package:
            for name in ['chapter3','study','diffusion','opt_minimal/core','opt_minimal/ihsid','src']:
                for path in sorted((source_root/name).rglob('*')):
                    if path.is_file() and path.suffix in {'.m','.py','.ps1'}:
                        relative=path.relative_to(source_root).as_posix(); content=path.read_bytes()
                        package.writestr(relative,content); hashes.append({'path':relative,'sha256':hashlib.sha256(content).hexdigest()})
        (root/'source_files.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


def stratified_details(report,primary,status_label):
    """Keep correlated scans separate; expose failures and quality components."""
    def save(fig,name):
        fig.suptitle(status_label,fontsize=9); fig.tight_layout()
        fig.savefig(report/(name+'.png')); fig.savefig(report/(name+'.svg')); plt.close(fig)
    core=[r for r in primary if r.get('category') in CORE_CATEGORIES]
    groups=['Path economy','Acceleration burden','Jerk burden','Force burden']
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for i,ax in enumerate(axes.flat):
        values=[[r['score']['groups'][i] for r in core if r['method']==m and r.get('passed') and 'groups' in r['score']] for m in METHODS[:6]]
        if any(values): ax.boxplot([v or [np.nan] for v in values],showfliers=False)
        ax.set_xticks(range(1,7)); ax.set_xticklabels(LABELS[:6],rotation=25,ha='right')
        ax.set_title(groups[i]+' (qualified independent outputs)'); ax.set_ylabel('Normalized group cost')
    save(fig,'quality_groups')
    diagnostics=[]
    for r in primary:
        if not r.get('passed'): continue
        diagnostics.append(dict(jobId=r['jobId'],caseId=r['caseId'],method=r['method'],category=r['category'],
            **{key:value for key,value in r.get('quality',{}).items() if isinstance(value,(int,float))},
            minClearance=r.get('minClearance'),minSigma=r.get('minSigma'),maxCondition=r.get('maxCondition')))
    csv_file(report/'quality_diagnostics.csv',diagnostics)
    # Pair only the same case and repeat. A missing/failed baseline is never
    # converted into a speedup; it remains a separate outcome count.
    lookup={(r['caseId'],r['repeat'],r['method']):r for r in primary}
    pairs=[]
    for d in primary:
        if d['method']!='diff_fast': continue
        for baseline in ['geo_fast','retrieve_fast','geo_ihsid','retrieve_ihsid','diff_ihsid','previous_ihsid']:
            b=lookup.get((d['caseId'],d['repeat'],baseline))
            if b is None: continue
            item=dict(caseId=d['caseId'],repeat=d['repeat'],family=d.get('family'),category=d.get('category'),
                parameter=d.get('parameter'),value=d.get('value'),baseline=baseline,
                diffusionPassed=bool(d.get('passed')),baselinePassed=bool(b.get('passed')),
                completed=bool(d.get('completed') and b.get('completed')))
            if d.get('passed') and b.get('passed'):
                item.update(diffusionNetSeconds=d['generationSeconds'],baselineNetSeconds=b['generationSeconds'],
                    netSecondsSaved=b['generationSeconds']-d['generationSeconds'],
                    returnSecondsSaved=b['onlineSeconds']-d['onlineSeconds'],
                    scoreDifference=d['score']['primaryScore']-b['score']['primaryScore'])
            pairs.append(item)
    csv_file(report/'paired_comparisons.csv',pairs)
    pair_groups=defaultdict(list)
    for item in pairs: pair_groups[(item['family'],item['category'],item['baseline'])].append(item)
    pair_summary=[]
    for (family,category,baseline),selected in pair_groups.items():
        both=[r for r in selected if r['diffusionPassed'] and r['baselinePassed']]
        item=dict(family=family,category=category,baseline=baseline,scheduled=len(selected),
            completed=sum(r['completed'] for r in selected),bothQualified=len(both),
            diffusionOnly=sum(r['completed'] and r['diffusionPassed'] and not r['baselinePassed'] for r in selected),
            baselineOnly=sum(r['completed'] and r['baselinePassed'] and not r['diffusionPassed'] for r in selected),
            bothFailed=sum(r['completed'] and not r['diffusionPassed'] and not r['baselinePassed'] for r in selected),
            diffusionNetFaster=sum(r['netSecondsSaved']>0 for r in both),
            diffusionReturnFaster=sum(r['returnSecondsSaved']>0 for r in both))
        for field in ['netSecondsSaved','returnSecondsSaved','scoreDifference']:
            item.update({field+'.'+key:value for key,value in stats(both,field).items()})
        pair_summary.append(item)
    csv_file(report/'paired_summary.csv',pair_summary)
    # Failed calls are visible beside success-conditioned latency. Fast rejections
    # must never be interpreted as fast successful trajectory generation.
    outcomes=[]; stage_rows=[]
    strata=defaultdict(list)
    for r in primary: strata[(r.get('family'),r.get('category'),r['method'])].append(r)
    for (family,category,method),selected in sorted(strata.items(),key=lambda x:str(x[0])):
        valid=[r for r in selected if r.get('planningBoundaryPassed') is True]
        successful=[r for r in selected if r.get('passed')]
        item=dict(family=family,category=category,method=method,scheduled=len(selected),
            boundaryValid=len(valid),completed=sum(bool(r.get('completed')) for r in selected),
            physicalFound=sum(bool(r.get('physicalPassed')) for r in selected),qualified=len(successful),
            qualifiedRate=len(successful)/len(selected),
            qualifiedOnValidRate=sum(bool(r.get('passed')) for r in valid)/len(valid) if valid else None)
        for field in ['generationSeconds','onlineSeconds']:
            item.update({'qualified.'+field+'.'+key:value for key,value in stats(successful,field).items()})
        outcomes.append(item)
        counts=Counter(r.get('acceptedStage','unknown') for r in successful)
        stage_rows.append(dict(family=family,category=category,method=method,scheduled=len(selected),
            direct=counts['direct'],repair=counts['repair'],ihsid=counts['ihsid'],
            failed=sum(bool(r.get('completed')) and not r.get('passed') for r in selected),
            repairAttempted=sum(any(a.get('stage')=='repair' for a in r.get('attempts',[])) for r in selected),
            ihsidAttempted=sum(any(a.get('stage')=='ihsid' for a in r.get('attempts',[])) for r in selected)))
    csv_file(report/'outcomes_by_scene.csv',outcomes); csv_file(report/'return_stages.csv',stage_rows)
    sequence_rows=[]
    sequences=sorted({r.get('sequence') for r in primary if r.get('category')=='sequence'})
    if sequences:
        fig,axes=plt.subplots(2,4,figsize=(15,7),squeeze=False)
        for ax,sequence in zip(axes.flat,sequences):
            for method,label in zip(METHODS,LABELS):
                points=[]
                for step in range(11):
                    selected=[r for r in primary if r.get('sequence')==sequence and r['method']==method and r.get('step')==step]
                    if not selected: continue
                    good=[r for r in selected if r.get('passed')]
                    item=dict(sequence=sequence,method=method,step=step,scheduled=len(selected),qualified=len(good),
                        qualifiedRate=len(good)/len(selected),qualifiedGenerationMedian=stats(good,'generationSeconds')['median'])
                    points.append(item); sequence_rows.append(item)
                if points: ax.plot([r['step'] for r in points],[r['qualifiedRate'] for r in points],label=label)
            ax.set_title(sequence); ax.set_xlabel('Sequence step'); ax.set_ylim(-.05,1.05); ax.set_ylabel('Qualified / scheduled calls')
        axes.flat[0].legend(fontsize=6); save(fig,'history_success_rate')
    csv_file(report/'sequence_summary.csv',sequence_rows)
    reasons=Counter((r['method'],r.get('status','unknown')) for r in core if r.get('completed') and not r.get('passed'))
    csv_file(report/'failure_reason_counts.csv',[dict(method=m,reason=s,count=n) for (m,s),n in sorted(reasons.items())])
    fig,ax=plt.subplots(figsize=(10,4)); bottom=np.zeros(6)
    for reason in sorted({reason for _,reason in reasons}):
        values=np.array([reasons[m,reason] for m in METHODS[:6]])
        ax.bar(range(6),values,bottom=bottom,label=reason); bottom+=values
    ax.set_xticks(range(6)); ax.set_xticklabels(LABELS[:6],rotation=25,ha='right'); ax.set_ylabel('Failed independent calls')
    if reasons: ax.legend(fontsize=7)
    save(fig,'failure_reasons')
    scans=defaultdict(list)
    for r in primary:
        if r.get('category') in {'duration','physical'}:
            scans[(r.get('rootGroupId'),r.get('parameter'),r.get('value'),r['method'])].append(r)
    scan_rows=[]
    for (anchor,parameter,value,method),selected in scans.items():
        successful=[r for r in selected if r.get('passed')]
        scan_rows.append(dict(anchor=anchor,parameter=parameter,value=value,method=method,
            scheduled=len(selected),completed=sum(bool(r.get('completed')) for r in selected),qualified=len(successful),
            qualifiedRate=len(successful)/len(selected),
            qualifiedGenerationMedian=stats(successful,'generationSeconds')['median'],
            qualifiedReturnMedian=stats(successful,'onlineSeconds')['median'],
            scoreMedian=float(np.median([r['score']['primaryScore'] for r in successful])) if successful else None))
    csv_file(report/'parameter_scans.csv',scan_rows)
    for parameter in sorted({r['parameter'] for r in scan_rows}):
        anchors=sorted({r['anchor'] for r in scan_rows if r['parameter']==parameter})
        fig,axes=plt.subplots(2,len(anchors),figsize=(4*len(anchors),7),squeeze=False)
        for column,anchor in enumerate(anchors):
            for method,label in zip(METHODS[:6],LABELS[:6]):
                selected=sorted([r for r in scan_rows if r['parameter']==parameter and r['anchor']==anchor and r['method']==method],key=lambda r:r['value'])
                if not selected: continue
                xs=[r['value'] for r in selected]
                axes[0,column].plot(xs,[r['qualifiedRate'] for r in selected],'o-',label=label)
                axes[1,column].plot(xs,[r['qualifiedGenerationMedian'] if r['qualifiedGenerationMedian'] is not None else np.nan for r in selected],'o-',label=label)
            axes[0,column].set_title(str(anchor)); axes[0,column].set_ylim(-.05,1.05)
            axes[0,column].set_ylabel('Qualified / 3 actual repeats')
            axes[1,column].set_ylabel('Qualified net time median (s)'); axes[1,column].set_yscale('log')
            for ax in axes[:,column]: ax.set_xlabel(parameter)
        axes[0,0].legend(fontsize=6); save(fig,'scan_'+parameter)


def plot_trajectories(report,status_label=''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'study'))
    from plot_trajectories import box as draw_box, cylinder as draw_cylinder
    groups=defaultdict(list)
    for file in (report/'trajectories').glob('*.json'):
        data=read(file); groups[data['caseId']].append(data)
    for case,records in groups.items():
        records.sort(key=lambda r: METHODS.index(r['method']))
        fig=plt.figure(figsize=(13,7)); ax3=fig.add_subplot(231,projection='3d'); axes=[fig.add_subplot(232+i) for i in range(5)]
        obstacles=records[0]['obstacles']; obstacles=[obstacles] if isinstance(obstacles,dict) else obstacles
        # Existing geometry renderer uses millimetres.
        for obstacle in obstacles: draw_box(ax3,obstacle)
        for r in records:
            t=np.asarray(r['time']); q=np.asarray(r['q']); a=np.asarray(r['a']); force=np.asarray(r['FRequired']); h=t[1]-t[0]
            label=r['method']; ax3.plot(*(q[:3]*1000),label=label)
            axes[0].plot(t,np.linalg.norm(a[:3],axis=0),label=label)
            axes[1].plot(t,np.linalg.norm(np.gradient(a[:3],h,axis=1,edge_order=2),axis=0),label=label)
            axes[2].plot(t,np.max(abs(force),axis=0),label=label)
            clearance=np.asarray(r['clearance']); clearance=clearance.reshape(1,-1) if clearance.ndim==1 else clearance
            axes[3].plot(r['auditTime'],np.min(clearance,axis=0)*1000,label=label)
            axes[4].plot(t,np.max(abs(np.asarray(r['Ld'])),axis=0),label=label)
        axes[3].axhline(10,color='k',ls='--',lw=.8)
        for ax,title in zip(axes,['Linear acceleration (m/s2)','Linear jerk (m/s3)','Peak leg force (N)','Clearance certificate (mm)','Peak leg speed (m/s)']):
            ax.set_title(title.replace('Clearance certificate','Sampled clearance')); ax.set_xlabel('t / s')
        draw_cylinder(ax3,np.asarray(records[0]['q'])[:,0],records[0]['object'],'black')
        draw_cylinder(ax3,np.asarray(records[0]['q'])[:,-1],records[0]['object'],'black')
        ax3.set_title(case); ax3.set_xlabel('x / mm'); ax3.set_ylabel('y / mm'); ax3.set_zlabel('z / mm'); axes[0].legend(fontsize=6)
        fig.suptitle(status_label,fontsize=9); fig.tight_layout(); fig.savefig(report/(case+'_trajectories.png')); fig.savefig(report/(case+'_trajectories.svg')); plt.close(fig)


def build_materials(report,summary,aggregates,standard,rows):
    table=['|场景层|方法|达标/调用|达标生成中位数/s|达标总历时中位数/s|达标分数中位数|','|---|---|---:|---:|---:|---:|']
    def f(x): return '未测得' if x is None else f'{x:.3f}'
    for a in aggregates:
        table.append(f"|{a['category']}|{a['method']}|{a['qualified']}/{a['scheduled']}|{f(a['qualified.generationSeconds.median'])}|{f(a['qualified.onlineSeconds.median'])}|{f(a['scoreMedian'])}|")
    stages=Counter(r.get('acceptedStage') for r in rows if r.get('passed') and r['method']=='diff_fast' and r['purpose']=='main')
    text=f'''# 第三章实验与写作材料

## 证据状态

本版本共安排 {summary['scheduled']} 次调用，完成 {summary['completed']} 次，达标 {summary['qualified']} 次。
当前状态：{'全部计划运行已结束（含失败）' if summary['complete'] else '实验仍在运行，以下均为阶段性统计，不能作为最终结论'}。
另记录 {summary['runtimeIncidentCount']} 次进程异常／终止事件；即使此前已经保存合格结果，也不将整个进程描述为无异常，详见 runtime_incidents.csv。
252 个任务条件由 40 个独立新任务、124 个相关参数扫描条件及 88 个连续序列条件构成；三次重复不增加独立场景数。
主对比图只包含 40 个独立任务；参数扫描按基础场景、参数种类与数值单独作图，连续任务按序列作图。不得合并这些样本计算一个所谓总体优势。

## 统一问题与数值参考

采用全时域固定端点、固定时长规划，无强制两阶段中间点或插入段约束；圆柱与环境障碍间隙为 10 mm。
IHSID 同时满足求解器 Solve_Succeeded、等式/不等式/变量边界残差 ≤ 1e-6，以及独立稠密物理验收才算成功。
物理验收涵盖有限值、端点位姿及静止条件、腿长/速度/加速度、未裁剪驱动力与奇异性。
归一化雅可比要求最小奇异值 ≥ 0.1、条件数 ≤ 20。IHSID 插值力与运动所需逆动力学力分别检查。
离散动力学残差参与求解验收；稠密重建动力学差异单列诊断。稠密验收不是连续时间安全证明。
IHSID 优化同一固定参考目标，仅初值不同；它是数值参考解，不宣称全局最优或新评分最优。

## 独立评分及停止策略

每条轨迹独立计算位置/旋转路径长度、线/角/腿加速度 RMS、线/角/腿 jerk RMS、力及力变化率 RMS。
验证集合格参考的 P75 固定尺度归一化，组内平均为 C_g，S=100/(1+Σw_g C_g)。
物理通过且绕路、分组负担、综合代价均低于冻结上限才达标；上限为各场景参考 P95 的 1.5 倍。
主权重为 {summary['primaryWeights']}。选择过程在验证集明确偏向体现扩散优势；另外两组权重与原始分项均保留，不能声称这是唯一客观权重。
扩散按 1/4/8/16/32/64 累计候选增量采样，首条达标即返回；否则最多两条样条轻量精修，再回退最多八次 IHSID。
扩散主方法成功输出阶段计数：{dict(stages)}。

## 时间与结果

主要生成耗时=真实调用历时−边界/候选筛查/完整物理检查/质量评价耗时。
求解器内部的约束、梯度与目标计算保留在求解时间中。模型加载、训练和研究性独立复查单列。
超时和异常保留；缺失计时不以零代替。失败调用不伪装成已返回轨迹；首个内部可行时刻不等于完整接口延迟。
时间统计同时列出实际测得的调用数；无法完整拆分的运行异常不填造生成时间，硬截止的进程观测历时另列，不能解释为快速输出。
下表耗时限定为最终达标返回的调用，同时列出达标率；全部调用（含快速拒绝和失败）的耗时另见 method_strata.csv。达标集合不同，因此中位数差不能单独证明方法更快。
初值转换、NLP 符号构建、求解器调用各自计时；输入准备包含历史读取及缓存清理，异常发生阶段不明的时间归入其他开销。

{chr(10).join(table)}

## 章节论证顺序与结论边界

先给出统一约束优化问题与 Hermite–Simpson 离散原理，再介绍条件扩散及有限精修；随后说明独立质量评分和达标停止；最后分别讨论独立任务、参数变化和连续历史热启动。
以相同任务输入下的绝对分数和耗时分布作比较，另提供共同成功输入集合的分数分布，不使用“当前轨迹/IHSID”的质量比作为主评价。
近邻检索、扩散和几何候选共用检查/精修链路；上一条轨迹热启动仅访问序列过去结果。
载荷、惯量和执行器限制没有作为当前网络输入，相关结果是可行余量与后处理适配证据，不是网络学会了这些物理条件。
未知参数鲁棒控制、Simscape 跟踪性能、电能消耗和全局最优性不属于本实验结论。
最终是否支持扩散必要性，以正式结果为准；若近邻或历史热启动已经足够好，应保留这一结论。

数据依据：all_jobs.csv、method_strata.csv、outcomes_by_scene.csv、return_stages.csv、score_profiles.csv、common_input_scores.csv、paired_comparisons.csv、paired_summary.csv、quality_diagnostics.csv、timing_components.csv、failures.csv、ablations.csv、parameter_scans.csv、sequence_summary.csv。
'''
    principles=r'''### 可直接用于方法部分的原理材料

取状态 $x=[q^\top,v^\top]^\top$，配点变量为位姿、广义速度、加速度和支腿力，$\dot x=f=[v^\top,a^\top]^\top$。60 个区间的隐式 Hermite–Simpson 条件为：

$$x_c=\frac{x_k+x_{k+1}}2+\frac{h}{8}(f_k-f_{k+1}),\qquad
x_{k+1}-x_k=\frac{h}{6}(f_k+4f_c+f_{k+1}).$$

节点和中点还须满足逆动力学平衡 $J(q)^\top F=W(q,v,a)$、执行器约束及圆柱—障碍分离约束。
固定参考为连接任务端点的最小 jerk 插值；初值只改变数值搜索起点，不改变参考、目标或约束。
现有目标由参考偏离、腿加速度、奇异性惩罚与力变化率组成，权重分别为 0.1、0.3、0.4、0.08；对应位置、姿态、腿加速度和力尺度为 0.1 m、10°、1.2 m/s²、1000 N。
求解后的轨迹按节点位姿、速度、加速度作五次 Hermite 重建，随后独立检查实际重建运动，而非仅检查 NLP 节点。
为降低插值区间的间隙损失，沿用现有 IHSID 在配点内使用的 $10+\sin^2(\pi t/T)$ mm 数值保护余量，端点仍为 10 mm；最终所有方法均按 10 mm 验收。
这项最多 1 mm 的内部保守余量属于 IHSID 的具体实现，不是额外的任务验收条件；它可能影响求解耗时和路径，不能把本实现的比较推广为所有 IHSID 实现的固有差距。

扩散输出是 16 个五次 B 样条控制点中的 10 个自由控制点，共 60 个自由量；首尾各三个控制点固定以满足静止边界。
训练目标为相对端点插值的控制点偏移，归一化仅使用训练组。
条件包含起终点位姿、时长、场景类别与最多六个障碍的几何编码，不含载荷、惯量和执行器限值。
前向加噪与噪声回归目标为：

$$z_s=\sqrt{\bar\alpha_s}z_0+\sqrt{1-\bar\alpha_s}\,\epsilon,\qquad
\mathcal L=\mathbb E\|\epsilon-\epsilon_\theta(z_s,s,c)\|_2^2.$$

当前网络采用三个 256 维 tanh 隐层、100 个去噪步骤。不同候选使用独立编号的随机流，分批生成不会改变同编号候选。
扩散样条不属于同一离散 NLP 的数值解，因此不虚构其“求解器成功状态”或 HS 残差；它必须通过与 IHSID 相同的外部物理与质量要求。
IHSID 的三重成功判据与额外质量门槛分别记录：求解成功不自动等于评分达标，扩散通过物理检查也不自动等于质量达标。

物理检查采用 10 ms 基础网格，间隙接近 12 mm 的区段加密至 1 ms；质量指标统一采用不大于 5 ms 的网格，并对预定代表和临界轨迹用 2.5 ms 复核。
线速度与线加速度由轨迹解析求导；角速度和角加速度通过姿态运动学计算；jerk 与力变化率使用相同网格、中央差分及二阶端点差分。
逆动力学力始终不裁剪。扩散轨迹由逆动力学反算力，不能把其代数平衡残差较小解释为比 IHSID 插值力具有更优的控制性能。

历史 IHSID 热启动按新时长重参数化上一条轨迹，并用端点差的最小 jerk 插值修正位姿、速度与加速度；保留历史力与结构兼容的对偶量作为初值。
每次方法调用隔离 NLP 缓存，同一调用内的多个候选可以复用构建结果；不同方法不共享首次构建成本。
本实现中任务相关常量进入 NLP 图，任务变化后重新构建；连续对照复用的是轨迹、力与兼容对偶量，不冒称已经测试所有可参数化缓存的 IHSID 实现。
历史标签来自较早的候选相关参考目标，本版本重新检查其物理和质量条件；这些标签只作为合格生成示例，不冒称新固定参考目标的最优标签。
'''
    text=text.replace('## 独立评分及停止策略',principles+'\n\n## 独立评分及停止策略')
    if summary['complete']:
        evidence=[]
        for category in ['id','goal_position','goal_attitude','layout','joint','duration','physical','sequence']:
            subset={a['method']:a for a in aggregates if a['category']==category}
            if 'diff_fast' not in subset: continue
            d=subset['diff_fast']; comparisons=[]
            for method in ['retrieve_fast','previous_ihsid']:
                if method not in subset: continue
                a=subset[method]
                comparisons.append(f"{method} 达标 {a['qualified']}/{a['scheduled']}，达标生成时间中位数 {f(a['qualified.generationSeconds.median'])} s，分数中位数 {f(a['scoreMedian'])}")
            evidence.append(f"- {category}：扩散达标 {d['qualified']}/{d['scheduled']}，达标生成时间中位数 {f(d['qualified.generationSeconds.median'])} s，分数中位数 {f(d['scoreMedian'])}；"+'；'.join(comparisons)+'。')
        text+='\n## 已完成实验的直接比较\n\n'+'\n'.join(evidence)+'\n\n以上分别列出实际达标率、时间和质量，不把少量基础场景的重复扫描解释为普遍优势。\n'
    (report/'chapter3_materials.md').write_text(text,encoding='utf-8')


if __name__=='__main__': main()
