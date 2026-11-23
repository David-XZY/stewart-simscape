"""Summarize frozen records; never adjusts tasks, planners, or success flags."""
import argparse
import collections
import csv
import json
import math
from pathlib import Path
import statistics

METHODS = ['geo_ihsid', 'diff_ihsid', 'geo_fast', 'retrieve_fast', 'diff_direct', 'diff_fast']


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def numeric(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def quantile(values, fraction):
    values = sorted(x for x in values if numeric(x))
    if not values:
        return None
    position = (len(values) - 1) * fraction
    left = int(position)
    return values[left] + (values[min(left + 1, len(values)-1)] - values[left]) * (position-left)


def flatten(row):
    result = {}
    for key, value in row.items():
        if isinstance(value, dict):
            result.update({key + '_' + k: v for k, v in flatten(value).items()})
        elif isinstance(value, list):
            result[key] = json.dumps(value, ensure_ascii=False)
        else:
            result[key] = value
    return result


def save_csv(path, rows):
    if not rows:
        path.write_text('', encoding='utf-8-sig')
        return
    rows = [flatten(row) for row in rows]
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def describe(rows):
    complete = [r for r in rows if r.get('completed')]
    times = [r['onlineSeconds'] for r in complete if numeric(r.get('onlineSeconds'))]
    legal = [r for r in complete if r.get('planningBoundaryPassed')]
    truth_legal = [r for r in complete if r.get('truthBoundaryPassed')]
    legal_times = [r['onlineSeconds'] for r in legal if numeric(r.get('onlineSeconds'))]
    success_times = [r['onlineSeconds'] for r in complete if r.get('passed') and numeric(r.get('onlineSeconds'))]
    nominal = [r for r in complete if r.get('nominalPassed') and r.get('withinBudget')]
    return dict(scheduled=len(rows), completed=len(complete), unfinished=len(rows)-len(complete),
        independentGroups=len(set(r['groupId'] for r in rows)),
        distinctCases=len(set(r['caseId'] for r in rows)),
        passed=sum(bool(r.get('passed')) for r in complete),
        boundaryLegal=len(legal), boundaryLegalPassed=sum(bool(r.get('passed')) for r in legal),
        truthBoundaryLegal=len(truth_legal),truthBoundaryLegalPassed=sum(bool(r.get('passed')) for r in truth_legal),
        nominalAccepted=len(nominal), falseAccepted=sum(bool(r.get('falseAcceptance')) for r in nominal),
        directlyFeasibleTasks=sum(bool(r.get('directTaskTruthPassed')) for r in complete),
        rawCandidates=sum(r.get('rawCandidateCount', 0) for r in complete),
        rawPassed=sum(r.get('rawCandidatePasses', 0) for r in complete),
        repairAttempted=sum(r.get('repairAttempts', 0) > 0 for r in complete),
        repairSucceeded=sum(r.get('acceptedStage') == 'repair' and bool(r.get('passed')) for r in complete),
        timedJobs=len(times), meanSeconds=statistics.mean(times) if times else None,
        medianSeconds=quantile(times, .5), p90Seconds=quantile(times, .9), maxSeconds=max(times) if times else None,
        legalMedianSeconds=quantile(legal_times,.5),legalP90Seconds=quantile(legal_times,.9),
        successMedianSeconds=quantile(success_times,.5),successP90Seconds=quantile(success_times,.9),
        reusedPlans=sum(bool(r.get('planningReused')) for r in complete))


def aggregate(rows, keys):
    groups = collections.defaultdict(list)
    for row in rows:
        groups[tuple(row.get(k) for k in keys)].append(row)
    return [dict(zip(keys, key), **describe(group)) for key, group in sorted(groups.items(), key=lambda x: str(x[0]))]


def fmt(value):
    return '—' if value is None else f'{value:.3g}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory
    report = root / 'report'
    report.mkdir(exist_ok=True)
    manifest, jobs = read(root/'manifest.json'), read(root/'jobs.json')
    details={r['jobId']:r for r in read(root/'quality_details.json')} if (root/'quality_details.json').exists() else {}
    diagnostics={r['jobId']:r for r in read(root/'failure_details.json')} if (root/'failure_details.json').exists() else {}
    timing_details={r['jobId']:r for r in read(root/'timing_details.json')} if (root/'timing_details.json').exists() else {}
    cases = {c['caseId']: c for c in manifest['cases'] + manifest['probes']}
    rows = []
    for job in jobs:
        item = cases[job['caseId']]
        row = {k: item[k] for k in ['caseId', 'category', 'parameter', 'value', 'family', 'mode', 'groupId']}
        row.update(job)
        row.update(completed=False, passed=False, status='unfinished', core=item['core'],
                   planningBoundaryPassed=bool(item['planningTask'] and item['planningTask']['endpointPassed']),
                   truthBoundaryPassed=bool(item['truthTask'] and item['truthTask']['endpointPassed']))
        path = root/'jobs'/job['jobId']/'result.json'
        if path.exists():
            row.update(read(path))
        if row.get('timeoutPhase')=='audit' and row.get('onlineSeconds')==0:
            row['onlineSeconds']=None; row['timingUnavailable']=True
        if job['jobId'] in timing_details:
            timing=timing_details[job['jobId']]
            for key in ['onlineSeconds','firstFeasibleSeconds','nominalPassed','withinBudget']:
                row[key]=timing[key]
            row['timingSource']=timing['source']; row['timingUnavailable']=False
        post_file=root/'post_audit'/(job['jobId']+'.json')
        if post_file.exists():
            post=read(post_file); row['postAudit']=post
            if row.get('nominalPassed') and row.get('withinBudget'):
                row['truthPassed']=post['passed']; row['falseAcceptance']=not post['passed']
            if row.get('passed') and not post['passed']:
                row['originalReportedPassed']=True; row['passed']=False; row['status']='post_audit_failed'
            row['firstTruthPassed']=bool(row.get('firstTruthPassed') and post['firstPassed'])
            events=row.get('anytime') or []; events=[events] if isinstance(events,dict) else events
            event_passes=post['eventPassed']; event_passes=[event_passes] if isinstance(event_passes,bool) else event_passes
            for event,accepted in zip(events,event_passes): event['truthPassed']=bool(event.get('truthPassed') and accepted)
            row['anytime']=events
        if isinstance(row.get('quality'),dict):
            quality=row['quality']; duration=item['truthTask']['T']
            quality['positionAccelSquaredIntegral']=quality['positionAccelRms']**2*duration
            quality['angularAccelSquaredIntegral']=quality['angularAccelRms']**2*duration
            quality.update({k:v for k,v in details.get(job['jobId'],{}).items() if k!='jobId'})
        if job['jobId'] in diagnostics:
            row['diagnostics']=diagnostics[job['jobId']]
        rows.append(row)
    save_csv(report/'all_jobs.csv', rows)
    save_csv(report/'fresh_process_audit.csv',[r['postAudit'] for r in rows if 'postAudit' in r])
    save_csv(report/'failures_and_unfinished.csv', [r for r in rows if not r.get('passed')])
    if (root/'runtime_incidents.json').exists():
        save_csv(report/'runtime_incidents.csv',read(root/'runtime_incidents.json'))
    if (root/'boundary_details.json').exists():
        save_csv(report/'boundary_details.csv',read(root/'boundary_details.json'))
    rejections=[]
    for row in rows:
        if row['purpose']!='main' or row['repeat']!=1: continue
        for reason in row.get('diagnostics',{}).get('rawFirstReasons',[]):
            rejections.append(dict(caseId=row['caseId'],category=row['category'],parameter=row['parameter'],value=row['value'],
                              mode=row['mode'],method=row['method'],firstReason=reason['reason'],candidates=reason['count']))
    save_csv(report/'candidate_rejection_reasons.csv',rejections)
    main_rows = [r for r in rows if r['purpose'] == 'main']
    core = [r for r in main_rows if r['core'] and r['repeat'] == 1]
    summary = aggregate(core, ['method'])
    save_csv(report/'core_methods_seed1.csv', summary)
    save_csv(report/'all_method_strata.csv', aggregate(main_rows, ['category', 'mode', 'family', 'method', 'repeat']))
    scans = [r for r in main_rows if r['repeat'] == 1]
    save_csv(report/'parameter_success.csv', aggregate(scans, ['category', 'parameter', 'value', 'mode', 'method']))
    counts = [r for r in rows if r['core'] and r['method'] == 'diff_direct']
    save_csv(report/'candidate_counts.csv', aggregate(counts, ['count', 'repeat', 'family']))
    save_csv(report/'unknown_parameters.csv', [r for r in rows if r['mode'] == 'unknown'])
    # Keep runtime failures and missing measurements visible in time breakdowns.
    timing_fields=['boundarySeconds','samplingSeconds','rankingSeconds','validationSeconds',
                   'repairSeconds','solverBuildSeconds','solverSeconds']
    timing_rows=[]
    for row in main_rows:
        record={k:row.get(k) for k in ['jobId','caseId','category','mode','method','repeat',
                                      'completed','passed','status','onlineSeconds','planningReused']}
        for key in timing_fields: record[key]=row.get(key)
        measured=all(numeric(record[k]) for k in timing_fields)
        record['allComponentsAvailable']=measured
        record['unassignedOnlineSeconds']=(record['onlineSeconds']-sum(record[k] for k in timing_fields)
             if measured and numeric(record['onlineSeconds']) else None)
        record['researchAuditSeconds']=row.get('researchAuditSeconds')
        timing_rows.append(record)
    save_csv(report/'timing_breakdown.csv',timing_rows)
    timing_summary=[]
    for method in METHODS:
        cohort=[r for r in core if r['method']==method and r['planningBoundaryPassed']]
        for metric in ['onlineSeconds','firstFeasibleSeconds']+timing_fields+['researchAuditSeconds']:
            values=[r.get(metric) for r in cohort if numeric(r.get(metric))]
            timing_summary.append(dict(method=method,metric=metric,legalScheduled=len(cohort),
                measured=len(values),mean=statistics.mean(values) if values else None,
                median=quantile(values,.5),p90=quantile(values,.9),maximum=max(values) if values else None))
    save_csv(report/'core_timing_components.csv',timing_summary)
    unknown_summary=[]
    for method in ['geo_ihsid','diff_ihsid','diff_fast']:
        cohort=[r for r in main_rows if r['mode']=='unknown' and r['method']==method]
        accepted=[r for r in cohort if r.get('nominalPassed') and r.get('withinBudget')]
        failures=[r for r in accepted if r.get('falseAcceptance')]
        record=dict(method=method,**describe(cohort),falseAcceptanceDenominator=len(accepted),
                    falseAcceptanceRate=len(failures)/len(accepted) if accepted else None)
        for metric in ['forceMismatchPeak','forceMismatchRms']:
            values=[r[metric] for r in accepted if numeric(r.get(metric))]
            record[metric+'Measured']=len(values); record[metric+'Median']=quantile(values,.5)
            record[metric+'Maximum']=max(values) if values else None
        unknown_summary.append(record)
    save_csv(report/'unknown_summary.csv',unknown_summary)
    save_csv(report/'false_acceptances.csv',[r for r in main_rows if r.get('falseAcceptance')])

    baseline = {r['caseId']: r for r in main_rows if r['method'] == 'geo_ihsid'}
    paired = []
    for row in main_rows:
        base = baseline.get(row['caseId'])
        if not base or row['method'] == 'geo_ihsid':
            continue
        pair = {k: row[k] for k in ['caseId', 'groupId', 'category', 'mode', 'family', 'method', 'repeat']}
        pair.update(baselineCompleted=base['completed'], methodCompleted=row['completed'],
                    baselinePassed=base['passed'], methodPassed=row['passed'])
        if base['passed'] and row['passed']:
            pair['speedup'] = base['onlineSeconds']/max(row['onlineSeconds'], 1e-12)
            if row.get('firstTruthPassed') and numeric(row.get('firstFeasibleSeconds')):
                pair['firstFeasibleSpeedup'] = base['firstFeasibleSeconds']/max(row['firstFeasibleSeconds'], 1e-12)
            for key, value in row.get('quality', {}).items():
                other = base.get('quality', {}).get(key)
                if key != 'seconds' and numeric(value) and numeric(other) and abs(other) > 1e-12:
                    pair['qualityRatio_'+key] = value/other
        paired.append(pair)
    save_csv(report/'paired_comparisons.csv', paired)
    # Attribute learning separately from the shared fast checker/repair machinery.
    fast_pairs=[]
    for control in ['geo_fast','retrieve_fast']:
        controls={r['caseId']:r for r in core if r['method']==control}
        for row in main_rows:
            if not row['core'] or row['method']!='diff_fast': continue
            base=controls.get(row['caseId'])
            if base is None: continue
            pair=dict(caseId=row['caseId'],family=row['family'],category=row['category'],repeat=row['repeat'],
                      control=control,controlCompleted=base['completed'],diffusionCompleted=row['completed'],
                      controlPassed=base['passed'],diffusionPassed=row['passed'])
            if base['passed'] and row['passed']:
                pair['speedup']=base['onlineSeconds']/max(row['onlineSeconds'],1e-12)
                for key,value in row.get('quality',{}).items():
                    other=base.get('quality',{}).get(key)
                    if key!='seconds' and numeric(value) and numeric(other) and abs(other)>1e-12:
                        pair['qualityRatio_'+key]=value/other
            fast_pairs.append(pair)
    save_csv(report/'fast_control_pairs.csv',fast_pairs)
    quality_summary=[]
    for scope in ['core_known','scan_known','scan_unknown']:
        for method in METHODS[1:]:
            sample=[p for p in paired if p['method']==method and p['repeat']==1
                    and (('core' in scope)==(p['category'] in ['id','goal_position','goal_attitude','layout']))
                    and p['mode']==('unknown' if scope.endswith('unknown') else 'known')]
            metrics=sorted({k for p in sample for k in p if k.startswith('qualityRatio_')})
            for key in metrics:
                matches=[p for p in sample if key in p]; values=[p[key] for p in matches]
                worst=max(matches,key=lambda p:p[key])
                quality_summary.append(dict(scope=scope,method=method,metric=key,pairedTasks=len(values),
                     independentGroups=len(set(p['groupId'] for p in matches)),
                     median=quantile(values,.5),p10=quantile(values,.1),p90=quantile(values,.9),
                     minimum=min(values),maximum=max(values),maximumCase=worst['caseId']))
    save_csv(report/'paired_quality_summary.csv',quality_summary)
    gains = []
    for scope in ['core_known','scan_known','scan_unknown']:
        for method in METHODS[1:]:
            records = [p for p in paired if p['method'] == method and p['repeat'] == 1
                       and (('core' in scope)==(p['category'] in ['id','goal_position','goal_attitude','layout']))
                       and p['mode']==('unknown' if scope.endswith('unknown') else 'known')]
            gains.append(dict(scope=scope,method=method, scheduled=len(records),
                 independentGroups=len(set(p['groupId'] for p in records)),
                 pairedSuccess=sum('speedup' in p for p in records),
                 **{f'speedupAtLeast{x}': sum(p.get('speedup', 0) >= x for p in records) for x in [2,3,5,10]}))
    save_csv(report/'speedup_counts.csv', gains)
    initial_guesses=[]
    for row in main_rows:
        if row['method'] not in ('geo_ihsid','diff_ihsid'):
            continue
        records=row.get('solverRecords') or []
        if isinstance(records,dict): records=[records]
        metric={k:row.get(k) for k in ['caseId','groupId','family','category','mode','method','repeat','passed','status','onlineSeconds','solveAttempts','solverBuildSeconds','solverSeconds']}
        metric['completedSolverAttempts']=len(records)
        metric['completedIterations']=sum(s.get('iterations',0) for s in records)
        metric['solverStatuses']=[s.get('solverStatus') for s in records]
        metric['acceptedObjective']=next((s['objective'] for s in records if s.get('passed')),None)
        initial_guesses.append(metric)
    save_csv(report/'initial_guess_diagnostics.csv',initial_guesses)
    ablations=[]
    lookup={(r['caseId'],r['method'],r['count'],r['repeat']):r for r in rows}
    for row in rows:
        if row['purpose']!='checker_ablation': continue
        fast=lookup.get((row['caseId'],'diff_direct',8,1))
        if not fast: continue
        record=dict(caseId=row['caseId'],legacyCompleted=row['completed'],fastCompleted=fast['completed'],
                    legacyPassed=row['passed'],fastPassed=fast['passed'],legacyStatus=row['status'],fastStatus=fast['status'],
                    legacySeconds=row.get('onlineSeconds'),fastSeconds=fast.get('onlineSeconds'))
        if numeric(row.get('onlineSeconds')) and numeric(fast.get('onlineSeconds')):
            record['elapsedTimeRatio']=row['onlineSeconds']/max(fast['onlineSeconds'],1e-12)
            if row['passed'] and fast['passed']:
                record['pairedSuccessSpeedup']=record['elapsedTimeRatio']
        ablations.append(record)
    save_csv(report/'checker_ablation.csv',ablations)
    permutation=[]
    for row in rows:
        if row['purpose']!='permutation': continue
        original=lookup.get((row['caseId'].removesuffix('_permuted'),'diff_direct',64,row['repeat']))
        if original:
            permutation.append(dict(caseId=original['caseId'],repeat=row['repeat'],
                 originalCompleted=original['completed'],permutedCompleted=row['completed'],
                 originalPassed=original['passed'],permutedPassed=row['passed'],
                 originalRawPassed=original.get('rawCandidatePasses'),permutedRawPassed=row.get('rawCandidatePasses'),
                 originalSeconds=original.get('onlineSeconds'),permutedSeconds=row.get('onlineSeconds')))
    save_csv(report/'obstacle_order.csv',permutation)
    reliance=[]
    for case,item in cases.items():
        if not item.get('core'): continue
        geo=lookup.get((case,'geo_ihsid',8,1)); warm=lookup.get((case,'diff_ihsid',8,1))
        fast=lookup.get((case,'diff_fast',64,1)); direct=lookup.get((case,'diff_direct',64,1))
        if all([geo,warm,fast,direct]):
            reliance.append(dict(caseId=case,family=item['family'],allCompleted=all(r['completed'] for r in [geo,warm,fast,direct]),
                rawDirect=direct['passed'],finiteRepairPipeline=fast['passed'],
                geometricIhsid=geo['passed'],diffusionIhsid=warm['passed'],
                onlyIhsidOfTestedPipelines=bool(not fast['passed'] and (geo['passed'] or warm['passed']))))
    save_csv(report/'ihsid_reliance.csv',reliance)
    checkpoints = []
    for row in rows:
        events = row.get('anytime') or []
        if isinstance(events, dict):
            events = [events]
        for deadline in [1,2,5,10,20,30]:
            eligible = [e for e in events if e['seconds'] <= deadline]
            chosen = eligible[-1] if eligible else None
            record = {k: row[k] for k in ['jobId','caseId','groupId','method','count','repeat','mode']}
            record.update(budget=deadline, nominalAvailable=bool(chosen),
                          truthPassed=bool(chosen and chosen.get('truthPassed')))
            if chosen:
                record.update(availableAt=chosen['seconds'], quality=chosen.get('quality'))
            checkpoints.append(record)
    save_csv(report/'anytime_quality.csv', checkpoints)
    selection=[]
    for row in main_rows:
        if not row.get('passed') or not row.get('firstTruthPassed'): continue
        first=row.get('firstQuality',{}); best=row.get('quality',{})
        if numeric(first.get('commonCostMean')) and first['commonCostMean']>0:
            selection.append(dict(caseId=row['caseId'],method=row['method'],repeat=row['repeat'],mode=row['mode'],
                firstFeasibleSeconds=row['firstFeasibleSeconds'],totalSeconds=row['onlineSeconds'],
                completedSelectionCostOverFirst=best['commonCostMean']/first['commonCostMean'],
                firstCost=first['commonCostMean'],selectedCost=best['commonCostMean']))
    save_csv(report/'first_vs_selected_quality.csv',selection)
    reference_file=root/'reference_checkpointed_v1'/'audited_results.json'
    if reference_file.exists():
        references=read(reference_file); comparisons=[]
        runtime_file=root/'reference_checkpointed_v1'/'runtime_audit.json'
        reference_incidents=read(runtime_file) if runtime_file.exists() else []
        runtime_lookup={r['jobId']:r for r in reference_incidents}
        save_csv(report/'reference_runtime_incidents.csv',reference_incidents)
        for reference in references:
            record=dict(reference)
            incident=runtime_lookup.get(record['jobId'])
            record['nativeRuntimeFailureObserved']=bool(incident)
            record['completionSavedBeforeRuntimeFailure']=bool(incident and incident.get('resultSavedBeforeProcessExit'))
            for method in ['geo_ihsid','diff_fast']:
                match=next((r for r in core if r['caseId']==reference['caseId'] and r['method']==method),None)
                record[method+'_passed']=bool(match and match.get('passed'))
                if match and match.get('passed') and reference.get('auditPassed'):
                    record[method+'_costOverReference']=match['quality']['commonCostMean']/reference['quality']['commonCostMean']
            comparisons.append(record)
        save_csv(report/'long_reference_comparison.csv',comparisons)
    failures = collections.Counter(r['status'] for r in rows if r['completed'] and not r['passed'])
    complete = sum(bool(r['completed']) for r in rows)
    payload = dict(scheduled=len(rows), completed=complete, unfinished=len(rows)-complete,
                   passed=sum(bool(r['passed']) for r in rows), failures=dict(failures),
                   coreSeed1=summary, speedupCounts=gains)
    (report/'summary.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# 冻结扩散模型实验：记录汇总', '',
        f'已完成 {complete}/{len(rows)} 次计划对照；未完成 {len(rows)-complete} 次。未完成项不计为失败。', '',
        '## 核心 32 个场景：第一固定种子', '',
        '三随机种子的完整结果另见 all_method_strata.csv。成功数保留边界拒绝和失败；耗时列限边界合法的已完成任务，包含求解失败和超时。全部任务耗时另见 CSV。', '',
        '| 方法 | 完成/计划 | 成功 | 边界合法成功/完成合法 | 合法任务中位耗时/s | P90/s |',
        '|---|---:|---:|---:|---:|---:|']
    for s in summary:
        lines.append(f"| {s['method']} | {s['completed']}/{s['scheduled']} | {s['passed']} | "
                     f"{s['boundaryLegalPassed']}/{s['boundaryLegal']} | {fmt(s['legalMedianSeconds'])} | {fmt(s['legalP90Seconds'])} |")
    lines += ['', '## 统计口径', '',
        '- 成功要求在线验收、预算和独立真实模型审计均通过。未知参数组单独报告。',
        '- 参数扫描包含同一基础任务的派生条件，不能视为同等数量的独立场景。',
        '- 配对质量与提速只在双方成功时计算；paired_comparisons.csv 同时保留双方失败状态。',
        '- 参数未知时复用名义规划输出，但沿用原在线耗时；不把缓存审计当作零秒规划。',
        '- 总耗时包含任务边界检查、生成、排序、构建、验收、修正及失败求解。研究审计单列。',
        '- 候选数量实验独立计时，使用公共随机流；不假设候选独立。',
        '- 不同姿态以 SO(3) 旋转角和角速度度量；机械功是负担代理。', '',
        '## 失败状态', '', *[f'- {k}: {v}' for k,v in sorted(failures.items())], '',
        '## 文献边界', '',
        '扩散先验、B 样条和代价引导已见 [MPD](https://arxiv.org/abs/2412.19948)；',
        '多初值并行运动优化已见 [cuRobo](https://arxiv.org/abs/2310.17274)。',
        '本实验需要以匹配基线结果判断 Stewart 约束下的贡献，不能只凭组合命名断言方法创新。']
    (report/'results.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    plots(report, core, counts, paired, main_rows)
    print(json.dumps({k: payload[k] for k in ['scheduled','completed','unfinished','passed','failures']}, ensure_ascii=False))


def plots(report, core, counts, paired, main_rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    plt.rcParams.update({'font.size': 9, 'figure.dpi': 140})
    fig, axes = plt.subplots(1,2,figsize=(12,4))
    summaries = {x['method']: x for x in aggregate(core,['method'])}
    x=np.arange(len(METHODS))
    axes[0].bar(x,[summaries.get(m,{}).get('passed',0) for m in METHODS],label='Audited success')
    axes[0].bar(x,[summaries.get(m,{}).get('unfinished',0) for m in METHODS],
                bottom=[summaries.get(m,{}).get('completed',0) for m in METHODS],color='.8',label='Unfinished')
    axes[0].set_ylabel('Tasks (32 planned per method)'); axes[0].legend()
    for i, method in enumerate(METHODS):
        times=[r['onlineSeconds'] for r in core if r['method']==method and numeric(r.get('onlineSeconds')) and r['onlineSeconds']>0]
        if times: axes[1].scatter(np.full(len(times),i),times,s=10,alpha=.5)
    axes[1].set_yscale('log'); axes[1].set_ylabel('All completed online times (s)')
    for ax in axes: ax.set_xticks(x, METHODS,rotation=35,ha='right'); ax.grid(axis='y',alpha=.2)
    fig.tight_layout(); fig.savefig(report/'core_success_time.png'); plt.close(fig)
    fig, axes=plt.subplots(1,2,figsize=(10,4))
    for family in ['simple','hood']:
        group=[r for r in counts if r['family']==family and r['repeat']==1]
        summary=sorted(aggregate(group,['count']),key=lambda r:r['count']); xs=[r['count'] for r in summary]
        ys=[r['passed']/r['completed'] if r['completed'] else np.nan for r in summary]
        axes[0].plot(xs,ys,'o-',label=family)
        axes[1].plot(xs,[r['medianSeconds'] or np.nan for r in summary],'o-',label=family)
    axes[0].set_ylabel('Audited task success / completed'); axes[1].set_ylabel('Median online time (s)')
    for ax in axes: ax.set_xlabel('Independently timed candidate count'); ax.set_xscale('log',base=2); ax.legend(); ax.grid(alpha=.2)
    fig.suptitle('Core seed 1: 16 scheduled tasks per scene family; boundary rejections retained')
    fig.tight_layout(); fig.savefig(report/'candidate_counts.png'); plt.close(fig)
    fig, ax=plt.subplots(figsize=(7,5))
    for method in METHODS[1:]:
        points=[p for p in paired if p['method']==method and p['repeat']==1 and p.get('mode')=='known' and 'qualityRatio_commonCostMean' in p]
        ax.scatter([p['speedup'] for p in points],[p['qualityRatio_commonCostMean'] for p in points],s=22,label=method,alpha=.65)
    ax.axhline(1,color='k',lw=.6); ax.axvline(1,color='k',lw=.6); ax.set_xscale('log'); ax.set_yscale('log')
    ax.set_xlabel('Paired online speedup vs geometric IHSID'); ax.set_ylabel('Common cost / geometric IHSID cost')
    ax.set_title('Known parameters, seed 1: paired successes\nParameter variants share base tasks')
    ax.legend(fontsize=8); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(report/'quality_speed.png'); plt.close(fig)
    parameters=['payloadMass','com_1','com_2','com_3','inertiaScale','platformMass','speedScale','accelerationScale','forceScale','duration']
    fig,axes=plt.subplots(2,5,figsize=(16,6))
    for parameter,ax in zip(parameters,axes.flat):
        for mode,style in [('known','-'),('unknown','--')]:
            subset=[r for r in main_rows if r['method']=='diff_fast' and r['parameter']==parameter and r['mode']==mode]
            values=sorted(aggregate(subset,['value']),key=lambda r:r['value'])
            ax.plot([r['value'] for r in values],[r['passed']/r['completed'] if r['completed'] else np.nan for r in values],style+'o',label=mode)
        ax.set_title(parameter); ax.set_ylim(-.05,1.05); ax.grid(alpha=.2)
    axes.flat[0].legend(); fig.supylabel('Audited success / completed derived conditions')
    fig.suptitle('Each point uses four base tasks; lines connect tested settings only')
    fig.text(.5,.005,'COM x = -0.025 m includes one runtime failure; force limit x0.5 includes one boundary / true-model rejection.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.03,1,1]); fig.savefig(report/'parameter_generalization.png'); plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4)); budgets=[1,2,5,10,20,30]
    baseline={r['caseId']:r for r in core if r['method']=='geo_ihsid' and r['passed']}
    for method in METHODS:
        selected=[r for r in core if r['method']==method]
        available=[]; costs=[]
        for budget in budgets:
            count=0; ratios=[]
            for row in selected:
                events=row.get('anytime') or []; events=[events] if isinstance(events,dict) else events
                eligible=[e for e in events if e['seconds']<=budget]
                if not eligible or not eligible[-1].get('truthPassed'): continue
                count+=1; reference=baseline.get(row['caseId'])
                if reference: ratios.append(eligible[-1]['quality']['commonCostMean']/reference['quality']['commonCostMean'])
            available.append(count); costs.append(quantile(ratios,.5) or np.nan)
        axes[0].plot(budgets,available,'o-',label=method); axes[1].plot(budgets,costs,'o-',label=method)
    axes[0].set_ylabel('Tasks with audited feasible output at checkpoint'); axes[1].set_ylabel('Median common cost / baseline (paired available tasks)')
    axes[1].axhline(1,color='k',lw=.7)
    for ax in axes: ax.set_xscale('log'); ax.set_xticks(budgets,[str(b) for b in budgets]); ax.set_xlabel('Online checkpoint (s)'); ax.grid(alpha=.2)
    axes[0].legend(fontsize=7); fig.suptitle('Core seed 1: internal result availability (not API return); paired cohort changes with time')
    fig.tight_layout(); fig.savefig(report/'anytime_quality.png'); plt.close(fig)


if __name__ == '__main__':
    main()
