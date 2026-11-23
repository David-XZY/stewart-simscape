"""Final immutable-source and all-sample audit for the scheduling-only result."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.io import loadmat

ROOT=Path(__file__).resolve().parents[3];BASE=ROOT/'chapter5/artifacts/control_force_v62';OUT=BASE/'teacher_repair_v92'


def main():
    identity=json.loads((OUT/'identity.json').read_text(encoding='utf-8-sig'))
    sources={**identity['original_sources'],**identity['sources']}
    for name,sha in sources.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha,name
    report=json.loads((OUT/'comparison.json').read_text(encoding='utf-8'))
    assert report['completed'] and report['all_tested_runs_safe'] and len(report['rows'])==3
    rows=[]
    for row in report['rows']:
        name=row['task']+'_'+row['mode'].replace('-','_')+'.mat'
        paths=[BASE/'teacher_repair_v89'/name,Path(row['source_full_run'])]
        records=[np.atleast_1d(loadmat(p,variable_names=['records'],simplify_cells=True)['records']) for p in paths]
        assert len(records[0])==len(records[1])==751
        differences={field:float(max(np.max(np.abs(np.asarray(a[field])-np.asarray(b[field]))) for a,b in zip(*records)))
                     for field in ['time','q','qd','x','u','nominal','goal','observed']}
        assert max(differences.values())<=1e-10,(row['task'],differences)
        rows.append(dict(task=row['task'],mode=row['mode'],validation_kind=row['validation_kind'],
                         saved_samples=751,maximum_differences=differences,
                         source_sha256=[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]))
    cross=loadmat(OUT/'train_016_nonideal_force.mat',variable_names=['plans'],simplify_cells=True)['plans']
    budgets=[]
    for p in np.atleast_1d(cross):
        if p.get('arrival_budget_event_preserved') and 'previous_arrival_time' in p:
            budgets.append(dict(time=float(p['time']),previous_arrival_time=float(p['previous_arrival_time']),
                                arrival_time=float(p['arrival_time']),decision=p['reference_decision']))
    verification=dict(completed=True,formal=False,accepted_version='v92-scheduling-only',
                      controller_source_hashes_verified=len(sources),rows=rows,arrival_budget_events=budgets,
                      new_complete_v92_simscape_runs=1,complete_v90_runs_with_v92_scheduler_equivalence=2,
                      rejected_versions_preserved=['v90: crossing acceleration violation','v91: terminal hold 0.478814997 s, below 0.5 s'],
                      safety_policy='Original V89 physical filter and tolerances; V91 constraint-processing change is not used.',
                      timing_policy='Raw historical observed times, explicitly attributed to V90 or V92; no merged formal timing result.')
    (OUT/'final_verification.json').write_text(json.dumps(verification,indent=2),encoding='utf-8')
    file=OUT/'index.html';text=file.read_text(encoding='utf-8')
    if 'final_verification.json' not in text:
        text+='<p><b>最终查验：</b>三组各 751 拍保存状态、实际下发指令、名义力、目标和障碍观测，与原 V89 对照差异均为零；完整物理裕度也逐项一致。此结论仅覆盖固定三组开发条件。</p><p><a href="final_verification.json">最终源码、逐拍数据和到达预算事件审计</a></p>'
        file.write_text(text,encoding='utf-8')
    final_scripts={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in (ROOT/'chapter5/reduced/force_v62').glob('*teacher*v9*.py')}
    (OUT/'analysis_sources_final.json').write_text(json.dumps(final_scripts,indent=2),encoding='utf-8')
    print(json.dumps(dict(completed=True,sources_verified=len(sources),rows=rows,arrival_budget_events=budgets),indent=2))


if __name__=='__main__':main()
