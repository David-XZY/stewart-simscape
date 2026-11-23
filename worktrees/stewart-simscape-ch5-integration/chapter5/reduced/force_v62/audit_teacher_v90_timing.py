"""Expose iteration counts and historical timing comparability, without normalization."""
from pathlib import Path
import json
import numpy as np
from scipy.io import loadmat

BASE=Path(__file__).resolve().parents[3]/'chapter5/artifacts/control_force_v62'


def cells(value):
    return [value] if isinstance(value,dict) else list(np.asarray(value,dtype=object).reshape(-1))


def collect(path):
    plans=cells(loadmat(path,variable_names=['plans'],simplify_cells=True)['plans'])
    iterations=0;solver_seconds=0.;first=[]
    for pi,p in enumerate(plans):
        for name in ['plan','backup_plan']:
            plan=p.get(name)
            if not isinstance(plan,dict):continue
            for a in cells(plan.get('attempts',[])):
                if not isinstance(a,dict) or not a.get('solver_called',True):continue
                iterations+=int(a.get('iterations',0));solver_seconds+=float(a.get('solver_seconds',0))
                if pi==0 and name=='plan':
                    first.append(dict(iterations=int(a.get('iterations',0)),objective=float(a.get('objective',0)),
                                      solver_seconds=float(a.get('solver_seconds',0))))
    return dict(iterations=iterations,solver_seconds=solver_seconds,initial_four_starts=first)


def main():
    out=BASE/'teacher_repair_v90';summary=json.loads((out/'summary.json').read_text(encoding='utf-8-sig'))
    rows=[]
    for row in summary['rows']:
        name=row['task']+'_'+row['mode'].replace('-','_')+'.mat'
        a=collect(BASE/'teacher_repair_v89'/name);b=collect(out/name)
        pairs=list(zip(a['initial_four_starts'],b['initial_four_starts']))
        same=(len(pairs)==4 and all(x['iterations']==y['iterations'] and abs(x['objective']-y['objective'])<1e-12 for x,y in pairs))
        rows.append(dict(task=row['task'],mode=row['mode'],before=a,after=b,
                         initial_iterations_and_objectives_match=same,
                         initial_solver_time_ratio=sum(y['solver_seconds'] for _,y in pairs)/sum(x['solver_seconds'] for x,_ in pairs)))
    report=dict(completed=summary['completed'],formal=False,rows=rows,
                interpretation='V89 and V90 are historical matched-task wall timings, not interleaved benchmarks. Identical initial iteration/objective comparisons reveal execution-speed variation; raw wall times are retained and not normalized or attributed entirely to scheduling.')
    (out/'timing_comparability.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Timing/iteration comparability audit saved.')


if __name__=='__main__':main()
