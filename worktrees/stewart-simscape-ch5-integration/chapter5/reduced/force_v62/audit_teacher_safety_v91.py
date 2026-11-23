"""Recompute physical safety margins from dense raw traces, keeping provenance."""
from pathlib import Path
import csv
import json
import numpy as np
from scipy.io import loadmat

BASE=Path(__file__).resolve().parents[3]/'chapter5/artifacts/control_force_v62'
OUT=BASE/'teacher_repair_v91'


def margins(path):
    d=loadmat(path,variable_names=['audit','r'],simplify_cells=True)
    a=d['audit'];r=d['r'];p=r['physical'];act=p['actuator'];cfg=r['strict']
    length=np.atleast_2d(a['leg_length']);speed=np.atleast_2d(a['leg_speed'])
    acc=np.maximum(np.abs(a['leg_acceleration_model_actual_force']),np.abs(a['leg_acceleration_measured']))
    force=np.atleast_2d(a['actual_force'])
    return dict(simulated_end_seconds=float(np.ravel(a['time'])[-1]),
                minimum_clearance_mm=float(np.min(a['clearance']))*1000,
                clearance_margin_mm=float(np.min(a['clearance'])-r['c']['safe_distance'])*1000,
                stroke_margin_mm=float(min(np.min(length-p['lmin']),np.min(p['lmax']-length)))*1000,
                speed_margin_mps=float(np.min(act['ldotMax']-np.abs(speed))),
                acceleration_margin_mps2=float(np.min(act['lddotMax']-acc)),
                singularity_margin=float(np.min(a['sigma_actual'])-cfg['sigmaSafe']),
                actual_force_margin_N=float(min(np.min(force-cfg['forceMin']),np.min(cfg['forceMax']-force))),
                collision=bool(np.any(a['collision'])))


def main():
    report=json.loads((OUT/'comparison.json').read_text(encoding='utf-8'))
    assert report['completed'] and report['all_tested_runs_safe']
    rows=[]
    for row in report['rows']:
        name=row['task']+'_'+row['mode'].replace('-','_')+'.mat'
        a=margins(BASE/'teacher_repair_v89'/name);b=margins(Path(row['source_full_run']))
        assert not b['collision'] and b['simulated_end_seconds']>=15-1e-8
        for metric in ['speed_margin_mps','acceleration_margin_mps2','singularity_margin','actual_force_margin_N']:
            assert b[metric]>=-1e-6,(row['task'],metric,b[metric])
        rows.append(dict(task=row['task'],mode=row['mode'],validation_kind=row['validation_kind'],before=a,after=b))
    result=dict(completed=True,formal=False,rows=rows,
                boundary='Dense finite development traces, not continuous-time/global safety. All physical bounds retain original tolerances; margins can differ despite all constraints passing.')
    (OUT/'safety_margins.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    flat=[]
    for row in rows:
        for version in ['before','after']:
            flat.append(dict(task=row['task'],mode=row['mode'],comparison=version,**row[version]))
    with (OUT/'safety_margins.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
