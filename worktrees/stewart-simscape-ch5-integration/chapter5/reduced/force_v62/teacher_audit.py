"""Read-only audit of excluded teachers; preserves every original episode.

Separates acceleration computed from measured force from the numerical speed
derivative, and exports fixed worst-case controller states for MATLAB replay.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
import scipy.io as sio

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'chapter5/artifacts/control_force_v62'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    assert not (OUT / 'teacher_audit.json').exists(), 'Preserve previous audit.'
    excluded = json.loads((ROOT / 'chapter5/artifacts/control_force_v57/teacher_moving_failure_audit.json').read_text('utf-8'))['rows']
    rows, snapshots = [], []
    for source in excluded:
        file = ROOT / 'chapter5/artifacts/teacher' / Path(source['file']).with_suffix('.mat')
        d = sio.loadmat(file, simplify_cells=True, variable_names=['records', 'audit'])
        a, records = d['audit'], d['records']
        t = np.asarray(a['time']); model = np.abs(a['leg_acceleration_model_actual_force'])
        measured = np.abs(a['leg_acceleration_measured'])
        model_peak = np.unravel_index(np.argmax(model), model.shape)
        measured_peak = np.unravel_index(np.argmax(measured), measured.shape)
        ct = np.array([v['time'] for v in records])
        idx = min(max(int(np.searchsorted(ct, t[model_peak[0]], side='right') - 1), 0), len(records)-1)
        rec = records[idx]
        row = dict(file=file.name, sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                   split=source['split'], mode='nonideal-force' if 'nonideal' in file.name else 'ideal-force',
                   model_acceleration_max=float(model[model_peak]), model_peak_time=float(t[model_peak[0]]),
                   measured_acceleration_max=float(measured[measured_peak]), measured_peak_time=float(t[measured_peak[0]]),
                   model_exceeded=bool(model.max()>1.2+1e-6), measured_exceeded=bool(measured.max()>1.2+1e-6),
                   replanned_at_peak=bool(rec['replanned']), qp_feasible_at_peak=bool(rec['diagnostic']['feasible']),
                   qp_fallback_at_peak=bool(rec['diagnostic']['usedFallback']),
                   command_jump_at_peak_N=float(np.max(np.abs(rec['u']-rec['previous_u']))),
                   other_violation=source['other_violation'], arrival=source['arrival'])
        rows.append(row)
        # A bounded set is chosen by task identity, before any proposed repair.
        if file.name.startswith(('train_004_', 'train_008_', 'train_012_', 'validation_004_', 'validation_008_', 'validation_012_')):
            for ci in sorted(set([max(0,idx-1),idx,min(idx+1,len(records)-1)])):
                z = records[ci]
                snapshots.append(dict(source=file.name, split=source['split'], mode=row['mode'],
                                      time=float(z['time']), x=z['x'], previous=z['previous_u'],
                                      u=z['u'], nominal=z['nominal'], goal=z['goal'], observed=z['observed']))
        del d, a, records
    report = dict(rows=rows, total=len(rows),
                  model_exceeded=sum(v['model_exceeded'] for v in rows),
                  measured_exceeded=sum(v['measured_exceeded'] for v in rows),
                  measured_only=sum(v['measured_exceeded'] and not v['model_exceeded'] for v in rows),
                  replanned_at_model_peak=sum(v['replanned_at_peak'] for v in rows),
                  qp_infeasible_at_model_peak=sum(not v['qp_feasible_at_peak'] for v in rows),
                  snapshots=len(snapshots), formal=False)
    (OUT/'teacher_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    sio.savemat(OUT/'teacher_snapshots.mat',dict(snapshots=np.array(snapshots,dtype=object)))
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}))


if __name__ == '__main__':
    main()
