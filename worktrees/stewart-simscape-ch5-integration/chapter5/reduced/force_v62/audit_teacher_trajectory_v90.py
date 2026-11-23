"""Compare the two complete orbit traces to their original V89 counterparts."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.io import loadmat

BASE=Path(__file__).resolve().parents[3]/'chapter5/artifacts/control_force_v62'


def main():
    rows=[]
    for mode in ['ideal_force','nonideal_force']:
        name='train_012_'+mode+'.mat';paths=[BASE/('teacher_repair_'+v)/name for v in ['v89','v90']]
        records=[np.atleast_1d(loadmat(p,variable_names=['records'],simplify_cells=True)['records']) for p in paths]
        assert len(records[0])==len(records[1])==751
        difference={}
        for field in ['time','q','qd','x','u','nominal','goal','observed']:
            difference[field]=float(max(np.max(np.abs(np.asarray(a[field])-np.asarray(b[field]))) for a,b in zip(*records)))
        rows.append(dict(task='train_012',mode=mode,records=751,maximum_differences=difference,
                         state_force_equivalent=all(x<=1e-10 for x in difference.values()),
                         source_sha256=[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]))
    report=dict(completed=True,formal=False,rows=rows,
                scope='All saved control states, inputs, goals and observations of two full 15 s orbit tasks; timing/solver attempts intentionally differ.')
    (BASE/'teacher_repair_v90/trajectory_equivalence_v89_v90.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
