"""Audit complete repaired teachers without materializing or training a dataset."""
from pathlib import Path
import hashlib,json
import numpy as np
import scipy.io as sio
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'chapter5/artifacts/control_force_v62'

def main():
    rows=[]
    for version in ['teacher_full_v67','teacher_full_v72']:
        directory=OUT/version
        for file in sorted(directory.glob('train_004_*.mat')):
            rowfile=file.with_suffix('.json')
            if not rowfile.exists():continue
            source=json.loads(rowfile.read_text(encoding='utf-8-sig'));m=source['metrics']
            data=sio.loadmat(file,simplify_cells=True,variable_names=['records','r'])
            records=data['records'];t=np.array([r['time'] for r in records]);u=np.array([r['u'] for r in records]);previous=np.array([r['previous_u'] for r in records])
            state=np.array([r['x'] for r in records]);goal=np.array([r['goal'] for r in records]);observed=np.array([r['observed'] for r in records])
            previous_error=float(np.max(np.abs(u[:-1]-previous[1:])))
            finite=bool(all(np.isfinite(a).all() for a in [t,u,previous,state,goal,observed]))
            uniform=bool(len(t)==751 and abs(t[0])<1e-10 and abs(t[-1]-15)<1e-8 and np.max(np.abs(np.diff(t)-.02))<1e-8)
            dimensions=bool(state.shape[1]==(30 if source['mode']=='nonideal-force' else 12) and u.shape==(751,6) and goal.shape==(751,6) and observed.shape==(751,22,8))
            initial_error=float(np.max(np.abs(state[0,:12]-np.r_[data['r']['task']['q0'],data['r']['task']['qd0']])))
            eligible=bool(m['safe_success'] and m['all_constraints_valid'] and m['simulated_end_seconds']>=15-1e-8 and not source['simulation_error'] and finite and uniform and dimensions and previous_error<1e-10 and initial_error<1e-7)
            rows.append({'version':version,'task':source['task'],'mode':source['mode'],'split':'train','eligible':eligible,
                'finite':finite,'uniform_control_grid':uniform,'dimensions_valid':dimensions,'samples':len(t),
                'previous_command_consistency_N':previous_error,'force_increment_rms_N':float(np.sqrt(np.mean((u-previous)**2))),
                'initial_state_consistency':initial_error,
                'source_file':str(file.relative_to(ROOT)),'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),
                'used_for_training':False,'formal':False})
    report={'formal':False,'materialized':False,'trained':False,'rows':rows,
        'boundary':'Versioned development teachers; duplicate task/actuator conditions across versions are not independent tasks. Only the final selected baseline version can supply a new dataset. Context uses observed obstacles; future truth is excluded.'}
    (OUT/'teacher_admission_v72.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Teacher integrity eligible:',sum(r['eligible'] for r in rows),'/',len(rows),'(not trained)')

if __name__=='__main__':main()
