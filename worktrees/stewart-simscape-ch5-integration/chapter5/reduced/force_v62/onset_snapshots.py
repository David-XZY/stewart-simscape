"""Export pre-violation states without admitting any failed teacher as data."""
from pathlib import Path
import numpy as np
import scipy.io as sio

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'chapter5/artifacts/control_force_v62'
rows=[]
for split in ['train','validation']:
    for task in [4,8,12]:
        for profile in ['ideal','nonideal']:
            name=f'{split}_{task:03d}_{profile}_force_baseline_0.mat'
            d=sio.loadmat(ROOT/'chapter5/artifacts/teacher'/name,simplify_cells=True,variable_names=['records','audit'])
            a=d['audit'];unsafe=np.any(np.abs(a['leg_acceleration_model_actual_force'])>1.2+1e-6,axis=1)
            if not unsafe.any():continue
            first=float(a['time'][np.flatnonzero(unsafe)[0]])
            records=d['records'];ct=np.array([v['time'] for v in records]);index=max(0,int(np.searchsorted(ct,first,side='right')-1))
            for i in sorted(set(max(0,index-j) for j in range(4))):
                z=records[i]
                rows.append(dict(source=name,split=split,mode=f'{profile}-force',time=z['time'],first_violation_time=first,
                                 x=z['x'],previous=z['previous_u'],u=z['u'],nominal=z['nominal'],observed=z['observed'],
                                 clf=z['diagnostic']['clf'],cbf=z['diagnostic']['cbf']))
assert not (OUT/'onset_snapshots.mat').exists()
sio.savemat(OUT/'onset_snapshots.mat',dict(snapshots=np.array(rows,dtype=object)))
print('Exported',len(rows),'pre-violation diagnostic states; excluded from training')
