"""Compare interrupted/resumed optimization with an uninterrupted small fixture."""
from pathlib import Path
import shutil
import numpy as np
import scipy.io as sio
import torch
from control_force import OUT, train, write_json, sha

directory=OUT/'resume_check'
assert not directory.exists(),'Preserve previous reproducibility attempts.'
directory.mkdir()
source=OUT/'pilot/repaired.mat'
d=sio.loadmat(source,simplify_cells=True)
split=np.asarray(d['split']).reshape(-1);accepted=np.asarray(d['accepted']).reshape(-1).astype(bool)
idx=np.concatenate([np.flatnonzero(accepted & (split=='train'))[:32],np.flatnonzero(accepted & (split=='validation'))[:4]])
fixture=dict(context=d['context'][:,idx],residual=d['residual'][:,idx],accepted=np.ones((1,len(idx))),
    split=split[idx][None,:],physical_J=d['physical_J'][idx],physical_g=d['physical_g'][:,idx])
for name in ['direct','resumed']:
    path=directory/name;path.mkdir();sio.savemat(path/'repaired.mat',fixture)
train(directory/'direct',5901,5)
train(directory/'resumed',5901,5,pause_after=2)
assert not (directory/'resumed/seed_5901/model.pt').exists()
train(directory/'resumed',5901,5)
a=torch.load(directory/'direct/seed_5901/model.pt',weights_only=False)
b=torch.load(directory/'resumed/seed_5901/model.pt',weights_only=False)
error=max(float((a['state_dict'][k]-b['state_dict'][k]).abs().max()) for k in a['state_dict'])
logs=(directory/'direct/seed_5901/training.csv').read_bytes()==(directory/'resumed/seed_5901/training.csv').read_bytes()
report=dict(accepted=error==0 and logs,model_max_difference=error,logs_exact_equal=logs,fixture_source_sha256=sha(source),
    training_only=True,formal=False)
write_json(directory/'report.json',report)
assert report['accepted']
print(report)
