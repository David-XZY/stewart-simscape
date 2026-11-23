"""Persistent CPU inference; identical frozen temporal model, explicit noise.

Imported once by MATLAB's out-of-process Python session. No Python process
is started in a control sample; no network service or future events are used.
"""
from pathlib import Path
import hashlib,json,time
import numpy as np
import torch
from temporal import Temporal,sample
_cache={}
def model_at(checkpoint):
    path=Path(checkpoint).resolve();key=str(path)
    if key not in _cache:
        metadata=json.loads((path.parent/'metadata.json').read_text(encoding='utf-8'))
        assert hashlib.sha256(path.read_bytes()).hexdigest()==metadata['checkpoint_sha256'],'Checkpoint hash mismatch.'
        torch.set_num_threads(1)
        state=torch.load(path,weights_only=False,map_location='cpu')
        model=Temporal();model.load_state_dict(state['state_dict']);model.eval()
        _cache[key]=(model,state)
    return _cache[key]
def predict(context,previous,noise,checkpoint):
    model,state=model_at(str(checkpoint));start=time.perf_counter()
    c=np.asarray(context,dtype=np.float64).reshape(219,-1,order='F').T
    z=np.asarray(noise,dtype=np.float64).reshape(360,-1,order='F').T
    if len(c)==1:c=np.repeat(c,len(z),axis=0)
    assert len(c)==len(z) and np.isfinite(c).all() and np.isfinite(z).all()
    norm=(c-state['context_mean'])/state['context_std']
    clean=sample(model,torch.tensor(norm,dtype=torch.float32),torch.tensor(z,dtype=torch.float32),torch.tensor(state['alpha_bar'])).numpy()
    residual=clean.astype(np.float64)*state['force_std']+state['force_mean']
    p=np.asarray(previous,dtype=np.float64).reshape(6)
    force=residual.T+np.tile(p,60)[:,None]
    return (force.ravel(order='F'),clean.T.astype(np.float64).ravel(order='F'),time.perf_counter()-start)
