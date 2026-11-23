"""Task-equally weighted first-round models, with MATLAB-compatible export."""
from __future__ import annotations
import copy
import hashlib
from pathlib import Path
import time
from run_boundary import read, write


def train(dataset, output, method, seed, device='cuda', epochs=300):
    import numpy as np
    import torch
    from torch import nn
    started=time.perf_counter(); data=read(dataset); output=Path(output)
    if method not in ['M3_N','M4'] or epochs<1:
        raise ValueError('Unsupported first-round training configuration')
    x=np.asarray(data['x'],dtype=float); y=np.asarray(data['y'],dtype=float)
    split=np.asarray(data['split']); groups=np.asarray(data['groups']); tasks=np.asarray(data['taskIds'])
    if set(split)-{'train','validation'}:
        raise ValueError('Training export must not contain formal test labels')
    if x.ndim!=2 or x.shape[1]!=93 or y.ndim!=2 or y.shape[0]!=len(x) or y.shape[1]%6:
        raise ValueError('Invalid conditional trajectory data shape')
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError('Nonfinite labels')
    if set(groups[split=='train']) & set(groups[split=='validation']):
        raise ValueError('Group leakage')
    ti=np.flatnonzero(split=='train'); vi=np.flatnonzero(split=='validation')
    if not len(ti) or not len(vi): raise ValueError('Train and validation labels required')
    def weights(ids):
        unique,counts=np.unique(tasks[ids],return_counts=True); count=dict(zip(unique,counts))
        w=np.array([1/count[t] for t in tasks[ids]],dtype=float); return w/w.sum()
    tw=weights(ti); vw=weights(vi)
    xm=np.average(x[ti],axis=0,weights=tw); ym=np.average(y[ti],axis=0,weights=tw)
    xs=np.maximum(np.sqrt(np.average((x[ti]-xm)**2,axis=0,weights=tw)),1e-6)
    ys=np.maximum(np.sqrt(np.average((y[ti]-ym)**2,axis=0,weights=tw)),1e-5)
    torch.manual_seed(seed); dim=y.shape[1]; input_dim=93+(dim+2 if method=='M4' else 0)
    model=nn.Sequential(nn.Linear(input_dim,256),nn.Tanh(),nn.Linear(256,256),nn.Tanh(),nn.Linear(256,256),nn.Tanh(),nn.Linear(256,dim)).to(device)
    optimizer=torch.optim.AdamW(model.parameters(),lr=1e-3)
    X=torch.tensor((x-xm)/xs,dtype=torch.float32,device=device); Y=torch.tensor((y-ym)/ys,dtype=torch.float32,device=device)
    train_ids=torch.tensor(ti,device=device); val_ids=torch.tensor(vi,device=device)
    train_weights=torch.tensor(tw,dtype=torch.float32,device=device); val_weights=torch.tensor(vw,dtype=torch.float32,device=device)
    beta=torch.linspace(1e-4,.02,100,device=device); abar=torch.cumprod(1-beta,0)
    def loss(ids,fixed=False):
        if method=='M3_N': return (model(X[ids])-Y[ids]).square().mean(1)
        generator=torch.Generator(device=device).manual_seed(seed+11) if fixed else None
        t=torch.randint(100,(len(ids),),device=device,generator=generator)
        noise=torch.randn((len(ids),dim),device=device,generator=generator)
        z=abar[t,None].sqrt()*Y[ids]+(1-abar[t,None]).sqrt()*noise; phase=t[:,None]/99*torch.pi
        return (model(torch.cat([X[ids],z,phase.sin(),phase.cos()],1))-noise).square().mean(1)
    output.mkdir(parents=True,exist_ok=True)
    if (output/'model.json').exists(): raise ValueError('Model output exists; use a new version instead of overwriting')
    best=float('inf'); state=None; stale=0; log=[]
    for epoch in range(epochs):
        model.train()
        sampled=train_ids[torch.multinomial(train_weights,len(ti),replacement=True)]
        for batch in sampled.split(128):
            optimizer.zero_grad(); value=loss(batch).mean(); value.backward(); nn.utils.clip_grad_norm_(model.parameters(),10); optimizer.step()
        model.eval()
        with torch.no_grad(): validation=float((loss(val_ids,True)*val_weights).sum())
        if validation<best: best=validation; state=copy.deepcopy(model.state_dict()); stale=0
        else: stale+=1
        log.append(dict(epoch=epoch,taskWeightedValidationLoss=validation))
        if stale>=30: break
    model.load_state_dict(state); model.eval()
    if str(device).startswith('cuda'): torch.cuda.synchronize()
    layers=[dict(W=l.weight.detach().cpu().tolist(),b=l.bias.detach().cpu().tolist()) for l in model if isinstance(l,nn.Linear)]
    asset=dict(kind='boundary-'+method,method=method,status='trained',seed=seed,datasetHash=hashlib.sha256(Path(dataset).read_bytes()).hexdigest(),
               inputMean=xm.tolist(),inputScale=xs.tolist(),outputMean=ym.tolist(),outputScale=ys.tolist(),layers=layers,
               beta=beta.cpu().tolist() if method=='M4' else [],controlCount=dim//6+6,taskEqualWeight=True,
               epochsCompleted=len(log),bestValidationLoss=best,device=str(device),torchVersion=torch.__version__,
               trainingWallSeconds=time.perf_counter()-started,trainingConfig=dict(hiddenWidths=[256]*3,optimizer='AdamW',learningRate=.001,batchSize=128,patience=30))
    torch.save(dict(model=state,method=method,datasetHash=asset['datasetHash'],seed=seed),output/'checkpoint.pt')
    write(output/'model.json',asset); write(output/'training.json',log)
    probe=torch.linspace(-.5,.5,input_dim,device=device).reshape(1,-1)
    with torch.no_grad(): expected=model(probe).cpu().tolist()
    write(output/'network_probe.json',dict(input=probe.cpu().tolist(),output=expected))
    return asset
