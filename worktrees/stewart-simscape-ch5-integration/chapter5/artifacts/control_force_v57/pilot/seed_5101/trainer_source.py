"""Isolated V57: invertible force coordinates, bounded v reconstruction, and training.

Flat vectors use (time, leg) ordering, matching MATLAB reshape(vector, 6, H).
The output remains an input-increment decision; no pose reference is predicted.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
import time
from pathlib import Path
import h5py
import numpy as np
import scipy.io as sio
import torch
from torch import nn

HERE = Path(__file__).resolve().parent
CH5 = HERE.parents[1]
OUT = CH5 / 'artifacts' / 'control_force_v57'
CONFIG = json.loads((HERE / 'config.json').read_text('utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.pending')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    os.replace(temp, path)


def force_residual(delta):
    a = np.asarray(delta)
    return np.cumsum(a.reshape(*a.shape[:-1], 60, 6), axis=-2).reshape(a.shape)


def increments(residual):
    a = np.asarray(residual)
    u = a.reshape(*a.shape[:-1], 60, 6)
    return np.diff(u, axis=-2, prepend=np.zeros_like(u[..., :1, :])).reshape(a.shape)


def schedule(steps=100):
    grid = np.arange(steps + 1, dtype=np.float64)
    a = np.cos((grid / steps + .008) / 1.008 * math.pi / 2) ** 2
    a = a[1:] / a[0]
    # Exact zero SNR, including the last inference timestep; no beta clipping.
    a[-1] = 0.
    return a.astype(np.float32)


def reconstruct(noisy, velocity, alpha):
    return alpha.sqrt() * noisy - (1 - alpha).sqrt() * velocity


class Denoiser(nn.Module):
    def __init__(self):
        super().__init__()
        dims = [360 + 219 + 32, *CONFIG['hidden'], 360]
        self.layers = nn.ModuleList(nn.Linear(a, b) for a, b in zip(dims, dims[1:]))
        nn.init.zeros_(self.layers[-1].weight)
        nn.init.zeros_(self.layers[-1].bias)

    def forward(self, z, c, t):
        frequency = torch.exp(-math.log(10000) * torch.arange(16, device=z.device) / 15)
        phase = t[:, None] * frequency[None, :]
        value = torch.cat([z, c, phase.sin(), phase.cos()], dim=-1)
        for layer in self.layers[:-1]:
            value = torch.tanh(layer(value))
        return self.layers[-1](value)  # v, with no epsilon identity skip


@torch.no_grad()
def sample(model, context, noise, alpha):
    steps = np.unique(np.rint(np.linspace(99, 0, CONFIG['ddim_steps'])).astype(int))[::-1]
    z = noise.clone()
    for i, t in enumerate(steps):
        a = alpha[t]
        v = model(z, context, torch.full((len(z),), int(t), dtype=torch.long))
        clean = reconstruct(z, v, a)
        eps = (1-a).sqrt()*z + a.sqrt()*v
        if i+1 < len(steps):
            nxt = alpha[steps[i+1]]
            z = nxt.sqrt()*clean + (1-nxt).sqrt()*eps
    return clean


def algebra():
    rng = np.random.default_rng(5701)
    d = rng.normal(size=(32, 360))
    mu, scale = rng.normal(size=360), rng.uniform(.1, 10, 360)
    z = (force_residual(d)-mu)/scale
    decoded = increments(mu+scale*z)
    ez = rng.normal(size=z.shape)*.03
    eu = force_residual(increments(mu+scale*(z+ez))) - force_residual(d)
    roundtrip = float(np.max(abs(decoded-d)))
    identity = float(np.max(abs(eu-scale*ez)))
    z0 = torch.tensor(z, dtype=torch.float64)
    eps = torch.tensor(rng.normal(size=z.shape), dtype=torch.float64)
    errors = []
    gains = []
    for a in schedule():
        a = torch.tensor(float(a), dtype=torch.float64)
        noisy = a.sqrt()*z0+(1-a).sqrt()*eps
        v = a.sqrt()*eps-(1-a).sqrt()*z0
        errors.append(float((reconstruct(noisy, v, a)-z0).abs().max()))
        dv = torch.tensor(ez, dtype=torch.float64)
        gains.append(float((reconstruct(noisy, v+dv, a)-z0).norm()/dv.norm()))
    assert roundtrip < 1e-12 and identity < 1e-12 and max(errors) < 1e-12
    assert max(gains) <= 1+1e-12 and schedule()[-1] == 0
    OUT.mkdir(parents=True, exist_ok=True)
    result = dict(roundtrip_max=roundtrip, force_error_identity_max=identity,
                  v_recovery_max=max(errors), max_v_perturbation_gain=max(gains),
                  terminal_alpha=float(schedule()[-1]), accepted=True, source_sha256=sha(__file__))
    write_json(OUT/'algebra.json', result)
    print(json.dumps(result))


def select(count):
    out = OUT/'pilot'
    out.mkdir(parents=True, exist_ok=True)
    assert not (out/'selection.mat').exists(), 'Preserve existing selections.'
    rows, contexts, deltas, splits, indices, sources, tasks = [], [], [], [], [], [], []
    old = sio.loadmat(CH5/'artifacts/reduced_probe/teacher_candidate_audit_v39/audit.mat', simplify_cells=True)['report']['rows']
    for split in ['train', 'validation']:
        teacher = CH5/'artifacts/teacher'
        with h5py.File(teacher/f'{split}_windows.h5') as f:
            c, d = np.array(f['context']), np.array(f['delta'])
        provenance = sio.loadmat(teacher/f'{split}_windows_provenance.mat', simplify_cells=True)['provenance']
        # Selection is fixed before any V57 controller outcome is available.
        chosen = np.unique(np.rint(np.linspace(0, len(d)-1, count)).astype(int)) if split == 'train' else np.array([r['window_index']-1 for r in old])
        for index in chosen:
            source = provenance['sources'][index]
            contexts.append(c[index]);deltas.append(d[index]);splits.append(split)
            indices.append(int(index+1));sources.append(source['run']);tasks.append(source['task'])
    assert not set(t for t,s in zip(tasks,splits) if s=='train') & set(t for t,s in zip(tasks,splits) if s=='validation')
    sio.savemat(out/'selection.mat', dict(context=np.array(contexts).T, delta=np.array(deltas).T,
        split=np.array(splits,dtype=object)[None,:], window_index=np.array(indices),
        source=np.array(sources,dtype=object)[None,:], task=np.array(tasks,dtype=object)[None,:]))
    write_json(out/'selection.json', dict(train_requested=count, validation_requested=len(old),
        selection_sha256=sha(out/'selection.mat'), protocol='uniform_index_train_fixed_v39_validation_before_outcomes',
        source_h5_sha256={s:sha(CH5/f'artifacts/teacher/{s}_windows.h5') for s in ['train','validation']},
        formal=False))
    print('Selected',len(contexts),'windows')


def repaired_data(directory):
    data = sio.loadmat(directory/'repaired.mat', simplify_cells=True)
    accepted = np.asarray(data['accepted'],dtype=bool).reshape(-1)
    split = np.asarray(data['split']).reshape(-1)
    c, u = data['context'].T.astype(np.float32), data['residual'].T.astype(np.float32)
    train = accepted & (split=='train'); val = accepted & (split=='validation')
    assert train.any() and val.any()
    assert np.isfinite(u[accepted]).all() and np.isfinite(c[accepted]).all()
    return data,c,u,train,val


def train(directory, seed, epochs):
    modeldir = directory/f'seed_{seed}'
    modeldir.mkdir(parents=True,exist_ok=True)
    assert not (modeldir/'model.pt').exists(), 'Completed model is immutable; use a new attempt.'
    torch.set_num_threads(2);torch.manual_seed(seed);np.random.seed(seed)
    torch.use_deterministic_algorithms(True)
    data,c,u,trainmask,valmask = repaired_data(directory)
    mu,sd = u[trainmask].mean(0),np.maximum(u[trainmask].std(0),1e-3)
    cm,cs = c[trainmask].mean(0),np.maximum(c[trainmask].std(0),1e-3)
    z=torch.tensor((u-mu)/sd);ct=torch.tensor((c-cm)/cs)
    tr=np.flatnonzero(trainmask);va=np.flatnonzero(valmask)
    alpha=torch.tensor(schedule());model=Denoiser();optimizer=torch.optim.AdamW(model.parameters(),lr=CONFIG['learning_rate'])
    fixed=torch.Generator().manual_seed(5702)
    vt=torch.randint(0,100,(len(va),),generator=fixed);ve=torch.randn((len(va),360),generator=fixed)
    vn=alpha[vt,None].sqrt()*z[va]+(1-alpha[vt,None]).sqrt()*ve
    vv=alpha[vt,None].sqrt()*ve-(1-alpha[vt,None]).sqrt()*z[va]
    mean,scale=torch.tensor(mu),torch.tensor(sd)
    # Per-window exact dynamics sensitivities exported by MATLAB at the admitted label.
    physJ=torch.tensor(data['physical_J'],dtype=torch.float32)
    physg=torch.tensor(data['physical_g'].T,dtype=torch.float32)
    best=float('inf');bestepoch=0;start=time.perf_counter()
    with (modeldir/'training.csv').open('w',newline='',encoding='utf-8') as f:
        log=csv.writer(f);log.writerow(['epoch','loss','validation_v_mse','validation_force_rms_N','validation_score'])
        for epoch in range(1,epochs+1):
            losses=[];model.train()
            order=tr[torch.randperm(len(tr)).numpy()]
            for offset in range(0,len(order),CONFIG['batch_size']):
                idx=order[offset:offset+CONFIG['batch_size']]
                t=torch.randint(0,100,(len(idx),));eps=torch.randn((len(idx),360));a=alpha[t,None]
                noisy=a.sqrt()*z[idx]+(1-a).sqrt()*eps
                target=a.sqrt()*eps-(1-a).sqrt()*z[idx]
                pred=model(noisy,ct[idx],t);clean=reconstruct(noisy,pred,a)
                force_error=(clean-z[idx])*scale
                delta_error=torch.diff(force_error.reshape(-1,60,6),dim=1,prepend=torch.zeros(len(idx),1,6))
                low=(a[:,0]>=CONFIG['physical_alpha_threshold']).float()
                # Local physics is an auxiliary low-noise loss, never a feasibility certificate.
                g=physg[idx]+torch.bmm(physJ[idx],force_error[:,:,None]).squeeze(-1)
                physics=(g.relu().square().mean(1)*low).mean()
                loss=(pred-target).square().mean()+CONFIG['force_loss_weight']*(force_error/20).square().mean()+CONFIG['increment_loss_weight']*(delta_error/20).square().mean()+CONFIG['physical_loss_weight']*physics
                optimizer.zero_grad(set_to_none=True);loss.backward();nn.utils.clip_grad_norm_(model.parameters(),10);optimizer.step();losses.append(float(loss))
            model.eval()
            with torch.no_grad():
                pred=model(vn,ct[va],vt);vl=float((pred-vv).square().mean())
                vf=float((((reconstruct(vn,pred,alpha[vt,None])-z[va])*scale).square().mean()).sqrt())
                score=vl+.1*(vf/20)**2
            log.writerow([epoch,np.mean(losses),vl,vf,score]);f.flush()
            if score<best:
                best=score;bestepoch=epoch
                torch.save(dict(state_dict=model.state_dict(),seed=seed,epoch=epoch,score=score,parameterization='v_force_residual'),modeldir/'best.pt')
            # Durable optimizer/RNG state permits continuation after an interrupted pilot.
            torch.save(dict(state_dict=model.state_dict(),optimizer=optimizer.state_dict(),epoch=epoch,
                torch_rng=torch.get_rng_state(),numpy_rng=np.random.get_state()),modeldir/'resume.pt')
            if epoch%20==0: print(f'epoch {epoch} val_v={vl:.5f} force={vf:.3f} N',flush=True)
    chosen=torch.load(modeldir/'best.pt',weights_only=False);model.load_state_dict(chosen['state_dict'])
    torch.save({**chosen,'force_mean':mu,'force_std':sd,'context_mean':cm,'context_std':cs,'alpha_bar':schedule()},modeldir/'model.pt')
    export=dict(force_mean=mu[:,None],force_std=sd[:,None],context_mean=cm[:,None],context_std=cs[:,None],alpha_bar=schedule()[:,None],
        parameterization='v_force_residual',residual_noise_skip=0,revision=CONFIG['revision'])
    for i,l in enumerate(model.layers,1): export[f'W{i}']=l.weight.detach().numpy();export[f'b{i}']=l.bias.detach().numpy()[:,None]
    generator=torch.Generator().manual_seed(5703);goldz=torch.randn(4,360,generator=generator);goldc=ct[va[:4]];goldt=torch.tensor([0,17,63,99])
    with torch.no_grad():
        goldv=model(goldz,goldc,goldt);goldclean=reconstruct(goldz,goldv,alpha[goldt,None])
        sampled=sample(model,goldc,goldz,alpha)
    export.update(golden_z=goldz.numpy().T,golden_context=goldc.numpy().T,golden_t=goldt.numpy()[None,:],golden_v=goldv.numpy().T,
        golden_clean=goldclean.numpy().T,golden_sample=sampled.numpy().T)
    sio.savemat(modeldir/'weights.mat',export)
    write_json(modeldir/'metadata.json',dict(seed=seed,completed=True,development_only=True,epochs=epochs,best_epoch=bestepoch,
        validation_score=best,train_windows=len(tr),validation_windows=len(va),seconds=time.perf_counter()-start,
        dataset_sha256=sha(directory/'repaired.mat'),trainer_sha256=sha(__file__),config_sha256=sha(HERE/'config.json'),matlab_sha256=sha(modeldir/'weights.mat')))
    print('Completed',modeldir, 'best epoch',bestepoch,flush=True)


def candidates(directory,seed):
    state=torch.load(directory/f'seed_{seed}/model.pt',weights_only=False)
    model=Denoiser();model.load_state_dict(state['state_dict']);model.eval();torch.set_num_threads(2)
    selection=sio.loadmat(directory/'selection.mat',simplify_cells=True)
    idx=np.flatnonzero(np.asarray(selection['split']).reshape(-1)=='validation')
    c=selection['context'].T[idx].astype(np.float32)
    c=(c-state['context_mean'])/state['context_std']
    generator=torch.Generator().manual_seed(5710+seed)
    context=torch.tensor(np.repeat(c,4,axis=0));noise=torch.randn(len(context),360,generator=generator)
    clean=sample(model,context,noise,torch.tensor(state['alpha_bar'])).numpy()
    residual=clean*state['force_std']+state['force_mean']
    sio.savemat(directory/f'candidates_{seed}.mat',dict(residual=residual.T,noise=noise.numpy().T,selection_index=np.repeat(idx+1,4)[None,:],candidate=np.tile(np.arange(1,5),len(idx))[None,:],seed=seed))
    print('Generated',len(residual),'candidates')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['algebra','select','train','candidates'])
    parser.add_argument('--count',type=int,default=1024);parser.add_argument('--seed',type=int,default=5101)
    parser.add_argument('--epochs',type=int,default=CONFIG['epochs']);parser.add_argument('--directory',type=Path,default=OUT/'pilot')
    args=parser.parse_args()
    if args.action=='algebra': algebra()
    elif args.action=='select': select(args.count)
    elif args.action=='train': train(args.directory,args.seed,args.epochs)
    else: candidates(args.directory,args.seed)
