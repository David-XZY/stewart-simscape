"""Development pilot: diffusion over bijective feedback input innovations.

Only admitted training windows fit normalizers/weights. Full fixed validation
draws select a checkpoint by terminal and mechanical proxy; MATLAB then audits
the unchanged nonlinear gate. No optimizer, clipping or tail repair in sampling.
"""
from pathlib import Path
import csv, json, time, shutil
import numpy as np
import scipy.io as sio
import torch
from torch import nn
import temporal as network
from control_force import schedule, reconstruct, sha, write_json

HERE=Path(__file__).resolve().parent
CH5=HERE.parents[1]
OUT=CH5/'artifacts/control_force_v62/raw_v74'
DATA=CH5/'artifacts/control_force_v62/raw_v73/innovation_data.mat'
CONFIG=dict(epochs=300,batch_size=256,seed=5101,learning_rate=.001,ddim_steps=20,
    boundary_noise_fraction=.1,terminal_weight=.2,mechanical_weight=.2,
    clean_weight=.1,physical_alpha_threshold=.5,architecture='temporal_cnn_v64',
    parameterization='v_feedback_innovation',formal=False)

def load():
    d=sio.loadmat(DATA,simplify_cells=True)
    accepted=np.asarray(d['accepted'],dtype=bool);splits=np.asarray(d['split'])
    tr=np.flatnonzero(accepted & (splits=='train'));va=np.flatnonzero(accepted & (splits=='validation'))
    y=np.asarray(d['innovation'].T,dtype=np.float32);c=np.asarray(d['context'].T,dtype=np.float32)
    mu=y[tr].mean(0);sd=np.maximum(y[tr].std(0),1e-3);cm=c[tr].mean(0);cs=np.maximum(c[tr].std(0),1e-3)
    return d,tr,va,y,c,mu,sd,cm,cs

def train():
    wait_start=time.monotonic();notice=wait_start
    while not DATA.exists() or not DATA.with_suffix('.json').exists():
        if time.monotonic()-wait_start>600:raise TimeoutError('MATLAB innovation data export')
        if time.monotonic()-notice>30:print('Awaiting complete MATLAB data export',flush=True);notice=time.monotonic()
        time.sleep(1)
    OUT.mkdir(exist_ok=True,parents=True);directory=OUT/'seed_5101';directory.mkdir(exist_ok=True)
    assert not (directory/'metadata.json').exists(),'Preserve completed model.'
    identity=dict(config=CONFIG,dataset_sha256=sha(DATA),trainer_sha256=sha(__file__),network_sha256=sha(HERE/'temporal.py'))
    if (directory/'identity.json').exists():assert json.loads((directory/'identity.json').read_text())==identity
    else:write_json(directory/'identity.json',identity);shutil.copy2(__file__,directory/'trainer_source.py');shutil.copy2(HERE/'temporal.py',directory/'network_source.py')
    torch.set_num_threads(2);torch.manual_seed(5101);np.random.seed(5101);torch.use_deterministic_algorithms(True)
    d,tr,va,y,c,mu,sd,cm,cs=load();z=torch.tensor((y-mu)/sd);ct=torch.tensor((c-cm)/cs);scale=torch.tensor(sd)
    alpha=torch.tensor(schedule());J=torch.tensor(d['physical_J']);g=torch.tensor(d['physical_g'].T)
    JT=torch.tensor(d['terminal_J']);e=torch.tensor(d['terminal_e'].T)
    model=network.Temporal();optimizer=torch.optim.AdamW(model.parameters(),lr=CONFIG['learning_rate']);best=float('inf');best_epoch=0;start_epoch=1
    if (directory/'resume.pt').exists():
        resume=torch.load(directory/'resume.pt',weights_only=False);model.load_state_dict(resume['state_dict']);optimizer.load_state_dict(resume['optimizer'])
        torch.set_rng_state(resume['torch_rng']);np.random.set_state(resume['numpy_rng']);start_epoch=resume['epoch']+1;best=resume['best_score'];best_epoch=resume['best_epoch']
    generator=torch.Generator().manual_seed(10811);vn=torch.randn(len(va)*4,360,generator=generator);ids=np.repeat(va,4);vc=ct[ids]
    fields=['epoch','loss','innovation_rms_N','terminal_ratio_mean','terminal_ratio_p95','mechanical_proxy','validation_score']
    start=time.perf_counter()
    with (directory/'training.csv').open('a' if start_epoch>1 else 'w',newline='',encoding='utf-8') as handle:
        log=csv.writer(handle)
        if start_epoch==1:log.writerow(fields)
        for epoch in range(start_epoch,CONFIG['epochs']+1):
            model.train();losses=[];order=tr[torch.randperm(len(tr)).numpy()]
            for offset in range(0,len(order),CONFIG['batch_size']):
                ix=order[offset:offset+CONFIG['batch_size']];t=torch.randint(0,100,(len(ix),));t[:max(1,len(ix)//10)]=99
                noise=torch.randn(len(ix),360);a=alpha[t,None];noisy=a.sqrt()*z[ix]+(1-a).sqrt()*noise;target=a.sqrt()*noise-(1-a).sqrt()*z[ix]
                pred=model(noisy,ct[ix],t);clean=reconstruct(noisy,pred,a);error=(clean-z[ix])*scale
                mech=g[ix]+torch.bmm(J[ix],error[:,:,None]).squeeze(-1)
                terminal=e[ix]+torch.bmm(JT[ix],error[:,:,None]).squeeze(-1)
                # Reconstructed samples, including t=99, must obey the proxy. Squared
                # violation is not logarithmically compressed to hide large defects.
                gate=(a[:,0]>=CONFIG['physical_alpha_threshold']).float();gate[t==99]=1
                physics=(mech.relu().square().mean(1)*gate).mean()
                terminal_loss=(terminal.square().sum(1)*gate).mean()
                loss=(pred-target).square().mean()+CONFIG['clean_weight']*(clean-z[ix]).square().mean()+CONFIG['mechanical_weight']*physics+CONFIG['terminal_weight']*terminal_loss
                optimizer.zero_grad(set_to_none=True);loss.backward();nn.utils.clip_grad_norm_(model.parameters(),10);optimizer.step();losses.append(float(loss.detach()))
            if epoch%10 and epoch!=1:continue
            model.eval()
            with torch.no_grad():
                generated=network.sample(model,vc,vn,alpha);error=(generated-z[ids])*scale
                terminal=e[ids]+torch.bmm(JT[ids],error[:,:,None]).squeeze(-1);ratio=terminal.square().sum(1)
                mech=g[ids]+torch.bmm(J[ids],error[:,:,None]).squeeze(-1);mp=mech.relu().square().mean()
                rms=float(error.square().mean().sqrt());tm=float(ratio.mean());tp=float(torch.quantile(ratio,.95));physics=float(mp)
                score=tm+physics+float((generated-z[ids]).square().mean())*.05
            log.writerow([epoch,np.mean(losses),rms,tm,tp,physics,score]);handle.flush()
            if score<best:best=score;best_epoch=epoch;torch.save(dict(state_dict=model.state_dict(),epoch=epoch,score=score),directory/'best.pt')
            torch.save(dict(state_dict=model.state_dict(),optimizer=optimizer.state_dict(),torch_rng=torch.get_rng_state(),numpy_rng=np.random.get_state(),epoch=epoch,best_score=best,best_epoch=best_epoch),directory/'resume.pt')
            if epoch%50==0:print('V74',epoch,'innovation',round(rms,3),'terminal',round(tm,3),'score',round(score,3),flush=True)
    state=torch.load(directory/'best.pt',weights_only=False);model.load_state_dict(state['state_dict']);model.eval()
    state.update(force_mean=mu,force_std=sd,context_mean=cm,context_std=cs,alpha_bar=schedule(),seed=5101,parameterization='v_feedback_innovation')
    torch.save(state,directory/'model.pt');network.export(model,state,ct,va,directory)
    exported=sio.loadmat(directory/'weights.mat');exported={k:v for k,v in exported.items() if not k.startswith('__')};exported['parameterization']='v_feedback_innovation';sio.savemat(directory/'weights.mat',exported)
    write_json(directory/'metadata.json',dict(**identity,completed=True,development_only=True,train_windows=len(tr),validation_windows=len(va),moving_coverage=False,
        best_epoch=best_epoch,best_score=best,seconds=time.perf_counter()-start,matlab_sha256=sha(directory/'weights.mat'),checkpoint_sha256=sha(directory/'model.pt'),
        selection_rule='fixed_20_DDIM_free_terminal_pose_and_mechanical_proxy_plus_latent_error; nonlinear_gate_separately_audited',inference_postprocessing=False))
    selection=sio.loadmat(CH5/'artifacts/control_force_v57/pilot/selection.mat',simplify_cells=True);indices=np.flatnonzero(selection['split']=='validation')
    # Include the one rejected-label validation condition, preserving the old 96 draws.
    context=(selection['context'].T[indices]-cm)/cs;generator=torch.Generator().manual_seed(10811);noise=torch.randn(len(indices)*4,360,generator=generator)
    with torch.no_grad():generated=network.sample(model,torch.tensor(np.repeat(context,4,axis=0),dtype=torch.float32),noise,alpha)
    sio.savemat(OUT/'candidates_5101.mat',dict(innovation=(generated.numpy()*sd+mu).T,noise=noise.numpy().T,
        selection_index=np.repeat(indices+1,4)[None,:],candidate=np.tile(np.arange(1,5),len(indices))[None,:],seed=5101))
    print('Completed V74',directory,flush=True)

if __name__=='__main__':train()
