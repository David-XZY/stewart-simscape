"""V75: train the actual 20-DDIM sampler, using all constraint classes.

Starts from the frozen V74 development model, preserves its labels/task split,
and selects by fixed validation proxy, never by the MATLAB raw pass counts.
"""
from pathlib import Path
import csv,json,time,shutil
import numpy as np
import scipy.io as sio
import torch
import temporal as network
from control_force import sha,write_json
from innovation_v74 import load,CH5

HERE=Path(__file__).resolve().parent
OUT=CH5/'artifacts/control_force_v62/raw_v75'
PRIOR=CH5/'artifacts/control_force_v62/raw_v74/seed_5101/model.pt'
CONFIG=dict(iterations=200,batch_size=32,learning_rate=.0002,seed=5101,ddim_steps=20,
    terminal_weight=3.,physical_weight=2.,imitation_weight=1.,validation_every=10,
    sampler_gradients=True,all_mechanical_classes=True,formal=False)

def sample_grad(model,c,noise,alpha):
    z=noise;steps=np.unique(np.rint(np.linspace(99,0,20)).astype(int))[::-1]
    for i,t in enumerate(steps):
        a=alpha[t];v=model(z,c,torch.full((len(z),),int(t),dtype=torch.long));clean=a.sqrt()*z-(1-a).sqrt()*v;eps=(1-a).sqrt()*z+a.sqrt()*v
        if i+1<len(steps):
            nxt=alpha[steps[i+1]];z=nxt.sqrt()*clean+(1-nxt).sqrt()*eps
    return clean

def train():
    start=time.monotonic();notice=start;geometry=OUT/'geometry.mat'
    while not geometry.exists() or not geometry.with_suffix('.json').exists():
        if time.monotonic()-start>600:raise TimeoutError('V75 geometry')
        if time.monotonic()-notice>30:print('Awaiting all-class MATLAB geometry',flush=True);notice=time.monotonic()
        time.sleep(1)
    directory=OUT/'seed_5101';directory.mkdir(exist_ok=True,parents=True);assert not (directory/'metadata.json').exists(),'Preserve models.'
    identity=dict(config=CONFIG,prior_sha256=sha(PRIOR),geometry_sha256=sha(geometry),dataset_sha256=sha(CH5/'artifacts/control_force_v62/raw_v73/innovation_data.mat'),trainer_sha256=sha(__file__),network_sha256=sha(HERE/'temporal.py'))
    assert not (directory/'identity.json').exists(),'Use a new version to repeat fine tuning.'
    write_json(directory/'identity.json',identity);shutil.copy2(__file__,directory/'trainer_source.py');shutil.copy2(HERE/'temporal.py',directory/'network_source.py')
    torch.set_num_threads(2);torch.manual_seed(5101);np.random.seed(5101);torch.use_deterministic_algorithms(True)
    d,tr,va,y,c,mu,sd,cm,cs=load();z=torch.tensor((y-mu)/sd);ct=torch.tensor((c-cm)/cs);scale=torch.tensor(sd);state=torch.load(PRIOR,weights_only=False);alpha=torch.tensor(state['alpha_bar'])
    g=sio.loadmat(geometry,simplify_cells=True);J=torch.tensor(np.concatenate([g['physical_J'],d['physical_J'][:,11:]],axis=1));g0=torch.tensor(np.concatenate([g['physical_g'].T,d['physical_g'].T[:,11:]],axis=1))
    JT=torch.tensor(d['terminal_J']);e=torch.tensor(d['terminal_e'].T);model=network.Temporal();model.load_state_dict(state['state_dict']);optimizer=torch.optim.AdamW(model.parameters(),lr=CONFIG['learning_rate'])
    generator=torch.Generator().manual_seed(10811);vn=torch.randn(len(va)*4,360,generator=generator);ids=np.repeat(va,4);vc=ct[ids];best=float('inf');best_iteration=0;start=time.perf_counter()
    def loss_for(generated,ix):
        error=(generated-z[ix])*scale;terminal=e[ix]+torch.bmm(JT[ix],error[:,:,None]).squeeze(-1);ratio=terminal.square().sum(1)
        physical=g0[ix]+torch.bmm(J[ix],error[:,:,None]).squeeze(-1);violation=physical.relu().max(1).values.square().mean();imitation=(generated-z[ix]).square().mean()
        loss=CONFIG['terminal_weight']*ratio.mean()+CONFIG['physical_weight']*violation+CONFIG['imitation_weight']*imitation
        return loss,ratio,violation,imitation
    def validate(iteration):
        nonlocal best,best_iteration
        model.eval()
        with torch.no_grad():
            generated=network.sample(model,vc,vn,alpha);loss,ratio,physical,imitation=loss_for(generated,ids);score=float(loss)
            rms=float(((generated-z[ids])*scale).square().mean().sqrt());tm=float(ratio.mean());tp=float(torch.quantile(ratio,.95));phy=float(physical)
        if score<best:best=score;best_iteration=iteration;torch.save(dict(state_dict=model.state_dict(),iteration=iteration,score=score),directory/'best.pt')
        return rms,tm,tp,phy,score
    fields=['iteration','training_loss','innovation_rms_N','terminal_ratio_mean','terminal_ratio_p95','maximum_physical_proxy','validation_score']
    with (directory/'training.csv').open('w',newline='',encoding='utf-8') as handle:
        log=csv.writer(handle);log.writerow(fields);log.writerow([0,None,*validate(0)]);handle.flush()
        for iteration in range(1,CONFIG['iterations']+1):
            model.train();ix=np.random.choice(tr,CONFIG['batch_size'],replace=False);noise=torch.randn(len(ix),360);generated=sample_grad(model,ct[ix],noise,alpha);loss,*_=loss_for(generated,ix)
            optimizer.zero_grad(set_to_none=True);loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),10);optimizer.step()
            if iteration%CONFIG['validation_every']:continue
            stats=validate(iteration);log.writerow([iteration,float(loss.detach()),*stats]);handle.flush()
            torch.save(dict(state_dict=model.state_dict(),optimizer=optimizer.state_dict(),iteration=iteration,torch_rng=torch.get_rng_state(),numpy_rng=np.random.get_state()),directory/'resume.pt')
            if iteration%20==0:print('V75',iteration,'terminal',round(stats[1],3),'physics',round(stats[3],4),'elapsed',round(time.perf_counter()-start,1),flush=True)
    checkpoint=torch.load(directory/'best.pt',weights_only=False);model.load_state_dict(checkpoint['state_dict']);state['state_dict']=checkpoint['state_dict'];state['iteration']=best_iteration;state['score']=best;model.eval();torch.save(state,directory/'model.pt');network.export(model,state,ct,va,directory)
    exported=sio.loadmat(directory/'weights.mat');exported={k:v for k,v in exported.items() if not k.startswith('__')};exported['parameterization']='v_feedback_innovation';sio.savemat(directory/'weights.mat',exported)
    write_json(directory/'metadata.json',dict(**identity,completed=True,development_only=True,train_windows=len(tr),validation_windows=len(va),moving_coverage=False,
        best_iteration=best_iteration,best_score=best,seconds=time.perf_counter()-start,matlab_sha256=sha(directory/'weights.mat'),checkpoint_sha256=sha(directory/'model.pt'),
        selection_rule='fixed_20_DDIM_terminal_and_maximum_all_class_physics_proxy_plus_imitation; nonlinear_audit_separate',inference_postprocessing=False))
    selection=sio.loadmat(CH5/'artifacts/control_force_v57/pilot/selection.mat',simplify_cells=True);indices=np.flatnonzero(selection['split']=='validation');context=(selection['context'].T[indices]-cm)/cs
    generator=torch.Generator().manual_seed(10811);noise=torch.randn(len(indices)*4,360,generator=generator)
    generated=network.sample(model,torch.tensor(np.repeat(context,4,axis=0),dtype=torch.float32),noise,alpha)
    sio.savemat(OUT/'candidates_5101.mat',dict(innovation=(generated.numpy()*sd+mu).T,noise=noise.numpy().T,selection_index=np.repeat(indices+1,4)[None,:],candidate=np.tile(np.arange(1,5),len(indices))[None,:],seed=5101))
    print('Completed V75',directory,flush=True)
if __name__=='__main__':train()
