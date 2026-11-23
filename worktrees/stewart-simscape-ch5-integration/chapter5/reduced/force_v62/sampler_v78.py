"""Train/distill the actual ten-step sampler; retain all physical loss classes.

The teacher is the frozen V76 twenty-step network. Both use the same task split,
normalizers and draws. Checkpoints are selected by held-out proxy losses, never
by a closed-loop test. Proxy physics remains distinct from nonlinear admission.
"""
from pathlib import Path
import csv, shutil, time
import numpy as np
import scipy.io as sio
import torch
import temporal as network
from control_force import sha, write_json
from innovation_v74 import load, CH5

HERE=Path(__file__).resolve().parent
OUT=CH5/'artifacts/control_force_v62/raw_v78'
PRIOR=CH5/'artifacts/control_force_v62/raw_v76/seed_5101/model.pt'
STEPS=np.arange(99,-1,-11,dtype=np.int64)
CONFIG=dict(iterations=200,batch_size=32,learning_rate=.0002,seed=5101,
    ddim_steps=10,timesteps=STEPS.tolist(),terminal_weight=3.,physical_weight=2.,
    imitation_weight=1.,distillation_weight=1.,validation_every=10,
    sampler_gradients=True,all_mechanical_classes=True,formal=False)

def sample(model,c,noise,alpha,steps=STEPS):
    z=noise
    for i,t in enumerate(steps):
        a=alpha[t];v=model(z,c,torch.full((len(z),),int(t),dtype=torch.long))
        clean=a.sqrt()*z-(1-a).sqrt()*v;eps=(1-a).sqrt()*z+a.sqrt()*v
        if i+1<len(steps):
            nxt=alpha[steps[i+1]];z=nxt.sqrt()*clean+(1-nxt).sqrt()*eps
    return clean

def main():
    directory=OUT/'seed_5101';directory.mkdir(parents=True,exist_ok=True)
    assert not (directory/'identity.json').exists(),'Preserve training attempts in a new version.'
    geometry=CH5/'artifacts/control_force_v62/raw_v75/geometry.mat'
    identity=dict(config=CONFIG,prior_sha256=sha(PRIOR),geometry_sha256=sha(geometry),
        dataset_sha256=sha(CH5/'artifacts/control_force_v62/raw_v73/innovation_data.mat'),
        trainer_sha256=sha(__file__),network_sha256=sha(HERE/'temporal.py'))
    write_json(directory/'identity.json',identity);shutil.copy2(__file__,directory/'trainer_source.py')
    shutil.copy2(HERE/'temporal.py',directory/'network_source.py')
    torch.set_num_threads(2);torch.manual_seed(5101);np.random.seed(5101);torch.use_deterministic_algorithms(True)
    d,tr,va,y,c,mu,sd,cm,cs=load();state=torch.load(PRIOR,weights_only=False)
    for key,value in [('force_mean',mu),('force_std',sd),('context_mean',cm),('context_std',cs)]:
        np.testing.assert_array_equal(state[key],value)
    supported=np.asarray(state['context_supported'],dtype=bool)
    z=torch.tensor((y-mu)/sd);ct=torch.tensor((c-cm)/cs);ct[:,~supported]=0;scale=torch.tensor(sd)
    alpha=torch.tensor(state['alpha_bar']);g=sio.loadmat(geometry,simplify_cells=True)
    J=torch.tensor(np.concatenate([g['physical_J'],d['physical_J'][:,11:]],axis=1))
    g0=torch.tensor(np.concatenate([g['physical_g'].T,d['physical_g'].T[:,11:]],axis=1))
    JT=torch.tensor(d['terminal_J']);e=torch.tensor(d['terminal_e'].T)
    teacher=network.Temporal();teacher.load_state_dict(state['state_dict']);teacher.eval()
    for p in teacher.parameters():p.requires_grad_(False)
    model=network.Temporal();model.load_state_dict(state['state_dict'])
    optimizer=torch.optim.AdamW(model.parameters(),lr=CONFIG['learning_rate'])
    generator=torch.Generator().manual_seed(10811);vn=torch.randn(len(va)*4,360,generator=generator)
    ids=np.repeat(va,4);vc=ct[ids]
    with torch.no_grad():teacher_validation=network.sample(teacher,vc,vn,alpha)
    best=float('inf');best_iteration=0;start=time.perf_counter()
    def loss_for(generated,ix,target):
        error=(generated-z[ix])*scale
        terminal=e[ix]+torch.bmm(JT[ix],error[:,:,None]).squeeze(-1);ratio=terminal.square().sum(1)
        physical=g0[ix]+torch.bmm(J[ix],error[:,:,None]).squeeze(-1)
        violation=physical.relu().max(1).values.square().mean()
        imitation=(generated-z[ix]).square().mean();distillation=(generated-target).square().mean()
        loss=3*ratio.mean()+2*violation+imitation+distillation
        return loss,ratio,violation,imitation,distillation
    def validate(iteration):
        nonlocal best,best_iteration
        model.eval()
        with torch.no_grad():
            generated=sample(model,vc,vn,alpha)
            loss,ratio,physical,imitation,distillation=loss_for(generated,ids,teacher_validation);score=float(loss)
            stats=[float(((generated-z[ids])*scale).square().mean().sqrt()),float(ratio.mean()),
                float(torch.quantile(ratio,.95)),float(physical),float(imitation),float(distillation),score]
        if score<best:
            best=score;best_iteration=iteration
            torch.save(dict(state_dict=model.state_dict(),iteration=iteration,score=score),directory/'best.pt')
        return stats
    fields=['iteration','training_loss','innovation_rms_N','terminal_ratio_mean','terminal_ratio_p95',
        'maximum_physical_proxy','imitation','distillation','validation_score']
    with (directory/'training.csv').open('w',newline='',encoding='utf-8') as handle:
        log=csv.writer(handle);log.writerow(fields);initial=validate(0);log.writerow([0,None,*initial]);handle.flush()
        for iteration in range(1,CONFIG['iterations']+1):
            model.train();ix=np.random.choice(tr,CONFIG['batch_size'],replace=False);noise=torch.randn(len(ix),360)
            with torch.no_grad():target=network.sample(teacher,ct[ix],noise,alpha)
            generated=sample(model,ct[ix],noise,alpha);loss,*_=loss_for(generated,ix,target)
            optimizer.zero_grad(set_to_none=True);loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),10);optimizer.step()
            with torch.no_grad():model.embed[0].weight[:,:219][:,torch.tensor(~supported)]=0
            if iteration%CONFIG['validation_every']:continue
            stats=validate(iteration);log.writerow([iteration,float(loss.detach()),*stats]);handle.flush()
            torch.save(dict(state_dict=model.state_dict(),optimizer=optimizer.state_dict(),iteration=iteration,
                torch_rng=torch.get_rng_state(),numpy_rng=np.random.get_state()),directory/'resume.pt')
            if iteration%20==0:print('V78',iteration,'terminal',round(stats[1],3),'score',round(stats[-1],4),'elapsed',round(time.perf_counter()-start,1),flush=True)
    checkpoint=torch.load(directory/'best.pt',weights_only=False);model.load_state_dict(checkpoint['state_dict']);model.eval()
    state.update(state_dict=checkpoint['state_dict'],iteration=best_iteration,score=best,ddim_steps=10,ddim_timesteps=STEPS,
        parameterization='v_feedback_innovation',context_supported=supported);torch.save(state,directory/'model.pt')
    network.export(model,state,ct,va,directory)
    exported={k:v for k,v in sio.loadmat(directory/'weights.mat').items() if not k.startswith('__')}
    gc=torch.tensor(exported['golden_context'].T);gz=torch.tensor(exported['golden_z'].T)
    with torch.no_grad():golden=sample(model,gc,gz,alpha)
    exported.update(parameterization='v_feedback_innovation',context_supported=supported[:,None],
        ddim_steps=10,ddim_timesteps=STEPS[None,:],golden_sample=golden.numpy().T)
    sio.savemat(directory/'weights.mat',exported)
    selection=sio.loadmat(CH5/'artifacts/control_force_v57/pilot/selection.mat',simplify_cells=True)
    indices=np.flatnonzero(selection['split']=='validation');context=(selection['context'].T[indices]-cm)/cs
    generator=torch.Generator().manual_seed(10811);noise=torch.randn(len(indices)*4,360,generator=generator)
    condition=torch.tensor(np.repeat(context,4,axis=0),dtype=torch.float32)
    with torch.no_grad():
        generated=sample(model,condition,noise,alpha);untrained=sample(teacher,condition,noise,alpha)
        old=network.sample(teacher,condition,noise,alpha)
    sio.savemat(OUT/'candidates_5101.mat',dict(innovation=(generated.numpy()*sd+mu).T,
        untrained_innovation=(untrained.numpy()*sd+mu).T,teacher_innovation=(old.numpy()*sd+mu).T,
        noise=noise.numpy().T,selection_index=np.repeat(indices+1,4)[None,:],seed=5101))
    write_json(directory/'metadata.json',dict(**identity,completed=True,development_only=True,
        train_windows=len(tr),validation_windows=len(va),moving_coverage=False,best_iteration=best_iteration,
        initial_validation_score=initial[-1],best_score=best,seconds=time.perf_counter()-start,
        matlab_sha256=sha(directory/'weights.mat'),checkpoint_sha256=sha(directory/'model.pt'),
        selection_rule='fixed_10_step_validation_proxy_plus_20_step_distillation; nonlinear_audit_separate',
        inference_postprocessing=False,nonlinear_training_loss=False))
    print('Completed ten-step sampler',directory,flush=True)

if __name__=='__main__':main()
