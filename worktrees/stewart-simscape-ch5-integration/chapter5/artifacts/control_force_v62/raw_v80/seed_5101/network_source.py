"""Small action-sequence CNN, v prediction, fixed full-DDIM validation.

This pilot uses the immutable admitted V57 data. Missing moving teachers keep
it development-only; MATLAB inference never starts Python in a control step.
"""
from pathlib import Path
import argparse,csv,hashlib,json,math,sys,time
import numpy as np
import scipy.io as sio
import torch
from torch import nn

HERE=Path(__file__).resolve().parent
CH5=HERE.parents[1]
sys.path.insert(0,str(HERE.parent/'force_v57'))
from control_force import schedule,reconstruct,sha,write_json,repaired_data,coverage
DATA=CH5/'artifacts/control_force_v57/pilot'
OUT=CH5/'artifacts/control_force_v62/temporal_v64'
CONFIG=dict(width=32,dilations=[1,2,4,8,16],condition_width=64,epochs=300,batch_size=256,
            learning_rate=.001,force_weight=.1,increment_weight=.1,increment_scale_N=2.,
            physics_weight=.05,physical_alpha_threshold=.8,ddim_steps=20,validation_every=10,
            parameterization='v_force_residual',architecture='temporal_cnn_v64',formal=False)


class Block(nn.Module):
    def __init__(self,dilation):
        super().__init__();w=CONFIG['width']
        self.c1=nn.Conv1d(w,w,3,padding=dilation,dilation=dilation)
        self.c2=nn.Conv1d(w,w,3,padding=dilation,dilation=dilation)
        self.film=nn.Linear(CONFIG['condition_width'],2*w)
    def forward(self,value,condition):
        scale,bias=self.film(condition).chunk(2,dim=1)
        residual=torch.tanh(self.c1(value)*(1+.1*torch.tanh(scale[:,:,None]))+bias[:,:,None])
        return value+.1*torch.tanh(self.c2(residual))


class Temporal(nn.Module):
    def __init__(self):
        super().__init__();w=CONFIG['width'];e=CONFIG['condition_width']
        self.input=nn.Conv1d(9,w,3,padding=1)
        self.embed=nn.Sequential(nn.Linear(251,e),nn.Tanh(),nn.Linear(e,e),nn.Tanh())
        self.blocks=nn.ModuleList(Block(d) for d in CONFIG['dilations'])
        self.output=nn.Conv1d(w,6,1);nn.init.zeros_(self.output.weight);nn.init.zeros_(self.output.bias)
    def forward(self,z,c,t):
        frequency=torch.exp(-math.log(10000)*torch.arange(16,device=z.device)/15)
        phase=t[:,None]*frequency[None,:];condition=self.embed(torch.cat([c,phase.sin(),phase.cos()],1))
        j=torch.linspace(0,1,60,device=z.device)
        position=torch.stack([j,torch.sin(math.pi*j),torch.cos(math.pi*j)])[None].expand(len(z),-1,-1)
        value=torch.tanh(self.input(torch.cat([z.reshape(-1,60,6).transpose(1,2),position],1)))
        for block in self.blocks:value=block(value,condition)
        return self.output(value).transpose(1,2).reshape(-1,360)


@torch.no_grad()
def sample(model,c,noise,alpha):
    z=noise.clone();steps=np.unique(np.rint(np.linspace(99,0,20)).astype(int))[::-1]
    for i,t in enumerate(steps):
        a=alpha[t];v=model(z,c,torch.full((len(z),),int(t),dtype=torch.long))
        clean=reconstruct(z,v,a);eps=(1-a).sqrt()*z+a.sqrt()*v
        if i+1<len(steps):
            nxt=alpha[steps[i+1]];z=nxt.sqrt()*clean+(1-nxt).sqrt()*eps
    return clean


def export(model,state,c,va,directory):
    result={k:np.asarray(state[k])[:,None] for k in ['force_mean','force_std','context_mean','context_std','alpha_bar']}
    result.update(parameterization='v_force_residual',architecture='temporal_cnn_v64',residual_noise_skip=0,dilations=np.array(CONFIG['dilations']))
    def convolution(name,layer):
        a=layer.weight.detach().numpy();result[name+'W']=a.transpose(0,2,1).reshape(a.shape[0],-1);result[name+'b']=layer.bias.detach().numpy()[:,None]
    convolution('input',model.input);convolution('output',model.output)
    for i,layer in [(1,model.embed[0]),(2,model.embed[2])]:result[f'embedW{i}']=layer.weight.detach().numpy();result[f'embedb{i}']=layer.bias.detach().numpy()[:,None]
    for i,block in enumerate(model.blocks,1):
        convolution(f'block{i}c1',block.c1);convolution(f'block{i}c2',block.c2)
        result[f'filmW{i}']=block.film.weight.detach().numpy();result[f'filmb{i}']=block.film.bias.detach().numpy()[:,None]
    generator=torch.Generator().manual_seed(5703);z=torch.randn(4,360,generator=generator);gc=c[va[:4]];t=torch.tensor([0,17,63,99]);alpha=torch.tensor(state['alpha_bar'])
    with torch.no_grad():v=model(z,gc,t);clean=reconstruct(z,v,alpha[t,None]);generated=sample(model,gc,z,alpha)
    result.update(golden_z=z.numpy().T,golden_context=gc.numpy().T,golden_t=t.numpy()[None,:],golden_v=v.numpy().T,golden_clean=clean.numpy().T,golden_sample=generated.numpy().T)
    sio.savemat(directory/'weights.mat',result)


def train(seed,development_only):
    cov=coverage(DATA,not development_only)
    geometry=CH5/'artifacts/control_force_v62/training_geometry.mat'
    assert geometry.exists(),'Require terminal curvature / collision geometry export.'
    OUT.mkdir(parents=True,exist_ok=True);directory=OUT/f'seed_{seed}';directory.mkdir(exist_ok=True)
    assert not (directory/'metadata.json').exists(),'Preserve completed model.'
    identity=dict(seed=seed,dataset_sha256=sha(DATA/'repaired.mat'),geometry_sha256=sha(geometry),trainer_sha256=sha(__file__),config=CONFIG)
    if (directory/'identity.json').exists():assert json.loads((directory/'identity.json').read_text())==identity
    else:write_json(directory/'identity.json',identity);(directory/'trainer_source.py').write_bytes(Path(__file__).read_bytes())
    torch.set_num_threads(2);torch.manual_seed(seed);np.random.seed(seed);torch.use_deterministic_algorithms(True)
    d,c,u,trmask,vamask=repaired_data(DATA);tr=np.flatnonzero(trmask);va=np.flatnonzero(vamask)
    mu=u[tr].mean(0);sd=np.maximum(u[tr].std(0),1e-3);cm=c[tr].mean(0);cs=np.maximum(c[tr].std(0),1e-3)
    z=torch.tensor((u-mu)/sd);ct=torch.tensor((c-cm)/cs);scale=torch.tensor(sd);alpha=torch.tensor(schedule())
    g=sio.loadmat(geometry,simplify_cells=True)
    J=torch.tensor(d['physical_J'][:,:11],dtype=torch.float32);g0=torch.tensor(d['physical_g'][:11].T,dtype=torch.float32)
    JT=torch.tensor(g['terminal_J'],dtype=torch.float32);e0=torch.tensor(g['terminal_e'].T,dtype=torch.float32)
    JC=torch.tensor(g['collision_J'],dtype=torch.float32);gc0=torch.tensor(g['collision_g'].T,dtype=torch.float32)
    model=Temporal();optimizer=torch.optim.AdamW(model.parameters(),lr=CONFIG['learning_rate']);best=float('inf');best_epoch=0;start_epoch=1;start=time.perf_counter()
    if (directory/'resume.pt').exists():
        resume=torch.load(directory/'resume.pt',weights_only=False);model.load_state_dict(resume['state_dict']);optimizer.load_state_dict(resume['optimizer'])
        torch.set_rng_state(resume['torch_rng']);np.random.set_state(resume['numpy_rng']);start_epoch=resume['epoch']+1;best=resume['best_score'];best_epoch=resume['best_epoch']
    generator=torch.Generator().manual_seed(5710+seed);vn=torch.randn(len(va)*4,360,generator=generator);vc=ct[va].repeat_interleave(4,0);vtarget=torch.tensor(u[va]).repeat_interleave(4,0)
    fields=['epoch','training_loss','sample_force_rms_N','sample_increment_error_rms_N','sample_physics_proxy','validation_score']
    with (directory/'training.csv').open('a' if start_epoch>1 else 'w',newline='',encoding='utf-8') as handle:
        log=csv.writer(handle)
        if start_epoch==1:log.writerow(fields)
        for epoch in range(start_epoch,CONFIG['epochs']+1):
            model.train();losses=[];order=tr[torch.randperm(len(tr)).numpy()]
            for offset in range(0,len(order),CONFIG['batch_size']):
                ix=order[offset:offset+CONFIG['batch_size']];t=torch.randint(0,100,(len(ix),));noise=torch.randn(len(ix),360);a=alpha[t,None]
                noisy=a.sqrt()*z[ix]+(1-a).sqrt()*noise;target=a.sqrt()*noise-(1-a).sqrt()*z[ix]
                pred=model(noisy,ct[ix],t);clean=reconstruct(noisy,pred,a);error=(clean-z[ix])*scale
                delta=torch.diff(error.reshape(-1,60,6),dim=1,prepend=torch.zeros(len(ix),1,6))
                mechanical=g0[ix]+torch.bmm(J[ix],error[:,:,None]).squeeze(-1)
                collision=gc0[ix]+torch.bmm(JC[ix],error[:,:,None]).squeeze(-1)
                terminal=e0[ix]+torch.bmm(JT[ix],error[:,:,None]).squeeze(-1)
                residual=torch.cat([mechanical,collision,terminal.square().sum(1,keepdim=True)-1],1)
                low=(a[:,0]>=CONFIG['physical_alpha_threshold']).float();physics=(torch.log1p(residual.relu()).mean(1)*low).mean()
                loss=(pred-target).square().mean()+.1*(error/20).square().mean()+.1*(delta/2).square().mean()+.05*physics
                optimizer.zero_grad(set_to_none=True);loss.backward();nn.utils.clip_grad_norm_(model.parameters(),10);optimizer.step();losses.append(float(loss.detach()))
            if epoch%CONFIG['validation_every'] and epoch!=1:continue
            model.eval()
            with torch.no_grad():
                generated=sample(model,vc,vn,alpha);force=torch.tensor(mu)+scale*generated;error=force-vtarget
                delta=torch.diff(error.reshape(-1,60,6),dim=1,prepend=torch.zeros(len(error),1,6))
                vf=float(error.square().mean().sqrt());vd=float(delta.square().mean().sqrt())
                ids=np.repeat(va,4);mech=g0[ids]+torch.bmm(J[ids],error[:,:,None]).squeeze(-1);coll=gc0[ids]+torch.bmm(JC[ids],error[:,:,None]).squeeze(-1)
                term=e0[ids]+torch.bmm(JT[ids],error[:,:,None]).squeeze(-1)
                vp=float(torch.log1p(torch.cat([mech,coll,term.square().sum(1,keepdim=True)-1],1).relu()).mean())
                score=(vf/20)**2+.1*(vd/2)**2+.05*vp
            log.writerow([epoch,np.mean(losses),vf,vd,vp,score]);handle.flush()
            if score<best:
                best=score;best_epoch=epoch;torch.save(dict(state_dict=model.state_dict(),epoch=epoch,score=score),directory/'best.pt')
            torch.save(dict(state_dict=model.state_dict(),optimizer=optimizer.state_dict(),torch_rng=torch.get_rng_state(),numpy_rng=np.random.get_state(),epoch=epoch,best_score=best,best_epoch=best_epoch),directory/'resume.pt')
            if epoch%50==0:print('Temporal',epoch,'force',round(vf,3),'delta_error',round(vd,3),'score',round(score,4),flush=True)
    state=torch.load(directory/'best.pt',weights_only=False);model.load_state_dict(state['state_dict']);model.eval()
    state.update(force_mean=mu,force_std=sd,context_mean=cm,context_std=cs,alpha_bar=schedule(),seed=seed)
    torch.save(state,directory/'model.pt');export(model,state,ct,va,directory)
    write_json(directory/'metadata.json',dict(**identity,completed=True,development_only=True,train_windows=len(tr),validation_windows=len(va),moving_coverage=cov['full_training_allowed'],
        best_epoch=best_epoch,best_score=best,seconds=time.perf_counter()-start,parameters=sum(p.numel() for p in model.parameters()),matlab_sha256=sha(directory/'weights.mat'),checkpoint_sha256=sha(directory/'model.pt'),
        selection_rule='fixed_full_20_step_DDIM_force_rate_and_local_physics_proxy; nonlinear feasibility checked separately by MATLAB'))
    generate(seed);print('Completed temporal pilot',directory,flush=True)


def generate(seed):
    directory=OUT/f'seed_{seed}';state=torch.load(directory/'model.pt',weights_only=False);model=Temporal();model.load_state_dict(state['state_dict']);model.eval();torch.set_num_threads(2)
    s=sio.loadmat(DATA/'selection.mat',simplify_cells=True);idx=np.flatnonzero(np.asarray(s['split']).reshape(-1)=='validation')
    c=(s['context'].T[idx]-state['context_mean'])/state['context_std'];generator=torch.Generator().manual_seed(5710+seed)
    noise=torch.randn(len(idx)*4,360,generator=generator);context=torch.tensor(np.repeat(c,4,axis=0),dtype=torch.float32)
    residual=sample(model,context,noise,torch.tensor(state['alpha_bar'])).numpy()*state['force_std']+state['force_mean']
    sio.savemat(OUT/f'candidates_{seed}.mat',dict(residual=residual.T,noise=noise.numpy().T,selection_index=np.repeat(idx+1,4)[None,:],candidate=np.tile(np.arange(1,5),len(idx))[None,:],seed=seed))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,default=5101);parser.add_argument('--development-only',action='store_true');args=parser.parse_args();train(args.seed,args.development_only)
