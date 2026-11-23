"""Offline differentiable rollout through the exact exported Stewart stages.

Stage A/B derivatives were built before native compilation in MATLAB. Reverse
recursion includes actuator states, previous input and the innovation feedback.
No finite-difference or teacher-local dynamics proxy is used by task gradients.
"""
from pathlib import Path
import json, os, subprocess, time
import numpy as np
import scipy.io as sio
import torch
import casadi as ca
from innovation_v74 import CH5
from control_force import sha, write_json

OUT=CH5/'artifacts/control_force_v62/nonlinear_v80'
GCC=Path('C:/msys64/ucrt64/bin/gcc.exe')

class Bank:
    def __init__(self,metadata):
        self.metadata=metadata;self.n=metadata['n'];self.stage_maps={};self.equilibrium_maps={}
        for kind in ['stage','equilibrium']:
            source=OUT/metadata[kind+'_file'];library=source.with_suffix('.dll')
            if not library.exists():
                result=subprocess.run([str(GCC),'-O2','-shared',str(source),'-o',str(library)],
                    capture_output=True,text=True,timeout=180)
                (source.with_suffix('.compile.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
                if result.returncode:raise RuntimeError(result.stderr)
            setattr(self,kind,ca.external(metadata[kind+'_function'],str(library)))
    def mapped(self,B):
        if B not in self.stage_maps:self.stage_maps[B]=self.stage.map(B)
        return self.stage_maps[B]
    def equilibrium_mapped(self,B):
        if B not in self.equilibrium_maps:self.equilibrium_maps[B]=self.equilibrium.map(B)
        return self.equilibrium_maps[B]

class Rollout(torch.autograd.Function):
    @staticmethod
    def forward(ctx,V,x,previous,L,anchor,bank):
        B,H,_=V.shape;n=bank.n;dtype=V.dtype
        vi=V.detach().double().numpy();state=x.detach().double().numpy().copy()
        prev=previous.detach().double().numpy().copy();feedback=L.detach().double().numpy();steady=anchor.detach().double().numpy()
        states=np.empty((B,H+1,n));commands=np.empty((B,H,6));As=np.empty((B,H,n,n));Bs=np.empty((B,H,n,6));states[:,0]=state
        stage=bank.mapped(B)
        for j in range(H):
            error=np.concatenate([state,prev],axis=1)-steady
            u=prev+vi[:,j]-np.einsum('bij,bj->bi',feedback,error)
            next_state,a,b=stage(state.T,u.T)
            state=np.asarray(next_state).T.copy();commands[:,j]=u;states[:,j+1]=state
            As[:,j]=np.asarray(a).reshape(n,n,B,order='F').transpose(2,0,1)
            Bs[:,j]=np.asarray(b).reshape(n,6,B,order='F').transpose(2,0,1);prev=u
        ctx.save_for_backward(torch.from_numpy(As),torch.from_numpy(Bs),torch.from_numpy(feedback))
        ctx.input_dtype=dtype
        return torch.from_numpy(states),torch.from_numpy(commands)
    @staticmethod
    def backward(ctx,gX,gU):
        As,Bs,L=ctx.saved_tensors;B,H,n,_=As.shape
        if gX is None:gX=torch.zeros(B,H+1,n,dtype=torch.float64)
        if gU is None:gU=torch.zeros(B,H,6,dtype=torch.float64)
        cx=torch.zeros(B,n,1,dtype=torch.float64);cp=torch.zeros(B,6,1,dtype=torch.float64)
        gradient=torch.empty(B,H,6,dtype=torch.float64)
        for j in range(H-1,-1,-1):
            lam=gX[:,j+1,:,None]+cx
            du=gU[:,j,:,None]+cp+torch.bmm(Bs[:,j].transpose(1,2),lam)
            gradient[:,j]=du[:,:,0]
            cx=torch.bmm(As[:,j].transpose(1,2),lam)-torch.bmm(L[:,:,:n].transpose(1,2),du)
            cp=du-torch.bmm(L[:,:,n:].transpose(1,2),du)
        return gradient.to(ctx.input_dtype),None,None,None,None,None

class Equilibrium(torch.autograd.Function):
    @staticmethod
    def forward(ctx,q,bank):
        B=len(q);u,j=bank.equilibrium_mapped(B)(q.detach().double().numpy().T)
        jac=np.asarray(j).reshape(6,6,B,order='F').transpose(2,0,1).copy()
        ctx.save_for_backward(torch.from_numpy(jac));ctx.input_dtype=q.dtype
        return torch.from_numpy(np.asarray(u).T.copy())
    @staticmethod
    def backward(ctx,gradient):
        (jac,)=ctx.saved_tensors
        return torch.bmm(jac.transpose(1,2),gradient[:,:,None]).squeeze(-1).to(ctx.input_dtype),None

def banks():
    metadata=json.loads((OUT/'dynamics.json').read_text(encoding='utf-8-sig'))
    assert metadata['completed'] and ca.__version__==metadata['casadi_version']
    os.environ['PATH']=str(GCC.parent)+os.pathsep+os.environ['PATH']
    return {m['mode']:Bank(m) for m in metadata['modes']}

def task_cost(X,U,previous,goal,bank):
    ps=torch.tensor([.1,.1,.1,np.deg2rad(10),np.deg2rad(10),np.deg2rad(10)],dtype=torch.float64)
    vs=torch.tensor([.1,.1,.1,.2,.2,.2],dtype=torch.float64)
    qs=X[:,-1,:6];us=Equilibrium.apply(qs,bank);delta=torch.diff(U,dim=1,prepend=previous[:,None].double())
    stage=((X[:,1:,:6]-qs[:,None])/ps).square().sum((1,2))+(X[:,1:,6:12]/vs).square().sum((1,2))
    stage+=.05*((X[:,1:,:6]-goal[:,None])/ps).square().sum((1,2))
    stage+=.01*(delta/160).square().sum((1,2))+.001*((U-us[:,None])/2000).square().sum((1,2))
    cost=.02*stage+100*((qs-goal)/ps).square().sum(1)
    normalization=(100*((X[:,0,:6]-goal)/ps).square().sum(1)).clamp_min(1)
    return cost/normalization

def verify():
    OUT.mkdir(exist_ok=True,parents=True);assert not (OUT/'parity.json').exists(),'Preserve kernel verification.'
    start=time.monotonic();notice=start
    while not (OUT/'dynamics.json').exists():
        if time.monotonic()-start>1800:raise TimeoutError('Dynamics export')
        if time.monotonic()-notice>30:print('Awaiting exact MATLAB dynamics export',flush=True);notice=time.monotonic()
        time.sleep(1)
    kernels=banks();data=sio.loadmat(OUT/'anchors.mat',simplify_cells=True)
    report=dict(completed=False,formal=False,rows=[],gradient_rows=[])
    for row in np.atleast_1d(data['golden']):
        bank=kernels[row['mode']];n=bank.n
        V=torch.tensor(row['innovation'].T[None],dtype=torch.float64,requires_grad=True)
        x=torch.tensor(row['x'][None],dtype=torch.float64);previous=torch.tensor(row['previous'][None],dtype=torch.float64)
        L=torch.tensor(row['L'][None],dtype=torch.float64);anchor=torch.tensor(row['xi'][None],dtype=torch.float64)
        X,U=Rollout.apply(V,x,previous,L,anchor,bank);goal=x[:,:6]+.01
        loss=task_cost(X,U,previous,goal,bank).mean();loss.backward();gradient=V.grad.detach().numpy().reshape(-1)
        next_state,a,b=bank.stage(row['x'],row['U'][:,0])
        report['rows'].append(dict(mode=row['mode'],state_error=float(np.max(np.abs(X.detach().numpy()[0]-row['X'].T))),
            command_error_N=float(np.max(np.abs(U.detach().numpy()[0]-row['U'].T))),
            stage_error=float(np.max(np.abs(np.asarray(next_state).ravel()-row['next']))),
            A_error=float(np.max(np.abs(np.asarray(a)-row['A']))),B_error=float(np.max(np.abs(np.asarray(b)-row['B'])))))
        for coordinate in [0,7,100,170,277,359]:
            values=[];h=.01
            with torch.no_grad():
                for direction in [1,-1]:
                    trial=V.detach().clone();trial.reshape(-1)[coordinate]+=direction*h
                    tx,tu=Rollout.apply(trial,x,previous,L,anchor,bank)
                    values.append(float(task_cost(tx,tu,previous,goal,bank).mean()))
            finite_difference=(values[0]-values[1])/(2*h);analytic=float(gradient[coordinate])
            report['gradient_rows'].append(dict(mode=row['mode'],coordinate=coordinate,analytic=analytic,finite_difference=finite_difference,
                absolute_error=abs(analytic-finite_difference),accepted=abs(analytic-finite_difference)<=1e-6+1e-4*abs(finite_difference)))
    report['accepted']=all(r['state_error']<1e-9 and r['command_error_N']<1e-7 and r['A_error']<1e-9 and r['B_error']<1e-9 for r in report['rows']) and all(r['accepted'] for r in report['gradient_rows'])
    report['completed']=True;report['casadi_version']=ca.__version__
    report['source_hashes']={p.name:sha(p) for p in OUT.glob('*.c')};report['library_hashes']={p.name:sha(p) for p in OUT.glob('*.dll')}
    report['implementation_sha256']=sha(__file__);write_json(OUT/'parity.json',report)
    if not report['accepted']:raise RuntimeError('Nonlinear rollout or reverse derivatives failed parity.')
    print('Exact nonlinear rollout and gradient parity passed',flush=True)

if __name__=='__main__':verify()
