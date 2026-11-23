"""Run against a fresh MATLAB/CasADi oracle; do not manufacture training labels."""
import argparse
import time
import numpy as np
import torch
from pg_core import read,write,schedule,basis
from pg_torch import evaluate_physics,clearance_lower,trajectory_constraint_loss

def audit(path,out):
    torch.set_num_threads(1); start=time.perf_counter(); o=read(path); errors={}
    def t(x): return torch.tensor(x,dtype=torch.float64)
    q,v,a=t(o['q']),t(o['v']),t(o['a'])
    for name,task,expected in [('nominal',o['task'],o['F']),('box',o['boxTask'],o['boxForce']),('heavy',o['heavyTask'],o['heavyForce'])]:
        p=evaluate_physics(q,v,a,task['model'],task['derived'])
        error=float((p['F']-t(expected)).abs().max()/t(expected).abs().max().clamp_min(1))
        assert error<1e-6,(name,error); errors[name+'ForceRelative']=error
    p=evaluate_physics(q,v,a,o['robot'],o['task']['derived'])
    for field in ['L','Ld','Ldd','W']:
        error=float((p[field]-t(o[field])).abs().max()); assert error<1e-8,(field,error); errors[field+'Absolute']=error
    for key,actual in zip(['basis','basisD','basisDD'],basis()):
        error=float(np.max(np.abs(np.array(o[key])-actual))); assert error<1e-8,(key,error); errors[key]=error
    for key in ['beta','alphaBar','posteriorVariance','posteriorMeanClean','posteriorMeanNoisy']:
        np.testing.assert_allclose(schedule()[key],o['schedule'][key],atol=1e-14,rtol=1e-12)
    for name,task in [('cylinder',o['task']),('box',o['boxTask'])]:
        gap=clearance_lower(q[:30],task)
        error=float((gap-t(o[name+'Gap'])).abs().max()); assert error<1e-8,(name,error); errors[name+'GapAbsolute']=error
        smooth=clearance_lower(q[:30],task,True); assert bool((smooth<=gap+1e-12).all())
    state=torch.cat([q[:30],v[:30],a[:30]],-1).requires_grad_(True)
    p=evaluate_physics(state[:,:6],state[:,6:12],state[:,12:],o['robot'],o['task']['derived'])
    grad=torch.autograd.grad(p['F'].square().sum()/1e6,state)[0]
    error=float((grad-t(o['forceGradient'])).abs().max()/t(o['forceGradient']).abs().max().clamp_min(1))
    assert error<1e-6,error; errors['casadiTorchGradientRelative']=error
    # Constraint-loss gradient directional finite differences away from switches.
    torch.manual_seed(20260919)
    free=(t(o['baseline']).T[3:-3].flatten()+.015*torch.randn(60,dtype=torch.float64)).requires_grad_(True)
    loss,_=trajectory_constraint_loss(free,o['task'],o['envelope']); gradient=torch.autograd.grad(loss,free)[0]
    torch.manual_seed(20260919); direction=torch.randn_like(free); direction/=torch.linalg.vector_norm(direction)
    eps=1e-6
    plus=trajectory_constraint_loss(free+eps*direction,o['task'],o['envelope'])[0]
    minus=trajectory_constraint_loss(free-eps*direction,o['task'],o['envelope'])[0]
    numerical=(plus-minus)/(2*eps); analytic=gradient@direction
    error=float(((numerical-analytic).abs()/torch.maximum(numerical.abs(),analytic.abs()).clamp_min(1e-6)).detach()); assert error<1e-3,error
    errors['constraintDirectionalRelative']=error
    assert bool(torch.isfinite(gradient).all())
    switch=t(o['baseline']).T[3:-3].flatten().clone().requires_grad_(True)
    sl=trajectory_constraint_loss(switch,o['task'],o['envelope'])[0]; sg=torch.autograd.grad(sl,switch)[0]
    assert bool(torch.isfinite(sg).all())
    trials=[trajectory_constraint_loss(switch-10.**(-k)*sg/sg.square().mean().sqrt().clamp_min(1e-8),o['task'],o['envelope'])[0] for k in range(2,7)]
    assert any(float(x.detach())<float(sl.detach()) for x in trials),'No descent at symmetric collision switch'
    r=dict(passed=True,statesPerLoad=1000,payloads=3,errors=errors,seconds=time.perf_counter()-start,torch=torch.__version__,device='cpu',threads=1)
    write(out,r); print(r)

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('oracle'); p.add_argument('output'); a=p.parse_args(); audit(a.oracle,a.output)
