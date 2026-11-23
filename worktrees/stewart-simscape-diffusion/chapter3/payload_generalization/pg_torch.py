"""Differentiable rigid-body kernel. World spatial angular kinematics, float64."""
import torch
from pg_core import basis

def tensor(x, like): return torch.as_tensor(x,dtype=like.dtype,device=like.device)

def rotation(q,v,a):
    r,b,y=q[...,3],q[...,4],q[...,5]; cr,sr=torch.cos(r),torch.sin(r); cb,sb=torch.cos(b),torch.sin(b); cy,sy=torch.cos(y),torch.sin(y)
    z=torch.zeros_like(r); o=torch.ones_like(r)
    R=torch.stack([cy*cb,cy*sb*sr-sy*cr,cy*sb*cr+sy*sr,sy*cb,sy*sb*sr+cy*cr,sy*sb*cr-cy*sr,-sb,cb*sr,cb*cr],-1).reshape(q.shape[:-1]+(3,3))
    E=torch.stack([cy*cb,-sy,z,sy*cb,cy,z,-sb,z,o],-1).reshape(q.shape[:-1]+(3,3))
    rd,bd,yd=v[...,3],v[...,4],v[...,5]
    w=(E@v[...,3:,None]).squeeze(-1)
    conv=torch.stack([(-sy*yd*cb-cy*sb*bd)*rd-cy*yd*bd,(cy*yd*cb-sy*sb*bd)*rd-sy*yd*bd,-cb*bd*rd],-1)
    alpha=(E@a[...,3:,None]).squeeze(-1)+conv
    return R,w,alpha

def evaluate_physics(q,v,a,robot,derived):
    R,w,alpha=rotation(q,v,a)
    anchor=tensor(robot['A'],q).T; local=tensor(robot['B'],q).T[tensor(robot['legMap'],q).long()-1]
    rb=torch.einsum('...ij,kj->...ki',R,local); d=q[...,:3].unsqueeze(-2)+rb-anchor
    L=torch.linalg.vector_norm(d,dim=-1); u=d/L[...,None]
    point_v=v[...,:3].unsqueeze(-2)+torch.linalg.cross(w.unsqueeze(-2).expand_as(rb),rb)
    point_a=a[...,:3].unsqueeze(-2)+torch.linalg.cross(alpha.unsqueeze(-2).expand_as(rb),rb)+torch.linalg.cross(w.unsqueeze(-2).expand_as(rb),torch.linalg.cross(w.unsqueeze(-2).expand_as(rb),rb))
    Ld=(u*point_v).sum(-1); Ldd=(u*point_a).sum(-1)+(point_v.square().sum(-1)-Ld.square())/L
    J=torch.cat([u,torch.linalg.cross(rb,u)],-1)
    com=tensor(derived['comP'],q); inertia=tensor(derived['inertiaCOM'],q); mass=tensor(derived['totalMass'],q)
    rc=(R@com[...,None]).squeeze(-1); I=R@inertia@R.transpose(-1,-2)
    f=mass[...,None]*(a[...,:3]+torch.linalg.cross(alpha,rc)+torch.linalg.cross(w,torch.linalg.cross(w,rc))-tensor(robot['g'],q))
    moment=(I@alpha[...,None]).squeeze(-1)+torch.linalg.cross(w,(I@w[...,None]).squeeze(-1))+torch.linalg.cross(rc,f)
    W=torch.cat([f,moment],-1); F=torch.linalg.solve(J.transpose(-1,-2),W[...,None]).squeeze(-1)
    sv=torch.linalg.svdvals(J*tensor([1,1,1,2,2,2],q))
    return dict(L=L,Ld=Ld,Ldd=Ldd,F=F,J=J,W=W,R=R,omega=w,alpha=alpha,sigma=sv[...,-1],condition=sv[...,0]/sv[...,-1])

def reconstruct_spline(free, q0, qf, T, count=33):
    # Free vector is column-major MATLAB [6,n-6], represented here [n-6,6].
    n=free.shape[-1]//6+6
    cp=torch.cat([q0[...,None,:].expand(*q0.shape[:-1],3,6),free.reshape(*free.shape[:-1],n-6,6),qf[...,None,:].expand(*qf.shape[:-1],3,6)],-2)
    mats=basis(n,count); states=[]
    for order,B in enumerate(mats):
        result=torch.einsum('ns,...nd->...sd',tensor(B,free),cp)/T[...,None,None]**order
        states.append(result)
    return states

def clearance_lower(q,task,smooth=False):
    """Conservative directional support certificate; smooth abs OVERestimates support."""
    R,_,_=rotation(q,torch.zeros_like(q),torch.zeros_like(q)); p=task['payload']
    center=q[...,:3]+(R@tensor([0,0,.15],q)[...,None]).squeeze(-1)
    obstacles=task['scene']['hood']['obstacles']; obstacles=obstacles if isinstance(obstacles,list) else [obstacles]
    results=[]; eps=1e-12
    absolute=lambda x: torch.sqrt(x*x+eps) if smooth else torch.abs(x)
    for ob in obstacles:
        O=tensor(ob['R_S'],q); oc=tensor(ob['center_S'],q); half=tensor(ob['halfSize'],q)
        eye=torch.eye(3,dtype=q.dtype,device=q.device).expand_as(R)
        delta=oc-center; dirs=torch.cat([eye,R.transpose(-1,-2),O.T.expand_as(R),delta.unsqueeze(-2)],-2)
        # Exact same finite direction family as MATLAB, with degenerate normals zeroed.
        crosses=torch.linalg.cross(R.transpose(-1,-2).unsqueeze(-2).expand(*R.shape[:-2],3,3,3),O.T.expand(*R.shape[:-2],3,3).unsqueeze(-3).expand(*R.shape[:-2],3,3,3)).reshape(*R.shape[:-2],9,3)
        # Cross-axis directions become undefined at parallel axes. Exclude them
        # from the differentiable training family, retaining conservative bounds.
        if not smooth: dirs=torch.cat([dirs,crosses],-2)
        dirs=torch.cat([dirs,-dirs],-2)
        norm=torch.linalg.vector_norm(dirs,dim=-1,keepdim=True); valid=norm.squeeze(-1)>1e-12; dirs=dirs/norm.clamp_min(1e-12)
        local=dirs@R; obs_local=dirs@O
        if p['shape']=='box': support=(absolute(local)*tensor(p['dimensions'],q)/2).sum(-1)
        else:
            support=p['dimensions'][0]/2*absolute(local[...,0])+p['dimensions'][1]*torch.sqrt(local[...,1:].square().sum(-1)+(eps if smooth else 0))
        gap=(dirs*delta.unsqueeze(-2)).sum(-1)-(absolute(obs_local)*half).sum(-1)-support
        gap=gap.masked_fill(~valid,-1e6); results.append(gap.max(-1).values)
    if not results: return q.new_empty(q.shape[:-1]+(0,))
    return torch.stack(results,-1)

def envelope_lower(q,task,envelope,smooth=False):
    R,_,_=rotation(q,torch.zeros_like(q),torch.zeros_like(q)); robot=task['model']; pos=q[...,:3]
    A=tensor(robot['A'],q).T; B=tensor(robot['B'],q).T[tensor(robot['legMap'],q).long()-1]
    tips=pos.unsqueeze(-2)+torch.einsum('...ij,kj->...ki',R,B)
    obstacles=task['scene']['hood']['obstacles']; obstacles=obstacles if isinstance(obstacles,list) else [obstacles]
    allg=[]; eps=1e-12; absolute=lambda x:torch.sqrt(x*x+eps) if smooth else torch.abs(x)
    for ob in obstacles:
        O=tensor(ob['R_S'],q); dirs=torch.cat([torch.eye(3,dtype=q.dtype,device=q.device),O.T],0); dirs=torch.cat([dirs,-dirs],0)
        box=(absolute(dirs@O)*tensor(ob['halfSize'],q)).sum(-1); oc=tensor(ob['center_S'],q)
        ax=R[..., :,2]@dirs.T
        support=envelope['platformHalfHeight']*absolute(ax)+envelope['platformRadius']*torch.sqrt((1-ax.square()).clamp_min(0)+eps)
        allg.append(((oc-pos)@dirs.T-box-support).max(-1).values)
        ax=dirs[:,2]; support=envelope['baseHalfHeight']*absolute(ax)+envelope['baseRadius']*torch.sqrt((1-ax.square()).clamp_min(0)+eps)
        base=((oc-tensor(envelope['baseCenter'],q))@dirs.T-box-support).max(-1).values
        allg.append(base.expand(q.shape[:-1]))
        center=(tips+A)/2; half=(tips-A)/2
        gap=(oc-center)@dirs.T-box-absolute(half@dirs.T)-envelope['legRadius']
        allg.extend(gap.max(-1).values.unbind(-1))
    return torch.stack(allg,-1) if allg else q.new_empty(q.shape[:-1]+(0,))

def trajectory_constraint_loss(free,task,envelope, count=33):
    free=free.double(); q0=tensor(task['q0'],free); qf=tensor(task['qGoal'],free); T=tensor(task['T'],free)
    q,v,a,_=reconstruct_spline(free,q0,qf,T,count)
    state=evaluate_physics(q,v,a,task['model'],task['derived']); m=task['model']; relu=torch.nn.functional.relu
    groups=[]
    def mean_penalty(x): return relu(x).square().mean()
    groups.append(mean_penalty(torch.cat([(tensor(m['lmin'],free)-state['L'])/tensor(m['lmax'],free),(state['L']-tensor(m['lmax'],free))/tensor(m['lmax'],free)],-1)))
    groups.append(mean_penalty(state['Ld'].abs()/tensor(m['actuator']['ldotMax'],free)-1))
    groups.append(mean_penalty(state['Ldd'].abs()/tensor(m['actuator']['lddotMax'],free)-1))
    groups.append(mean_penalty(torch.cat([(tensor(m['actuator']['forceMin'],free)-state['F'])/2000,(state['F']-tensor(m['actuator']['forceMax'],free))/2000],-1)))
    gap=torch.cat([clearance_lower(q,task,True),envelope_lower(q,task,envelope,True)],-1)
    groups.append(mean_penalty((.01-gap)/.01) if gap.numel() else free.sum()*0)
    groups.append(mean_penalty((.1-state['sigma'])/.1)); groups.append(mean_penalty(state['condition']/20-1))
    return torch.stack(groups).sum(),dict(zip(['length','speed','acceleration','force','clearance','sigma','condition'],groups))
