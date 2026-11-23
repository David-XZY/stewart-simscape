"""Formal specification sampling. Uses no solver or generated-path outcomes."""
import hashlib
import numpy as np
from pg_core import digest,VERSION

def region(shape,x):
    high=x>.75
    if shape=='cylinder': return bool(high[0] and high[1])
    return bool((high[0] and high[1]) or (high[2] and high[3]))

def physical(shape,x):
    if shape=='cylinder': return 50+350*x[0],[.8+.5*x[1],.10+.05*x[2]]
    return 50+350*x[0],[.8+.5*x[1],.20+.10*x[2],.16+.12*x[3]]

def sample_load(rng,shape,split,combo=False):
    dim=3 if shape=='cylinder' else 4
    for _ in range(100000):
        cell=rng.integers(0,10,dim); x=(cell+rng.uniform(.05,.95,dim))/10
        key=shape+':'+','.join(map(str,cell)); bucket=int(hashlib.sha256(key.encode()).hexdigest()[:8],16)%10
        target=(bucket<7 if split=='train' else bucket in [7,8] if split=='validation' else bucket==9)
        if region(shape,x)==combo and target:
            mass,dims=physical(shape,x); return mass,dims,key
    raise RuntimeError('No legal parameter macrocell under frozen split rule')

def formal_tasks(ntrain,seed=20260919):
    if ntrain not in [150,300,600]: raise ValueError('Unregistered training size')
    rng=np.random.default_rng(seed+901); all_tasks=[]; prototypes={'cylinder':[],'box':[]}
    for split,nscenes,per_scene in [('train',60,10),('validation',24,5),('test',24,8)]:
        for i in range(nscenes):
            family=['open','dual_fixture'][i%2]; cls=['translation','attitude','combined'][(i//2)%3]
            scene=f'formal_{split}_{i:03d}'; q0=np.array([-.12,-.18,1.04,0,0,0.]); qf=np.array([.12,.18,1.08,0,0,0.]); delta=rng.uniform(-.02,.02,3); q0[:3]+=delta; qf[:3]+=delta
            if cls=='attitude': qf[:3]=q0[:3]
            if cls!='translation': qf[5]=rng.choice([-10,-5,5,10])*np.pi/180
            params=rng.uniform(-.04,.04,2).tolist()+[float(qf[5]),float(rng.choice([2,3,5,7.5]))]
            for k in range(per_scene):
                shape=['cylinder','box'][(k+(i//2 if split=='validation' else 0))%2]; group=['G0','G1','G2','G3'][k//2] if split=='test' else 'development_selection' if split=='validation' else 'training'
                # Validation reserves some combination cells, distinct from test cells.
                combo=(split=='test' and group=='G2') or (split=='validation' and k>=3)
                mass,dims,cell=sample_load(rng,shape,split,combo)
                if split=='test' and group=='G0': mass,dims,cell=prototypes[shape][i%len(prototypes[shape])]
                if split=='test' and group=='G3':
                    # Both extrapolated factors occur in both environment families.
                    if (i//2)%2: mass=float(rng.uniform(400,440))
                    else: dims[0]=float(rng.uniform(1.30,1.35))
                spec=dict(taskId=f'{scene}_{shape}_{k:02d}',baseSceneId=scene,split=split,taskClass=cls,family=family,generalizationType=group,
                          payloadRegionId=cell,q0=q0.tolist(),qGoal=qf.tolist(),parameters=params,shape=shape,dimensions=dims,mass=float(mass))
                spec['specHash']=digest(spec); all_tasks.append(spec)
                if split=='train' and len(prototypes[shape])<4: prototypes[shape].append((mass,list(dims),cell))
    train=[t for t in all_tasks if t['split']=='train']
    # Include the eight declared G0 prototypes in every nested library manifest.
    proto_keys={digest([shape,m,list(d)]) for shape,values in prototypes.items() for m,d,_ in values}
    train.sort(key=lambda t:(digest([t['shape'],t['mass'],t['dimensions']]) not in proto_keys,digest(t['taskId'])))
    selected=train[:ntrain]+[t for t in all_tasks if t['split']!='train']
    return dict(version=VERSION,status='specifications_only_not_feasibility_evidence',tasks=selected,taskHash=digest(selected),trainingSize=ntrain,seed=seed,
                splitRule='base_scene_disjoint_and_parameter_macrocell_70_20_10_guard5pct',g0ObservedPrototypeCheckRequired=True)
