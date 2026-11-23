"""Versioned payload contract, frozen manifests, and diffusion schedule (SI)."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

VERSION = 'payload_generalization_v1'
CONTEXT = {'basic': ('pg_basic99_v1', 99), 'signature': ('pg_signature354_v1', 354)}
SEEDS = [20260919, 20260920, 20260921]

def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def write(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')

def payload(shape, dimensions, mass):
    d = np.asarray(dimensions, dtype=float)
    if not np.isfinite(mass) or mass <= 0 or not np.isfinite(d).all() or (d <= 0).any():
        raise ValueError('Positive finite payload parameters required')
    if shape == 'cylinder' and d.shape == (2,):
        length, radius = d
        inertia = mass * np.diag([radius**2 / 2, (3*radius**2+length**2)/12, (3*radius**2+length**2)/12])
        bounds = [length, 2*radius, 2*radius]; volume = np.pi*radius**2*length
    elif shape == 'box' and d.shape == (3,):
        inertia = mass/12*np.diag([d[1]**2+d[2]**2, d[0]**2+d[2]**2, d[0]**2+d[1]**2])
        bounds = d.tolist(); volume = np.prod(d)
    else:
        raise ValueError('Unsupported shape or dimensions')
    mount = np.array([0., 0., .15]); com = mass/(50+mass)*mount
    def shift(m, r): return m*(np.dot(r, r)*np.eye(3)-np.outer(r, r))
    composite = np.diag([3.125, 3.125, 6.25])+inertia+shift(50, -com)+shift(mass, mount-com)
    return {'shape': shape, 'dimensions': d.tolist(), 'mass': float(mass), 'boundingDimensions': bounds,
            'mount': {'R': np.eye(3).tolist(), 'center': mount.tolist()}, 'density': float(mass/volume),
            'inertiaCOM': inertia.tolist(), 'derived': {'totalMass': float(50+mass), 'comP': com.tolist(), 'inertiaCOM': composite.tolist()}}

def schedule(n=100):
    t = np.arange(n+1, dtype=float)/n
    f = np.cos((t+.008)/1.008*np.pi/2)**2
    beta = np.minimum(1-f[1:]/f[:-1], .999); alpha = 1-beta; ab = np.cumprod(alpha); prev = np.r_[1., ab[:-1]]
    return dict(name='cosine_s008_cap999', steps=n, beta=beta.tolist(), alpha=alpha.tolist(), alphaBar=ab.tolist(),
                posteriorVariance=(beta*(1-prev)/(1-ab)).tolist(), posteriorMeanClean=(beta*np.sqrt(prev)/(1-ab)).tolist(),
                posteriorMeanNoisy=((1-prev)*np.sqrt(alpha)/(1-ab)).tolist())

def basis(n=16, count=33, degree=5):
    """Clamped B-spline basis and three analytic derivatives; no SciPy dependency."""
    knots=np.r_[np.zeros(degree+1), np.arange(1,n-degree)/(n-degree), np.ones(degree+1)]
    u=np.linspace(0,1,count); x=np.minimum(u, np.nextafter(1.,0.))
    levels=[np.array([((x>=knots[i]) & (x<knots[i+1])).astype(float) for i in range(len(knots)-1)])]
    for p in range(1,degree+1):
        prev=levels[-1]; cur=np.zeros((len(knots)-p-1,count))
        for i in range(len(cur)):
            a=knots[i+p]-knots[i]; b=knots[i+p+1]-knots[i+1]
            if a>0: cur[i]+=(x-knots[i])/a*prev[i]
            if b>0: cur[i]+=(knots[i+p+1]-x)/b*prev[i+1]
        levels.append(cur)
    def derivative(p, order):
        if order==0: return levels[p]
        lower=derivative(p-1,order-1); out=np.zeros((len(knots)-p-1,count))
        for i in range(len(out)):
            a=knots[i+p]-knots[i]; b=knots[i+p+1]-knots[i+1]
            if a>0: out[i]+=p/a*lower[i]
            if b>0: out[i]-=p/b*lower[i+1]
        return out
    out=[derivative(degree,k) for k in range(4)]
    out[0][:,-1]=0; out[0][-1,-1]=1
    return out

def development_tasks():
    rng=np.random.default_rng(SEEDS[0]); tasks=[]; durations=[2.,3.,5.,7.5]
    for env in ['open','dual_fixture']:
        for cls in ['translation','attitude','combined']:
            for instance in range(4):
                scene=f'dev_{env}_{cls}_{instance:02d}'; split='dev_validation' if instance==3 or (instance==2 and cls=='combined') else 'dev_train'
                # 2 environments x (three instance-3 + one instance-2) = 8 validation scenes.
                q0=np.array([-.12,-.18,1.04,0,0,0]); qf=np.array([.12,.18,1.08,0,0,0.])
                delta=rng.uniform(-.02,.02,3); q0[:3]+=delta; qf[:3]+=delta
                yaw=np.deg2rad([5,-5,10,-10][instance])
                if cls=='attitude': qf[:3]=q0[:3]; qf[5]=yaw
                elif cls=='combined': qf[5]=yaw
                offsets=rng.uniform(-.04,.04,2).tolist(); T=durations[(instance+['translation','attitude','combined'].index(cls))%4]
                for shape in ['cylinder','box']:
                    for mass in [100.,400.]:
                        for size in [0,1]:
                            dims=([1.,.12],[1.3,.15])[size] if shape=='cylinder' else ([1.,.24,.20],[1.3,.30,.28])[size]
                            task=dict(taskId=f'{scene}_{shape}_{int(mass)}_{size}',baseSceneId=scene,split=split,taskClass=cls,family=env,generalizationType='development',
                                      q0=q0.tolist(),qGoal=qf.tolist(),parameters=offsets+[float(qf[5]),T],shape=shape,dimensions=dims,mass=mass)
                            task['specHash']=digest(task); tasks.append(task)
    assert len(tasks)==192 and len({t['baseSceneId'] for t in tasks if t['split']=='dev_train'})==16
    # Balanced pilot: two shapes x two environments x four durations. No solver results used.
    selected=[]
    for env in ['open','dual_fixture']:
        for shape in ['cylinder','box']:
            for i,T in enumerate(durations):
                candidates=[t for t in tasks if t['family']==env and t['shape']==shape and t['parameters'][3]==T and t['mass']==[100.,400.][i%2] and t['dimensions'][0]==[1.,1.3][(i//2)%2]]
                selected.append(sorted(candidates,key=lambda t:t['taskId'])[0]['taskId'])
    return {'version':VERSION,'taskSeed':SEEDS[0],'tasks':tasks,'pilotTaskIds':selected,'taskHash':digest(tasks),'pilotHash':digest(selected)}

def check_context(model, mode='basic'):
    schema,dim=CONTEXT[mode]
    if model.get('contextSchema')!=schema or len(model.get('inputMean',[]))!=dim:
        raise ValueError('Incompatible condition schema; legacy 93/96 checkpoints cannot be used')

def reference_scale(mean_seconds, cap_hours=72):
    choices=[n for n in [150,300,600] if 1.25*mean_seconds*(8*n+4*120+4*192)<=cap_hours*3600]
    return max(choices) if choices else None
