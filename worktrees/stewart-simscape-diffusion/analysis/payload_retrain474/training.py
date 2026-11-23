"""Train-only normalization, task-equal labels, cosine DDPM and bounded guidance."""
from __future__ import annotations
import copy
import hashlib
import math
import subprocess
from pathlib import Path
import time
import numpy as np
import torch
from torch import nn
from common import score_panels
from pg_core import CONTEXT,read,write,schedule,digest
from pg_torch import trajectory_constraint_loss

METHODS={'regression','regression_signature','ddpm_basic','ddpm_signature'}

def signature_mode(method): return 'signature' in method

def network(input_dim,output_dim,width=256):
    return nn.Sequential(nn.Linear(input_dim,width),nn.Tanh(),nn.Linear(width,width),nn.Tanh(),nn.Linear(width,width),nn.Tanh(),nn.Linear(width,output_dim))

def closest_width(output_dim):
    def size(inp,w): return (inp+1)*w+2*(w+1)*w+(w+1)*output_dim
    target=size(354+output_dim+2,256)
    return min(range(256,400),key=lambda w:abs(size(99+output_dim+2,w)-target))

def model_context(row,method):
    x=list(row['context'])
    if signature_mode(method):
        s=np.asarray(row['signature'],float).reshape(5,51).copy()
        if method.endswith('no_accel'): s[:,6:42]=0
        if method.endswith('no_gap'): s[:,42:48]=0
        x+=s.flatten().tolist()
    return x


def project_clean(clean, asset):
    """Optional TRAIN-ONLY sampling envelope; never a physical acceptance test.

    Legacy assets omit this field and retain their original sampler exactly.
    """
    bounds=asset.get('cleanProjection')
    if not bounds: return clean
    lo=torch.as_tensor(bounds['lower'],device=clean.device,dtype=clean.dtype)
    hi=torch.as_tensor(bounds['upper'],device=clean.device,dtype=clean.dtype)
    return torch.maximum(lo,torch.minimum(hi,clean))

def guide(clean,model,task,envelope,eta,deadline,clock=time.perf_counter):
    """One standardized-space physical step; no gradient update after deadline."""
    if clock()>=deadline: return clean,0
    z=clean.detach().double().requires_grad_(True)
    mean=torch.as_tensor(model['outputMean'],device=z.device,dtype=z.dtype)
    scale=torch.as_tensor(model['outputScale'],device=z.device,dtype=z.dtype)
    before,_=trajectory_constraint_loss(z*scale+mean,task,envelope)
    grad=torch.autograd.grad(before,z)[0]
    if not torch.isfinite(grad).all() or clock()>=deadline: return clean,1
    direction=grad/grad.square().mean().sqrt().clamp_min(1e-8); step=min(eta,.1)
    for b in range(4):
        if clock()>=deadline: break
        trial=z-step/2**b*direction
        with torch.no_grad(): after,_=trajectory_constraint_loss(trial*scale+mean,task,envelope)
        if torch.isfinite(after) and after<before and clock()<deadline:
            return trial.detach().to(clean.dtype),1
    return clean,1

def sample(net,asset,context,candidate_id,seed,deadline=math.inf,task=None,envelope=None,guidance=False,end_guidance=False,eta=.03,noise=None):
    device=next(net.parameters()).device; dtype=next(net.parameters()).dtype; begun=time.perf_counter()
    x=torch.tensor((np.asarray(context)-asset['inputMean'])/asset['inputScale'],dtype=dtype,device=device)
    dim=len(asset['outputMean']); calls=0; seconds=0.
    if asset['method'].startswith('regression'):
        if candidate_id!=1: return None,dict(reason='deterministic_single_candidate')
        with torch.no_grad(): z=net(x)
    else:
        s=asset['noiseSchedule']; steps=s['steps']
        generator=torch.Generator(device=device).manual_seed((int(seed)+7919*(candidate_id-1))%(2**32-1))
        noise=torch.randn((steps,dim),generator=generator,dtype=dtype,device=device) if noise is None else torch.as_tensor(noise,dtype=dtype,device=device)
        z=noise[0].clone()
        for j in range(steps-1,-1,-1):
            if time.perf_counter()>=deadline: return None,dict(reason='incomplete_candidate',guidanceCalls=calls,guidanceSeconds=seconds)
            phase=j/(steps-1)*math.pi
            with torch.no_grad():
                eps=net(torch.cat([x,z,torch.tensor([math.sin(phase),math.cos(phase)],device=device,dtype=dtype)]))
                clean=(z-math.sqrt(1-s['alphaBar'][j])*eps)/math.sqrt(s['alphaBar'][j])
                clean=project_clean(clean,asset)
            if guidance and j+1 in [20,15,10,5,1]:
                mark=time.perf_counter(); clean,n=guide(clean,asset,task,envelope,eta,deadline); seconds+=time.perf_counter()-mark; calls+=n
            z=s['posteriorMeanClean'][j]*clean+s['posteriorMeanNoisy'][j]*z
            if j>0: z=z+math.sqrt(s['posteriorVariance'][j])*noise[j]
        if end_guidance:
            for _ in range(5):
                mark=time.perf_counter(); z,n=guide(z,asset,task,envelope,eta,deadline); seconds+=time.perf_counter()-mark; calls+=n
    if str(device).startswith('cuda'): torch.cuda.synchronize()
    if time.perf_counter()>=deadline: return None,dict(reason='incomplete_candidate',guidanceCalls=calls,guidanceSeconds=seconds)
    controls=z.detach().cpu().numpy()*asset['outputScale']+asset['outputMean']
    return controls,dict(reason='complete',guidanceCalls=calls,guidanceSeconds=seconds,seconds=time.perf_counter()-begun)

def validate_dataset(data):
    if data.get('version')!='pg_dataset_v1': raise ValueError('Wrong dataset schema')
    rows=data['rows']; by_scene={}
    for r in rows:
        if r['split'] not in ['train','validation']: raise ValueError('Test labels cannot enter training')
        by_scene.setdefault(r['baseSceneId'],set()).add(r['split'])
        if not r['rawPassed'] or not r['fittedPassed']: raise ValueError('Nonqualified label')
        if len(r['context'])!=99 or len(r['controls'])%6: raise ValueError('Wrong condition/output layout')
        if not np.isfinite(r['context']+r['controls']).all(): raise ValueError('Nonfinite label')
    if any(len(s)>1 for s in by_scene.values()): raise ValueError('Base scene leakage')
    for split,n in [('train',50),('validation',20)]:
        subset=[r for r in rows if r['split']==split]
        if len({r['taskId'] for r in subset})<n: raise ValueError(f'Label gate: {split} requires {n} qualified conditions')
        if {r['task']['payload']['shape'] for r in subset}!={'box','cylinder'}: raise ValueError('Both shapes required')
    return rows

def train(dataset,output,method,seed,validator,config,device='cuda',width=None,physics_weight=0.,resume=False,validation_manifest=None):
    """validator(candidates) must run the shared complete physical acceptance.

    No fallback from physical validation to denoising loss is permitted.
    """
    if method not in METHODS: raise ValueError('Unsupported method')
    if not callable(validator): raise ValueError('Shared physical checkpoint validator is required')
    start=time.perf_counter(); data=read(dataset); rows=validate_dataset(data); output=Path(output)
    if output.exists() and not resume: raise ValueError('Immutable model run: use a new output directory or explicit resume')
    output.mkdir(parents=True,exist_ok=True); tc=config['training']; deadline=start+tc['wallHours']*3600
    torch.manual_seed(seed); rng=np.random.default_rng(seed)
    by_task={}
    for i,r in enumerate(rows): by_task.setdefault(r['taskId'],[]).append(i)
    if method.startswith('regression'):
        chosen={k:min(v,key=lambda i:(rows[i]['objective'],rows[i]['labelId'])) for k,v in by_task.items()}
        rows=[rows[i] for i in chosen.values()]; by_task={r['taskId']:[i] for i,r in enumerate(rows)}
    train_tasks=sorted(k for k,ids in by_task.items() if rows[ids[0]]['split']=='train')
    val_tasks=sorted(k for k,ids in by_task.items() if rows[ids[0]]['split']=='validation')
    # Fixed task IDs, condition equal. Full validation selection is a later step.
    validation_conditions=read(validation_manifest)['checkpointRows'] if validation_manifest else data.get('validationTasks')
    if not validation_conditions: raise ValueError('Full validation condition manifest required, including failed-reference conditions')
    if any(r['task']['split'] not in ['validation','dev_validation'] for r in validation_conditions): raise ValueError('Validation condition leakage')
    checkpoint_rows=validation_conditions
    if validation_manifest:
        assert len(checkpoint_rows)==194
        train_scenes={r['baseSceneId'] for r in rows if r['split']=='train'}
        assert not train_scenes & {r['task']['baseSceneId'] for r in checkpoint_rows}
    x=np.array([model_context(r,method) for r in rows]); y=np.array([r['controls'] for r in rows])
    train_ids=[i for k in train_tasks for i in by_task[k]]; weights=np.array([1/len(by_task[rows[i]['taskId']]) for i in train_ids]); weights/=weights.sum()
    xm=np.average(x[train_ids],axis=0,weights=weights); ym=np.average(y[train_ids],axis=0,weights=weights)
    xs=np.maximum(np.sqrt(np.average((x[train_ids]-xm)**2,axis=0,weights=weights)),1e-6)
    ys=np.maximum(np.sqrt(np.average((y[train_ids]-ym)**2,axis=0,weights=weights)),1e-5)
    dim=y.shape[1]; diffusion=not method.startswith('regression'); width=width or (closest_width(dim) if method=='ddpm_basic_wide' else 256)
    net=network(x.shape[1]+(dim+2 if diffusion else 0),dim,width).to(device); optimizer=torch.optim.AdamW(net.parameters(),lr=tc['learningRate'],betas=tuple(tc['betas']),eps=tc['eps'],weight_decay=tc['weightDecay'],amsgrad=False)
    X=torch.tensor((x-xm)/xs,dtype=torch.float32,device=device); Y=torch.tensor((y-ym)/ys,dtype=torch.float32,device=device)
    sc=schedule(); ab=torch.tensor(sc['alphaBar'],dtype=torch.float32,device=device)
    schema=CONTEXT['signature' if signature_mode(method) else 'basic'][0]
    asset=dict(method=method,contextSchema=schema,inputMean=xm.tolist(),inputScale=xs.tolist(),outputMean=ym.tolist(),outputScale=ys.tolist(),
               outputRepresentation='absolute_free_controls_column_major',noiseSchedule=sc,noiseScheduleHash=digest(sc),controlCount=dim//6+6,seed=seed,
               datasetHash=hashlib.sha256(Path(dataset).read_bytes()).hexdigest(),width=width)
    asset['normalizationHash']=digest([xm.tolist(),xs.tolist(),ym.tolist(),ys.tolist()])
    asset['trainingCodeHash']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    asset['trainingConfigHash']=digest(config)
    asset['validationManifestHash']=hashlib.sha256(Path(validation_manifest).read_bytes()).hexdigest()
    asset['batchSize']=tc['batch']
    if diffusion and config.get('sampling',{}).get('cleanProjection')=='train_standardized_envelope':
        radius=np.maximum(1.,1.5*np.max(np.abs((y[train_ids]-ym)/ys),axis=0))
        asset['cleanProjection']=dict(kind='train_standardized_envelope',lower=(-radius).tolist(),upper=radius.tolist(),source='training_labels_only',physicalCertificate=False)
    best=None; best_key=None; best_epoch=0; log=[]; validation_seconds=0.; physics_seconds=0.
    completed_epoch=0; updates=0; previous_elapsed=0.
    state_path=output/'resume_state.pt'
    identity={k:asset[k] for k in ['method','seed','datasetHash','trainingCodeHash','trainingConfigHash','normalizationHash','validationManifestHash']}
    physics_device=config.get('physicsLoss',{}).get('device',device)
    identity.update(device=device,width=width,physicsWeight=physics_weight,physicsDevice=physics_device)
    if resume:
        saved=torch.load(state_path,map_location=device,weights_only=False)
        if saved['identity']!=identity: raise ValueError('Resume identity mismatch; preserve old run and use a new version')
        net.load_state_dict(saved['net']); optimizer.load_state_dict(saved['optimizer'])
        rng.bit_generator.state=saved['numpyRng']; torch.set_rng_state(saved['torchRng'].cpu())
        if torch.cuda.is_available() and saved['cudaRng'] is not None: torch.cuda.set_rng_state_all([v.cpu() for v in saved['cudaRng']])
        best=saved['best']; best_key=saved['bestKey']; best_epoch=saved['bestEpoch']; log=saved['log']
        completed_epoch=saved['epoch']; updates=saved['updates']; previous_elapsed=saved['elapsedSeconds']
        validation_seconds=saved['validationSeconds']; physics_seconds=saved['physicsSeconds']; train_tasks=saved['trainTasks']
        deadline=start+max(0,tc['wallHours']*3600-previous_elapsed)
    def save_state(epoch):
        temp=state_path.with_suffix('.tmp')
        torch.save(dict(identity=identity,net=net.state_dict(),optimizer=optimizer.state_dict(),numpyRng=rng.bit_generator.state,
                        torchRng=torch.get_rng_state(),cudaRng=torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
                        best=best,bestKey=best_key,bestEpoch=best_epoch,log=log,epoch=epoch,updates=updates,
                        elapsedSeconds=previous_elapsed+time.perf_counter()-start,validationSeconds=validation_seconds,physicsSeconds=physics_seconds,trainTasks=train_tasks),temp)
        temp.replace(state_path)
    def prediction_loss(ids,generator=None):
        if not diffusion: return (net(X[ids])-Y[ids]).square().mean()
        t=torch.randint(100,(len(ids),),device=device,generator=generator); noise=torch.randn((len(ids),dim),device=device,generator=generator)
        noisy=ab[t,None].sqrt()*Y[ids]+(1-ab[t,None]).sqrt()*noise; phase=t[:,None]/99*torch.pi
        return (net(torch.cat([X[ids],noisy,phase.sin(),phase.cos()],1))-noise).square().mean()
    epoch=completed_epoch
    for epoch in range(completed_epoch+1,tc['epochsMax']+1):
        if time.perf_counter()>=deadline: break
        net.train(); rng.shuffle(train_tasks); losses=[]
        ids=[int(rng.choice(by_task[k])) for k in train_tasks]
        # A small development dataset must still cover the 100 noise levels.
        # Every extra sweep samples conditions equally; expose the update count.
        target=max(len(ids),int(tc.get('stepsPerEpochFloor',0))*tc['batch'])
        while len(ids)<target:
            rng.shuffle(train_tasks); ids.extend(int(rng.choice(by_task[k])) for k in train_tasks)
        ids=ids[:target]
        for off in range(0,len(ids),tc['batch']):
            if time.perf_counter()>=deadline: break
            batch=ids[off:off+tc['batch']]; optimizer.zero_grad(); loss=prediction_loss(batch)
            if 'physloss' in method and epoch>20:
                mark=time.perf_counter(); subset=batch[:min(32,len(batch))]; times=torch.randint(0,30,(len(subset),),device=device)
                noise=torch.randn((len(subset),dim),device=device); noisy=ab[times,None].sqrt()*Y[subset]+(1-ab[times,None]).sqrt()*noise
                phase=times[:,None]/99*torch.pi; pred=net(torch.cat([X[subset],noisy,phase.sin(),phase.cos()],1)); clean=(noisy-(1-ab[times,None]).sqrt()*pred)/ab[times,None].sqrt()
                physical=[]
                for j,i in enumerate(subset):
                    controls=clean[j].to(device=physics_device,dtype=torch.float64)*torch.tensor(ys,dtype=torch.float64,device=physics_device)+torch.tensor(ym,dtype=torch.float64,device=physics_device)
                    physical.append(trajectory_constraint_loss(controls,rows[i]['task'],config['geometryEnvelope'])[0])
                loss=loss+physics_weight*min(1,(epoch-20)/30)*torch.stack(physical).mean().to(device)
                if str(device).startswith('cuda'): torch.cuda.synchronize()
                physics_seconds+=time.perf_counter()-mark
            if not torch.isfinite(loss): raise FloatingPointError('Nonfinite training loss; checkpoint not eligible')
            loss.backward(); nn.utils.clip_grad_norm_(net.parameters(),tc['gradClip'],error_if_nonfinite=True); optimizer.step(); losses.append(float(loss.detach()))
            if any(not torch.isfinite(p).all() for p in net.parameters()): raise FloatingPointError('Nonfinite weights')
            updates+=1
        if epoch%tc['validationEvery']==0 and time.perf_counter()<deadline:
            mark=time.perf_counter(); net.eval(); val_ids=[i for k in val_tasks for i in by_task[k]]
            with torch.no_grad():
                val_loss=float(torch.stack([prediction_loss(by_task[k],torch.Generator(device=device).manual_seed(seed+11)) for k in val_tasks]).mean())
            candidates=[];metadata=[]
            for row in checkpoint_rows:
                for sample_seed in (tc['validationSeeds'] if diffusion else tc['validationSeeds'][:1]):
                    controls,info=sample(net,asset,model_context(row,method),1,sample_seed,deadline)
                    candidates.append(dict(task=row['task'],controls=None if controls is None else controls.tolist(),taskId=row['taskId']))
                    metadata.append(dict(panel=row['panel'],samplingSeed=sample_seed))
            try: checks=validator(candidates,deadline)
            except (TimeoutError,subprocess.TimeoutExpired): break
            if len(checks)!=len(candidates): raise ValueError('Incomplete checkpoint physical validation')
            rate,panel_rates=score_panels(checks,metadata); key=(rate,-val_loss,-epoch)
            elapsed=time.perf_counter()-mark; validation_seconds+=elapsed
            if time.perf_counter()>deadline: break
            log.append(dict(epoch=epoch,updates=updates,loss=float(np.mean(losses)),physicalRate=rate,panelRates=panel_rates,predictionLoss=val_loss,validationSeconds=elapsed,
                            failureReasons={reason:sum(r['reason']==reason for r in checks) for reason in sorted({r['reason'] for r in checks})}))
            if best_key is None or key>best_key:
                best_key=key; best=copy.deepcopy(net.state_dict()); best_epoch=epoch
                torch.save(best,output/'best_validated_weights.pt')
                write(output/'best_checkpoint.json',dict(status='validated_checkpoint',epoch=epoch,physicalRate=rate,predictionLoss=val_loss,datasetHash=asset['datasetHash'],normalizationHash=asset['normalizationHash']))
            write(output/'progress.json',dict(log=log,elapsedSeconds=previous_elapsed+time.perf_counter()-start,bestEpoch=best_epoch,updates=updates))
            save_state(epoch)
            if tc.get('earlyStopping',True) and epoch-best_epoch>=tc['patienceEpochs'] and updates>=tc.get('minimumUpdatesBeforeEarlyStop',0): break
        elif epoch%tc.get('checkpointEvery',10)==0:
            save_state(epoch)
    if updates!=tc['epochsMax']*tc['stepsPerEpochFloor']:
        raise TimeoutError('Training budget interrupted before fixed update budget; resume state preserved')
    if best is None: raise RuntimeError('No physically validated checkpoint completed before deadline')
    net.load_state_dict(best); net.eval()
    asset['layers']=[dict(W=l.weight.detach().cpu().tolist(),b=l.bias.detach().cpu().tolist()) for l in net if isinstance(l,nn.Linear)]
    asset.update(status='trained' if best_key[0]>0 else 'trained_not_qualified',epochsCompleted=epoch,bestEpoch=best_epoch,bestPhysicalRate=best_key[0],parameterCount=sum(p.numel() for p in net.parameters()),device=device,
                 elapsedSeconds=previous_elapsed+time.perf_counter()-start,optimizerUpdates=updates,stepsPerEpochFloor=tc.get('stepsPerEpochFloor',0),validationSeconds=validation_seconds,physicsForwardSeconds=physics_seconds,physicsDevice=physics_device,physicsWeight=physics_weight,log=log,
                 taskEqualWeight=True,deterministicLabelSelection='lowest_original_objective' if not diffusion else None,torchVersion=torch.__version__)
    write(output/'model.json',asset); torch.save(best,output/'weights.pt'); return asset
