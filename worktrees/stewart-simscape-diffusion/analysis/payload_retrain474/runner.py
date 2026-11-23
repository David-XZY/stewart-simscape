"""Resumable 474-label campaign. No test-set access, no IHSID fallback."""
import os
for _v in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:os.environ[_v]='1'
import argparse
import collections
import copy
import hashlib
import subprocess
import sys
import time
import traceback
from common import *

def status(stage,**kw):atomic(OUT/'status.json',dict(status='running',stage=stage,pid=os.getpid(),updatedUnix=time.time(),**kw))

def matlab(expr,log='matlab',timeout=None):
    env=os.environ.copy();env.setdefault('STEWART_DEPENDENCY_ROOT',str(ROOT.parent/'stewart-simscape-abfd'))
    executable=os.environ.get('PG_MATLAB','D:/matlab2019a/bin/matlab.exe')
    with (OUT/(log+'.log')).open('a',encoding='utf-8') as h:
        subprocess.run([executable,'-wait','-singleCompThread','-batch',
            "addpath('chapter3/payload_generalization','analysis/payload_retrain474'); "+expr],cwd=ROOT,env=env,stdout=h,stderr=subprocess.STDOUT,timeout=timeout,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))

def ml(p):return "'"+str(p).replace('\\','/').replace("'","''")+"'"

def prepare():
    if sha(SOURCE/'dataset.json')!=DATA_SHA:raise ValueError('474 dataset changed')
    d=read(SOURCE/'dataset.json');train=[r for r in d['rows'] if r['split']=='train'];val=[r for r in d['rows'] if r['split']=='validation']
    assert len(train)==474 and len(val)==53 and len({r['taskId'] for r in train})==474
    assert all(len(r['controls'])==60 for r in d['rows'])
    manifest=OUT/'validation.json'
    if not manifest.exists():
        from pg_formal_tasks import formal_tasks
        forbidden={physical_identity(spec_from_task(r['task'])) for r in d['rows']+d['validationTasks']}
        pool=[]
        for seed in range(20260927,20260947):
            for s in formal_tasks(600,seed)['tasks']:
                if s['split']!='validation' or physical_identity(s) in forbidden:continue
                index=int(s['taskId'].rsplit('_',1)[1]);s=copy.deepcopy(s)
                s['taskId']=f'p474_{seed}_'+s['taskId'];s['baseSceneId']=f'p474_{seed}_'+s['baseSceneId']
                s['combinationClass']='combination' if index>=3 else 'ordinary'
                s['geometryIdentity']=hashlib.sha256(json.dumps([s['family'],s['q0'],s['qGoal'],s['parameters'][:3]],sort_keys=True).encode()).hexdigest()
                pool.append(s)
        atomic(OUT/'prepare_request.json',dict(dataset=str(SOURCE/'dataset.json'),pool=pool))
        matlab(f'p474Prepare({ml(OUT/"prepare_request.json")},{ml(manifest)});','prepare')
    v=read(manifest);assert len(v['tasks'])==240 and len(v['checkpointRows'])==194
    assert collections.Counter(r['task']['T'] for r in v['tasks'] if r['panel']=='longtime')=={5:24,7.5:24,10:24,12.5:24,15:24}
    train_scenes={r['baseSceneId'] for r in train};assert not train_scenes&{r['task']['baseSceneId'] for r in v['tasks']}
    old={r['taskId']:r for r in d['validationTasks']}
    # R2019a JSON round-trips can shorten doubles. Keep legacy rows verbatim
    # from their frozen source instead of passing them through MATLAB JSON.
    changed=False
    for field in ['tasks','checkpointRows']:
        for i,row in enumerate(v[field]):
            if row['panel']=='legacy':
                exact=dict(old[row['taskId']],panel='legacy')
                changed=changed or row!=exact;v[field][i]=exact
    if changed:atomic(manifest,v)
    files=[SOURCE/'dataset.json',SOURCE/'library.json',manifest]+sorted(HERE.glob('*.py'))+sorted(HERE.glob('*.m'))+[HERE/'protocol.json']
    files+=sorted((ROOT/'chapter3/payload_generalization').glob('*.m'))
    files+=sorted((ROOT/'chapter3/boundary').glob('*.m'))
    files+=sorted((ROOT/'diffusion').glob('*.m'))
    files+=sorted((ROOT/'chapter3').glob('*.m'))
    files+=sorted((ROOT/'src').glob('*.m'))
    library=read(SOURCE/'library.json')
    files += [SOURCE/r['file'] for r in library['records'] if r['split']=='train']
    identities={str(p.relative_to(ROOT)):sha(p) for p in files}
    if (OUT/'inputs.json').exists() and read(OUT/'inputs.json')!=identities:
        if any((OUT/'models').glob('*/resume_state.pt')) or any((OUT/'calls').glob('*/*/result.json')):
            raise ValueError('Frozen campaign source changed; preserve and version the experiment')
        atomic(OUT/'preflight_input_revisions'/(str(time.time_ns())+'.json'),read(OUT/'inputs.json'))
    atomic(OUT/'inputs.json',identities)
    atomic(OUT/'preflight.json',dict(passed=True,trainLabels=474,validationLabels=53,validationTasks=240,checkpointTasks=194,batch=64,stepsPerEpoch=64,expectedUpdates=19200))
    return v

def validator(candidates,deadline):
    tag=str(time.time_ns());folder=OUT/'checkpoint_validation';folder.mkdir(exist_ok=True)
    inp=folder/(tag+'_input.json');out=folder/(tag+'_output.json');atomic(inp,candidates)
    matlab(f'pgCheckpointValidate({ml(inp)},{ml(out)});','checkpoint',max(.01,deadline-time.perf_counter()))
    return read(out)

def train_all():
    import torch
    import platform
    torch.set_num_threads(1)
    from training import train
    assert read(OUT/'runtime_verification_status.json')['status']=='passed'
    assert read(OUT/'training_resume_tests.json')['passed']
    assert read(OUT/'diagnostic_tests.json')['passed']
    config=read(HERE/'protocol.json');models=[]
    assert torch.cuda.is_available(),'CUDA training worker required'
    atomic(OUT/'environment.json',dict(host=platform.node(),python=sys.version,torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(),cpuThreads=1,onlineDevice='cpu',trainingDevice='cuda'))
    for method in LEARNED:
        for seed in SEEDS:
            key=f'{method}_{seed}';folder=OUT/'models'/key;path=folder/'model.json';status('training',model=key)
            if not path.exists():
                train(SOURCE/'dataset.json',folder,method,seed,validator,config,resume=(folder/'resume_state.pt').exists(),validation_manifest=OUT/'validation.json')
            a=read(path);assert a['optimizerUpdates']==19200 and a['batchSize']==64 and a['datasetHash']==DATA_SHA
            assert a['validationManifestHash']==sha(OUT/'validation.json') and a['trainingCodeHash']==sha(HERE/'training.py')
            models.append(dict(key=key,path=str(path),method=method,seed=seed,sha=sha(path),weightsSha=sha(folder/'weights.pt')))
    atomic(OUT/'frozen_models.json',models);return models

def units(v,models):
    result=[]
    for ti,row in enumerate(v['tasks']):
        for rep,sampling in enumerate(SAMPLE_SEEDS):
            block=[]
            for method in METHODS:
                chosen=[m for m in models if m['method']==method] if method in LEARNED else [dict(key='',seed=0)]
                for m in chosen:
                    uid=hashlib.sha256(f"{row['taskId']}|{method}|{m['seed']}|{sampling}".encode()).hexdigest()[:24]
                    block.append(dict(unitId=uid,spec=spec_from_task(row['task']),panel=row['panel'],method=method,modelKey=m['key'],trainingSeed=m['seed'],samplingSeed=sampling,repeat=rep))
            # Rotate method blocks rather than running all diffusion calls first.
            offset=(ti+rep)%len(METHODS);order=METHODS[offset:]+METHODS[:offset]
            block.sort(key=lambda u:order.index(u['method']));result.extend(block)
    return result

def execute_jobs(name,jobs,models,identity):
    if not jobs:return []
    request=OUT/'requests'/(name+'.json');folder=OUT/'calls'/name
    atomic(request,dict(identity=identity,models=models,library=str(SOURCE/'library.json'),output=str(folder),jobs=jobs))
    matlab(f'p474Batch({ml(request)});',name)
    rows=[read(folder/j['id']/'result.json') for j in jobs]
    assert all(r['identity']==identity for r in rows);return rows

def compare(v,models):
    identity=hashlib.sha256(json.dumps([read(OUT/'inputs.json'),models],sort_keys=True).encode()).hexdigest()
    all_units=units(v,models);assert len(all_units)==10080
    for B in [2,5,10]:
        status('finite_comparison',budget=B)
        jobs=[dict(u,id=u['unitId'],B=B,sliceBudget=B,previousState='') for u in all_units]
        execute_jobs(f'finite_{B}',jobs,models,identity)
    # Success is absorbing only within this separate cumulative campaign.
    for method in METHODS:
        path=OUT/'unlimited'/(method+'.json')
        state=read(path) if path.exists() else dict(identity=identity,method=method,allowance=20,history=[],units={},status='running')
        if state['identity']!=identity:raise ValueError('Unlimited resume mismatch')
        if state['status']=='completed':continue
        selected=[u for u in all_units if u['method']==method]
        while True:
            allowance=state['allowance'];status('unlimited_comparison',method=method,allowance=allowance)
            jobs=[];expanded=True
            for u in selected:
                prev=state['units'].get(u['unitId'])
                if prev and (prev['finalQualified'] or prev['searchExhausted']):continue
                used=prev['cumulativeSeconds'] if prev else 0
                if used>=allowance:expanded=False;continue
                jobs.append(dict(u,id=u['unitId'],B=allowance,sliceBudget=allowance-used,previousState=prev['statePath'] if prev else ''))
            rows=execute_jobs(f'unlimited_{method}_{allowance}',jobs,models,identity)
            for row in rows:state['units'][row['unitId']]=row
            successful=sum(x['finalQualified'] for x in state['units'].values())
            terminal=len(state['units'])==len(selected) and all(x['finalQualified'] or x['searchExhausted'] for x in state['units'].values())
            state['history'].append(dict(allowance=allowance,successes=successful,expanded=expanded,units=len(selected),active=sum(not(x['finalQualified'] or x['searchExhausted']) for x in state['units'].values())))
            stop,reason=plateau(state['history'],terminal);state['stopReason']=reason
            state['status']='completed' if stop else 'running';state['allowance']=allowance if stop else allowance*2;atomic(path,state)
            if stop:break

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',choices=['prepare','train','compare','all'],default='all');p.add_argument('--worker',action='store_true');args=p.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    if args.worker:
        sys.stdout=(OUT/'stdout.log').open('a',encoding='utf-8',buffering=1);sys.stderr=(OUT/'stderr.log').open('a',encoding='utf-8',buffering=1)
    lock=(OUT/'worker.lock').open('a+b');lock.seek(0)
    if os.name=='nt':
        import msvcrt
        msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    try:
        status('preparing');v=prepare()
        if args.stage=='prepare':status('preparation_completed');return
        models=train_all() if args.stage!='compare' else read(OUT/'frozen_models.json')
        if args.stage=='train':status('training_completed');return
        compare(v,models)
        from report import finish
        status('failure_diagnosis');finish(v,models,matlab)
        atomic(OUT/'status.json',dict(status='completed',stage='report_complete',updatedUnix=time.time(),models=12,testSetUsed=False))
    except BaseException:
        atomic(OUT/'status.json',dict(status='failed',updatedUnix=time.time(),error=traceback.format_exc()));raise
    finally:lock.close()

if __name__=='__main__':main()
