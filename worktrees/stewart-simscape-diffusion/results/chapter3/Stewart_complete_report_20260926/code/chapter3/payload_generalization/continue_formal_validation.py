"""Resumable single-worker FORMAL-DATA VALIDATION continuation. Never reads formal test labels.

Run using pythonw through Win32_Process.Create to detach from SSH. Each completed
stage is immutable; in-flight training resumes from an atomic optimizer/RNG state.
The numerical benchmark has its own committed per-call resume records.
"""
from __future__ import annotations
import argparse
import collections
import hashlib
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
for _thread_variable in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:
    os.environ[_thread_variable]='1'
from pg_core import read,write,digest
from run_payload import HERE,ROOT,RESULTS,matlab,matlab_literal,checkpoint_validator,freeze,file_hash

SEEDS=[20260919,20260920,20260921]
JOB=RESULTS/'payload_formal_validation_job_v1'
MODELS=RESULTS/'payload_formal_models_v1'
DATA=RESULTS/'payload_formal_dataset_v1/dataset.json'
LIBRARY=RESULTS/'payload_formal_dataset_v1/library.json'
MANIFEST=RESULTS/'payload_formal_preparation_v1/train_validation_tasks.json'
CONFIG=HERE/'protocol_learning_v2.json'

def atomic(path,value):
    path=Path(path); temp=path.with_suffix(path.suffix+'.tmp');write(temp,value);temp.replace(path)

class Continuation:
    def __init__(self):
        self.state=dict(status='running',pid=os.getpid(),startedUnix=time.time(),scope='formal_training_validation_only_no_test_access',completed=[])
        if (JOB/'status.json').exists():self.state['previousStatus']=read(JOB/'status.json').get('status')
        self.models={};self.data=read(DATA)
        self.specs=[s for s in read(MANIFEST)['tasks'] if s['split']=='validation']
        representatives={}
        for s in sorted(self.specs,key=lambda x:x['taskId']):representatives.setdefault((s['family'],s['taskClass'],s['shape']),s['taskId'])
        self.representatives=set(representatives.values())
        assert len(self.specs)==120 and len(self.data['validationTasks'])==120, 'Full validation coverage required'
        assert {s['taskId'] for s in self.specs}=={r['taskId'] for r in self.data['validationTasks']}, 'Validation identity mismatch'
        self.identities=dict(dataset=file_hash(DATA),library=file_hash(LIBRARY),manifest=file_hash(MANIFEST),config=file_hash(CONFIG))
        if (JOB/'inputs.json').exists() and read(JOB/'inputs.json')!=self.identities:raise ValueError('Continuation input changed; create a new experiment version')
        atomic(JOB/'inputs.json',self.identities)

    def status(self,stage,**extra):
        self.state.update(stage=stage,updatedUnix=time.time(),**extra);atomic(JOB/'status.json',self.state)

    def training(self,method,seed,weight=.1,config=CONFIG,suffix=''):
        key=f'{method}_{seed}{suffix}';folder=MODELS/key;model=folder/'model.json'
        self.status('training_'+key)
        if not model.exists():
            args=[sys.executable,str(HERE/'run_payload.py'),'train','--method',method,'--dataset',str(DATA),'--output',str(folder),'--seed',str(seed),'--config',str(config),'--physics-weight',str(weight)]
            if (folder/'resume_state.pt').exists():args.append('--resume')
            elif folder.exists():raise RuntimeError(f'Uncommitted training directory without resume state: {folder}; preserve and inspect')
            subprocess.run(args,cwd=ROOT,check=True)
        asset=read(model)
        if asset['datasetHash']!=file_hash(DATA) or asset.get('trainingConfigHash')!=digest(read(config)) or asset['method']!=method or asset['seed']!=seed:raise ValueError('Existing model provenance mismatch')
        if asset.get('trainingCodeHash')!=file_hash(HERE/'pg_learning.py'):raise ValueError('Training code changed; use a new version')
        self.models[key]=model;self.state['completed'].append(key);self.status('trained_'+key)
        return key,model

    def direct(self,key,path):
        """Entire 120-condition validation, retaining illegal endpoints."""
        out=JOB/'direct'/(key+'.json')
        if out.exists():
            result=read(out)
            if result['modelHash']!=file_hash(path):raise ValueError('Validation model identity changed')
            return result
        self.status('direct_validation_'+key)
        import numpy as np
        import torch
        from pg_learning import network,sample,model_context
        torch.set_num_threads(1);a=read(path);dim=len(a['outputMean']);inp=len(a['inputMean'])+(0 if a['method'].startswith('regression') else dim+2)
        net=network(inp,dim,a['width']);net.load_state_dict(torch.load(path.parent/'weights.pt',map_location='cpu',weights_only=True));net.eval()
        candidates=[];metadata=[]
        for row in self.data['validationTasks']:
            z,info=sample(net,a,model_context(row,a['method']),1,a['seed']+17)
            candidates.append(dict(task=row['task'],taskId=row['taskId'],controls=z.tolist()))
            t=row['task'];metadata.append(dict(taskId=row['taskId'],stratum=t['family']+'_'+t['taskClass']+'_'+t['payload']['shape'],generationSeconds=info['seconds']))
        checks=checkpoint_validator(candidates,time.perf_counter()+1800)
        for m,c in zip(metadata,checks):m.update(c)
        groups=collections.defaultdict(list)
        for r in metadata:groups[r['stratum']].append(int(r['passed']))
        result=dict(scope='formal_data_validation',modelHash=file_hash(path),model=key,count=len(metadata),passed=sum(r['passed'] for r in metadata),stratumEqualRate=float(np.mean([np.mean(v) for v in groups.values()])),records=metadata)
        atomic(out,result);return result

    def benchmark(self,name,entries,budgets=(2,),eta=.03,signature=False):
        """Each budget is a distinct real call, all methods interleaved per block."""
        self.status('benchmark_'+name)
        protocol_file=JOB/'protocols'/(name+'.json');run='payload_formal_'+name+'_v1'
        if not protocol_file.exists():
            identity=JOB/'matlab_identity.json';matlab(f'pgWriteCodeIdentity({matlab_literal(identity)});')
            jobs=[];models=[];seen=set()
            for e in entries:
                if e.get('key') and e['key'] not in seen:
                    seen.add(e['key']);models.append(dict(key=e['key'],file=str(e['file'])))
            if not models:raise ValueError('Benchmark entries must include a trained model')
            for ti,spec in enumerate(self.specs):
                for bi,B in enumerate(budgets):
                    repeats=sorted({e['repeat'] for e in entries})
                    for repeat in repeats:
                        block=[e for e in entries if e['repeat']==repeat];offset=(ti+bi+repeat)%len(block);block=block[offset:]+block[:offset]
                        for e in block:
                            rid=digest([name,spec['taskId'],B,e['method'],repeat])[:24]
                            jobs.append(dict(runId=rid,spec=spec,B=B,method=e['method'],returnPolicy='physical',modelKey=e.get('key',''),trainingSeed=e.get('seed',0),samplingSeed=SEEDS[repeat],repeat=repeat,preselectedRepresentative=spec['taskId'] in self.representatives))
            selected=read(RESULTS/'payload_formal_validation_selection_v1/validation_selection.json')
            request=dict(experimentVersion='payload_generalization_v1',scope='formal_data_validation',matlabCodeHash=read(identity)['matlabCodeHash'],controlCount=self.data['controlCount'],models=models,jobs=jobs,
                         qualityCaps=read(RESULTS/'payload_formal_dataset_v1/quality_caps.json'),validationSelection=selected,tasksFile=str(MANIFEST),datasetFile=str(DATA),libraryFile=str(LIBRARY),guidanceEta=eta,integrationAuditFile=str(RESULTS/'payload_formal_integration_v1/integration.json'))
            req=protocol_file.with_suffix('.request.json');write(req,request);freeze(req,protocol_file)
        subprocess.run([sys.executable,str(HERE/'run_payload.py'),'benchmark','--protocol',str(protocol_file),'--run',run],cwd=ROOT,check=True)
        rows=[read(p) for p in (RESULTS/run/'calls').glob('*/result.json')]
        expected=len(read(protocol_file)['jobs'])
        if len(rows)!=expected:raise ValueError('Incomplete call set')
        summary={}
        for method in sorted({r['method'] for r in rows}):
            rr=[r for r in rows if r['method']==method];groups=collections.defaultdict(list)
            for r in rr:
                s=r['job']['spec'];groups[s['family']+'_'+s['taskClass']+'_'+s['shape']].append(bool(r['finalQualified']))
            summary[method]=dict(calls=len(rr),qualified=sum(r['finalQualified'] for r in rr),stratumEqualRate=sum(sum(v)/len(v) for v in groups.values())/len(groups),overBudget=sum(not r['returnedWithinBudget'] for r in rr))
        atomic(JOB/(name+'_summary.json'),dict(run=run,scope='formal_data_validation',summary=summary));return summary

    def entry(self,method,seed,key=None,path=None):
        return dict(method=method,seed=seed,repeat=SEEDS.index(seed),key=key or '',file=path)

    def execute(self):
        diag=read(RESULTS/'payload_diagnosis_v2/diagnosis_report.json')
        if not diag.get('passed'):raise ValueError('Failure diagnosis audit not passed')
        audit=read(RESULTS/'payload_followup_audit_v2/tests.json')
        if not audit.get('passed') or not read(RESULTS/'payload_followup_audit_v2/penalty_audit.json').get('passed'):raise ValueError('Solver acceptance or parameterized penalty audit failed')
        selection=RESULTS/'payload_formal_validation_selection_v1/validation_selection.json'
        if not selection.exists():
            self.status('rerun_retrieval_selection_after_acceptance_fix')
            subprocess.run([sys.executable,str(HERE/'run_payload.py'),'select-retrieval','--library',str(LIBRARY),'--manifest',str(MANIFEST),'--run','payload_formal_validation_selection_v1'],cwd=ROOT,check=True)
        # Full same-seed ablation isolates training effort from sampling projection.
        # Update-only diagnostic already completed in development; do not repeat.
        base=[];base_rates={};signature_rates={};physics_rates={}
        for seed in SEEDS:
            for method in ['ddpm_basic','regression']:
                k,p=self.training(method,seed);rate=self.direct(k,p)
                if method=='ddpm_basic':base_rates[seed]=rate['stratumEqualRate']
                base.append(self.entry(method,seed,k,p))
            base.extend([self.entry('ihsid',seed),self.entry('retrieval',seed)])
        self.benchmark('basic_validation',base)
        # Feature and capacity controls are chosen before any formal test use.
        enhanced=[]
        for method in ['ddpm_signature','ddpm_signature_no_accel','ddpm_signature_no_gap','ddpm_basic_wide','regression_signature']:
            for seed in SEEDS:
                k,p=self.training(method,seed);rate=self.direct(k,p);enhanced.append(self.entry(method,seed,k,p))
                if method=='ddpm_signature':signature_rates[seed]=rate['stratumEqualRate']
        # Compare in-denoising and end-only guidance for each validation eta.
        guidance_scores=[]
        for eta in [.01,.03,.1]:
            path=MODELS/f'ddpm_basic_{SEEDS[0]}'/'model.json';key=f'ddpm_basic_{SEEDS[0]}'
            entries=[self.entry(m,SEEDS[0],key,path) for m in ['ddpm_guided','ddpm_end_guided']]
            result=self.benchmark('guidance_eta_'+str(eta).replace('.','p'),entries,eta=eta)
            guidance_scores.append(dict(eta=eta,score=result['ddpm_guided']['stratumEqualRate']))
        chosen_eta=max(guidance_scores,key=lambda x:(x['score'],-x['eta']))['eta']
        # CPU float64 penalty (measured faster on this small grid); GPU network.
        choices=[]
        for weight in [.01,.1,1.]:
            suffix='_lambda_'+str(weight).replace('.','p')
            k,p=self.training('ddpm_physloss',SEEDS[0],weight=weight,suffix=suffix)
            result=self.direct(k,p);choices.append(dict(weight=weight,key=k,file=str(p),score=result['stratumEqualRate']))
        chosen=max(choices,key=lambda x:(x['score'],-x['weight']))
        for seed in SEEDS:
            if seed==SEEDS[0]:k,p=chosen['key'],Path(chosen['file'])
            else:k,p=self.training('ddpm_physloss',seed,weight=chosen['weight'])
            rate=self.direct(k,p);physics_rates[seed]=rate['stratumEqualRate'];enhanced.append(self.entry('ddpm_physloss',seed,k,p))
        mean=lambda d:sum(d.values())/len(d)
        ds=mean(signature_rates)-mean(base_rates);dp=mean(physics_rates)-mean(base_rates)
        combine=min(ds,dp)>=-.02 and max(ds,dp)>=.02
        atomic(JOB/'validation_selection.json',dict(scope='formal_data_validation',guidance=guidance_scores,selectedEta=chosen_eta,physicsWeights=choices,selectedPhysicsWeight=chosen['weight'],signatureDifference=ds,physicsDifference=dp,combinedTriggered=combine))
        if combine:
            for seed in SEEDS:
                k,p=self.training('ddpm_signature_physloss',seed,weight=chosen['weight']);self.direct(k,p);enhanced.append(self.entry('ddpm_signature_physloss',seed,k,p))
        for seed in SEEDS:
            path=MODELS/f'ddpm_basic_{seed}'/'model.json';key=f'ddpm_basic_{seed}'
            enhanced.extend([self.entry(m,seed,key,path) for m in ['ddpm_guided','ddpm_end_guided']])
            enhanced.append(self.entry('retrieval_signature',seed))
        self.benchmark('enhanced_validation',base+enhanced,budgets=(2,5),eta=chosen_eta)
        self.status('validation_complete_test_protocol_freeze_pending',status='completed',formalTestExperimentsStarted=False)

def main():
    p=argparse.ArgumentParser();p.add_argument('--worker',action='store_true');args=p.parse_args()
    JOB.mkdir(exist_ok=True,parents=True)
    # Inherited disk handles survive SSH closure; WMI supplies the independent parent.
    if args.worker:
        sys.stdout=open(JOB/'stdout.log','a',encoding='utf-8',buffering=1);sys.stderr=open(JOB/'stderr.log','a',encoding='utf-8',buffering=1)
    for name in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:os.environ[name]='1'
    lock=open(JOB/'worker.lock','a+b');lock.seek(0)
    if os.name=='nt':
        import msvcrt
        msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    runner=None
    try:
        runner=Continuation();runner.execute()
    except BaseException as e:
        if runner:runner.status('failed',status='failed',failureType=type(e).__name__,failureReason=str(e))
        else:atomic(JOB/'status.json',dict(status='failed',failureType=type(e).__name__,failureReason=str(e)))
        traceback.print_exc();raise
    finally:lock.close()

if __name__=='__main__':main()
