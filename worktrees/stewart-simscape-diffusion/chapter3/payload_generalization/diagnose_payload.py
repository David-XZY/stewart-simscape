"""Reproduce the original failure, paired stabilization ablation, production parity."""
import copy
import json
import math
import time
from pathlib import Path
import numpy as np
import torch
from pg_core import read,write
from pg_learning import network,sample,model_context
from run_payload import RESULTS,matlab,matlab_literal,checkpoint_validator

def run():
    torch.set_num_threads(1)
    out=RESULTS/'payload_diagnosis_v2';out.mkdir(exist_ok=True)
    data=read(RESULTS/'payload_dataset_v1/dataset.json'); train=[r for r in data['rows'] if r['split']=='train']
    val=sorted(data['validationTasks'],key=lambda r:r['taskId'])
    folder=RESULTS/'payload_models_v1/ddpm_basic_20260919';a=read(folder/'model.json')
    net=network(161,60);net.load_state_dict(torch.load(folder/'weights.pt',map_location='cpu',weights_only=True));net.eval()
    Y=(np.array([r['controls'] for r in train])-a['outputMean'])/a['outputScale'];radius=np.maximum(1.,1.5*np.abs(Y).max(0))
    clipped=copy.deepcopy(a);clipped['cleanProjection']=dict(kind='train_standardized_envelope',lower=(-radius).tolist(),upper=radius.tolist(),source='training_labels_only',physicalCertificate=False)
    candidates=[];meta=[]
    for variant,asset in [('original',a),('projection_only_same_weights',clipped)]:
        for r in val:
            z,_=sample(net,asset,r['context'],1,20260936)
            candidates.append(dict(task=r['task'],controls=z.tolist(),taskId=r['taskId']))
            meta.append(dict(variant=variant,taskId=r['taskId'],standardizedAbsMax=float(np.max(np.abs((z-a['outputMean'])/a['outputScale'])))))
    for r in data['rows']:
        candidates.append(dict(task=r['task'],controls=r['controls'],taskId=r['taskId']))
        meta.append(dict(variant='qualified_label_roundtrip',taskId=r['taskId']))
    checks=checkpoint_validator(candidates,time.perf_counter()+900)
    for m,c in zip(meta,checks):m.update(c)
    # Same explicit tensor; independent framework precision, genuine MATLAB caller.
    noise=np.random.default_rng(731).normal(size=(100,60)); row=val[0]
    oracle=dict(task=row['task'],context=row['context'],model=clipped,noise=noise.tolist())
    ip=out/'sampler_input.json';op=out/'sampler_output.json';write(ip,oracle)
    matlab(f'pgSamplingAudit({matlab_literal(ip)},{matlab_literal(op)});')
    py,_=sample(net,clipped,row['context'],1,20260936,noise=noise)
    error=float(np.max(np.abs(py-read(op)['controls'])))
    z1,_=sample(net,clipped,row['context'],3,719);sample(net,clipped,row['context'],1,719);z2,_=sample(net,clipped,row['context'],3,719)
    prefix=bool(np.array_equal(z1,z2))
    results={}
    for variant in sorted({m['variant'] for m in meta}):
        group=[m for m in meta if m['variant']==variant]
        results[variant]=dict(count=len(group),passed=sum(m['passed'] for m in group),reasons={r:sum(m['reason']==r for m in group) for r in sorted({m['reason'] for m in group})})
    report=dict(status='completed',scope='development_validation_only',summary=results,records=meta,productionSamplerMaxAbsoluteError=error,randomPrefixStable=prefix,
                terminalCleanAmplification=1/math.sqrt(a['noiseSchedule']['alphaBar'][-1]),trainStandardizedAbsMax=float(np.abs(Y).max()),
                originalOptimizerUpdates=a['epochsCompleted'],trainConditionCount=len(train),explanation='Original small-set training performed one optimizer update per epoch. Unbounded epsilon inversion amplifies high-noise prediction error. Projection is a separate disclosed train-only sampler ablation, not a feasibility certificate.')
    report['passed']=error<1e-4 and prefix and results['qualified_label_roundtrip']['passed']==len(data['rows'])
    write(out/'diagnosis_report.json',report)
    assert error<1e-4 and prefix,'Production sampler parity/prefix failure'
    assert results['qualified_label_roundtrip']['passed']==len(data['rows']),'Qualified label roundtrip failed'
    print(json.dumps({k:v for k,v in report.items() if k!='records'}))

if __name__=='__main__':run()
