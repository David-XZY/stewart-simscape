"""Real optimizer/RNG restart test; mocked physical validator is labeled explicitly."""
import copy
import tempfile
import numpy as np
import torch
from common import *
import training

def main():
    torch.set_num_threads(1)
    config=read(HERE/'protocol.json');config['training'].update(epochsMax=2,stepsPerEpochFloor=8,validationEvery=1,checkpointEvery=1)
    original=training.sample
    training.sample=lambda net,asset,context,*args,**kwargs:(np.array(asset['outputMean']),dict(reason='test_stub'))
    records=[]
    try:
        parent=OUT/'verification_runs';parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as temp:
            folder=Path(temp)
            for method in ['ddpm_basic','regression','ddpm_signature','regression_signature']:
                calls=[]
                def validator(candidates,deadline):
                    calls.append(len(candidates));return [dict(passed=True,reason='test_stub') for c in candidates]
                kwargs=dict(dataset=SOURCE/'dataset.json',method=method,seed=20260919,validator=validator,config=config,device='cpu',validation_manifest=OUT/'validation.json')
                uninterrupted=folder/(method+'_full');training.train(output=uninterrupted,**kwargs)
                count=0
                def interrupt(candidates,deadline):
                    nonlocal count
                    count+=1
                    if count==2:raise RuntimeError('intentional_test_interruption')
                    return validator(candidates,deadline)
                resumed=folder/(method+'_resume')
                try:training.train(output=resumed,**dict(kwargs,validator=interrupt))
                except RuntimeError as e:assert str(e)=='intentional_test_interruption'
                else:raise AssertionError('Expected interruption')
                training.train(output=resumed,resume=True,**kwargs)
                a=torch.load(uninterrupted/'resume_state.pt',weights_only=False);b=torch.load(resumed/'resume_state.pt',weights_only=False)
                assert a['updates']==b['updates']==16
                assert all(torch.equal(a['net'][k],b['net'][k]) for k in a['net'])
                assert torch.equal(a['torchRng'],b['torchRng'])
                assert a['numpyRng']==b['numpyRng']
                assert all(n==(582 if method.startswith('ddpm') else 194) for n in calls)
                records.append(dict(method=method,updates=16,bitwiseIdenticalResume=True,physicalValidator='mocked_for_optimizer_test_only'))
    finally:training.sample=original
    atomic(OUT/'training_resume_tests.json',dict(passed=True,records=records,scope='CPU_optimizer_RNG_restart_not_physical_or_performance_evidence'))
    print('Four model optimizer/RNG resume tests passed')

if __name__=='__main__':main()
