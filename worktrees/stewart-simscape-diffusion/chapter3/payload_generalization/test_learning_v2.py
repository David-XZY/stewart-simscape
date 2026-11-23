"""Training restart and sampling tests; synthetic validator is never production."""
import copy
import tempfile
import unittest
from pathlib import Path
import numpy as np
import torch
from pg_core import read
from pg_learning import train,project_clean
from run_payload import RESULTS,HERE

class LearningTests(unittest.TestCase):
    def test_projection_is_explicit_and_finite(self):
        x=torch.tensor([-100.,.2,100.])
        self.assertTrue(torch.equal(project_clean(x,{}),x))
        np.testing.assert_allclose(project_clean(x,{'cleanProjection':{'lower':[-1]*3,'upper':[1]*3}}),[-1,.2,1],atol=1e-7)

    def test_resume_replays_uncommitted_updates(self):
        data=RESULTS/'payload_dataset_v1/dataset.json'
        if not data.exists(): self.skipTest('Requires recovered qualified development dataset')
        torch.set_num_threads(1)
        cfg=read(HERE/'protocol_learning_v2.json');cfg['training'].update(epochsMax=30,stepsPerEpochFloor=1,minimumUpdatesBeforeEarlyStop=0,patienceEpochs=100)
        def validator(candidates,deadline):return [dict(passed=False,reason='synthetic_test_only') for _ in candidates]
        calls=[0]
        def interrupted(candidates,deadline):
            calls[0]+=1
            if calls[0]==2:raise RuntimeError('simulated_process_interrupt')
            return validator(candidates,deadline)
        with tempfile.TemporaryDirectory(prefix='pg_resume_test_') as tmp:
            tmp=Path(tmp)
            full=train(data,tmp/'full','regression',91,validator,cfg,device='cpu')
            with self.assertRaisesRegex(RuntimeError,'simulated_process_interrupt'):
                train(data,tmp/'resumed','regression',91,interrupted,cfg,device='cpu')
            resumed=train(data,tmp/'resumed','regression',91,validator,cfg,device='cpu',resume=True)
            self.assertEqual(full['optimizerUpdates'],resumed['optimizerUpdates'])
            self.assertEqual(full['bestEpoch'],resumed['bestEpoch'])
            for a,b in zip(full['layers'],resumed['layers']):
                np.testing.assert_array_equal(a['W'],b['W']);np.testing.assert_array_equal(a['b'],b['b'])
            bad=copy.deepcopy(cfg);bad['training']['learningRate']=.002
            with self.assertRaisesRegex(ValueError,'identity mismatch'):
                train(data,tmp/'resumed','regression',91,validator,bad,device='cpu',resume=True)

if __name__=='__main__':unittest.main()
