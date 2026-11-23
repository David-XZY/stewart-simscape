import time
import unittest
import numpy as np
import torch
from pg_core import schedule,read
from pg_learning import network,sample,closest_width,validate_dataset
from pg_statistics import paired_bootstrap,allow_m34

class LearningTests(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(1); torch.manual_seed(42)
        self.net=network(161,60)
        self.asset=dict(method='ddpm_basic',inputMean=[0]*99,inputScale=[1]*99,outputMean=[0]*60,outputScale=[1]*60,noiseSchedule=schedule())

    def test_candidate_prefix(self):
        one,_=sample(self.net,self.asset,[0]*99,1,20260919)
        many=[sample(self.net,self.asset,[0]*99,k,20260919)[0] for k in range(1,5)]
        np.testing.assert_array_equal(one,many[0]); self.assertFalse(np.array_equal(many[0],many[1]))

    def test_deadline(self):
        result,info=sample(self.net,self.asset,[0]*99,1,1,deadline=time.perf_counter()-1)
        self.assertIsNone(result); self.assertEqual(info['reason'],'incomplete_candidate')

    def test_export_arithmetic(self):
        x=np.linspace(-.2,.2,161); y=x.copy()
        for layer in self.net:
            if isinstance(layer,torch.nn.Linear): y=layer.weight.detach().numpy().astype(float)@y+layer.bias.detach().numpy()
            else: y=np.tanh(y)
        with torch.no_grad(): reference=self.net(torch.tensor(x,dtype=torch.float32)).numpy()
        np.testing.assert_allclose(y,reference,atol=1e-7,rtol=1e-5)
        self.assertEqual(closest_width(60),305)

    def test_one_regression_candidate(self):
        self.asset['method']='regression'; net=network(99,60)
        result,_=sample(net,self.asset,[0]*99,2,1)
        self.assertIsNone(result)

    def test_no_labels_no_training(self):
        with self.assertRaises(ValueError): validate_dataset({'version':'pg_dataset_v1','rows':[]})

    def test_shared_seed_scene_bootstrap(self):
        rows=[]
        for scene in range(4):
            for task in range(2):
                for repeat in range(3):
                    for method,value in [('ddpm_basic',repeat==0),('retrieval',False)]:
                        rows.append(dict(method=method,B=2,generalizationType='G2',taskId=f'{scene}_{task}',baseSceneId=str(scene),repeat=repeat,finalQualified=value))
        result=paired_bootstrap(rows,'ddpm_basic','retrieval',replicates=200)
        self.assertAlmostEqual(result['difference'],1/3); self.assertEqual(result['tasks'],8); self.assertEqual(result['seedBlocks'],3)
        self.assertGreater(result['ci95'][1]-result['ci95'][0],.5)
        with self.assertRaises(ValueError): paired_bootstrap(rows[1:],'ddpm_basic','retrieval',replicates=10)
        self.assertTrue(allow_m34(.5,.52,.49)); self.assertFalse(allow_m34(.5,.53,.47))

if __name__=='__main__': unittest.main()
