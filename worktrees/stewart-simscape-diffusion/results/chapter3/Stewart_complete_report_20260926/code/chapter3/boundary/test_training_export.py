"""Synthetic arithmetic smoke test only; never used as experiment labels."""
from pathlib import Path
import tempfile
import unittest
import importlib.util
from run_boundary import read,write


@unittest.skipUnless(importlib.util.find_spec('torch'),'Numerical host has PyTorch')
class TrainingExportTests(unittest.TestCase):
    def test_two_models_export_and_probe(self):
        import numpy as np
        import torch
        from train_boundary import train
        torch.set_num_threads(1); rng=np.random.default_rng(123)
        with tempfile.TemporaryDirectory(prefix='boundary_training_unit_') as directory:
            root=Path(directory); dataset=root/'synthetic_unit_only.json'
            data=dict(kind='synthetic-unit-test-not-experiment',x=rng.normal(size=(12,93)).tolist(),y=rng.normal(size=(12,60)).tolist(),
                      split=['train']*8+['validation']*4,groups=['train_group']*8+['val_group']*4,
                      taskIds=['t1']*4+['t2']*4+['v1']*2+['v2']*2)
            write(dataset,data)
            for method in ['M3_N','M4']:
                model=train(dataset,root/method,method,123,'cpu',2)
                probe=read(root/method/'network_probe.json'); value=np.asarray(probe['input']).T
                for i,layer in enumerate(model['layers']):
                    value=np.asarray(layer['W'])@value+np.asarray(layer['b'])[:,None]
                    if i+1<len(model['layers']): value=np.tanh(value)
                self.assertLess(np.max(np.abs(value-np.asarray(probe['output']).T)),2e-6)
                self.assertTrue(model['taskEqualWeight']); self.assertEqual(model['controlCount'],16)
            data['split'][-1]='test_regular'; write(dataset,data)
            with self.assertRaises(ValueError): train(dataset,root/'forbidden','M4',123,'cpu',1)


if __name__=='__main__': unittest.main()
