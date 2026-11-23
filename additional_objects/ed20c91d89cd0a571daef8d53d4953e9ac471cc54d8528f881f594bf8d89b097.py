import unittest
import numpy as np
from pg_core import payload, schedule, basis, development_tasks, check_context, reference_scale

class ContractTests(unittest.TestCase):
    def test_inertia(self):
        p=payload('cylinder',[1.2,.15],100)
        np.testing.assert_allclose(p['derived']['comP'],[0,0,.1],atol=1e-15)
        np.testing.assert_allclose(p['derived']['inertiaCOM'],np.diag([5,16.4375,18.8125]),atol=1e-12)
        for shape,d in [('cylinder',[1.3,.1]),('box',[1.2,.3,.25])]:
            self.assertTrue((np.linalg.eigvalsh(payload(shape,d,440)['derived']['inertiaCOM'])>0).all())
        for d in [[0,.1],[np.nan,.1]]:
            with self.assertRaises(ValueError): payload('cylinder',d,100)

    def test_schedule(self):
        s=schedule(); self.assertLess(s['alphaBar'][-1],1e-6)
        self.assertEqual(s['posteriorVariance'][0],0)
        self.assertAlmostEqual(s['posteriorMeanClean'][0],1)
        self.assertEqual(s['posteriorMeanNoisy'][0],0)

    def test_basis(self):
        for n in [16,20,24]:
            b,d,dd,ddd=basis(n,101)
            np.testing.assert_allclose(b.sum(0),1,atol=1e-14)
            for x in [d,dd,ddd]: np.testing.assert_allclose(x.sum(0),0,atol=1e-8)
            cp=np.r_[np.zeros(3),np.linspace(.1,.9,n-6),np.ones(3)]
            self.assertLess(np.max(np.abs((cp@d)[[0,-1]])),1e-10)
            self.assertLess(np.max(np.abs((cp@dd)[[0,-1]])),1e-9)

    def test_frozen_tasks(self):
        m=development_tasks(); self.assertEqual(m,development_tasks()); self.assertEqual(len(m['tasks']),192)
        self.assertEqual(len(set(m['pilotTaskIds'])),16)
        scenes={}
        for t in m['tasks']: scenes.setdefault(t['baseSceneId'],set()).add(t['split'])
        self.assertTrue(all(len(s)==1 for s in scenes.values()))
        self.assertEqual(sum(next(iter(s))=='dev_validation' for s in scenes.values()),8)

    def test_schema_and_resource(self):
        for dim in [93,96,354]:
            with self.assertRaises(ValueError): check_context({'contextSchema':'pg_basic99_v1','inputMean':[0]*dim})
        self.assertEqual(reference_scale(30),600); self.assertIsNone(reference_scale(120))

if __name__=='__main__': unittest.main()
