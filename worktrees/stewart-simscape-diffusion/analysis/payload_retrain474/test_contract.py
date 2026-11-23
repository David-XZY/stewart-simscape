import unittest
from common import plateau,score_panels,read,HERE

class ContractTests(unittest.TestCase):
    def test_batch_budget(self):
        t=read(HERE/'protocol.json')['training']
        self.assertEqual(t['batch']*t['stepsPerEpochFloor'],4096)
        self.assertEqual(t['epochsMax']*t['stepsPerEpochFloor'],19200)
        self.assertFalse(t['earlyStopping'])

    def test_panel_equal_weight_not_row_weight(self):
        score,rates=score_panels([{'passed':True}]*2+[{'passed':False}]*8,[{'panel':'legacy'}]*2+[{'panel':'longtime'}]*8)
        self.assertEqual(score,.5)
        self.assertEqual(rates,{'legacy':1,'longtime':0})

    def test_incomplete_validation_rejected(self):
        with self.assertRaises(ValueError):score_panels([], [{'panel':'legacy'}])

    def test_plateau_minimum_and_three_expansions(self):
        h=[dict(allowance=b,successes=5) for b in [20,40,80]]
        self.assertFalse(plateau(h,False)[0])
        self.assertTrue(plateau(h+[dict(allowance=160,successes=5)],False)[0])
        self.assertFalse(plateau(h+[dict(allowance=160,successes=6)],False)[0])
        self.assertTrue(plateau([],True)[0])

if __name__=='__main__':unittest.main()
