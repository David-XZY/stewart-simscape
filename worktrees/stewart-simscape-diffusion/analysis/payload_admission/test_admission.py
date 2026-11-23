import unittest
from analyze_admission import select

class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.rows=[dict(labelId=f'attempt_{i:05d}',objective=float(4-i)) for i in range(1,4)]
        self.d={frozenset(('attempt_00001','attempt_00002')):.1,
                frozenset(('attempt_00001','attempt_00003')):.4,
                frozenset(('attempt_00002','attempt_00003')):.3}

    def test_threshold_equality_is_duplicate(self):
        actual=select(self.rows,self.d,.1,4)
        self.assertEqual([x['labelId'] for x in actual],['attempt_00001','attempt_00003'])

    def test_farthest_starts_with_best_objective(self):
        actual=select(self.rows,self.d,.1,2,'quality_farthest')
        self.assertEqual([x['labelId'] for x in actual],['attempt_00003','attempt_00001'])

    def test_exact_duplicate_does_not_pass_zero_threshold(self):
        actual=select(self.rows,{k:0. for k in self.d},0.,8)
        self.assertEqual(len(actual),1)

    def test_singleton(self):
        self.assertEqual(select(self.rows[:1],{},.15,4),self.rows[:1])

if __name__=='__main__':unittest.main()
