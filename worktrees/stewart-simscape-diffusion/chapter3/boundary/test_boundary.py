import copy
import unittest
from run_boundary import make_tasks,validate_tasks,protocol_jobs,canonical,quality_caps,validation_jobs


class ExperimentContractTests(unittest.TestCase):
    def setUp(self):
        self.manifest=make_tasks(dict(a=[-.02,.02],b=[-.02,.02],psi=[-.1745,.1745],T=[3,10]))

    def test_frozen_counts_and_determinism(self):
        self.assertEqual(validate_tasks(self.manifest),dict(train=600,validation=100,test_regular=100,test_combination=100))
        self.assertEqual(self.manifest,make_tasks(self.manifest['domain']))

    def test_combo_holdout(self):
        for t in self.manifest['tasks']:
            _,_,psi,duration=t['parameters']; holdout=abs(psi)>=.75*.1745 and duration<=4.75
            self.assertEqual(holdout,t['split']=='test_combination')

    def test_no_group_leak(self):
        bad=copy.deepcopy(self.manifest); bad['tasks'][0]['groupId']=bad['tasks'][-1]['groupId']; bad['hash']=canonical(bad['tasks'])
        with self.assertRaises(ValueError): validate_tasks(bad)

    def test_tamper_rejected(self):
        self.manifest['tasks'][0]['parameters'][0]+=.01
        with self.assertRaises(ValueError): validate_tasks(self.manifest)

    def test_actual_call_matrix(self):
        jobs=protocol_jobs([t for t in self.manifest['tasks'] if t['split'].startswith('test')])
        self.assertEqual(len(jobs),18000); self.assertEqual(len({j['runId'] for j in jobs}),18000)
        self.assertEqual(sum(j['B'] for j in jobs),66600)

    def test_validation_only_caps(self):
        rows=[dict(taskId='a',split='validation',passed=True,rawQuality=dict(positionDetour=1,rotationDetour=2)),
              dict(taskId='b',split='validation',passed=True,rawQuality=dict(positionDetour=3,rotationDetour=4))]
        self.assertEqual(quality_caps(rows),[4.5,6])
        rows[0]['split']='test_regular'
        with self.assertRaises(ValueError): quality_caps(rows)

    def test_repeats_are_not_new_tasks(self):
        tasks=[self.manifest['tasks'][0]]; jobs=protocol_jobs(tasks)
        self.assertEqual(len({j['taskId'] for j in jobs}),1)
        self.assertEqual(len(jobs),90)

    def test_validation_search_is_group_balanced_and_test_free(self):
        result=validation_jobs(self.manifest['tasks'],{},'methods')
        self.assertEqual(len(result['jobs']),1080)
        self.assertEqual(len({j['groupId'] for j in result['jobs']}),20)
        self.assertTrue(all(j['split']=='validation' and j['B']==2 for j in result['jobs']))
        self.assertEqual(len({j['runId'] for j in result['jobs']}),1080)
        with self.assertRaises(ValueError): validation_jobs(self.manifest['tasks'],{},'pipeline')
        pipeline=validation_jobs(self.manifest['tasks'],{},'pipeline',{})
        self.assertEqual(len(pipeline['jobs']),1080)


if __name__=='__main__': unittest.main()
