"""Synthetic fixtures are isolated in temporary directories, never experiment records."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class ReportTests(unittest.TestCase):
    def test_failures_missing_times_and_closure(self):
        with tempfile.TemporaryDirectory(prefix='c3_report_test_') as directory:
            root=Path(directory)
            jobs=[dict(jobId='fixture_ok',caseId='fixture',method='diff_fast',purpose='main',repeat=1,count=64),
                  dict(jobId='fixture_fail',caseId='fixture',method='geo_ihsid',purpose='main',repeat=1,count=8),
                  dict(jobId='scan_ok',caseId='scan',method='diff_fast',purpose='main',repeat=1,count=64),
                  dict(jobId='scan_fail',caseId='scan',method='geo_ihsid',purpose='main',repeat=1,count=8)]
            standard=dict(hash='fixture',primary=1,weightNames=['balanced','smooth','economy'])
            for name,value in [('jobs',jobs),('standard',standard),('manifest',{'cases':[]})]:
                (root/(name+'.json')).write_text(json.dumps(value))
            row=dict(jobs[0],completed=True,passed=True,category='id',groupId='fixture',status='passed',
                physicalPassed=True,timingMeasured=True,onlineSeconds=3,generationSeconds=2,evaluationSeconds=1,
                timing=dict(boundary=.1,generation=.5,screening=.1,physical=.5,quality=.3,repair=0,solverBuild=0,solver=0,other=1.5),
                auditPassed=True,standardHash='fixture',score={'scores':[50,51,49],'primaryScore':50,'groups':[1,1,1,1]},acceptedStage='direct')
            folder=root/'jobs/fixture_ok'; folder.mkdir(parents=True); (folder/'result.json').write_text(json.dumps(row))
            failure=dict(jobs[1],completed=True,passed=False,category='id',groupId='fixture',status='runtime_exit',timingMeasured=False)
            folder=root/'jobs/fixture_fail'; folder.mkdir(); (folder/'result.json').write_text(json.dumps(failure))
            for job,source in [(jobs[2],row),(jobs[3],failure)]:
                item=dict(source,**job); item.update(category='duration',rootGroupId='anchor_fixture',parameter='duration',value=3)
                folder=root/'jobs'/job['jobId']; folder.mkdir(); (folder/'result.json').write_text(json.dumps(item))
            script=Path(__file__).with_name('summarize_chapter3.py')
            result=subprocess.run([sys.executable,str(script),str(root)],capture_output=True,text=True,encoding='utf-8')
            self.assertEqual(result.returncode,0,result.stderr)
            summary=json.loads((root/'report/summary.json').read_text()); self.assertEqual(summary['qualified'],2)
            self.assertEqual(summary['mainFigureCalls'],2)
            self.assertEqual(summary['statuses']['runtime_exit'],2); self.assertFalse(summary['integrityErrors'])
            with (root/'report/parameter_scans.csv').open(encoding='utf-8-sig',newline='') as stream:
                scans=list(csv.DictReader(stream))
            self.assertEqual(len(scans),2)
            self.assertEqual(next(r for r in scans if r['method']=='geo_ihsid')['qualifiedGenerationMedian'],'')
            with (root/'report/paired_summary.csv').open(encoding='utf-8-sig',newline='') as stream:
                paired=list(csv.DictReader(stream))
            self.assertEqual(len(paired),2)
            self.assertTrue(all(r['diffusionOnly']=='1' and r['bothQualified']=='0' and r['netSecondsSaved.median']=='' for r in paired))
            self.assertTrue((root/'report/success_and_time.png').exists())
            row['onlineSeconds']=4; (root/'jobs/fixture_ok/result.json').write_text(json.dumps(row))
            result=subprocess.run([sys.executable,str(script),str(root)],capture_output=True,text=True,encoding='utf-8')
            self.assertNotEqual(result.returncode,0)
            summary=json.loads((root/'report/summary.json').read_text()); self.assertEqual(summary['integrityErrors'][0]['problem'],'timing_closure')


if __name__=='__main__': unittest.main()
