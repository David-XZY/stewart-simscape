"""Finalize the existing report once the authorized teacher batch is materialized."""
import json,time,subprocess,sys
from pathlib import Path
from report_v82 import ROOT,BASE

def refresh():
    subprocess.run([sys.executable,str(Path(__file__).with_name('report_v82.py'))],check=True)

def run():
    start=time.monotonic();result=BASE/'teacher_pilot_v82/dataset_report.json'
    status=ROOT/'chapter5/artifacts/performance_probe/worker_status.json'
    while not result.exists():
        try:worker=json.loads(status.read_text(encoding='utf-8-sig'))
        except (OSError,json.JSONDecodeError):worker={}
        if worker.get('id','').startswith('teacher_pilot_v82') and worker.get('state')=='failed':
            refresh();raise RuntimeError('Teacher collector failed; status and partial results remain available.')
        if time.monotonic()-start>7200:
            refresh();raise TimeoutError('Teacher materialization did not finish within two hours.')
        time.sleep(5)
    refresh()
    print('Teacher collection status refreshed in implementation report.',flush=True)

if __name__=='__main__':run()
