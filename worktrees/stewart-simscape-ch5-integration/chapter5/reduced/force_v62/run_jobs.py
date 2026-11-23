"""Sequential IPC jobs in the already running MATLAB process, no restart."""
from pathlib import Path
import json,os,time

ROOT=Path(__file__).resolve().parents[3]
IPC=ROOT/'chapter5/artifacts/performance_probe'
OUT=ROOT/'chapter5/artifacts/control_force_v62'
jobs=['ch5SpeedForceShortV63','ch5SpeedForceTrainingGeometryV63','ch5SpeedForceLabelAuditV63','ch5SpeedTeacherClosedV64']
progress=[]


def status():
    try:return json.loads((IPC/'worker_status.json').read_text('utf-8-sig'))
    except (OSError,json.JSONDecodeError):return {}


def wait(job=None):
    start=time.monotonic();notice=start
    while True:
        s=status()
        if s.get('state') in ['complete','failed'] and (job is None or s.get('id')==job):return s
        if time.monotonic()-notice>30:print('Waiting for',job or s.get('id'),flush=True);notice=time.monotonic()
        if time.monotonic()-start>1800:raise TimeoutError(job)
        time.sleep(1)


initial=wait()
assert initial['state']=='complete',initial
for function in jobs:
    job='v62-pipeline-'+function
    command=dict(id=job,action='run',function=function)
    temp=IPC/'worker_command.pending';temp.write_text(json.dumps(command),encoding='utf-8');os.replace(temp,IPC/'worker_command.json')
    print('Dispatched',function,flush=True);result=wait(job);progress.append(dict(function=function,result=result))
    (OUT/'pipeline_status.json').write_text(json.dumps(progress,indent=2),encoding='utf-8')
    print(function,result['state'],flush=True)
    if result['state']=='failed':raise RuntimeError(result['message'])
