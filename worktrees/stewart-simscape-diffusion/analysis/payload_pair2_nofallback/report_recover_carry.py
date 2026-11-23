"""Restart only a native MATLAB crash after checkpoint progress; preserve all attempts."""
import subprocess,sys,time
from prepare import HERE,OUT,read,write

def counts():
    return len(list((OUT/'formal/calls').glob('*/result.json'))),len(list((OUT/'carry/evaluations').glob('*.json')))

history=[];stalled=0
for attempt in range(1,11):
    before=counts();write(OUT/'recovery_status.json',dict(active=True,attempt=attempt,before=before,history=history))
    result=subprocess.run([sys.executable,str(HERE/'report_resume_carry.py')])
    after=counts();state=read(OUT/'status.json')
    history.append(dict(attempt=attempt,before=before,after=after,exit=result.returncode,unixTime=time.time(),error=state.get('error','')))
    if result.returncode==0:
        write(OUT/'recovery_status.json',dict(active=False,completed=True,history=history));break
    native='3221226356' in state.get('error','') or 'Heap corruption' in state.get('error','')
    stalled=stalled+1 if after==before else 0
    if not native or stalled>=2:
        write(OUT/'recovery_status.json',dict(active=False,completed=False,history=history));raise SystemExit(result.returncode)
else:
    write(OUT/'recovery_status.json',dict(active=False,completed=False,history=history));raise SystemExit('Native restart limit reached')
