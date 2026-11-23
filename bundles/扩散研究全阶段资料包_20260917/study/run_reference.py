"""Repeat all eight frozen references with accepted-result checkpoints.

Run only after the primary queue has stopped. No best-value pooling with the
original reference attempts; this is an explicitly separate persistence version.
"""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def read(path):
    try: return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError,ValueError): return None


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8'); os.replace(temporary,path)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run',default='frozen_v1')
    p.add_argument('--matlab',default='D:/matlab2019a/bin/matlab.exe')
    p.add_argument('--deadline',default='2026-09-17T03:46:51+00:00'); a=p.parse_args()
    project=Path(__file__).resolve().parents[1]; source=project/'results'/'study'/a.run
    progress=read(source/'progress.json'); completion=read(source/'supervisor_complete.json')
    if not completion or progress.get('active'): raise RuntimeError('Primary workers have not stopped.')
    queue=[(i+1,j) for i,j in enumerate(read(source/'jobs.json')) if j['purpose']=='reference']
    assert len(queue)==8 and all(j['budget']==600 and j['count']==8 for _,j in queue)
    output=source/'reference_checkpointed_v1'; output.mkdir(exist_ok=True)
    files=[Path(__file__),project/'study'/'launch_reference.py']+sorted((project/'study'/'reference').glob('*.m'))
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    protocol=dict(mainProtocol=read(source/'protocol.json'),sourceHashes=hashes,jobs=[j for _,j in queue],computeDeadlineUtc=a.deadline,
                  note='Same frozen eight cases and 600s budgets, checkpoint-only revision; do not pool with original reference runs.')
    previous=read(output/'protocol.json')
    if previous and previous!=protocol: raise RuntimeError('Reference resume fingerprint changed.')
    write(output/'protocol.json',protocol)
    lock=output/'supervisor.lock'; fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
    os.write(fd,str(os.getpid()).encode()); os.close(fd)
    deadline=dt.datetime.fromisoformat(a.deadline).timestamp()
    pending=[(i,j) for i,j in queue if not (output/j['jobId']/'completion.json').exists()]
    workers={}; handles=[]

    def stop(process):
        if process.poll() is None:
            subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)
            process.wait(timeout=30)

    try:
        while pending or workers:
            now=time.time()
            if now>=deadline:
                for process,_,_,_ in workers.values(): stop(process)
                workers.clear(); break
            while pending and len(workers)<2:
                index,job=pending.pop(0); folder=output/job['jobId']; folder.mkdir(exist_ok=True)
                (folder/'active.json').unlink(missing_ok=True)
                log=open(folder/'matlab.log','a',encoding='utf-8'); handles.append(log)
                expression=("setenv('STEWART_DEPENDENCY_ROOT','D:/XieZhenyu/stewart-simscape-abfd');"
                            "addpath('diffusion','study','study/reference');sgReferenceRun('%s',%d);"%(a.run,index))
                process=subprocess.Popen([a.matlab,'-wait','-singleCompThread','-batch',expression],
                    cwd=project,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                workers[job['jobId']]=(process,job,folder,time.time())
            for key,(process,job,folder,launched) in list(workers.items()):
                active=read(folder/'active.json'); status=None
                if process.poll() is not None:
                    status='worker_process_exit' if process.returncode else 'returned_without_completion'
                elif active and now-active['startedEpoch']>job['budget']+3:
                    stop(process); status='reference_budget_end'
                elif not active and now-launched>90:
                    stop(process); status='startup_timeout'
                if status:
                    if not (folder/'completion.json').exists():
                        write(folder/'completion.json',dict(status=status,jobId=job['jobId'],
                            elapsedSeconds=now-active['startedEpoch'] if active else None,
                            checkpointAvailable=(folder/'checkpoint.mat').exists()))
                    del workers[key]
            finished=sum((output/j['jobId']/'completion.json').exists() for _,j in queue)
            write(output/'progress.json',dict(epoch=time.time(),scheduled=8,completed=finished,
                  active={k:v[0].pid for k,v in workers.items()},deadlineEpoch=deadline))
            if workers: time.sleep(2)
        write(output/'supervisor_complete.json',dict(epoch=time.time(),reason='deadline' if time.time()>=deadline else 'finished'))
    finally:
        for process,_,_,_ in workers.values(): stop(process)
        for log in handles: log.close()
        lock.unlink(missing_ok=True)


if __name__=='__main__': main()
