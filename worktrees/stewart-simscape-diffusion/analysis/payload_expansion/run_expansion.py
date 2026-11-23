"""Hidden worker; resumable per-condition checkpoints; no training/test execution."""
import json
import os
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[2]
NAME='payload_condition_expansion_v2_longtime_20260926'

def main():
    out=ROOT/'results/chapter3'/NAME
    out.mkdir(parents=True,exist_ok=True)
    def update(**kw):
        temp=out/'worker_status.tmp'
        temp.write_text(json.dumps(dict(updatedUnix=time.time(),**kw)),encoding='utf-8')
        temp.replace(out/'worker_status.json')
    workers=8
    update(status='running',pid=os.getpid(),workers=workers)
    env=os.environ.copy();env['STEWART_DEPENDENCY_ROOT']='D:/XieZhenyu/stewart-simscape-abfd'
    for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:env[k]='1'
    children=[]; handles=[]
    try:
        for index in range(1,workers+1):
            stdout=(out/f'stdout_worker_{index}.log').open('a',encoding='utf-8')
            stderr=(out/f'stderr_worker_{index}.log').open('a',encoding='utf-8'); handles.extend([stdout,stderr])
            children.append(subprocess.Popen(['D:/matlab2019a/bin/matlab.exe','-wait','-singleCompThread','-nodesktop','-nosplash','-batch',
                f"addpath('analysis/payload_expansion'); pgExpandDataset({index},{workers});"],cwd=ROOT,env=env,stdout=stdout,stderr=stderr,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0)))
            if index==1:
                # pgConfig writes a shared provenance file on first use.
                # Initialize it before admitting other workers to that folder.
                while not (out/'identity.json').exists():
                    if children[0].poll() is not None:raise RuntimeError('Initial MATLAB worker failed before identity creation')
                    time.sleep(1)
        while any(p.poll() is None for p in children):
            if any(p.poll() not in (None,0) for p in children):raise RuntimeError('A MATLAB worker failed; inspect worker stderr logs')
            time.sleep(2)
        if any(p.returncode!=0 for p in children):raise RuntimeError('MATLAB worker failed')
        update(status='completed',pid=os.getpid())
    except BaseException:
        for child in children:
            if child.poll() is None:
                subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],capture_output=True,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        update(status='failed',error=traceback.format_exc());raise
    finally:
        for handle in handles:handle.close()

if __name__=='__main__':main()
