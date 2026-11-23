"""Wait for the bounded solve batch, then retain raw-safe trajectories and package."""
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
    def update(**kw):
        t=out/'finish_status.tmp';t.write_text(json.dumps(dict(updatedUnix=time.time(),**kw)),encoding='utf-8');t.replace(out/'finish_status.json')
    update(status='waiting_for_solve_batch',pid=os.getpid())
    try:
        while True:
            state=json.loads((out/'worker_status.json').read_text(encoding='utf-8'))
            if state['status']=='failed':raise RuntimeError(state['error'])
            if state['status']=='completed':break
            time.sleep(10)
        update(status='raw_physical_library',pid=os.getpid())
        env=os.environ.copy();env['STEWART_DEPENDENCY_ROOT']='D:/XieZhenyu/stewart-simscape-abfd'
        for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:env[k]='1'
        with (out/'finish_stdout.log').open('w',encoding='utf-8') as stdout,(out/'finish_stderr.log').open('w',encoding='utf-8') as stderr:
            subprocess.run(['D:/matlab2019a/bin/matlab.exe','-wait','-singleCompThread','-nodesktop','-nosplash','-batch',
                "addpath('analysis/payload_expansion'); pgExpansionRetimeRecovery(); pgExpansionRawLibrary();"],cwd=ROOT,env=env,stdout=stdout,stderr=stderr,check=True,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            subprocess.run(['D:/XieZhenyu/envs/stewart-abfd/Scripts/python.exe',str(ROOT/'analysis/payload_expansion/verify_and_package.py')],
                cwd=ROOT,env=env,stdout=stdout,stderr=stderr,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        update(status='completed',pid=os.getpid())
    except BaseException:
        update(status='failed',error=traceback.format_exc());raise

if __name__=='__main__':main()
