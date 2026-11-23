"""Hidden single-process materialization on the established Windows worker."""
import json
import os
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[2]
AUDIT='payload_admission_v2_20260926'
DATASET='payload_formal_dataset_admission_v2_20260926'

def main():
    out=ROOT/'results/chapter3'/AUDIT
    status=out/'materialize_status.json'
    def update(**kw):
        temp=status.with_suffix('.tmp');temp.write_text(json.dumps(dict(updatedUnix=time.time(),**kw)),encoding='utf-8');temp.replace(status)
    update(status='running',dataset=DATASET)
    env=os.environ.copy()
    env['STEWART_DEPENDENCY_ROOT']='D:/XieZhenyu/stewart-simscape-abfd'
    for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:env[k]='1'
    expr=("addpath('analysis/payload_admission'); pgAdmissionMaterialize("
          "'results/chapter3/payload_formal_dataset_v1/dataset.json',"
          f"'results/chapter3/{AUDIT}','results/chapter3/{AUDIT}/selection.json','{DATASET}');")
    try:
        with (out/'materialize_stdout.log').open('w',encoding='utf-8') as stdout,(out/'materialize_stderr.log').open('w',encoding='utf-8') as stderr:
            subprocess.run(['D:/matlab2019a/bin/matlab.exe','-wait','-singleCompThread','-nodesktop','-nosplash','-batch',expr],
                           cwd=ROOT,env=env,stdout=stdout,stderr=stderr,check=True,
                           creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        update(status='completed',dataset=DATASET)
    except BaseException:
        update(status='failed',error=traceback.format_exc());raise

if __name__=='__main__':main()
