"""Sequential jobs in the existing MATLAB worker; preserve every job status."""
from pathlib import Path
import argparse,json,os,time
ROOT=Path(__file__).resolve().parents[3]
IPC=ROOT/'chapter5/artifacts/performance_probe'
OUT=ROOT/'chapter5/artifacts/control_force_v62'
def state():
    try:return json.loads((IPC/'worker_status.json').read_text(encoding='utf-8-sig'))
    except (OSError,json.JSONDecodeError):return {}
def wait(job=None):
    start=notice=time.monotonic()
    while True:
        value=state()
        if value.get('state') in ['complete','failed'] and (job is None or value.get('id')==job):return value
        if time.monotonic()-notice>30:print('Waiting for',job or value.get('id'),flush=True);notice=time.monotonic()
        if time.monotonic()-start>1800:raise TimeoutError(job)
        time.sleep(1)
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--name',required=True);parser.add_argument('--after-ledger');parser.add_argument('--after-function');parser.add_argument('functions',nargs='+');args=parser.parse_args()
    if args.after_ledger:
        assert args.after_function,'Require the final function in the preceding ledger.'
        start=notice=time.monotonic()
        while True:
            path=OUT/args.after_ledger
            entries=json.loads(path.read_text(encoding='utf-8-sig')) if path.exists() else []
            if entries and entries[-1]['result']['state']=='failed':raise RuntimeError('Previous pipeline failed; do not overwrite its active command.')
            if entries and entries[-1]['function']==args.after_function and entries[-1]['result']['state']=='complete':break
            if time.monotonic()-notice>30:print('Awaiting preceding pipeline final ledger.',flush=True);notice=time.monotonic()
            if time.monotonic()-start>1800:raise TimeoutError(args.after_ledger)
            time.sleep(1)
    progress=OUT/f'{args.name}_status.json';assert not progress.exists(),'Preserve previous dispatch ledger.'
    previous=wait();ledger=[]
    for index,function in enumerate(args.functions):
        assert function.startswith('ch5Speed') and function.isalnum()
        job=f'{args.name}-{index}-{function}';command=dict(id=job,action='run',function=function)
        file=IPC/'worker_command.pending';file.write_text(json.dumps(command),encoding='utf-8');os.replace(file,IPC/'worker_command.json')
        print('Dispatched',function,flush=True);value=wait(job)
        ledger.append(dict(function=function,result=value));progress.write_text(json.dumps(ledger,ensure_ascii=False,indent=2),encoding='utf-8')
        print(function,value['state'],flush=True)
        if value['state']=='failed':raise RuntimeError(value.get('message','MATLAB job failed'))
if __name__=='__main__':main()
