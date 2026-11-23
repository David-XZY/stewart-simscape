"""Resume unchanged-source acceptance after the independently audited compiler replay.

The failed attempt stays in history and in the acceptance record. Only phases
already passed under the identical root source hash are retained.
"""
from pathlib import Path
import hashlib
import json
import msvcrt
import os
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import campaign
from repair_campaign import PHASES, ADMISSION_BYTES, source_hash, status
from process_resources import snapshot
ART=ROOT/'artifacts'


def main():
    assert not (ART/'freeze.json').exists()
    path=ART/'validation'/'repair_acceptance.json'
    with (ART/'repair.lock').open('r+b') as lock:
        lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
        record=json.loads(path.read_text('utf-8'));expected=record['source_hash']
        assert source_hash()==expected, 'Source changed; completed phases cannot be inherited.'
        assert record['control_revision']==campaign.CONFIG['control_revision']
        assert [p['name'] for p in record['phases']]==['numerics','narrow_nonideal','inference']
        assert all(p['returncode']==0 for p in record['phases'][:2]) and record['phases'][2]['returncode']!=0
        failed=json.loads((ART/'native'/'1791379932536181900.json').read_text('utf-8'))
        replay=json.loads((ART/'validation'/'compiler_replay_2280_31.finish.json').read_text('utf-8'))
        assert replay['returncode']==0 and replay['object_exists']
        assert all(p['exit_code']==0 for p in replay['process_exit_codes'].values())
        successes=[]
        for f in (ART/'native').glob('*.json'):
            value=json.loads(f.read_text('utf-8'))
            if value.get('key')==failed['key'] and value.get('returncode')==0:
                successes.append(value)
        assert len(successes)==1
        succeeded=successes[0]
        for key in ['source_sha256','compiled_source_sha256','key']:
            assert failed[key]==succeeded[key], 'A different compiler source cannot recover this attempt.'
        cached=ART/'native'/(failed['key']+'.obj');actual=ART/'validation'/'compiler_replay_2280_31.obj'
        assert hashlib.sha256(cached.read_bytes()).digest()==hashlib.sha256(actual.read_bytes()).digest()
        record['failed_phase_attempts']=[dict(record['phases'][2],error=record['error'])]
        record['phases']=record['phases'][:2];record.pop('error',None)
        record['recovery']=dict(kind='independently_audited_identical_source_replay',compiler_key=failed['key'],compiled_source_sha256=failed['compiled_source_sha256'],resumed_utc_seconds=time.time(),root_source_changed=False)
        campaign.atomic_json(path,record)
        try:
            for name,expression in PHASES[2:]:
                while snapshot(os.getpid())['available_commit_bytes']<ADMISSION_BYTES:
                    status('waiting_for_memory',phase=name,required_available_commit_bytes=ADMISSION_BYTES)
                    time.sleep(30)
                assert source_hash()==expected
                label=f'repair_acceptance_{name}_recovery1'
                child=campaign.launch_matlab(expression,label)
                phase=dict(name=name,pid=child.pid,attempt=2 if name=='inference' else 1,started_utc_seconds=time.time(),peak_tree_private_bytes=0,peak_compiler_private_bytes=0,minimum_available_commit_bytes=None,log_label=label)
                record['phases'].append(phase)
                with (ART/'validation'/(label+'_memory.jsonl')).open('w',encoding='utf-8') as handle:
                    while True:
                        sample=snapshot(child.pid);handle.write(json.dumps(sample)+'\n');handle.flush()
                        phase['peak_tree_private_bytes']=max(phase['peak_tree_private_bytes'],sample['private_bytes'])
                        phase['peak_compiler_private_bytes']=max([phase['peak_compiler_private_bytes'],*[p['private_bytes'] for p in sample['processes'] if p['name']=='cc1.exe']])
                        before=phase['minimum_available_commit_bytes'];phase['minimum_available_commit_bytes']=sample['available_commit_bytes'] if before is None else min(before,sample['available_commit_bytes'])
                        code=child.poll();status('running',phase=name,workers=[dict(pid=child.pid,returncode=code)],repairs_not_yet_validated=True)
                        campaign.atomic_json(path,record)
                        if code is not None:break
                        time.sleep(5)
                phase['returncode']=code;phase['finished_utc_seconds']=time.time();campaign.atomic_json(path,record)
                assert code==0, f'{name}: recovery acceptance failed; retain all attempts.'
                assert phase['minimum_available_commit_bytes']>=2*2**30
                assert source_hash()==expected
            record['passed']=True;record['finished_utc_seconds']=time.time();campaign.atomic_json(path,record)
            subprocess.run([sys.executable,str(ROOT/'repair_report.py')],cwd=ROOT.parent,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
            subprocess.run([sys.executable,str(ART/'validation'/'audit_repair_comparison.py')],cwd=ROOT.parent,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
            status('completed',repairs_not_yet_validated=False)
            subprocess.run([sys.executable,str(ROOT/'campaign.py'),'--stage','all','--start-stage','development'],cwd=ROOT.parent,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
        except BaseException as error:
            record['error']=str(error);campaign.atomic_json(path,record)
            status('failed',error=str(error),repairs_not_yet_validated=True)
            raise
        finally:
            lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_UNLCK,1)


if __name__=='__main__':main()
