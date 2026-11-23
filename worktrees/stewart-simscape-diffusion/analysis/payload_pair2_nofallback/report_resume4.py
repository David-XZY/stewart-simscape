"""User-authorized scope amendment: remove IHSID jobs, preserve numerical identity."""
import collections, os, sys, time, traceback, subprocess
from pathlib import Path
from prepare import ROOT,HERE,OUT,read,write,sha,digest
import runner

def amend():
    parent=read(OUT/'protocol.json')
    scope=dict(parent)
    scope['methods']=[m for m in parent['methods'] if m!='ihsid']
    scope['jobs']=[j for j in parent['jobs'] if j['method']!='ihsid']
    assert len(scope['methods'])==4 and len(scope['jobs'])==8760
    assert set(collections.Counter((j['method'],j['B']) for j in scope['jobs']).values())=={438}
    amendment=dict(reason='User requested removal of independent IHSID tests; all other settings unchanged',
        parentConfigHash=parent['configHash'],perCallIdentityUnchanged=True,
        methods=scope['methods'],retainedRunIds=[j['runId'] for j in scope['jobs']],
        retiredRunIds=[j['runId'] for j in parent['jobs'] if j['method']=='ihsid'],
        resumeSourceHashes={p.name:sha(p) for p in [HERE/'report_resume4.py',HERE/'report_summarize4.py']})
    amendment['scopeHash']=digest(amendment)
    path=OUT/'amendment_no_ihsid.json'
    if path.exists():assert read(path)==amendment,'Amended scope already frozen'
    else:write(path,amendment)
    scope['scopeHash']=amendment['scopeHash'];scope['scopeAmendment']='remove_independent_ihsid'
    # Keep configHash as the unchanged per-call identity; scopeHash identifies this subset.
    write(OUT/'protocol_four.json',scope)
    return parent,scope

def main():
    parent,p=amend();runner.preflight(parent)
    assert read(OUT/'verification/verification.json')['passed']
    assert read(OUT/'pilot/audit.json')['passed']
    assert read(OUT/'freeze.json')['configHash']==p['configHash']
    completed=sum((OUT/'formal/calls'/j['runId']/'result.json').is_file() for j in p['jobs'])
    request=dict(p,phase='formal',root=str(ROOT),output=str(OUT/'formal'))
    request.pop('pilotJobs',None);path=OUT/'formal_four_request.json';write(path,request)
    runner.status('formal_four_running',total=8760,completedAtResume=completed,scopeHash=p['scopeHash'],resumedUnix=time.time())
    runner.matlab(f'p2Batch({runner.ml(path)});','formal',True)
    runner.preflight(parent);runner.audit(p,'formal',p['jobs'])
    from report_summarize4 import summarize
    summarize()
    runner.status('four_completed',calls=8760,scopeHash=p['scopeHash'])
    subprocess.run([sys.executable,str(HERE/'report_figures4.py')],check=True)

if __name__=='__main__':
    import msvcrt
    lock=(OUT/'worker.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    try:main()
    except BaseException:
        runner.status('failed',error=traceback.format_exc());raise
    finally:lock.close()
