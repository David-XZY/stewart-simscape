"""Single MATLAB worker; verified inputs, pilot gate, per-call watchdog and resume."""
import os
for name in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']: os.environ[name]='1'
import argparse, collections, json, platform, subprocess, sys, time, traceback
from pathlib import Path
from prepare import ROOT,HERE,OUT,read,write,sha

def status(stage,**kw): write(OUT/'status.json',dict(stage=stage,updatedUnix=time.time(),pid=os.getpid(),**kw))
def ml(p): return "'"+str(p).replace('\\','/').replace("'","''")+"'"
def preflight(p):
    assert platform.node().upper()=='DESKTOP-3E5N8AV','Wrong workstation'
    for rel,h in p['sourceHashes'].items(): assert sha(ROOT/rel)==h,f'Source differs: {rel}'
    for m in p['models']: assert sha(ROOT/m['relativeFile'])==m['sha256'],f'Model differs: {m["key"]}'
    assert sha(ROOT/'results/chapter3/payload_formal_dataset_v1/dataset.json')==p['datasetHash']
    assert sha(ROOT/'results/chapter3/payload_formal_dataset_v1/library.json')==p['libraryHash']
    assert len(p['jobs'])==10950 and len({j['runId'] for j in p['jobs']})==10950
    assert set(collections.Counter((j['method'],j['B']) for j in p['jobs']).values())=={438}
    assert not {e['taskId'] for e in p['exclusions']}&{j['spec']['taskId'] for j in p['jobs']}
    write(OUT/'preflight.json',dict(passed=True,configHash=p['configHash'],host=platform.node(),python=sys.version,
       models=12,sourceFiles=len(p['sourceHashes']),cpuThreads=1,onlineDevice='CPU',environment={k:os.environ[k] for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']}))

def request(p,phase,jobs):
    folder=OUT/phase;folder.mkdir(parents=True,exist_ok=True)
    r=dict(p);r.update(phase=phase,root=str(ROOT),output=str(folder),jobs=jobs)
    r.pop('pilotJobs',None);path=OUT/(phase+'_request.json');write(path,r);return path

def matlab(expression,phase,watchdog=False):
    folder=OUT/phase;folder.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['STEWART_DEPENDENCY_ROOT']=str(ROOT.parent/'stewart-simscape-abfd')
    cmd=[os.environ.get('PG_MATLAB','D:/matlab2019a/bin/matlab.exe'),'-wait','-singleCompThread','-batch',
         "addpath('chapter3/payload_generalization','analysis/payload_pair2_nofallback'); "+expression]
    # stdout is durable; the watchdog owns only this newly spawned process tree.
    with (folder/'matlab.log').open('a',encoding='utf-8') as log:
        process=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        launched=time.time(); heartbeat_file=folder/'heartbeat.json'
        while process.poll() is None:
            time.sleep(.2)
            if watchdog and heartbeat_file.exists():
                h=read(heartbeat_file)
                if h['unixTime']>=launched and h['stage']=='online' and time.time()-h['unixTime']>h['B']+30:
                    write(folder/'watchdog_failure.json',dict(heartbeat=h,pid=process.pid,observedUnix=time.time()))
                    subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,check=False)
                    process.wait(timeout=30)
                    raise RuntimeError('B+30s online watchdog; incomplete attempt preserved; campaign stopped for diagnosis')
        if process.returncode: raise RuntimeError(f'MATLAB {phase} exit {process.returncode}; see {folder}/matlab.log')

def audit(p,phase,jobs):
    rows=[]
    for j in jobs:
        path=OUT/phase/'calls'/j['runId']/'result.json';r=read(path)
        assert r['configHash']==p['configHash'] and r['job']==j
        assert (path.parent/'trajectory.mat').is_file()
        assert r['rankingCalls']==r['rankingSeconds']==0
        if j['method']!='ihsid':assert r['ihsidCalls']==r['ihsidSeconds']==0
        assert not r['finalQualified'] or (r['onlinePassed'] and r['independentAuditPassed'] and r['returnedWithinBudget'] and (not r['fineAuditSelected'] or r['fineAuditPassed']))
        assert abs(sum(e['seconds'] for e in r['events'])+r['otherSeconds']-r['actualReturnTime'])<1e-6
        assert r['otherSeconds']>=-1e-6
        if phase=='pilot': assert r['reason']!='boundary_rejected'
        blocks=r['repairBlocks'];blocks=[blocks] if isinstance(blocks,dict) else blocks
        assert all(b['iterations']<=40 for b in blocks)
        if j['method'].startswith('ddpm'): assert len({b['candidateId'] for b in blocks})==len(blocks)
        if j['method'].startswith('regression'): assert r['candidateGenerated']<=1
        events=r['candidateEvents'];events=[events] if isinstance(events,dict) else events
        generated=[e for e in events if e['event']=='generated']
        assert len({e['id'] for e in generated})==len(generated)
        for e in events:
            if e['event']=='checked': assert any(g['id']==e['id'] and g['complete'] and g['batchComplete'] for g in generated)
        rows.append(r)
    counts=collections.Counter((r['method'],r['B']) for r in rows)
    write(OUT/phase/'audit.json',dict(passed=True,calls=len(rows),cells=[dict(method=m,B=b,calls=n) for (m,b),n in counts.items()],
       successes=sum(r['finalQualified'] for r in rows),onlineSeconds=sum(r['actualReturnTime'] for r in rows),offlineSeconds=sum(r['offlineAuditSeconds'] for r in rows)))
    return rows

def run(stage):
    p=read(OUT/'protocol.json');status('preflight');preflight(p)
    path=request(p,'verification',p['pilotJobs'])
    if not (OUT/'verification/verification.json').exists():
        status('flow_and_numerical_tests');matlab(f'p2Verify({ml(path)});','verification')
    assert read(OUT/'verification/verification.json')['passed']
    if stage=='verify':status('verification_complete');return
    path=request(p,'pilot',p['pilotJobs']);status('pilot_running',total=40)
    matlab(f'p2Batch({ml(path)});','pilot',True);rows=audit(p,'pilot',p['pilotJobs'])
    startup=read(OUT/'pilot/startup.json');assert startup['threads']==1 and startup['casadi']=='3.7.2'
    assert '2019a' in startup['matlab']
    mean=sum(r['actualReturnTime']+r['offlineAuditSeconds'] for r in rows)/len(rows)
    write(OUT/'pilot/estimate.json',dict(pilotMeanSeconds=mean,roughHours=mean*10950/3600,
        note='Four validation tasks and 2/20 second budgets only; rough extrapolation, not a completion deadline.'))
    if stage=='pilot':status('pilot_complete',calls=40);return
    frozen=dict(configHash=p['configHash'],status='frozen',pilotAudit=sha(OUT/'pilot/audit.json'),verification=sha(OUT/'verification/verification.json'),frozenUnix=time.time())
    if (OUT/'freeze.json').exists():assert read(OUT/'freeze.json')['configHash']==p['configHash']
    else:write(OUT/'freeze.json',frozen)
    preflight(p);path=request(p,'formal',p['jobs']);status('formal_running',total=10950)
    matlab(f'p2Batch({ml(path)});','formal',True);audit(p,'formal',p['jobs'])
    from summarize import summarize
    summarize();status('completed',calls=10950)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stage',choices=['verify','pilot','all'],default='all');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    lock=(OUT/'worker.lock').open('a+b');lock.seek(0)
    import msvcrt
    msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    try:run(args.stage)
    except BaseException:
        status('failed',error=traceback.format_exc());raise
    finally:lock.close()
