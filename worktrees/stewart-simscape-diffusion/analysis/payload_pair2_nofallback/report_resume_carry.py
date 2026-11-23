"""Budget coverage protocol: carry audited successes; never synthesize measured calls."""
import collections,json,sys,time,traceback,subprocess
from pathlib import Path
from prepare import ROOT,HERE,OUT,read,write,sha,digest
import runner

def key(row):return row['taskId'],row['method'],row['repeat']
def qualified(r):
    return bool(r['finalQualified'] and r['onlinePassed'] and r['returnedWithinBudget'] and r['independentAuditPassed'] and (not r.get('fineAuditSelected') or r['fineAuditPassed']))

def prepare():
    p=read(OUT/'protocol.json');runner.preflight(p)
    old=read(OUT/'ihsid_baseline_1_2_5_10.json');assert len(old)==1752
    jobs={(j['spec']['taskId'],j['method'],j['repeat'],j['B']):j for j in p['jobs']}
    for r in old:
        j=jobs[key(r)+(r['B'],)];assert j['spec']==r['job']['spec'] and r['samplingSeed']==j['samplingSeed'] and r['trainingSeed']==j['trainingSeed']
        assert r['codeHash']==p['originalCodeHash'] and r['libraryHash']==p['libraryHash']
        assert r['method']=='ihsid' and r['B'] in [1,2,5,10]
        if r['finalQualified']:
            assert qualified(r) and Path(r['trajectoryRef']).is_file(),'Missing historical successful trajectory'
    scope=dict(p)
    rule=dict(parentConfigHash=p['configHash'],rule='ascending_budget_absorbing_finalQualified_same_task_method_repeat_seeds',
       budgets=[1,2,5,10,20],historicalIHSIDBudgets=[1,2,5,10],historicalIHSIDHash=sha(OUT/'ihsid_baseline_1_2_5_10.json'),
       historicalIHSIDSelection='Always original booklet records for 1/2/5/10, independent of newer wins/losses',
       archivedHigherBudgetMeasurements='Preserved; coverage uses earliest qualifying budget even when a later actual call exists',
       timing='Source single-call time retained and labeled; not a new measurement; not cumulative failed-attempt cost',
       fineAudit='Original actual source-run audit selection; no fictional new run/audit selection for inherited cells',
       code={f:sha(HERE/f) for f in ['p3CarryBatch.m','report_resume_carry.py','report_summarize_carry.py']})
    rule['scopeHash']=digest(rule)
    f=OUT/'amendment_carry.json'
    if f.exists():assert read(f)==rule,'Carry protocol already frozen'
    else:write(f,rule)
    scope['scopeHash']=rule['scopeHash'];scope['scopeAmendment']='carry_shorter_success_and_restore_ihsid20'
    write(OUT/'protocol_carry.json',scope)
    return scope,old

def request(p,path,view,jobs,phase='formal',test=False):
    r=dict(p,root=str(ROOT),phase=phase,output=str(OUT/'formal'),viewOutput=str(view),jobs=jobs,
      ihsidBaselineFile=str(OUT/'ihsid_baseline_1_2_5_10.json'),expectedNoNewMeasurements=test)
    r.pop('pilotJobs',None);write(path,r)

def audit(p):
    views=[read(OUT/'carry/evaluations'/(j['runId']+'.json')) for j in p['jobs']]
    counts=collections.Counter();origins=collections.Counter();groups=collections.defaultdict(list)
    for j,v in zip(p['jobs'],views):
        s=v['sourceRow'];assert v['scopeHash']==p['scopeHash'] and v['evaluationId']==j['runId']
        assert key(s)==(j['spec']['taskId'],j['method'],j['repeat']) and s['job']['spec']==j['spec']
        assert s['trainingSeed']==j['trainingSeed'] and s['samplingSeed']==j['samplingSeed']
        assert v['sourceBudget']==s['B']<=j['B'] and v['sourceRunId']==s['runId']
        assert v['inherited']==(s['B']<j['B']) and v['usesMeasurementAtTargetBudget']==(not v['inherited'])
        assert v['inherited'] or v['targetBudgetWasMeasured']
        assert not v['inherited'] or qualified(s)
        assert v['finalQualified']==s['finalQualified']
        assert not s['finalQualified'] or qualified(s)
        assert abs(sum(e['seconds'] for e in s['events'])+s['otherSeconds']-s['actualReturnTime'])<1e-6
        if j['method']!='ihsid': assert s['rankingCalls']==s['ihsidCalls']==0
        if v['origin']=='new_measurement' and j['method']=='ihsid':assert j['B']==20
        counts[j['method'],j['B']]+=1;origins[v['origin']]+=1;groups[key(s)].append(v)
    assert len(views)==10950 and set(counts.values())=={438}
    for items in groups.values():
        items.sort(key=lambda x:x['targetBudget']);seen=False
        for v in items:
            if seen:assert v['finalQualified'] and v['inherited']
            seen=seen or v['finalQualified']
    measured=[v for v in views if not v['inherited']]
    result=dict(passed=True,logicalCells=len(views),inherited=sum(v['inherited'] for v in views),
       uniqueReferencedCalls=len({v['sourceRunId'] for v in views}),noninheritedCells=len(measured),actualTargetBudgetRecords=sum(v['targetBudgetWasMeasured'] for v in views),
       retainedNewBatchRawCalls=len(list((OUT/'formal/calls').glob('*/result.json'))),origins=dict(origins),scopeHash=p['scopeHash'])
    write(OUT/'carry/audit.json',result);return result

def main():
    p,old=prepare();assert read(OUT/'verification/verification.json')['passed'] and read(OUT/'pilot/audit.json')['passed']
    selected=next(r for r in old if r['B']==5 and r['finalQualified'])
    testJobs=[j for j in p['jobs'] if (j['spec']['taskId'],j['method'],j['repeat'])==key(selected)]
    testFile=OUT/'carry_test_request.json';request(p,testFile,OUT/'carry/test_evaluations',testJobs,test=True)
    mainFile=OUT/'carry_request.json';request(p,mainFile,OUT/'carry/evaluations',p['jobs'])
    runner.status('carry_running',logicalCells=10950,scopeHash=p['scopeHash'],resumedUnix=time.time())
    runner.matlab(f'p3CarryBatch({runner.ml(testFile)}); p3CarryBatch({runner.ml(mainFile)});','formal',True)
    tv=[read(OUT/'carry/test_evaluations'/(j['runId']+'.json')) for j in testJobs]
    assert [v['inherited'] for v in tv]==[False,False,False,True,True]
    write(OUT/'carry/verification.json',dict(passed=True,tests=['failed_budget_not_propagated','first_finalQualified_success_inherited','IHSID20_skipped_after_success','no_new_measurement_for_inherited_cell'],views=len(tv)))
    result=audit(p)
    from report_summarize_carry import summarize
    summarize();runner.status('carry_completed',**result)
    subprocess.run([sys.executable,str(HERE/'report_figures_carry.py')],check=True)

if __name__=='__main__':
    from pathlib import Path
    import msvcrt
    lock=(OUT/'worker.lock').open('a+b');lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    try:main()
    except BaseException:
        runner.status('failed',error=traceback.format_exc());raise
    finally:lock.close()
