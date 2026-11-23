"""Read-only integrity audit of frozen tasks and completed result records."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('directory',type=Path)
    parser.add_argument('--require-fresh-audit',action='store_true')
    args=parser.parse_args()
    root=args.directory; manifest=read(root/'manifest.json'); jobs=read(root/'jobs.json'); protocol=read(root/'protocol.json')
    failures=[]
    def check(condition,message):
        if not condition: failures.append(message)
    cases={c['caseId']:c for c in manifest['cases']}
    check(len(cases)==288,'Expected 288 distinct primary conditions.')
    check(sum(c['mode']=='known' for c in cases.values())==172,'Known count mismatch.')
    check(sum(c['mode']=='unknown' for c in cases.values())==116,'Unknown count mismatch.')
    check(len(set(j['jobId'] for j in jobs))==len(jobs),'Duplicate job IDs.')
    check(protocol['manifestHash']==manifest['hash'],'Manifest/protocol mismatch.')
    check(protocol['modelHash']==manifest['modelHash'],'Model/protocol mismatch.')
    model=root.parents[1]/'remote'/'diffusion'/'formal_v1'/'training'/'model.json'
    if not model.exists(): model=root.parents[1]/'diffusion'/'formal_v1'/'training'/'model.json'
    check(model.exists(),'Frozen model file unavailable for hash audit.')
    if model.exists(): check(hashlib.sha256(model.read_bytes()).hexdigest()==protocol['modelHash'],'Trained model hash changed.')
    nominal_model=manifest['anchors'][0]['model']
    conditions=collections.defaultdict(list)
    for item in cases.values():
        if not item['generationPassed']: continue
        planning=item['planningTask']; truth=item['truthTask']
        check(planning['T']==truth['T'],item['caseId']+': planning/truth duration differs.')
        check(planning['q0']==truth['q0'] and planning['qGoal']==truth['qGoal'],item['caseId']+': endpoint leakage.')
        if item['category']=='physical': conditions[item['groupId']].append(planning['context'])
        if item['mode']=='unknown':
            check(planning['model']==nominal_model,item['caseId']+': unknown planner model or actuator limits changed.')
            check(planning['model']['dynamics']['totalMass']==150,item['caseId']+': unknown planner mass changed.')
            check(planning['context']==truth['context'],item['caseId']+': physics entered diffusion context.')
            known=cases[item['caseId'].removesuffix('_unknown')]
            check(known['planningTask']==truth,item['caseId']+': known/unknown truth mismatch.')
    for group,contexts in conditions.items():
        check(all(x==contexts[0] for x in contexts),group+': physical scan changed diffusion conditions.')
    for item in cases.values():
        if not item['core']: continue
        for repeat in [1,2,3]:
            selected=[j for j in jobs if j['caseId']==item['caseId'] and j['method']=='diff_direct' and j['repeat']==repeat]
            check(sorted(j['count'] for j in selected)==[1,4,8,16,32,64],item['caseId']+': incomplete candidate-count schedule.')
            check(len(set(j['seed'] for j in selected))==1,item['caseId']+': candidate stream seed mismatch.')
    complete=0; passed=0; known_mismatch=0; nonfinite=0; fresh_successes=0
    for job in jobs:
        path=root/'jobs'/job['jobId']/'result.json'
        if not path.exists(): continue
        row=read(path); complete+=1
        check(row['caseId']==job['caseId'] and row['method']==job['method'],job['jobId']+': result mapping mismatch.')
        if row.get('mode')=='known' and row.get('falseAcceptance'): known_mismatch+=1
        records=row.get('solverRecords') or []; records=[records] if isinstance(records,dict) else records
        for record in records:
            check(record.get('commonReference') is True,job['jobId']+': solver did not use common reference.')
            if record.get('passed'):
                check(record['maxEquality']<=1e-6 and record['maxInequality']<=1e-6,job['jobId']+': accepted NLP residual violation.')
        if row.get('passed'):
            passed+=1
            post_file=root/'post_audit'/(job['jobId']+'.json')
            if post_file.exists():
                post=read(post_file)
                accepted=post['passed'] and not post['knownMismatch']
                events=post['eventPassed']; events=[events] if isinstance(events,bool) else events
                if row.get('mode')=='known':
                    accepted=accepted and post['firstPassed'] and all(events)
                if accepted: fresh_successes+=1
                check(accepted,job['jobId']+': fresh-process output audit failed.')
            elif args.require_fresh_audit:
                check(False,job['jobId']+': successful output lacks fresh-process audit.')
            check(row.get('truthPassed') and row.get('nominalPassed') and row.get('withinBudget'),job['jobId']+': invalid success flags.')
            check(0<row['onlineSeconds']<=row['budget'],job['jobId']+': online budget violation.')
            check(row['minClearance']>=.01-1e-8 and row['minSigma']>=.1 and row['maxCondition']<=20,job['jobId']+': clearance or singularity violation.')
            check(row['maxLegViolation']<=1e-6 and row['maxForceViolation']<=1e-6,job['jobId']+': actuator violation.')
            for key,value in row['quality'].items():
                if isinstance(value,(int,float)) and not math.isfinite(value): nonfinite+=1
    check(known_mismatch==0,'Known-model independent audit mismatch occurred.')
    check(nonfinite==0,'Nonfinite accepted quality metrics.')
    result=dict(passed=not failures,scheduled=len(jobs),completed=complete,auditedSuccessfulRecords=passed,
                freshAuditRequired=args.require_fresh_audit,freshAuditedSuccessfulRecords=fresh_successes,
                knownAuditMismatches=known_mismatch,physicalConditionsInvariant=True if not failures else None,
                modelHashVerified=model.exists(),failures=failures)
    (root/'record_integrity.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False)); raise SystemExit(1 if failures else 0)


if __name__=='__main__': main()
