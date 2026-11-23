"""User-requested 10 s extension; unchanged tasks, weights and numerical source."""
import pathlib,sys,os,time,traceback,msvcrt,copy
ROOT=pathlib.Path('D:/XieZhenyu/stewart-simscape-diffusion');P=ROOT/'results/chapter3';OUT=P/'payload_budget10_job_v1'
os.chdir(ROOT);sys.path.insert(0,str(ROOT/'chapter3/payload_generalization'))
from pg_core import read,write,digest
from run_payload import check_frozen,frozen_hashes,matlab,matlab_literal
lock=open(OUT/'worker.lock','a+b');lock.seek(0);lock.write(b'0');lock.flush();lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
def status(stage,**kw):write(OUT/'status.json',dict(status='running',stage=stage,updatedUnix=time.time(),pid=os.getpid(),**kw))
try:
 status('freeze_explicit_budget_extension')
 for name,source_budget,expected in [('main',5,4032),('controls',2,3456),('quality',5,1008)]:
  dest=OUT/(name+'_frozen.json')
  if dest.exists():check_frozen(dest);continue
  source=P/'payload_test_orchestration_v2'/(name+'_frozen.json');original=check_frozen(source);protocol=copy.deepcopy(original)
  protocol.pop('configHash');protocol['scope']='held_out_budget10_user_requested_extension';protocol['primaryComparison']='Supplemental 10 second budget; original G2 B2 primary comparison remains unchanged'
  protocol['extension']=dict(sourceProtocol=str(source),sourceConfigHash=original['configHash'],authorization='Explicit user request: increase test budget to ten seconds and rerun',changedFields=['job.B','job.runId','scope','primaryComparison','extension','configHash'],actualIndependentCalls=True)
  jobs=[]
  for job in original['jobs']:
   if job['B']!=source_budget:continue
   new=copy.deepcopy(job);new['B']=10;new['runId']=digest(['budget10',name,job['spec']['taskId'],job['method'],job['repeat']])[:24];jobs.append(new)
   assert all(new[k]==job[k] for k in job if k not in ['B','runId'])
  assert len(jobs)==expected and len({j['runId'] for j in jobs})==expected
  protocol['jobs']=jobs;assert frozen_hashes(protocol)==original['fileHashes'];protocol['configHash']=digest(protocol);write(dest,protocol);check_frozen(dest)
 write(OUT/'plan.json',dict(main=4032,controls=3456,quality=1008,total=8496,budgetSeconds=10,sourceNumericsUnchanged=True,oldResultsPreserved=True))
 for name in ['main','controls','quality']:
  protocol=OUT/(name+'_frozen.json');check_frozen(protocol);status('benchmark_'+name,plannedCalls=len(read(protocol)['jobs']))
  expr='pgBenchmark('+matlab_literal(protocol)+",'payload_heldout_"+name+"_budget10_v1');"
  logfile=OUT/(name+'_matlab_error.txt')
  matlab("try, "+expr+" catch e, fid=fopen("+matlab_literal(logfile)+",'w'); fprintf(fid,'%s',getReport(e,'extended','hyperlinks','off')); fclose(fid); rethrow(e); end;")
  rows=[read(f) for f in (P/('payload_heldout_'+name+'_budget10_v1')/'calls').glob('*/result.json')];assert len(rows)==len(read(protocol)['jobs'])
  write(OUT/(name+'_completion.json'),dict(calls=len(rows),qualified=sum(r['finalQualified'] for r in rows),completedUnix=time.time()))
 write(OUT/'status.json',dict(status='completed',stage='budget10_calls_complete',updatedUnix=time.time()))
except BaseException as e:
 (OUT/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');write(OUT/'status.json',dict(status='failed',error=str(e),updatedUnix=time.time()));raise
