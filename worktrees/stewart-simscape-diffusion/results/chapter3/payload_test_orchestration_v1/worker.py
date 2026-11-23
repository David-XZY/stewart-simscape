"""Frozen held-out execution. No model training or test-dependent configuration."""
import os,sys,time,pathlib,traceback,collections,msvcrt
ROOT=pathlib.Path('D:/XieZhenyu/stewart-simscape-diffusion');P=ROOT/'results/chapter3';OUT=P/'payload_test_orchestration_v1'
os.chdir(ROOT);sys.path.insert(0,str(ROOT/'chapter3/payload_generalization'))
for k in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']:os.environ[k]='1'
from pg_core import read,write,digest
from run_payload import freeze,check_frozen,matlab,matlab_literal,file_hash
SEEDS=[20260919,20260920,20260921]
def status(stage,**kw):write(OUT/'status.json',dict(stage=stage,pid=os.getpid(),updatedUnix=time.time(),**kw))
lock=open(OUT/'worker.lock','a+b');lock.seek(0);lock.write(b'0');lock.flush();lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
try:
 status('preflight_and_freeze',status='running')
 assert read(P/'payload_formal_validation_job_v1/status.json')['status']=='completed'
 selection=read(P/'payload_formal_validation_job_v1/validation_selection.json')
 tasksfile=P/'payload_formal_preparation_v1/all_tasks.json';alltasks=read(tasksfile)['tasks'];tasks=[s for s in alltasks if s['split']=='test'];assert len(tasks)==192
 assert collections.Counter(s['generalizationType'] for s in tasks)==dict(G0=48,G1=48,G2=48,G3=48)
 dataset=P/'payload_formal_dataset_v1/dataset.json';data=read(dataset);library=P/'payload_formal_dataset_v1/library.json'
 trainScenes={r['baseSceneId'] for r in data['rows']};assert not trainScenes.intersection(s['baseSceneId'] for s in tasks)
 seen={digest([r['task']['payload']['shape'],r['task']['payload']['mass'],r['task']['payload']['dimensions']]) for r in data['rows'] if r['split']=='train'}
 assert all(digest([s['shape'],s['mass'],s['dimensions']]) in seen for s in tasks if s['generalizationType']=='G0')
 rows=[read(f) for f in (P/'payload_formal_enhanced_validation_v1/calls').glob('*/result.json')];assert len(rows)==9360
 assert all(not r['finalQualified'] or (r['returnedWithinBudget'] and r['independentAuditPassed'] and (not r.get('fineAuditSelected') or r['fineAuditPassed'])) for r in rows)
 closure=max(abs(r['actualReturnTime']-r['otherSeconds']-sum(e['seconds'] for e in r['events'])) for r in rows);assert closure<1e-6
 write(OUT/'preflight.json',dict(validationCalls=len(rows),timingClosureMax=closure,auditRejections=sum(r['onlinePassed'] and not r['finalQualified'] for r in rows),testTasks=192,G0SeenInTraining=True,testResultsRead=False))
 identity=OUT/'code_identity.json';matlab('pgWriteCodeIdentity('+matlab_literal(identity)+');')
 main=['ihsid','retrieval','ddpm_basic','regression','ddpm_signature','ddpm_physloss','ddpm_guided']
 controls=['ddpm_signature_no_accel','ddpm_signature_no_gap','ddpm_basic_wide','regression_signature','retrieval_signature','ddpm_end_guided']
 # Two tasks per test base scene, balanced by group across scenes; predetermined by hashes.
 quality=[]
 scenes=sorted({s['baseSceneId'] for s in tasks})
 for i,scene in enumerate(scenes):
  for shape in ['cylinder','box']:
   candidates=[s for s in tasks if s['baseSceneId']==scene and s['shape']==shape and s['generalizationType']=='G'+str(i%4)]
   assert len(candidates)==1;quality+=candidates
 reps={}
 for s in sorted(tasks,key=lambda x:x['taskId']):reps.setdefault((s['family'],s['taskClass'],s['shape'],s['generalizationType']),s['taskId'])
 reps=set(reps.values())
 modelchecks={};dataset_sha=file_hash(dataset)
 plans=[('main',tasks,main,[.5,1,2,5],'physical'),('controls',tasks,controls,[2],'physical'),('quality',quality,main,[2,5],'quality')]
 for name,specs,methods,budgets,policy in plans:
  protocol=OUT/(name+'_frozen.json')
  if protocol.exists():check_frozen(protocol);continue
  models={};jobs=[]
  for ti,spec in enumerate(specs):
   for bi,B in enumerate(budgets):
    for repeat,seed in enumerate(SEEDS):
     shift=(ti+bi+repeat)%len(methods);order=methods[shift:]+methods[:shift]
     for method in order:
      key=''
      if method not in ['ihsid','retrieval','retrieval_signature']:
       modelmethod='ddpm_basic' if method in ['ddpm_guided','ddpm_end_guided'] else method
       key=f'{modelmethod}_{seed}'
       if method=='ddpm_physloss' and seed==SEEDS[0]:key=next(x['key'] for x in selection['physicsWeights'] if x['weight']==selection['selectedPhysicsWeight'])
       f=P/'payload_formal_models_v1'/key/'model.json'
       if key not in modelchecks:
        a=read(f);assert a['datasetHash']==dataset_sha;modelchecks[key]=True
       models[key]=dict(key=key,file=str(f))
      jobs.append(dict(runId=digest([name,spec['taskId'],B,method,repeat])[:24],spec=spec,B=B,method=method,returnPolicy=policy,modelKey=key,trainingSeed=seed,samplingSeed=seed,repeat=repeat,preselectedRepresentative=spec['taskId'] in reps))
  req=dict(experimentVersion='payload_generalization_v1',scope='held_out_test',matlabCodeHash=read(identity)['matlabCodeHash'],controlCount=data['controlCount'],models=list(models.values()),jobs=jobs,qualityCaps=read(P/'payload_formal_dataset_v1/quality_caps.json'),validationSelection=read(P/'payload_formal_validation_selection_v1/validation_selection.json'),tasksFile=str(tasksfile),datasetFile=str(dataset),libraryFile=str(library),guidanceEta=selection['selectedEta'],integrationAuditFile=str(P/'payload_formal_integration_v1/integration.json'),enhancementSelection=selection,primaryComparison='G2 B2 ddpm_basic minus retrieval; enhancements versus basic',noTestTuning=True)
  request=OUT/(name+'_request.json');write(request,req);freeze(request,protocol)
 write(OUT/'frozen_plan.json',dict(protocols=[dict(name=n,jobs=len(read(OUT/(n+'_frozen.json'))['jobs']),configHash=read(OUT/(n+'_frozen.json'))['configHash']) for n,*_ in plans],formalReferenceGeneration='pending separate executor; no test labels used in training',combinedTriggered=selection['combinedTriggered']))
 for name,*_ in plans:
  protocol=OUT/(name+'_frozen.json');check_frozen(protocol);status('benchmark_'+name,status='running',plannedCalls=len(read(protocol)['jobs']))
  matlab('pgBenchmark('+matlab_literal(protocol)+",'payload_heldout_"+name+"_v1');")
  calls=[read(f) for f in (P/('payload_heldout_'+name+'_v1')/'calls').glob('*/result.json')];assert len(calls)==len(read(protocol)['jobs'])
  write(OUT/(name+'_completion.json'),dict(calls=len(calls),qualified=sum(r['finalQualified'] for r in calls),completedUnix=time.time()))
 status('heldout_benchmarks_complete_statistics_and_reference_diagnostics_pending',status='completed')
except BaseException as e:
 (OUT/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');status('failed',status='failed',error=str(e));raise
