function p3CarryBatch(requestFile)
r=jsondecode(fileread(requestFile)); c=pgConfig(['payload_pair2_nofallback_v1/',r.phase]);
assert(strcmp(c.codeHash,r.originalCodeHash),'P2:Code','Original numerical code identity differs.');
c.controlCount=16; c.returnPolicy='physical'; models=struct();
for k=1:numel(r.models), models.(r.models(k).key)=jsondecode(fileread(fullfile(r.root,r.models(k).relativeFile))); end
startup=pgBuildProblem(c); startup.matlab=version; startup.casadi=casadi.CasadiMeta.version();
startup.host=getenv('COMPUTERNAME'); startup.threads=maxNumCompThreads; startup.onlineDevice='CPU';
dpWriteJson(fullfile(r.output,'startup.json'),startup);
jobs=r.jobs;
jobMap=containers.Map('KeyType','char','ValueType','any');
for jj=1:numel(jobs), jobMap(key(jobs(jj),jobs(jj).B))=jobs(jj); end
oldRows=jsondecode(fileread(r.ihsidBaselineFile)); if ~iscell(oldRows), oldRows=num2cell(oldRows); end
oldMap=containers.Map('KeyType','char','ValueType','any');
for jj=1:numel(oldRows), row=oldRows{jj}; oldMap(key(row.job,row.B))=row; end
if ~isfolder(r.viewOutput), mkdir(r.viewOutput); end
for k=1:numel(jobs)
 job=jobs(k); path=fullfile(r.output,'calls',job.runId); committed=fullfile(path,'result.json');
 [source,origin]=availableSource(job);
 if ~isempty(source)
  recordView(source,origin); heartbeat('resolved_existing'); continue;
 end
 assert(~isfield(r,'expectedNoNewMeasurements')||~r.expectedNoNewMeasurements,'P3:TestUnexpectedMeasurement','Carry test attempted new planning.');
 assert(~strcmp(job.method,'ihsid')||job.B==20,'P3:IHSIDScope','Only missing IHSID 20s calls may execute.');
 if isfolder(path)
  interrupted=fullfile(r.output,'interrupted'); if ~isfolder(interrupted), mkdir(interrupted); end
  target=fullfile(interrupted,[job.runId,'_',datestr(now,'yyyymmddTHHMMSSFFF')]);
  rootCanonical=[char(java.io.File(r.output).getCanonicalPath()),filesep];
  assert(startsWith(lower(char(java.io.File(path).getCanonicalPath())),lower(rootCanonical))&&startsWith(lower(char(java.io.File(target).getCanonicalPath())),lower(rootCanonical)));
  movefile(path,target);
 end
 mkdir(path); local=struct(); if ~isempty(job.modelKey), local.model=models.(job.modelKey); end
 % Heartbeat writes are outside the measured online call.
 heartbeat('online'); outer=tic; task=pgMakeTask(job.spec,c); prep=toc(outer);
 result=p2Plan(task,job.method,max(0,job.B-prep),job.samplingSeed,local,c);
 result.job=job; result.B=job.B; result.configHash=r.configHash; result.libraryHash=r.libraryHash;
 result.trainingSeed=job.trainingSeed; result.samplingSeed=job.samplingSeed; result.repeat=job.repeat;
 result.runId=job.runId; result.generalizationType=job.spec.generalizationType;
 if isfinite(result.firstPhysicalFoundTime), result.firstPhysicalFoundTime=result.firstPhysicalFoundTime+prep; end
 if isfinite(result.firstQualityFoundTime), result.firstQualityFoundTime=result.firstQualityFoundTime+prep; end
 previous=result.actualReturnTime; result.actualReturnTime=toc(outer); result.outerPreparationSeconds=prep;
 result.otherSeconds=result.otherSeconds+result.actualReturnTime-previous-prep;
 result.events{end+1}=struct('stage','outer_input','seconds',prep);
 result.returnedWithinBudget=result.actualReturnTime<=job.B; result.passed=result.passed&&result.returnedWithinBudget;
 result.timeoutOverrun=max(0,result.actualReturnTime-job.B); if ~result.returnedWithinBudget, result.reason='timeout'; end
 result.onlinePassed=result.passed; result.independentAuditPassed=false; result.finalQualified=false;
 heartbeat('audit'); offline=tic;
 if result.passed
  audit=pgValidate(result.bundle,task,c); result.independentAuditPassed=audit.passed;
  result.rawMetrics=pgQuality(result.bundle,task,c); metrics=result.rawMetrics;
  hash=dpHash(job.runId); randomFine=hex2dec(hash(1:8))<.1*2^32;
  fine=job.preselectedRepresentative||audit.minClearance<.012||max([metrics.maxSpeedUtilization,metrics.maxAccelerationUtilization,metrics.maxForceUtilization])>.95||randomFine;
  result.fineAuditSelected=fine; result.fineAuditPassed=[]; result.finalQualified=audit.passed;
  if fine, cc=c; cc.denseStep=.0025; fr=pgValidate(result.bundle,task,cc); result.fineAuditPassed=fr.passed; result.finalQualified=result.finalQualified&&fr.passed; end
 end
 result.offlineAuditSeconds=toc(offline); result.trajectoryRef=fullfile(path,'trajectory.mat');
 assert(result.rankingCalls==0&&result.rankingSeconds==0);
 assert(strcmp(job.method,'ihsid')||(result.ihsidCalls==0&&result.ihsidSeconds==0));
 save(result.trajectoryRef,'task','result','-v7'); compact=rmfield(result,{'bundle','solver','bestUnacceptedBundle'});
 if ~isempty(compact.report)
  phys=compact.report.physical;
  for name={'q','v','a','FRequired','FInterpolated','L','Ld','Ldd','clearance','time'}, if isfield(phys,name{1}), phys=rmfield(phys,name{1}); end; end
  compact.report.physical=phys;
 end
 dpWriteJson(fullfile(path,'result.tmp.json'),compact); movefile(fullfile(path,'result.tmp.json'),committed,'f');
 recordView(compact,'new_measurement'); heartbeat('committed'); fprintf('P2_CALL %d/%d %s B=%g qualified=%d elapsed=%.4f\n',k,numel(jobs),job.method,job.B,result.finalQualified,result.actualReturnTime);
end
 function kkey=key(j,b)
  kkey=sprintf('%s|%s|%d|%g',j.spec.taskId,j.method,j.repeat,b);
 end
 function [source,origin]=availableSource(j)
  source=[]; origin=''; same=[]; sameOrigin='';
  for b=[1,2,5,10,20]
   if b>j.B, break; end
   kk=key(j,b); candidate=[]; kind='';
   if strcmp(j.method,'ihsid')&&b<20
    assert(isKey(oldMap,kk),'P3:MissingBaseline','Missing existing IHSID budget.');
    candidate=oldMap(kk); kind='historical_ihsid';
   elseif isKey(jobMap,kk)
    priorJob=jobMap(kk); f=fullfile(r.output,'calls',priorJob.runId,'result.json');
    if isfile(f)
     candidate=jsondecode(fileread(f)); kind='existing_pair2';
     assert(strcmp(candidate.configHash,r.configHash)&&isfile(fullfile(r.output,'calls',priorJob.runId,'trajectory.mat')),'P3:Identity','Existing call differs.');
    end
   end
   if isempty(candidate), continue; end
   assert(strcmp(candidate.taskId,j.spec.taskId)&&strcmp(candidate.method,j.method)&&candidate.repeat==j.repeat&&candidate.trainingSeed==j.trainingSeed&&candidate.samplingSeed==j.samplingSeed,'P3:CrossBlock','Cannot inherit across tasks/methods/seeds.');
   if b==j.B, same=candidate; sameOrigin=kind; end
   if candidate.finalQualified
    assert(candidate.onlinePassed&&candidate.independentAuditPassed&&candidate.returnedWithinBudget&&candidate.actualReturnTime<=j.B,'P3:Unqualified','Only audited on-time success may be inherited.');
    if candidate.fineAuditSelected, assert(candidate.fineAuditPassed); end
    source=candidate; origin=kind; return;
   end
  end
  source=same; origin=sameOrigin;
 end
 function recordView(source,origin)
  inherited=source.B<job.B;
  assert(~inherited||source.finalQualified,'P3:FailureCarry','Failed output cannot propagate.');
  if isfield(r,'expectedNoNewMeasurements')&&r.expectedNoNewMeasurements
   assert(inherited==(job.B>5),'P3:CarryTest','Incorrect success propagation.');
   if inherited, assert(source.B==5&&source.finalQualified); end
  end
  v=struct('evaluationId',job.runId,'targetBudget',job.B,'taskId',job.spec.taskId,'method',job.method, ...
   'repeat',job.repeat,'trainingSeed',job.trainingSeed,'samplingSeed',job.samplingSeed,'scopeHash',r.scopeHash, ...
   'sourceRunId',source.runId,'sourceBudget',source.B,'inherited',inherited,'origin',origin, ...
   'usesMeasurementAtTargetBudget',~inherited,'targetBudgetWasMeasured',isfile(committed)||(strcmp(job.method,'ihsid')&&job.B<20), ...
   'finalQualified',source.finalQualified,'sourceRow',source);
  file=fullfile(r.viewOutput,[job.runId,'.json']); dpWriteJson([file,'.tmp'],v); movefile([file,'.tmp'],file,'f');
 end
 function heartbeat(stage)
  h=struct('phase',r.phase,'stage',stage,'index',k,'total',numel(jobs),'runId',job.runId,'B',job.B, ...
   'unixTime',posixtime(datetime('now','TimeZone','UTC')));
  tmp=fullfile(r.output,'heartbeat.tmp.json'); dpWriteJson(tmp,h); movefile(tmp,fullfile(r.output,'heartbeat.json'),'f');
 end
end
