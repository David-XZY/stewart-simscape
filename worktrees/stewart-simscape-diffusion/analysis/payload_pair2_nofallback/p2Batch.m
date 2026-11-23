function p2Batch(requestFile)
r=jsondecode(fileread(requestFile)); c=pgConfig(['payload_pair2_nofallback_v1/',r.phase]);
assert(strcmp(c.codeHash,r.originalCodeHash),'P2:Code','Original numerical code identity differs.');
c.controlCount=16; c.returnPolicy='physical'; models=struct();
for k=1:numel(r.models), models.(r.models(k).key)=jsondecode(fileread(fullfile(r.root,r.models(k).relativeFile))); end
startup=pgBuildProblem(c); startup.matlab=version; startup.casadi=casadi.CasadiMeta.version();
startup.host=getenv('COMPUTERNAME'); startup.threads=maxNumCompThreads; startup.onlineDevice='CPU';
dpWriteJson(fullfile(r.output,'startup.json'),startup);
jobs=r.jobs;
for k=1:numel(jobs)
 job=jobs(k); path=fullfile(r.output,'calls',job.runId); committed=fullfile(path,'result.json');
 if isfile(committed)
  old=jsondecode(fileread(committed)); assert(strcmp(old.configHash,r.configHash)&&isfile(fullfile(path,'trajectory.mat')),'P2:Resume','Identity/artifact mismatch.'); continue;
 end
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
 heartbeat('committed'); fprintf('P2_CALL %d/%d %s B=%g qualified=%d elapsed=%.4f\n',k,numel(jobs),job.method,job.B,result.finalQualified,result.actualReturnTime);
end
 function heartbeat(stage)
  h=struct('phase',r.phase,'stage',stage,'index',k,'total',numel(jobs),'runId',job.runId,'B',job.B, ...
   'unixTime',posixtime(datetime('now','TimeZone','UTC')));
  tmp=fullfile(r.output,'heartbeat.tmp.json'); dpWriteJson(tmp,h); movefile(tmp,fullfile(r.output,'heartbeat.json'),'f');
 end
end
