function pgBenchmark(jobsFile,runName)
c=pgConfig(runName); protocol=jsondecode(fileread(jobsFile));
assert(strcmp(protocol.experimentVersion,c.version)&&strcmp(protocol.status,'frozen'),'PG:Freeze','Frozen protocol required.');
assert(strcmp(protocol.matlabCodeHash,c.codeHash),'PG:Version','Numerical source changed after freeze.');
cold=tic; c.controlCount=protocol.controlCount; signatureIndex=any(strcmp({protocol.jobs.method},'retrieval_signature')); assets=pgAssets(protocol.libraryFile,'',signatureIndex); models=struct();
assets.distanceVariant=protocol.validationSelection.selectedDistance;
assert(assets.controlCount==c.controlCount,'PG:Representation','Library/model common representation mismatch.');
if isfield(protocol,'guidanceEta'), c.guidanceEta=protocol.guidanceEta; end
for k=1:numel(protocol.models), entry=protocol.models(k); models.(entry.key)=jsondecode(fileread(entry.file)); end
startup=pgBuildProblem(c); startup.totalLoadingAndWarmupSeconds=toc(cold); dpWriteJson(fullfile(c.output,'startup.json'),startup);
for k=1:numel(protocol.jobs)
    job=protocol.jobs(k); path=fullfile(c.output,'calls',job.runId); assert(~isfolder(path),'PG:Immutable','Existing call output.'); mkdir(path);
    local=assets; if ~isempty(job.modelKey), local.model=models.(job.modelKey); end
    c.returnPolicy=job.returnPolicy;
    if strcmp(job.returnPolicy,'quality'), cap=protocol.qualityCaps.(job.spec.shape); c.positionCap=cap.position; c.rotationCap=cap.rotation; c.qualityFrozen=true; end
    outer=tic; task=pgMakeTask(job.spec,c); prep=toc(outer);
    result=pgPlan(task,job.method,max(0,job.B-prep),job.samplingSeed,local,c);
    % Output assembly belongs to the call; all later disk writes are archival.
    result.job=job; result.B=job.B; result.configHash=protocol.configHash; result.libraryHash=assets.libraryHash;
    result.trainingSeed=job.trainingSeed; result.samplingSeed=job.samplingSeed; result.repeat=job.repeat; result.runId=job.runId; result.generalizationType=job.spec.generalizationType;
    if isfinite(result.firstPhysicalFoundTime), result.firstPhysicalFoundTime=result.firstPhysicalFoundTime+prep; end
    if isfinite(result.firstQualityFoundTime), result.firstQualityFoundTime=result.firstQualityFoundTime+prep; end
    previous=result.actualReturnTime; result.actualReturnTime=toc(outer); result.outerPreparationSeconds=prep;
    result.otherSeconds=result.otherSeconds+result.actualReturnTime-previous-prep;
    result.events{end+1}=struct('stage','outer_input','seconds',prep);
    result.returnedWithinBudget=result.actualReturnTime<=job.B; result.passed=result.passed&&result.returnedWithinBudget;
    result.timeoutOverrun=max(0,result.actualReturnTime-job.B); if ~result.returnedWithinBudget, result.reason='timeout'; end
    result.onlinePassed=result.passed; result.independentAuditPassed=false; result.finalQualified=false; offline=tic;
    if result.passed
        audit=pgValidate(result.bundle,task,c); result.independentAuditPassed=audit.passed;
        result.rawMetrics=pgQuality(result.bundle,task,c); metrics=result.rawMetrics;
        hash=dpHash(job.runId); randomFine=hex2dec(hash(1:8))<.1*2^32;
        fine=job.preselectedRepresentative||audit.minClearance<.012||max([metrics.maxSpeedUtilization,metrics.maxAccelerationUtilization,metrics.maxForceUtilization])>.95||randomFine;
        result.fineAuditSelected=fine; result.fineAuditPassed=[];
        result.finalQualified=audit.passed;
        if fine, cc=c; cc.denseStep=.0025; fr=pgValidate(result.bundle,task,cc); result.fineAuditPassed=fr.passed; result.finalQualified=result.finalQualified&&fr.passed; end
    end
    result.offlineAuditSeconds=toc(offline); result.trajectoryRef=fullfile(path,'trajectory.mat');
    save(result.trajectoryRef,'task','result','-v7'); compact=result;
    compact=rmfield(compact,{'bundle','solver','bestUnacceptedBundle'});
    if ~isempty(compact.report), phys=compact.report.physical; for name={'q','v','a','FRequired','FInterpolated','L','Ld','Ldd','clearance','time'}, if isfield(phys,name{1}), phys=rmfield(phys,name{1}); end; end; compact.report.physical=phys; end
    dpWriteJson(fullfile(path,'result.json'),compact); fprintf('PG_CALL %d/%d qualified=%d elapsed=%.4g\n',k,numel(protocol.jobs),result.finalQualified,result.actualReturnTime);
end
end
