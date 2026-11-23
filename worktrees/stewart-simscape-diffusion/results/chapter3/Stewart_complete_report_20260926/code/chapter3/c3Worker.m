function c3Worker(runName,limit)
if nargin<2, limit=inf; end
startup=tic; c=c3Setup(runName); z=load(fullfile(c.output,'manifest.mat')); manifest=z.manifest;
z=load(fullfile(c.output,'jobs.mat')); jobs=z.jobs; z=load(fullfile(c.output,'standard.mat')); standard=z.standard;
frozen=jsondecode(fileread(fullfile(c.output,'frozen.json')));
assert(strcmp(c3Fingerprint(c),frozen.implementation),'C3:Frozen','Implementation mismatch.');
assert(strcmp(dpHash(fileread(c.modelFile)),frozen.model) && strcmp(standard.hash,frozen.standard));
asset=c3Assets(c); sgPhysics(buildOptModelCustom());
dpWriteJson(fullfile(c.output,'startup_latest.json'),struct('seconds',toc(startup),'matlab',version,'threads',1));
completed=0;
for index=1:numel(jobs)
    job=jobs{index}; item=manifest.cases{job.caseIndex}; folder=fullfile(c.output,'jobs',job.jobId);
    target=fullfile(folder,'result.json'); if isfile(target), continue; end
    if completed>=limit, break; end
    if ~isfolder(folder), mkdir(folder); end
    row=struct('jobId',job.jobId,'caseId',item.caseId,'family',item.family,'category',item.category, ...
        'groupId',item.groupId,'rootGroupId',item.rootGroupId,'sequence',item.sequence,'step',item.step,'parameter',item.parameter,'value',item.value, ...
        'method',job.method,'repeat',job.repeat,'purpose',job.purpose,'count',job.count,'seed',job.seed, ...
        'completed',true,'passed',false,'status','','implementation',frozen.implementation, ...
        'taskHash',item.task.hash,'standardHash',standard.hash,'historySource','','timingMeasured',false);
    active=struct('job',job,'metadata',row,'phase','online','startedEpoch',posixtime(datetime('now','TimeZone','UTC')),'budget',job.budget);
    dpWriteJson(fullfile(c.output,'active.json'),active);
    callClock=tic; history=[]; historyFile='';
    % Never inherit a competing method's graph. Reuse within a call is allowed.
    % Changed goals/model values are graph constants, so adjacent tasks rebuild.
    clear c3Solve sgPhysics sgBasis sgRepairPoint
    if ~isempty(item.sequence)
        historyFile=fullfile(c.output,'history',sprintf('%s__%s__r%d.mat',item.sequence,job.method,job.repeat));
        if item.step>0 && isfile(historyFile)
            previous=load(historyFile); assert(previous.step<item.step,'C3:FutureHistory','Future history access.');
            history=previous.history; row.historySource=previous.jobId;
        end
    end
    out=[]; audit=[]; fine=[];
    try
        c.progressFile=fullfile(folder,'progress.json'); preparationSeconds=toc(callClock);
        plannerJob=job; plannerJob.budget=max(0,job.budget-preparationSeconds);
        out=c3Plan(item.task,plannerJob,c,asset,standard,history); returnedSeconds=toc(callClock);
        out.timing.inputPreparation=out.timing.inputPreparation+preparationSeconds;
        out.timing.other=out.timing.other+returnedSeconds-out.onlineSeconds-preparationSeconds;
        out.onlineSeconds=returnedSeconds; out.generationSeconds=returnedSeconds-out.evaluationSeconds;
        out.firstPhysicalSeconds=out.firstPhysicalSeconds+preparationSeconds;
        out.firstQualifiedSeconds=out.firstQualifiedSeconds+preparationSeconds;
        out.timingClosureError=returnedSeconds-sum(structfun(@double,out.timing));
        if returnedSeconds>job.budget, out.passed=false; out.reason='wall_budget'; end
        save(fullfile(folder,'online.mat'),'out','job','item','-v7');
        row.timingMeasured=true; row.status=out.reason; row.physicalPassed=out.physicalPassed;
        row.cachePolicy='Isolated per measured call; candidate attempts may reuse their own graph; history states/forces/compatible duals persist.';
        row.qualityPassed=out.qualityPassed; row.onlineSeconds=out.onlineSeconds; row.generationSeconds=out.generationSeconds;
        row.evaluationSeconds=out.evaluationSeconds; row.timing=out.timing; row.timingClosureError=out.timingClosureError;
        row.firstPhysicalSeconds=out.firstPhysicalSeconds; row.firstQualifiedSeconds=out.firstQualifiedSeconds;
        row.acceptedStage=out.acceptedStage; row.candidateCount=out.candidateCount; row.attempts=out.attempts;
        row.passed=out.passed;
        dpWriteJson(fullfile(folder,'online_summary.json'),row);
        active.phase='audit'; active.budget=600; active.startedEpoch=posixtime(datetime('now','TimeZone','UTC'));
        dpWriteJson(fullfile(c.output,'active.json'),active); research=tic;
        if out.passed
            audit=sgAudit(out.bundle,item.task,c); independent=c3Quality(out.bundle,item.task,c); independentScore=c3Score(independent,item.family,standard);
            row.auditPassed=audit.passed && independentScore.qualityPassed;
            assert(row.auditPassed,'C3:AuditMismatch','Online acceptance disagrees with independent audit.');
            row.quality=independent; row.score=independentScore; row.minClearance=audit.minClearance;
            row.minSigma=audit.minSigma; row.maxCondition=audit.maxCondition; row.denseDynamicResidual=audit.maxDenseDynamicResidual;
            % Predeclared representatives and threshold-near results get grid refinement.
            if endsWith(item.caseId,'_01') || min(abs(out.score.groups./standard.limits.(item.family).groups-1))<.02
                fineConfig=c; fineConfig.qualityStep=.0025; fine=c3Quality(out.bundle,item.task,fineConfig);
                row.gridRelativeChange=abs(fine.values-independent.values)./max(abs(independent.values),1e-12);
            end
            if ~isempty(historyFile)
                if ~isfolder(fileparts(historyFile)), mkdir(fileparts(historyFile)); end
                history=out.warm; step=item.step; jobId=job.jobId; save(historyFile,'history','step','jobId','-v7');
            end
        elseif ~isempty(out.bestUnqualified)
            row.bestUnqualifiedScore=out.bestUnqualified.score;
        end
        row.researchAuditSeconds=toc(research);
    catch e
        row.passed=false; row.status='exception'; row.error=[e.identifier,': ',e.message]; row.stack=e.stack;
        if ~isfield(row,'onlineSeconds')
            if ~isempty(out) && isfield(out,'onlineSeconds'), row.onlineSeconds=out.onlineSeconds;
            else, row.onlineSeconds=toc(callClock); end
            row.wallTimeMeasured=true; row.timingMeasured=false;
        end
        if strcmp(e.identifier,'C3:AuditMismatch'), dpWriteJson(fullfile(c.output,'safety_stop.json'),row); end
    end
    save(fullfile(folder,'result.mat'),'out','row','audit','fine','job','item','-v7');
    dpWriteJson(target,row); completed=completed+1;
    fprintf('C3_JOB %s %s\n',job.jobId,row.status);
    if isfile(fullfile(c.output,'safety_stop.json')), error('C3:SafetyStop','Acceptance mismatch.'); end
end
dpWriteJson(fullfile(c.output,'worker_complete.json'),struct('callsThisProcess',completed,'finishedEpoch',posixtime(datetime('now','TimeZone','UTC'))));
end
