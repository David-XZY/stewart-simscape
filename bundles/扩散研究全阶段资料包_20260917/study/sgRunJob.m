function row=sgRunJob(item,job,c,asset,fingerprint,activeFile)
folder=fullfile(c.output,'jobs',job.jobId); if ~isfolder(folder), mkdir(folder); end
row=struct('schemaVersion',1,'jobId',job.jobId,'caseId',item.caseId,'category',item.category, ...
    'parameter',item.parameter,'value',item.value,'family',item.family,'mode',item.mode, ...
    'groupId',item.groupId,'method',job.method,'count',job.count,'repeat',job.repeat, ...
    'seed',job.seed,'purpose',job.purpose,'budget',job.budget,'fingerprint',fingerprint, ...
    'completed',true,'passed',false,'nominalPassed',false,'truthPassed',false, ...
    'falseAcceptance',false,'withinBudget',false,'status','','reason','', ...
    'onlineSeconds',0,'firstFeasibleSeconds',NaN,'planningReused',false,'reuseSource','', ...
    'generationPassed',item.generationPassed,'planningBoundaryPassed',false,'truthBoundaryPassed',false);
out=[]; audit=[]; quality=[]; firstQuality=[]; raw=[]; diversity=[];
if ~item.generationPassed
    row.status='generation_failed'; row.reason=item.generationReason; finish(); return;
end
task=item.planningTask; truth=item.truthTask;
row.planningBoundaryPassed=task.endpointPassed; row.truthBoundaryPassed=truth.endpointPassed;
% Truth is deliberately unavailable to sgPlan. Reuse is nominal-only.
key=dpHash(struct('q0',task.q0,'goal',task.qGoal,'T',task.T,'model',task.model,'scene',task.scene, ...
    'method',job.method,'count',job.count,'seed',job.seed,'budget',job.budget,'modelHash',asset.modelHash));
cache=fullfile(c.output,'nominal_reuse',[key,'.mat']);
if strcmp(item.mode,'unknown') && isfile(cache)
    saved=load(cache); out=saved.out; row.planningReused=true; row.reuseSource=saved.jobId;
else
    c.progressFile=fullfile(folder,'online_progress.json');
    out=sgPlan(task,job,c,asset);
    if strcmp(item.mode,'unknown')
        if ~isfolder(fileparts(cache)), mkdir(fileparts(cache)); end
        jobId=job.jobId; tmp=[cache,'.',jobId,'.tmp.mat']; save(tmp,'out','jobId','-v7'); movefile(tmp,cache,'f');
    end
end
save(fullfile(folder,'online.mat'),'out','job','fingerprint','-v7');
setPhase('audit',600);
researchClock=tic;
row.onlineSeconds=out.onlineSeconds; row.firstFeasibleSeconds=out.firstFeasibleSeconds;
row.nominalPassed=out.passed; row.withinBudget=out.onlineSeconds<=job.budget;
row.acceptedStage=out.acceptedStage; row.candidate=out.candidate;
for name={'boundarySeconds','samplingSeconds','rankingSeconds','validationSeconds','repairSeconds','solverSeconds','solverBuildSeconds','directPasses','directChecked','repairAttempts','solveAttempts'}
    row.(name{1})=out.(name{1});
end
row.anytime=out.events; row.reason=out.reason; row.status=out.reason;
row.solverRecords=out.solverRecords;
if out.passed
    clock=tic; audit=sgAudit(out.bundle,truth,c); row.auditSeconds=toc(clock);
    row.truthPassed=audit.passed; row.falseAcceptance=~audit.passed;
    if strcmp(item.mode,'known') && ~audit.passed
        dpWriteJson(fullfile(c.output,'safety_stop.json'),struct('jobId',job.jobId, ...
            'reason','Known-model fast acceptance disagrees with independent audit.'));
    end
    row.passed=audit.passed && row.withinBudget;
    if ~audit.passed, row.status='truth_audit_failed';
    elseif ~row.withinBudget, row.status='budget_overrun'; else, row.status='passed'; end
    row.minClearance=audit.minClearance; row.minSigma=audit.minSigma; row.maxCondition=audit.maxCondition;
    row.maxLegViolation=audit.maxLegViolation; row.maxForceViolation=audit.maxForceViolation;
    row.denseDynamicResidual=audit.maxDenseDynamicResidual;
    interior=audit.time>=.1*task.T & audit.time<=.9*task.T;
    row.interiorMinClearance=min(min(audit.clearance(:,interior)));
    row.nearBoundaryTimeFraction=trapz(audit.time,double(min(audit.clearance,[],1)<.012))/task.T;
    quality=sgQuality(out.bundle,truth,c); row.quality=quality;
    firstQuality=sgQuality(out.firstBundle,truth,c); row.firstQuality=firstQuality;
    if strcmp(dpHash(out.firstBundle),dpHash(out.bundle)), firstAudit=audit;
    else, firstAudit=sgAudit(out.firstBundle,truth,c); end
    row.firstTruthPassed=firstAudit.passed;
    if strcmp(item.mode,'known') && ~firstAudit.passed
        dpWriteJson(fullfile(c.output,'safety_stop.json'),struct('jobId',job.jobId,'reason','First accepted output failed independent audit.'));
    end
    save(fullfile(folder,'first_audit.mat'),'firstAudit','-v7');
    nominalTr=dpReconstruct(out.bundle,audit.time); p=sgPhysics(task.model);
    [~,~,~,nominalForce]=p.point(nominalTr.q,nominalTr.v,nominalTr.a); mismatch=full(nominalForce)-audit.FRequired;
    row.forceMismatchPeak=max(abs(mismatch(:)));
    row.forceMismatchRms=sqrt(trapz(audit.time,mean(mismatch.^2,1))/task.T);
    for e=1:numel(out.eventBundles)
        row.anytime(e).quality=sgQuality(out.eventBundles{e},truth,c);
        if e==1, eventAudit=firstAudit;
        elseif e==numel(out.eventBundles), eventAudit=audit;
        else, eventAudit=sgAudit(out.eventBundles{e},truth,c); end
        row.anytime(e).truthPassed=eventAudit.passed;
        if strcmp(item.mode,'known') && ~eventAudit.passed
            dpWriteJson(fullfile(c.output,'safety_stop.json'),struct('jobId',job.jobId,'reason','Anytime output failed independent audit.'));
        end
    end
end
% Census is an explicitly OFFLINE diagnostic; no truth leakage into planning.
raw=struct('candidate',{},'passed',{},'reason',{},'seconds',{});
candidateCount=0; if ~isempty(out.candidates), candidateCount=size(out.candidates,3); end
for k=1:candidateCount
    st=tic; r=sgValidate(struct('kind','spline','cp',out.candidates(:,:,k),'T',truth.T),truth,c);
    raw(end+1)=struct('candidate',k,'passed',r.passed,'reason',r.reason,'seconds',toc(st)); %#ok<AGROW>
end
row.rawCandidateCount=numel(raw); row.rawCandidatePasses=sum([raw.passed]); row.rawCensusSeconds=sum([raw.seconds]);
row.directTaskTruthPassed=any([raw.passed]);
if size(out.candidates,3)>1
    diversity=sgDiversity(out.candidates,task);
    row.maxPositionRmsM=diversity.maxPositionRmsM; row.maxAttitudeRmsRad=diversity.maxAttitudeRmsRad;
    mask=triu(true(size(diversity.positionRmsPairsM)),1);
    row.meanPositionRmsM=mean(diversity.positionRmsPairsM(mask));
    row.meanAttitudeRmsRad=mean(diversity.attitudeRmsPairsRad(mask));
    duplicate=diversity.positionRmsPairsM<1e-9 & diversity.attitudeRmsPairsRad<1e-7;
    row.numericallyDistinctCandidates=sum(~any(tril(duplicate,-1),2));
end
row.researchAuditSeconds=toc(researchClock);
finish();
    function setPhase(phase,budget)
        active=struct('jobId',job.jobId,'caseId',item.caseId,'fingerprint',fingerprint, ...
            'phase',phase,'startedEpoch',posixtime(datetime('now','TimeZone','UTC')), ...
            'budget',budget,'job',job,'metadata',row);
        dpWriteJson(activeFile,active);
    end
    function finish()
        save(fullfile(folder,'result.mat'),'row','out','audit','quality','firstQuality','raw','diversity','job','fingerprint','-v7');
        dpWriteJson(fullfile(folder,'result.json'),row);
        fprintf('STUDY %s %s pass=%d raw=%d/%d t=%.3f reused=%d\n', ...
            item.caseId,job.method,row.passed,value(row,'rawCandidatePasses'),value(row,'rawCandidateCount'),row.onlineSeconds,row.planningReused);
    end
end
function x=value(s,name)
x=0; if isfield(s,name), x=s.(name); end
end
