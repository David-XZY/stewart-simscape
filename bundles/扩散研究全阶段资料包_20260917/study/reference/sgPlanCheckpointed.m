function out=sgPlanCheckpointed(task,job,c,asset)
clock=tic; c.clockHandle=clock; c.wallBudget=job.budget; c.fastBudget=job.budget;
out=struct('passed',false,'reason','','bundle',[],'firstBundle',[],'report',[], ...
    'candidates',[],'generation',[],'scores',[],'order',[],'attempts',[], ...
    'events',[],'firstFeasibleSeconds',NaN,'onlineSeconds',0,'samplingSeconds',0, ...
    'rankingSeconds',0,'validationSeconds',0,'repairSeconds',0,'solverSeconds',0, ...
    'solverBuildSeconds',0,'acceptedStage','','candidate',NaN,'bestOnlineScore',inf, ...
    'directPasses',0,'directChecked',0,'repairAttempts',0,'solveAttempts',0,'solverRecords',{{}},'eventBundles',{{}},'boundarySeconds',0);
st=tic; task=sgFinalizeTask(task); out.boundarySeconds=toc(st);
if ~task.endpointPassed, out.reason='planning_boundary_invalid'; out.onlineSeconds=toc(clock); return; end
if strncmp(job.method,'geo',3), kind='geometric'; elseif strncmp(job.method,'retrieve',8), kind='retrieval'; else, kind='diffusion'; end
st=tic; [cp,generation]=sgCandidates(kind,task,job.count,job.seed,asset); out.samplingSeconds=toc(st);
st=tic; [scores,order]=sgRank(cp,task); out.rankingSeconds=toc(st);
out.candidates=cp; out.generation=generation; out.scores=scores; out.order=order;
attempts=struct('candidate',{},'stage',{},'passed',{},'seconds',{},'reason',{});
events=struct('seconds',{},'stage',{},'candidate',{},'onlineScore',{});
isSolver=~isempty(strfind(job.method,'ihsid')) || strcmp(job.method,'geo_reference'); %#ok<STREMP>
best=inf;
for ix=1:job.count
    k=order(ix); if toc(clock)>=job.budget, out.reason='budget_exhausted'; break; end
    st=tic; accepted=false; reason=''; bundle=[]; r=[]; solver=[];
    try
        if isSolver
            out.solveAttempts=out.solveAttempts+1;
            c.maxCpuSeconds=min(120,max(.1,job.budget-toc(clock)-1)); solver=sgSolve(task,cp(:,:,k),c);
            out.solverRecords{end+1}=rmfield(solver,{'bundle','report','sourceSeed'});
            accepted=solver.passed; r=solver.report; bundle=solver.bundle; reason=solver.solverStatus;
            out.solverSeconds=out.solverSeconds+solver.solveSeconds;
            out.solverBuildSeconds=out.solverBuildSeconds+solver.buildSeconds;
            stage='ihsid'; score=solver.objective;
        else
            bundle=struct('kind','spline','cp',cp(:,:,k),'T',task.T);
            if strcmp(job.method,'diff_legacy_check')
                r=sgAudit(bundle,task,c); r.reason='legacy_rejected';
                if r.passed, r.reason='passed'; end
            else, r=sgValidate(bundle,task,c,clock,job.budget); end
            accepted=r.passed; reason=r.reason; stage='direct'; score=scores(k,2);
            out.directChecked=out.directChecked+1; out.directPasses=out.directPasses+accepted;
        end
    catch e
        reason=[e.identifier,': ',e.message]; stage='exception'; score=inf;
    end
    seconds=toc(st);
    if ~isSolver, out.validationSeconds=out.validationSeconds+seconds; end
    attempts(end+1)=struct('candidate',k,'stage',stage,'passed',accepted,'seconds',seconds,'reason',reason); %#ok<AGROW>
    if isfield(c,'progressFile')
        dpWriteJson(c.progressFile,struct('attempts',attempts,'solverRecords',{out.solverRecords}, ...
            'samplingSeconds',out.samplingSeconds,'rankingSeconds',out.rankingSeconds, ...
            'onlineSeconds',toc(clock),'solveAttempts',out.solveAttempts));
    end
    if accepted && toc(clock)<=job.budget
        acceptedResult(bundle,r,stage,k,score);
        if isSolver && ~strcmp(job.method,'geo_reference'), break; end
    end
end
canRepair=any(strcmp(job.method,{'geo_fast','retrieve_fast','diff_fast'}));
if ~out.passed && canRepair
    for ix=1:min(c.repairCandidates,job.count)
        if toc(clock)>=job.budget, break; end
        k=order(ix); st=tic; accepted=false; reason='';
        try
            out.repairAttempts=out.repairAttempts+1;
            [fixed,info]=sgRepair(task,cp(:,:,k),c,clock);
            out.repairSeconds=out.repairSeconds+info.seconds;
            bundle=struct('kind','spline','cp',fixed,'T',task.T);
            vt=tic; r=sgValidate(bundle,task,c,clock,job.budget); out.validationSeconds=out.validationSeconds+toc(vt);
            accepted=r.passed; reason=r.reason;
            if accepted
                [fixedScores,~]=sgRank(fixed,task); score=fixedScores(1,2);
                if toc(clock)<=job.budget, acceptedResult(bundle,r,'repair',k,score); end
            end
        catch e, reason=[e.identifier,': ',e.message]; end
        attempts(end+1)=struct('candidate',k,'stage','repair','passed',accepted,'seconds',toc(st),'reason',reason); %#ok<AGROW>
        if out.passed, break; end
    end
end
out.onlineSeconds=toc(clock); out.attempts=attempts; out.events=events;
if out.passed, out.reason='passed'; elseif isempty(out.reason), out.reason='no_certified_candidate'; end
out.scope='Online required checks included; best output uses the common 61-point score. Research audit and 5ms quality metrics follow separately.';
    function acceptedResult(b,r,stage,k,score)
        if ~isfinite(score), return; end
        elapsed=toc(clock);
        if ~out.passed, out.firstFeasibleSeconds=elapsed; out.firstBundle=b; end
        out.passed=true;
        if score<best
            best=score; out.bundle=b; out.report=r; out.candidate=k; out.acceptedStage=stage; out.bestOnlineScore=score;
            events(end+1)=struct('seconds',elapsed,'stage',stage,'candidate',k,'onlineScore',score); %#ok<AGROW>
            out.eventBundles{end+1}=b;
            % Save every improved accepted reference before another NLP attempt.
            checkpoint=out; checkpoint.events=events; checkpoint.attempts=attempts;
            checkpoint.onlineSeconds=toc(clock); referenceFingerprint=c.referenceFingerprint;
            temporary=[c.referenceCheckpointFile,'.tmp.mat'];
            save(temporary,'checkpoint','referenceFingerprint','-v7');
            movefile(temporary,c.referenceCheckpointFile,'f');
        end
    end
end
