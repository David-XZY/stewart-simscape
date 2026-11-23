function out=c3Plan(task,job,c,asset,standard,history)
if nargin<6, history=[]; end
clock=tic; c.clockHandle=clock; c.wallBudget=job.budget;
names={'inputPreparation','boundary','generation','screening','physical','quality','repair','solverInitialization','solverBuild','solver','other'};
timing=struct(); for k=1:numel(names), timing.(names{k})=0; end
out=struct('passed',false,'physicalPassed',false,'qualityPassed',false,'reason','no_qualified_output', ...
    'bundle',[],'report',[],'quality',[],'score',[],'warm',[],'firstPhysicalSeconds',NaN, ...
    'firstQualifiedSeconds',NaN,'acceptedStage','','candidate',NaN,'attempts',{{}},'candidateCount',0, ...
    'candidates',[],'bestUnqualified',[],'standardHash',standard.hash,'historyUsed',~isempty(history));
allCp=zeros(6,16,0); allScores=zeros(0,2); physical=false(0,1); qualities=inf(0,1); best=inf;
st=tic; task=sgFinalizeTask(task); timing.boundary=toc(st);
if ~task.endpointPassed, out.reason='boundary_rejected'; finish(); return; end
isSolver=~isempty(strfind(job.method,'ihsid')); %#ok<STREMP>
if strncmp(job.method,'geo',3), kind='geometric'; elseif strncmp(job.method,'retrieve',8), kind='retrieval'; else, kind='diffusion'; end
if strcmp(job.method,'previous_ihsid'), kind='retrieval'; end
maxCount=job.count; if isSolver, maxCount=c.solverStarts; end
totals=c.batchTotals(c.batchTotals<maxCount); totals=[totals,maxCount]; if isSolver, totals=maxCount; end
if isfield(job,'fullSweep') && job.fullSweep, totals=maxCount; end
for total=totals
    if expired(), break; end
    ids=(size(allCp,3)+1):total; st=tic;
    [cp,generation]=c3Candidates(kind,task,ids,job.seed,asset); %#ok<NASGU>
    if strcmp(job.method,'previous_ihsid') && ~isempty(history) && ids(1)==1
        tr=c3TransportWarm(history.bundle,task,linspace(0,task.T,201));
        [B,~,~]=sgBasis(linspace(0,task.T,201),task.T); seed=dpBaseline(task); seed(:,4:13)=0;
        seed(:,4:13)=(tr.q-seed*B)/B(4:13,:); cp(:,:,1)=seed;
    end
    timing.generation=timing.generation+toc(st);
    [scores,order,screenSeconds,sortSeconds]=c3Rank(cp,task,c,standard);
    timing.screening=timing.screening+screenSeconds; timing.other=timing.other+sortSeconds;
    allCp=cat(3,allCp,cp); allScores=[allScores;scores]; physical=[physical;false(numel(ids),1)]; qualities=[qualities;inf(numel(ids),1)];
    if strcmp(job.method,'previous_ihsid') && ~isempty(history), order=[1;order(order~=1)]; end
    for ix=order(:).'
        k=ids(ix); if expired(), break; end
        if isSolver
            warm=[]; if strcmp(job.method,'previous_ihsid') && ix==1, warm=history; end
            solveCandidate(cp(:,:,ix),k,warm);
        else
            evaluate(struct('kind','spline','cp',cp(:,:,ix),'T',task.T),'direct',k,[],[]);
        end
        if out.passed && ~(isfield(job,'fullSweep') && job.fullSweep), finish(); return; end
    end
end
allowRepair=~isSolver && (~isfield(job,'repairEnabled') || job.repairEnabled);
if ~out.passed && allowRepair && ~isempty(allScores)
    st=tic; ranks=[~physical,allScores(:,1),qualities,allScores(:,2)]; [~,order]=sortrows(ranks,[1,2,3,4]); timing.other=timing.other+toc(st);
    for ix=1:min(c.repairCandidates,numel(order))
        if expired(), break; end
        k=order(ix); st=tic;
        try
            [cp,info]=c3Repair(task,allCp(:,:,k),c,standard,clock);
        catch e
            timing.repair=timing.repair+toc(st); failure('repair',k,e); continue;
        end
        timing.repair=timing.repair+toc(st);
        evaluate(struct('kind','spline','cp',cp,'T',task.T),'repair',k,[],info);
        if out.passed, finish(); return; end
    end
end
allowFallback=~isSolver && (~isfield(job,'fallbackEnabled') || job.fallbackEnabled);
if ~out.passed && allowFallback && ~isempty(allScores)
    st=tic; [~,order]=sortrows([~physical,allScores(:,1),qualities,allScores(:,2)],[1,2,3,4]); timing.other=timing.other+toc(st);
    for ix=1:min(c.solverStarts,numel(order))
        if expired(), break; end
        k=order(ix); solveCandidate(allCp(:,:,k),k,[]);
        if out.passed, finish(); return; end
    end
end
finish();
    function yes=expired()
        yes=toc(clock)>=c.wallBudget; if yes && ~out.passed, out.reason='wall_budget'; end
    end
    function solveCandidate(seed,k,warm)
        st=tic;
        try
            c.maxCpuSeconds=min(120,max(.1,c.wallBudget-toc(clock)-1));
            solver=c3Solve(task,seed,c,warm);
        catch e
            timing.other=timing.other+toc(st); failure('ihsid_exception_unclassified',k,e); return;
        end
        timing.solverBuild=timing.solverBuild+solver.buildSeconds;
        timing.solverInitialization=timing.solverInitialization+solver.initializationSeconds;
        timing.solver=timing.solver+solver.solveSeconds;
        timing.physical=timing.physical+solver.validationSeconds;
        timing.other=timing.other+max(0,toc(st)-solver.initializationSeconds-solver.buildSeconds-solver.solveSeconds-solver.validationSeconds);
        evaluate(solver.bundle,'ihsid',k,solver,[]);
    end
    function evaluate(bundle,stage,k,solver,repairInfo)
        attempt=struct('candidate',k,'stage',stage,'physicalPassed',false,'qualityPassed',false,'passed',false,'reason','','seconds',toc(clock));
        if isempty(solver)
            st=tic; r=sgValidate(bundle,task,c,clock,c.wallBudget); timing.physical=timing.physical+toc(st);
            eligible=r.passed;
        else
            r=solver.report; eligible=solver.passed;
            attempt.solver=rmfield(solver,{'bundle','report','sourceSeed','warm'});
        end
        attempt.physicalPassed=r.passed; attempt.reason=r.reason; physical(k)=r.passed;
        q=[]; score=[];
        if r.passed
            if isnan(out.firstPhysicalSeconds), out.firstPhysicalSeconds=toc(clock); end
            out.physicalPassed=true;
            st=tic; q=c3Quality(bundle,task,c); score=c3Score(q,task.family,standard); timing.quality=timing.quality+toc(st);
            attempt.qualityPassed=score.qualityPassed; attempt.scores=score.scores; qualities(k)=score.costs(standard.primary);
            attempt.passed=eligible && score.qualityPassed && ~expired();
            if ~score.qualityPassed, attempt.reason='quality_below_standard'; end
            if ~eligible && ~isempty(solver), attempt.reason=['solver_gate:',solver.solverStatus]; end
            if qualities(k)<best
                best=qualities(k); out.bestUnqualified=struct('bundle',bundle,'quality',q,'score',score,'stage',stage,'candidate',k);
            end
        end
        if ~isempty(repairInfo), attempt.repair=repairInfo; end
        if attempt.passed && (~out.passed || score.primaryScore>out.score.primaryScore)
            if isnan(out.firstQualifiedSeconds), out.firstQualifiedSeconds=toc(clock); end
            out.passed=true; out.qualityPassed=true; out.reason='passed'; out.bundle=bundle; out.report=r; out.quality=q; out.score=score;
            out.acceptedStage=stage; out.candidate=k;
            if ~isempty(solver), out.warm=solver.warm; else, out.warm=struct('bundle',bundle); end
        end
        attempt.seconds=toc(clock); out.attempts{end+1}=attempt; checkpoint();
    end
    function failure(stage,k,e)
        out.attempts{end+1}=struct('candidate',k,'stage',stage,'passed',false,'physicalPassed',false,'qualityPassed',false, ...
            'reason',[e.identifier,': ',e.message],'seconds',toc(clock)); checkpoint();
    end
    function checkpoint()
        if isfield(c,'progressFile')
            progress=struct('elapsed',toc(clock),'timing',timing,'attempts',{out.attempts},'passed',out.passed);
            dpWriteJson(c.progressFile,progress);
            if out.passed
                checkpointOut=out; save([c.progressFile,'.mat'],'checkpointOut','-v7');
            end
        end
    end
    function finish()
        out.candidates=allCp; out.candidateCount=size(allCp,3);
        if isempty(out.bestUnqualified) && ~out.passed && ~isempty(allScores)
            [~,least]=min(allScores(:,1));
            out.bestUnqualified=struct('bundle',struct('kind','spline','cp',allCp(:,:,least),'T',task.T), ...
                'quality',[],'score',[],'stage','rejected_candidate','candidate',least);
        end
        out.timing=timing; total=toc(clock);
        exclusive=sum(structfun(@double,timing)); out.timing.other=timing.other+(total-exclusive);
        out.evaluationSeconds=timing.boundary+timing.screening+timing.physical+timing.quality;
        out.onlineSeconds=total; out.generationSeconds=total-out.evaluationSeconds;
        out.timingClosureError=total-sum(structfun(@double,out.timing));
        if total>c.wallBudget, out.passed=false; out.reason='wall_budget'; end
        assert(out.timing.other>=-1e-6 && out.generationSeconds>=-1e-6,'C3:TimerOverlap','Overlapping timing sections.');
    end
end
