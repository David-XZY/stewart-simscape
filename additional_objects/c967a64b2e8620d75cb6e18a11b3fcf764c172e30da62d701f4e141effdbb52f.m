function out=pgPlan(task,method,B,seed,asset,c)
% Complete online call. All acceptance and returned bundle construction count.
clock=tic; clear pgPenalty pgDetourPenalty; events={}; pool={}; solverEvidence=[]; accepted=false; activeCandidate=0;
out=struct('taskId',task.taskId,'taskHash',task.hash,'modelHash',task.modelHash,'method',method,'B',B,'T',task.T, ...
    'seed',seed,'returnPolicy',c.returnPolicy,'passed',false,'physicalPassed',false, ...
    'qualityPassed',false,'reason','no_qualified_output','bundle',[],'report',[], ...
    'candidateRequested',0,'candidateGenerated',0,'candidateChecked',0,'candidateEvents',{{}},'bestUnacceptedBundle',[],'selectedCandidateId',NaN,'returnStage','','firstPhysicalFoundTime',NaN, ...
    'firstQualityFoundTime',NaN,'actualReturnTime',NaN,'signatureSeconds',0,'signatureValidity',[],'guidanceCalls',0,'guidanceSeconds',0,'solver',[],'attempts',{{}});
st=tic; task=pgFinalizeTask(task,task.payload,c); endpoint=struct('passed',task.endpointPassed); event('input_and_endpoint',st);
if ~endpoint.passed, out.reason='boundary_rejected'; finish(); return; end
st=tic;
if startsWith(method,'retrieval')
    assert(isfield(asset,'library'),'PG:Library','A training-only source library is required.');
    for li=1:numel(asset.library), assert(any(strcmp(asset.library{li}.task.split,{'train','dev_train'})),'PG:Leakage','Nontraining history in independent retrieval.'); end
    asset.signatureDistance=strcmp(method,'retrieval_signature');
    if asset.signatureDistance, [asset.callSignature,sm]=pgPhysicsSignature(task); out.signatureSeconds=sm.seconds; out.signatureValidity=sm.validity; end
    distances=pgDistance(task,asset); [~,asset.callOrder]=sort(distances,'ascend');
elseif ~strcmp(method,'ihsid')
    pgModelCheck(asset.model,method,c);
    if contains(method,'signature'), [sig,sm]=pgPhysicsSignature(task); asset.callContext=[pgEncodeContext(task,'basic');sig]; out.signatureSeconds=sm.seconds; out.signatureValidity=sm.validity;
    else, asset.callContext=pgEncodeContext(task,'basic'); end
    if endsWith(method,'no_accel')||endsWith(method,'no_gap')
        signature=reshape(asset.callContext(100:end),51,5);
        if endsWith(method,'no_accel'), signature(7:42,:)=0; else, signature(43:48,:)=0; end
        asset.callContext(100:end)=signature(:);
    end
end
event('encoding_and_retrieval',st);
directEnd=B*c.stageFractions(1); repairEnd=B*sum(c.stageFractions(1:2));
if strcmp(method,'ihsid'), directEnd=B; end
last=0;
totals=c.batchTotals;
if startsWith(method,'regression'), totals=1; elseif strcmp(method,'ihsid'), totals=[1,4,8]; end
for total=totals
    if toc(clock)>=directEnd, break; end
    out.candidateRequested=out.candidateRequested+total-last;
    st=tic; [generated,gt]=pgCandidates(method,task,last+1:total,asset,clock,directEnd,c,seed); elapsed=toc(st);
    names=fieldnames(gt); subtotal=0; for gi=1:numel(names), events{end+1}=struct('stage',names{gi},'seconds',gt.(names{gi})); subtotal=subtotal+gt.(names{gi}); end
    events{end+1}=struct('stage','generation_other','seconds',max(0,elapsed-subtotal)); last=total;
    batchStart=numel(pool); pool=[pool,generated];
    out.candidateGenerated=out.candidateGenerated+sum(cellfun(@(x)~isempty(x.bundle),generated));
    st=tic; rank=inf(numel(generated),3);
    for jj=1:numel(generated)
        out.candidateEvents{end+1}=generated{jj}.meta;
        out.guidanceCalls=out.guidanceCalls+generated{jj}.meta.guidanceCalls; out.guidanceSeconds=out.guidanceSeconds+generated{jj}.meta.guidanceSeconds;
        if ~isempty(generated{jj}.bundle)&&toc(clock)<directEnd
            if strcmp(method,'ihsid'), rank(jj,:)=[0,0,generated{jj}.meta.id];
            else, rank(jj,:)=[pgCoarse(generated{jj}.bundle,task,c),generated{jj}.meta.id]; end
            pool{batchStart+jj}.rank=rank(jj,1:2);
        end
    end
    [~,batchOrder]=sortrows(rank,[1,2,3]); event('coarse_ranking',st);
    for j=batchOrder(:).' 
        activeCandidate=batchStart+j;
        if toc(clock)>=directEnd, break; end
        b=generated{j}.bundle; if isempty(b), continue; end
        if strcmp(method,'ihsid')
            if solve(b,directEnd,'ihsid'), finish(); return; end
        else
            if evaluate(b,'direct',[]), finish(); return; end
            if startsWith(method,'retrieval') && toc(clock)<directEnd
                st=tic;
                try
                    [timed,info]=pgRetime(b.path,task,clock,directEnd,c); event('retiming',st);
                    if info.solverSuccess && evaluate(timed,'retimed',[]), finish(); return; end
                catch e, event('retiming',st); failure('retiming',e); end
            end
        end
    end
    if isempty(generated), break; end
end
% Deterministic ranking: physical feasibility, then normalized violation score.
if ~isempty(pool) && ~strcmp(method,'ihsid')
    st=tic; keys=zeros(numel(pool),2);
    for k=1:numel(pool)
        keys(k,:)=[1,inf];
        if isfield(pool{k},'rank'), keys(k,:)=pool{k}.rank; end
    end
    [~,order]=sortrows(keys,[1,2]); order=order(~cellfun(@(x)isempty(x.bundle),pool(order))); event('selection',st);
    if ~isempty(order), out.bestUnacceptedBundle=pool{order(1)}.bundle; end
    for k=order(1:min(2,numel(order))).'
        if toc(clock)>=repairEnd, break; end
        activeCandidate=k;
        st=tic;
        try
            [b,info]=pgRepair(pool{k}.bundle,task,clock,repairEnd,c); event('repair',st);
            if evaluate(b,'repair',[]), finish(); return; end
        catch e, event('repair',st); failure('repair',e); end
    end
    for k=order(1:min(8,numel(order))).'
        if toc(clock)>=B, break; end
        activeCandidate=k;
        if solve(pool{k}.bundle,B,'ihsid_fallback'), finish(); return; end
    end
end
finish();
    function yes=solve(b,deadline,stage)
        yes=false; st=tic;
        try
            if strcmp(b.kind,'spline'), cp=b.cp;
            else
                t=linspace(0,task.T,201); tr=bdReconstruct(b,t); [basis,~,~]=bdBasis(t,task.T,c.controlCount);
                cp=bdBaseline(task,c.controlCount); cp(:,4:end-3)=0; cp(:,4:end-3)=(tr.q-cp*basis)/basis(4:end-3,:);
            end
            cc=c; cc.clockHandle=clock; cc.wallBudget=deadline;
            sol=pgSolve(task,cp,cc); elapsed=toc(st);
            parts={'initializationSeconds','buildSeconds','solveSeconds','validationSeconds'}; names={'solver_initialization','solver_build','solver','physical'}; subtotal=0;
            for ii=1:4, events{end+1}=struct('stage',names{ii},'seconds',sol.(parts{ii})); subtotal=subtotal+sol.(parts{ii}); end
            events{end+1}=struct('stage','solver_other','seconds',max(0,elapsed-subtotal));
            entry=struct('stage',stage,'passed',sol.passed,'elapsed',toc(clock),'reason',sol.solverStatus,'physicalPassed',sol.report.passed,'maxEquality',sol.maxEquality,'maxInequality',sol.maxInequality,'maxBounds',sol.maxBounds,'buildSeconds',sol.buildSeconds,'solveSeconds',sol.solveSeconds,'cacheHit',sol.problemCacheHit); out.attempts{end+1}=entry;
            if sol.passed, yes=evaluate(sol.bundle,stage,sol); end
        catch e, event('ihsid_including_internal_acceptance',st); failure(stage,e); end
    end
    function yes=evaluate(b,stage,sol)
        st=tic; [yes,r]=pgAccept(b,task,c.returnPolicy,clock,B,c,sol); elapsed=toc(st); events{end+1}=struct('stage','physical','seconds',r.physicalSeconds); events{end+1}=struct('stage','quality','seconds',r.qualitySeconds); events{end+1}=struct('stage','acceptance_other','seconds',max(0,elapsed-r.physicalSeconds-r.qualitySeconds));
        out.candidateChecked=out.candidateChecked+1;
        entry=struct('stage',stage,'elapsed',toc(clock),'passed',yes,'physicalPassed',r.physicalPassed,'qualityPassed',r.qualityPassed,'reason',r.reason);
        out.attempts{end+1}=entry;
        if r.physicalPassed && isnan(out.firstPhysicalFoundTime), out.firstPhysicalFoundTime=toc(clock); end
        if r.qualityPassed && isnan(out.firstQualityFoundTime), out.firstQualityFoundTime=toc(clock); end
        if ~yes && activeCandidate>0, pool{activeCandidate}.rank(1)=double(~r.physicalPassed); end
        if yes
            accepted=true; out.bundle=b; out.report=r; out.physicalPassed=true; out.qualityPassed=r.qualityPassed;
            out.returnStage=stage; out.reason='passed'; solverEvidence=sol;
            if activeCandidate>0, out.selectedCandidateId=pool{activeCandidate}.meta.id; end
        end
    end
    function failure(stage,e)
        out.attempts{end+1}=struct('stage',stage,'passed',false,'elapsed',toc(clock),'reason',[e.identifier,': ',e.message]);
    end
    function event(name,st)
        events{end+1}=struct('stage',name,'seconds',toc(st));
    end
    function finish()
        out.solver=solverEvidence; out.events=events; out.payloadShape=task.payload.shape; out.payloadMass=task.payload.mass; out.payloadDimensions=task.payload.dimensions; out.derivedInertia=task.derived.inertiaCOM; out.contextSchema=task.contextSchema; out.baseSceneId=task.baseSceneId; out.split=task.split; out.codeHash=c.codeHash; out.actualReturnTime=toc(clock);
        elapsed=0; for z=1:numel(events), elapsed=elapsed+events{z}.seconds; end
        out.otherSeconds=out.actualReturnTime-elapsed;
        assert(out.otherSeconds>=-1e-6,'BD:TimingOverlap','Mutually exclusive events overlap.');
        out.returnedWithinBudget=out.actualReturnTime<=B; out.passed=accepted&&out.returnedWithinBudget;
        out.timeoutOverrun=max(0,out.actualReturnTime-B);
        if ~out.returnedWithinBudget, out.reason='timeout'; out.passed=false; end
    end
end
