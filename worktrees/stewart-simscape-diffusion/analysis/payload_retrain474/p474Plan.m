function [out,state]=p474Plan(task,method,B,seed,asset,c,state)
% Budget is remaining cumulative wall time; state belongs only to this call.
clock=tic; out=struct('passed',false,'physicalPassed',false,'reason','no_qualified_output', ...
    'returnStage','','bundle',[],'report',[],'elapsed',0,'events',{{}},'attempts',{{}},'nativeExhausted',false,'searchExhausted',false);
if nargin<8||isempty(state)
    state=struct('nextCandidate',1,'nativeExhausted',false,'pool',{{}},'warm',[], ...
        'cumulativeSeconds',0,'candidateGenerated',0,'candidateChecked',0,'ihsidStarted',false,'events',{{}},'attempts',{{}},'pending',{{}});
end
% The caller starts its clock before pgMakeTask and subtracts preparation.
st=tic;task=pgFinalizeTask(task,task.payload,c);event('endpoint',st);
if ~task.endpointPassed,out.reason='boundary_rejected';state.nativeExhausted=true;finish();return;end
directEnd=.5*B;repairEnd=.9*B;
pending=state.pending;state.pending={};
for vi=1:numel(pending)
    if evaluate(pending{vi}.bundle,pending{vi}.stage),finish();return;end
end
if strcmp(method,'ihsid')
    if ~state.nativeExhausted&&toc(clock)<repairEnd
        st=tic;cc=c;cc.clockHandle=clock;cc.wallBudget=repairEnd;
        cp=bdBaseline(task,c.controlCount);
        try,sol=pgSolve(task,cp,cc,state.warm);
        catch err
            if strcmp(err.identifier,'BD:Deadline'),event('ihsid',st);out.reason='time_slice_before_solver';finish();return;end
            rethrow(err);
        end
        event('ihsid',st);state.ihsidStarted=true;
        state.warm=sol.warm;
        interrupted=contains(sol.solverStatus,'Maximum_Iterations')||contains(sol.solverStatus,'Maximum_CpuTime')||contains(sol.solverStatus,'User_Requested_Stop')||contains(sol.solverStatus,'task_wall_budget');
        if contains(sol.solverStatus,'Invalid_Number')||contains(sol.solverStatus,'Internal_Error'),error('P474:SolverError','%s',sol.solverStatus);end
        state.nativeExhausted=~interrupted;
        out.attempts{end+1}=struct('stage','ihsid','solverStatus',sol.solverStatus,'iterations',sol.iterations,'equalityResidual',sol.maxEquality,'inequalityResidual',sol.maxInequality);
        if evaluate(sol.bundle,'ihsid'),finish();return;end
    end
    finish();return;
end
st=tic;
if startsWith(method,'retrieval')
    asset.signatureDistance=true;[asset.callSignature,~]=pgPhysicsSignature(task);
    [~,asset.callOrder]=sort(pgDistance(task,asset),'ascend');
else
    pgModelCheck(asset.model,method,c);
    asset.callContext=pgEncodeContext(task,'basic');
    if contains(method,'signature'),asset.callContext=[asset.callContext;pgPhysicsSignature(task)];end
end
event('encoding_and_retrieval',st);
% Revisit interrupted retrieval retiming, preserving time controls.
for i=1:numel(state.pool)
    p=state.pool{i};
    if isfield(p,'retimePending')&&p.retimePending&&toc(clock)<directEnd
        st=tic;[timed,info]=p474Retime(p.original.path,task,clock,directEnd,c,p.timeControls);event('retiming',st);
        state.pool{i}.timeControls=timed.timeControls;state.pool{i}.retimePending=info.interrupted;
        if evaluate(timed,'retimed'),finish();return;end
        state.pool{i}.bundle=timed;
        if toc(clock)<directEnd,st=tic;state.pool{i}.rank=pgCoarse(timed,task,c);event('ranking',st);end
    end
end
while ~state.nativeExhausted&&toc(clock)<directEnd
    id=state.nextCandidate;
    if startsWith(method,'regression')&&id>1,state.nativeExhausted=true;break;end
    if startsWith(method,'retrieval')&&id>4*numel(asset.callOrder),state.nativeExhausted=true;break;end
    st=tic;[generated,~]=pgCandidates(method,task,id,asset,clock,directEnd,c,seed);event('generation',st);
    if isempty(generated),break;end
    p=generated{1};
    if isempty(p.bundle)
        if contains(p.meta.failureReason,'deadline'),break;end
        error('P474:CandidateError','%s',p.meta.failureReason);
    end
    state.nextCandidate=id+1;state.candidateGenerated=state.candidateGenerated+1;
    p.original=p.bundle;p.exhausted=false;p.retimePending=false;p.timeControls=[];
    st=tic;p.rank=pgCoarse(p.bundle,task,c);event('ranking',st);
    state.pool{end+1}=p;k=numel(state.pool);
    if evaluate(p.bundle,'direct'),finish();return;end
    if startsWith(method,'retrieval')
        if toc(clock)<directEnd
            st=tic;[timed,info]=p474Retime(p.bundle.path,task,clock,directEnd,c,[]);event('retiming',st);
            state.pool{k}.timeControls=timed.timeControls;state.pool{k}.retimePending=info.interrupted;state.pool{k}.bundle=timed;
            if evaluate(timed,'retimed'),finish();return;end
            if toc(clock)<directEnd,st=tic;state.pool{k}.rank=pgCoarse(timed,task,c);event('ranking',st);end
        else,state.pool{k}.retimePending=true;
        end
    end
end
if startsWith(method,'regression')&&state.nextCandidate>1,state.nativeExhausted=true;end
if startsWith(method,'retrieval')&&state.nextCandidate>4*numel(asset.callOrder),state.nativeExhausted=true;end
% Repair candidates in feasibility order. A 40-iteration block is not a cap.
while toc(clock)<repairEnd
    ids=find(~cellfun(@(p)p.exhausted,state.pool));if isempty(ids),break;end
    rank=zeros(numel(ids),3);
    for j=1:numel(ids),rank(j,:)=[state.pool{ids(j)}.rank(:).',ids(j)];end
    [~,order]=sortrows(rank,[1,2,3]);k=ids(order(1));p=state.pool{k};st=tic;
    [b,info]=p474Repair(p.bundle,task,clock,repairEnd,c);event('repair',st);
    state.pool{k}.bundle=b;state.pool{k}.exhausted=info.exhausted;
    if evaluate(b,'repair'),finish();return;end
    if toc(clock)<repairEnd,st=tic;state.pool{k}.rank=pgCoarse(b,task,c);event('ranking',st);end
    if strcmp(info.reason,'time_slice'),break;end
end
finish();
    function yes=evaluate(b,stage)
        yes=false;if isempty(b),return;end
        if toc(clock)>=B,state.pending{end+1}=struct('bundle',b,'stage',stage);return;end
        st=tic;r=pgValidate(b,task,c,clock,B);event('physical',st);state.candidateChecked=state.candidateChecked+1;
        out.attempts{end+1}=struct('stage',stage,'reason',r.reason,'physicalPassed',r.passed,'elapsed',toc(clock));
        if strcmp(r.reason,'timeout'),state.pending{end+1}=struct('bundle',b,'stage',stage);end
        if r.passed&&toc(clock)<B,out.passed=true;out.physicalPassed=true;out.reason='passed';out.returnStage=stage;out.bundle=b;out.report=r;yes=true;end
    end
    function event(name,st),out.events{end+1}=struct('stage',name,'seconds',toc(st));end
    function finish()
        out.elapsed=toc(clock);state.cumulativeSeconds=state.cumulativeSeconds+out.elapsed;
        state.events=[state.events,out.events];state.attempts=[state.attempts,out.attempts];
        out.nativeExhausted=state.nativeExhausted;
        pending=any(cellfun(@(p)isfield(p,'retimePending')&&p.retimePending,state.pool));
        out.searchExhausted=state.nativeExhausted&&all(cellfun(@(p)p.exhausted,state.pool))&&~pending;
        if strcmp(method,'ihsid'),out.searchExhausted=state.nativeExhausted;end
        out.searchExhausted=out.searchExhausted&&isempty(state.pending);
        if ~task.endpointPassed,out.searchExhausted=true;end
        if out.elapsed>B,out.passed=false;out.reason='timeout';end
        out.candidateGenerated=state.candidateGenerated;out.candidateChecked=state.candidateChecked;
        if ~out.passed&&out.searchExhausted&&task.endpointPassed,out.reason='search_exhausted';end
    end
end
