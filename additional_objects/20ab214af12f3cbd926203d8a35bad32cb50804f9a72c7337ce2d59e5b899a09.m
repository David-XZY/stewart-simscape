function out=bdPlan(task,method,B,seed,asset,c)
% Complete online call. All acceptance and returned bundle construction count.
clock=tic; events={}; pool={}; solverEvidence=[]; accepted=false;
out=struct('taskId',task.taskId,'taskHash',task.hash,'method',method,'B',B,'T',task.T, ...
    'seed',seed,'returnPolicy',c.returnPolicy,'passed',false,'physicalPassed',false, ...
    'qualityPassed',false,'reason','no_qualified_output','bundle',[],'report',[], ...
    'candidateGenerated',0,'candidateChecked',0,'returnStage','','firstPhysicalFoundTime',NaN, ...
    'firstQualityFoundTime',NaN,'actualReturnTime',NaN,'solver',[],'attempts',{{}});
st=tic; endpoint=bdEndpointCheck(task,c); event('endpoint',st);
if ~endpoint.passed, out.reason='boundary_rejected'; finish(); return; end
directEnd=B*c.stageFractions(1); repairEnd=B*sum(c.stageFractions(1:2));
if strcmp(method,'M0'), directEnd=B; end
last=0;
for total=c.batchTotals
    if toc(clock)>=directEnd, break; end
    st=tic; [generated,gt]=bdCandidates(method,task,last+1:total,asset,clock,directEnd,c,seed); elapsed=toc(st);
    names=fieldnames(gt); subtotal=0; for gi=1:numel(names), events{end+1}=struct('stage',names{gi},'seconds',gt.(names{gi})); subtotal=subtotal+gt.(names{gi}); end
    events{end+1}=struct('stage','generation_other','seconds',max(0,elapsed-subtotal)); last=total;
    for j=1:numel(generated)
        pool{end+1}=generated{j}; out.candidateGenerated=out.candidateGenerated+1;
        if toc(clock)>=directEnd, break; end
        b=generated{j}.bundle;
        if strcmp(method,'M0')
            if solve(b,directEnd,'ihsid'), finish(); return; end
        else
            if evaluate(b,'direct',[]), finish(); return; end
            if any(strcmp(method,{'M2_H','M2_C'})) && toc(clock)<directEnd
                st=tic;
                try
                    [timed,info]=bdRetime(b.path,task,clock,directEnd,c); event('retiming',st);
                    if info.solverSuccess && evaluate(timed,'retimed',[]), finish(); return; end
                catch e, event('retiming',st); failure('retiming',e); end
            end
        end
    end
    if isempty(generated), break; end
end
% Deterministic ranking: physical feasibility, then normalized violation score.
if ~isempty(pool) && ~strcmp(method,'M0')
    st=tic; keys=zeros(numel(pool),2);
    for k=1:numel(pool)
        keys(k,:)=[1,inf];
        if isfield(pool{k},'rank'), keys(k,:)=pool{k}.rank; end
    end
    [~,order]=sortrows(keys,[1,2]); event('selection',st);
    for k=order(1:min(2,numel(order))).'
        if toc(clock)>=repairEnd, break; end
        st=tic;
        try
            [b,info]=bdRepair(pool{k}.bundle,task,clock,repairEnd,c); event('repair',st);
            if evaluate(b,'repair',[]), finish(); return; end
        catch e, event('repair',st); failure('repair',e); end
    end
    for k=order(1:min(8,numel(order))).'
        if toc(clock)>=B, break; end
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
            sol=bdSolve(task,cp,cc); elapsed=toc(st);
            parts={'initializationSeconds','buildSeconds','solveSeconds','validationSeconds'}; names={'solver_initialization','solver_build','solver','physical'}; subtotal=0;
            for ii=1:4, events{end+1}=struct('stage',names{ii},'seconds',sol.(parts{ii})); subtotal=subtotal+sol.(parts{ii}); end
            events{end+1}=struct('stage','solver_other','seconds',max(0,elapsed-subtotal));
            entry=struct('stage',stage,'passed',sol.passed,'elapsed',toc(clock),'reason',sol.solverStatus,'physicalPassed',sol.report.passed,'maxEquality',sol.maxEquality,'maxInequality',sol.maxInequality,'maxBounds',sol.maxBounds,'buildSeconds',sol.buildSeconds,'solveSeconds',sol.solveSeconds,'cacheHit',sol.problemCacheHit); out.attempts{end+1}=entry;
            if sol.passed, yes=evaluate(sol.bundle,stage,sol); end
        catch e, event('ihsid_including_internal_acceptance',st); failure(stage,e); end
    end
    function yes=evaluate(b,stage,sol)
        st=tic; [yes,r]=bdAccept(b,task,c.returnPolicy,clock,B,c,sol); elapsed=toc(st); events{end+1}=struct('stage','physical','seconds',r.physicalSeconds); events{end+1}=struct('stage','quality','seconds',r.qualitySeconds); events{end+1}=struct('stage','acceptance_other','seconds',max(0,elapsed-r.physicalSeconds-r.qualitySeconds));
        out.candidateChecked=out.candidateChecked+1;
        entry=struct('stage',stage,'elapsed',toc(clock),'passed',yes,'physicalPassed',r.physicalPassed,'qualityPassed',r.qualityPassed,'reason',r.reason);
        out.attempts{end+1}=entry;
        if r.physicalPassed && isnan(out.firstPhysicalFoundTime), out.firstPhysicalFoundTime=toc(clock); end
        if r.qualityPassed && isnan(out.firstQualityFoundTime), out.firstQualityFoundTime=toc(clock); end
        if ~yes && ~isempty(pool) && toc(clock)<B
            pool{end}.rank=[double(~r.physicalPassed),violation(b)];
        end
        if yes
            accepted=true; out.bundle=b; out.report=r; out.physicalPassed=true; out.qualityPassed=r.qualityPassed;
            out.returnStage=stage; out.reason='passed'; solverEvidence=sol;
        end
    end
    function score=violation(b)
        st=tic;
        try
            tr=bdReconstruct(b,linspace(0,task.T,31)); p=sgPhysics(task.model); m=task.model;
            [L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); g=sgClearanceBatch(tr.q,task);
            v=[max(0,(m.lmin-full(L))/.1);max(0,(full(L)-m.lmax)/.1);max(0,abs(full(Ld))./m.actuator.ldotMax-1);max(0,abs(full(Ldd))./m.actuator.lddotMax-1);max(0,abs(full(F))/2000-1);max(0,(.01-g)/.01)];
            score=sum(v(:).^2); if ~isfinite(score), score=inf; end
        catch, score=inf; end
        event('candidate_ranking',st);
    end
    function failure(stage,e)
        out.attempts{end+1}=struct('stage',stage,'passed',false,'elapsed',toc(clock),'reason',[e.identifier,': ',e.message]);
    end
    function event(name,st)
        events{end+1}=struct('stage',name,'seconds',toc(st));
    end
    function finish()
        out.solver=solverEvidence; out.events=events; out.actualReturnTime=toc(clock);
        elapsed=0; for z=1:numel(events), elapsed=elapsed+events{z}.seconds; end
        out.otherSeconds=out.actualReturnTime-elapsed;
        assert(out.otherSeconds>=-1e-6,'BD:TimingOverlap','Mutually exclusive events overlap.');
        out.returnedWithinBudget=out.actualReturnTime<=B; out.passed=accepted&&out.returnedWithinBudget;
        out.timeoutOverrun=max(0,out.actualReturnTime-B);
        if ~out.returnedWithinBudget, out.reason='timeout'; out.passed=false; end
    end
end
