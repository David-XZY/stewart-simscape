function report=pgValidate(bundle,task,c,clockHandle,budget)
% Early rejection, identical dense grid, no incomplete acceptance.
if nargin<4, clockHandle=tic; budget=inf; end
timer=tic; report=emptyReport();
if ~isfield(bundle,'T') || ~isfinite(bundle.T) || abs(bundle.T-task.T)>1e-12*max(1,task.T)
    report.reason='duration_mismatch'; report.seconds=toc(timer); return;
end
if strcmp(bundle.kind,'path_time_map') && ~isempty(bundle.timeControls)
    d=bundle.timeControls(:);
    if numel(d)<6 || any(~isfinite(d)) || any(diff(d)<-1e-10) || max(abs(d(1:3)))>1e-10 || max(abs(d(end-2:end)-1))>1e-10
        report.reason='time_map_contract'; report.seconds=toc(timer); return;
    end
end
t=linspace(0,task.T,ceil(task.T/c.denseStep)+1);
if strcmp(bundle.kind,'ihsid')
    knots=bundle.tNode(2:end-1); t=unique([t,knots,max(0,knots-1e-9)]);
end
[ok,data,reason]=check(t);
if ~ok, report.reason=reason; report.seconds=toc(timer); return; end
near=[]; if ~isempty(data.clearance), near=find(min(data.clearance,[],1)<.012); end; extra=[];
for k=near(:).'
    left=t(max(1,k-1)); right=t(min(numel(t),k+1));
    extra=[extra,linspace(left,right,max(2,ceil((right-left)/c.nearStep)+1))]; %#ok<AGROW>
end
extra=setdiff(unique(extra),t);
if ~isempty(extra)
    [ok,new,reason]=check(extra);
    if ~ok, report.reason=reason; report.seconds=toc(timer); return; end
    allTime=[t,extra]; [t,order]=sort(allTime);
    for name={'q','v','a','FRequired','FInterpolated','L','Ld','Ldd','clearance'}
        val=[data.(name{1}),new.(name{1})]; data.(name{1})=val(:,order);
    end
    data.minSigma=min(data.minSigma,new.minSigma); data.maxCondition=max(data.maxCondition,new.maxCondition);
    data.maxDenseDynamicResidual=max(data.maxDenseDynamicResidual,new.maxDenseDynamicResidual);
end
boundary=max(abs([data.q(:,1)-task.q0;data.q(:,end)-task.qGoal;reshape(data.v(:,[1,end]),[],1);reshape(data.a(:,[1,end]),[],1)]));
report=data; report.time=t; report.sampleCount=numel(t); report.boundaryError=boundary;
report.minClearance=inf; if ~isempty(data.clearance), report.minClearance=min(data.clearance(:)); end; report.maxLegViolation=0; report.maxForceViolation=0;
report.passed=boundary<=1e-6 && toc(clockHandle)<=budget; report.reason='passed';
if boundary>1e-6, report.reason='boundary'; elseif toc(clockHandle)>budget, report.reason='timeout'; end
report.seconds=toc(timer); report.scope='Complete dense physical check; no continuous-time proof.';
    function [ok,d,reason]=check(times)
        ok=false; d=[]; reason='timeout'; if toc(clockHandle)>budget, return; end
        reason='nonfinite_or_invalid_trajectory';
        try, tr=bdReconstruct(bundle,times); catch, return; end
        reason='nonfinite';
        if any(~isfinite([tr.q(:);tr.v(:);tr.a(:)])), return; end
        p=pgPhysics(task.model); m=task.model;
        [L,Ld,Ldd,F,J,W]=p.point(tr.q,tr.v,tr.a);
        L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F); J=full(J); W=full(W);
        if ~isfield(tr,'F'), tr.F=F; end
        if any(~isfinite([L(:);Ld(:);Ldd(:);F(:);tr.F(:);J(:);W(:)])), return; end
        reason='leg_kinematics';
        if any(any(L<m.lmin-1e-6 | L>m.lmax+1e-6 | abs(Ld)>m.actuator.ldotMax+1e-6 | abs(Ldd)>m.actuator.lddotMax+1e-6)), return; end
        reason='force';
        if any(any(F<m.actuator.forceMin-1e-6 | F>m.actuator.forceMax+1e-6 | tr.F<m.actuator.forceMin-1e-6 | tr.F>m.actuator.forceMax+1e-6)), return; end
        lo=inf; hi=0; residual=0;
        for v=1:numel(times)
            jj=J(:,(v-1)*6+(1:6)); sv=svd(jj*diag([1,1,1,2,2,2]));
            lo=min(lo,min(sv)); hi=max(hi,max(sv)/min(sv)); residual=max(residual,max(abs(jj.'*tr.F(:,v)-W(:,v))));
        end
        reason='singularity'; if lo<.1 || hi>20, return; end
        clearance=pgClearanceBatch(tr.q,task); reason='clearance_not_certified';
        uncertain=[]; if ~isempty(clearance), uncertain=find(min(clearance,[],1)<.01-1e-9); end
        for v=uncertain(:).'
            if toc(clockHandle)>budget, reason='timeout'; return; end
            clearance(:,v)=pgClearance(tr.q(:,v),task,true).';
            if min(clearance(:,v))<.01-1e-8, return; end
        end
        if ~isempty(clearance) && (any(~isfinite(clearance(:))) || min(clearance(:))<.01-1e-8), return; end
        if toc(clockHandle)>budget, reason='timeout'; return; end
        coverage=bdRobotEnvelope(task,tr.q,c);
        if ~coverage.passed, reason='robot_envelope_not_certified'; return; end
        d=struct('q',tr.q,'v',tr.v,'a',tr.a,'FRequired',F,'FInterpolated',tr.F, ...
            'L',L,'Ld',Ld,'Ldd',Ldd,'clearance',clearance,'minSigma',lo,'maxCondition',hi, ...
            'maxDenseDynamicResidual',residual); ok=true; reason='passed';
    end
end
function r=emptyReport()
r=struct('passed',false,'reason','not_checked','seconds',0,'minClearance',NaN, ...
    'maxLegViolation',NaN,'maxForceViolation',NaN,'minSigma',NaN,'maxCondition',NaN, ...
    'boundaryError',NaN,'sampleCount',0,'maxDenseDynamicResidual',NaN);
end
