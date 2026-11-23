function report=sgAudit(bundle,task,c)
% No reuse of NLP normals. Numerical dense acceptance, not continuous proof.
timer=tic; t=linspace(0,task.T,ceil(task.T/c.denseStep)+1);
if strcmp(bundle.kind,'ihsid')
    % Include both one-sided acceleration limits at knots.
    knots=bundle.tNode(2:end-1); t=unique([t,knots,max(0,knots-1e-9)]);
end
tr=dpReconstruct(bundle,t); clearance=distances(tr.q);
close=find(min(clearance,[],1)<.012); extra=[];
for k=close
    left=t(max(1,k-1)); right=t(min(numel(t),k+1));
    extra=[extra,linspace(left,right,max(2,ceil((right-left)/c.nearStep)+1))]; %#ok<AGROW>
end
if ~isempty(extra)
    t=unique([t,extra]); tr=dpReconstruct(bundle,t); clearance=distances(tr.q);
end
p=dpPhysics(task.model); m=task.model;
[L,Ld,Ldd,F,J,W]=p.point(tr.q,tr.v,tr.a); L=full(L); Ld=full(Ld); Ldd=full(Ldd); F=full(F); J=full(J); W=full(W);
svmin=inf; condmax=0; residual=0;
for k=1:numel(t)
    jk=J(:,(k-1)*6+(1:6)); sv=svd(jk*diag([1,1,1,2,2,2]));
    svmin=min(svmin,min(sv)); condmax=max(condmax,max(sv)/min(sv));
    if isfield(tr,'F'), residual=max(residual,max(abs(jk.'*tr.F(:,k)-W(:,k)))); end
end
legViolation=max([0;reshape(m.lmin-L,[],1);reshape(L-m.lmax,[],1);reshape(abs(Ld)-m.actuator.ldotMax,[],1);reshape(abs(Ldd)-m.actuator.lddotMax,[],1)]);
forceViolation=max([0;reshape(m.actuator.forceMin-F,[],1);reshape(F-m.actuator.forceMax,[],1)]);
if isfield(tr,'F'), forceViolation=max([forceViolation;reshape(m.actuator.forceMin-tr.F,[],1);reshape(tr.F-m.actuator.forceMax,[],1)]); else, tr.F=F; end
boundary=max(abs([tr.q(:,1)-task.q0;tr.q(:,end)-task.qGoal;reshape(tr.v(:,[1,end]),[],1);reshape(tr.a(:,[1,end]),[],1)]));
finite=all(isfinite([tr.q(:);tr.v(:);tr.a(:);tr.F(:);L(:);Ld(:);Ldd(:);F(:);J(:);W(:);clearance(:)]));
report=struct('passed',finite && min(clearance(:))>=.01-1e-8 && legViolation<=1e-6 && ...
    forceViolation<=1e-6 && svmin>=.1 && condmax<=20 && boundary<=1e-6, ...
    'minClearance',min(clearance(:)),'maxLegViolation',legViolation,'maxForceViolation',forceViolation, ...
    'minSigma',svmin,'maxCondition',condmax,'boundaryError',boundary,'maxDenseDynamicResidual',residual, ...
    'sampleCount',numel(t),'time',t,'clearance',clearance,'L',L,'Ld',Ld,'Ldd',Ldd, ...
    'FRequired',F,'FInterpolated',tr.F,'q',tr.q,'v',tr.v,'a',tr.a, ...
    'scope','Cylinder/environment, numerical dense acceptance; dense dynamics residual is diagnostic.');
report.seconds=toc(timer);
    function ds=distances(qs)
        ds=zeros(numel(task.scene.hood.obstacles),size(qs,2));
        for i=1:size(qs,2), ds(:,i)=dpClearance(qs(:,i),task,true).'; end
    end
end
