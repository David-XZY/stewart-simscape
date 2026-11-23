function [u,d]=ch5TeacherIntervalProjectV62(x,previous,nominal,o,r,clf,cbf,guess)
% Preserve the original seven-variable CLF-CBF-QP; add sampled hold constraints.
% Nonlinear interval and transient checks remain mandatory, no CBF slack added.
cfg=r.strict;u=guess;attempts={};baseCbf=cbf;
for iteration=1:4
    [check,~]=ch5ReducedHoldAuditV22(x,previous,u,o,r);
    hard=max([baseCbf.A*u-baseCbf.b;0]);
    if check.accepted&&hard<=10*cfg.solver.constraintTolerance
        d=struct('feasible',true,'usedFallback',false,'exitflag',1,'interval',check,'attempts',{attempts}, ...
            'fallbackSafetyCertified',false,'cbf',baseCbf,'clf',clf,'status','sampled_hold_checked');return;
    end
    kernel=intervalKernel(r);[g,J]=kernel(x,u,o);g=full(g);J=full(J);
    augmented=baseCbf;augmented.A=[baseCbf.A;J];augmented.b=[baseCbf.b;-g+J*u];
    augmented.names=[baseCbf.names;"hold_"+string((1:numel(g))')];
    [candidate,solve]=solveStrictClfCbfQp(nominal,previous,clf,augmented,cfg);
    attempts{end+1}=struct('solve',solve,'maximum_interval_residual',max(g),'input_change_N',norm(candidate-u,inf));
    u=candidate;
    if ~solve.feasible,break;end
end
[check,~]=ch5ReducedHoldAuditV22(x,previous,u,o,r);hard=max([baseCbf.A*u-baseCbf.b;0]);
d=struct('feasible',check.accepted&&hard<=10*cfg.solver.constraintTolerance,'usedFallback',true,'exitflag',-1,'interval',check, ...
    'attempts',{attempts},'fallbackSafetyCertified',false,'cbf',baseCbf,'clf',clf,'status','sampled_hold_projection_failed');
end
function kernel=intervalKernel(r)
persistent cache
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);if isKey(cache,key),kernel=cache(key);return;end
import casadi.*
x=MX.sym('teacher_hold_x',r.plant.n);u=MX.sym('teacher_hold_u',6);o=MX.sym('teacher_hold_o',22,8);
% Differentiate the original non-JIT graph. Differentiating a loaded JIT
% wrapper inherits its DLL name and attempts to relink a Windows-loaded DLL.
[~,trace]=r.plant.fd(x,u,MX.zeros(6,1));parts=cell(1,size(trace,2));
for j=1:numel(parts),parts{j}=r.plant.mechanicalValues(trace(:,j),u);end
values=horzcat(parts{:});
lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
scale=[ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2;1000*ones(6,1)];
finiteHi=find(isfinite(hi));g=[(values(finiteHi,:)-hi(finiteHi))./scale(finiteHi);(lo-values)./scale];g=reshape(g,numel(g),1);
for j=1:size(trace,2)
    ot=[o(1:3,:)+o(16:18,:)*r.plant.traceTime(j);o(4:22,:)];
    for oi=1:8
        gap=r.collision.single(trace(1:6,j),ot(:,oi));
        g=[g;if_else(o(22,oi)>.5,(r.c.safe_distance+r.c.sample_hold_margin-gap)/.1,-1)]; %#ok<AGROW>
    end
end
kernel=Function('ch5_teacher_interval_projection',{x,u,o},{g,jacobian(g,u)},ch5ProjectionJitOptions());cache(key)=kernel;
end
