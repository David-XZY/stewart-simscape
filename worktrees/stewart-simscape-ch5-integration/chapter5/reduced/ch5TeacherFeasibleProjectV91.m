function [u,d]=ch5TeacherFeasibleProjectV91(x,previous,nominal,o,r,clf,cbf,seed)
% Solve the same force/rate/CLF objective from an audited feasible centre.
% A shrinking force trust region limits nonlinear linearization error.
u=seed;attempts={};[kernel,fixed,tolerance]=intervalKernel(r);[g,J]=kernel(x,seed,o);g=full(g);J=full(J);
% Known initial/delay states do not depend on the present command. They
% must pass exactly the existing physical audit tolerance; contradictory
% zero-Jacobian rounding rows do not belong to the decision-variable QP.
if any(g(fixed)>tolerance(fixed))
    d=struct('feasible',false,'exitflag',-1,'usedFallback',true,'attempts',{{}}, ...
        'status','uncontrollable_known_state_failed','fallbackSafetyCertified',false);return;
end
removed=sum(fixed);g=g(~fixed);J=J(~fixed,:);
augmented=cbf;augmented.A=[cbf.A;J];augmented.b=[cbf.b;-g+J*seed];
augmented.names=[cbf.names;"hold_"+string((1:numel(g))')];
for radius=[40,20,10,5,1]
    cfg=r.strict;cfg.forceMin=max(cfg.forceMin,seed-radius);cfg.forceMax=min(cfg.forceMax,seed+radius);
    [candidate,solve]=solveStrictClfCbfQp(nominal,previous,clf,augmented,cfg);
    [check,~]=ch5TeacherHoldAuditV91(x,previous,candidate,o,r);
    hard=max([cbf.A*candidate-cbf.b;0]);
    attempts{end+1}=struct('radius_N',radius,'solve',solve,'check',check,'hard_cbf_violation',hard);
    if solve.feasible&&check.accepted&&hard<=10*r.strict.solver.constraintTolerance
        u=candidate;d=struct('feasible',true,'exitflag',1,'usedFallback',false,'interval',check,'attempts',{attempts}, ...
            'status','feasible_seed_trust_region_qp','known_rows_checked_then_removed',removed,'fallbackSafetyCertified',false);return;
    end
end
d=struct('feasible',false,'exitflag',-1,'usedFallback',true,'attempts',{attempts}, ...
    'status','feasible_seed_trust_region_rejected','known_rows_checked_then_removed',removed,'fallbackSafetyCertified',false);
end

function [kernel,fixed,tolerance]=intervalKernel(r)
persistent cache
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);if isKey(cache,key),entry=cache(key);kernel=entry.kernel;fixed=entry.fixed;tolerance=entry.tolerance;return;end
import casadi.*
x=MX.sym('teacher_restart_x',r.plant.n);u=MX.sym('teacher_restart_u',6);o=MX.sym('teacher_restart_o',22,8);
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
kernel=Function('ch5_teacher_feasible_projection',{x,u,o},{g,jacobian(g,u)},ch5ProjectionJitOptions());
known=false(size(r.plant.traceTime));
if r.mode=="nonideal-force",known=r.plant.traceTime<=r.plant.profile.delay+1e-14;end
rows=numel(finiteHi)+numel(lo);
fixed=[reshape(repmat(known,rows,1),[],1);reshape(repmat(known,8,1),[],1)];
tolerance=[repmat([1e-6./scale(finiteHi);1e-6./scale],numel(known),1);ones(8*numel(known),1)*1e-6/.1];
% The kernel retains identical future reserved constraints. At known delay
% samples the present command cannot recover a design reserve retroactively.
reserve=r.physical.actuator.lddotMax-ch5PredictionAccelerationLimit(r);
allowance=zeros(numel(lo),1);allowance(13:18)=reserve;
knownAllowance=[allowance(finiteHi)./scale(finiteHi);allowance./scale];
addition=[reshape(repmat(knownAllowance,1,numel(known)).*repmat(known,rows,1),[],1);zeros(8*numel(known),1)];
tolerance=tolerance+addition;
cache(key)=struct('kernel',kernel,'fixed',fixed,'tolerance',tolerance);
end
