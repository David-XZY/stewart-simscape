function [U,check,trajectory,cost,info]=ch5ReducedAuditV22(x,previous,U,goal,o,r,k,prior,incumbent,repairTerminal)
% Same preview, arithmetic, limits and physical gate; screen objective first.
if nargin<9,incumbent=[];end
if nargin<10,repairTerminal=true;end
info=struct('screened_for_cost',false,'full_audit_performed',false,'terminal_repair',[], ...
    'preview_count',0,'phases',struct('preview',0,'terminal',0,'objective',0,'physical',0,'transient',0));
[invalid,rate]=inputInvalid(previous,U,r);
if invalid
    [check,trajectory,cost]=ch5ReducedAuditGuardedV10(x,previous,U,goal,o,r,k,prior);return;
end
timer=tic;[nodes,values,trace]=preview(x,U,k);info.preview_count=1;info.phases.preview=toc(timer);
timer=tic;[finite,ua,steady,terminalRatio]=terminal(nodes,values,trace,U,r,k);
if repairTerminal&&terminalRatio>1
    [U,info.terminal_repair]=ch5ReducedTerminal(x,U,r,k);
    info.phases.terminal=toc(timer);[invalid,rate]=inputInvalid(previous,U,r);
    if invalid
        [check,trajectory,cost]=ch5ReducedAuditGuardedV10(x,previous,U,goal,o,r,k,prior);return;
    end
    timer=tic;[nodes,values,trace]=preview(x,U,k);info.preview_count=2;info.phases.preview=info.phases.preview+toc(timer);
    timer=tic;[finite,ua,steady,terminalRatio]=terminal(nodes,values,trace,U,r,k);
end
info.phases.terminal=info.phases.terminal+toc(timer);timer=tic;
D=diff([previous,U],1,2);assert(isequal(size(prior),size(U)),'ch5:PriorShape','Physical prior shape changed.');
ps=[.1;.1;.1;deg2rad(10)*ones(3,1)];vs=[.1;.1;.1;.2;.2;.2];qa=nodes(1:6,end);
cost=r.c.Ts*(sum(((nodes(1:6,2:end)-qa)./ps).^2,'all')+r.reduced.velocity_weight*sum((nodes(7:12,2:end)./vs).^2,'all')+ ...
    .05*sum(((nodes(1:6,2:end)-goal)./ps).^2,'all')+.01*sum((D/160).^2,'all')+.001*sum((U/2000).^2,'all'))+ ...
    .05*r.c.Ts*sum(((D-prior)/160).^2,'all')+100*sum(((qa-goal)./ps).^2);
trajectory=struct('X',nodes,'U',U,'delta',D,'artificial_pose',qa,'artificial_force',ua);
info.phases.objective=toc(timer);
if ~isempty(incumbent)&&incumbent.accepted&&finite&&isfinite(cost)&&cost>=incumbent.cost
    info.screened_for_cost=true;
    check=struct('accepted',false,'finite',finite,'equality_tolerance_ratio',terminalRatio, ...
        'inequality_violation',nan,'command_rate_violation',rate,'terminal_checked',true, ...
        'admissibility_checked',false,'scope','objective_screen','skipped_for_cost',true, ...
        'rejection_reason','nonimproving_objective');return;
end
timer=tic;info.full_audit_performed=true;
if finite
    internal=ch5ReducedCheckRolloutV22(nodes,values,trace,U,o,r);sv=full(r.plant.mechanicalValues(steady,ua));
    lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
    hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
    inputErrors=[reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1); ...
        reshape(abs(D)-r.strict.forceRateLimit*r.c.Ts,[],1);abs(ua-U(:,end))-r.strict.forceRateLimit*r.c.Ts;lo-sv;sv-hi;0];
    violation=max(max(inputErrors),internal.maximum_violation);
else,violation=inf;internal=struct('accepted',false);end
accepted=finite&&terminalRatio<=1&&violation<=1e-6;
check=struct('accepted',accepted,'finite',finite,'equality_tolerance_ratio',terminalRatio, ...
    'inequality_violation',violation,'internal_check',internal,'terminal_velocity',nodes(7:12,end), ...
    'scope','full_horizon','terminal_checked',true,'admissibility_checked',true,'cheap_reject',false,'skipped_for_cost',false);
info.phases.physical=toc(timer);timer=tic;
if check.accepted
    extra=ch5ReducedTransient(nodes,U,o,r);check.transient=extra;
    check.accepted=extra.accepted;check.inequality_violation=max(check.inequality_violation,extra.maximum_violation);
end
check.command_rate_violation=rate;check.accepted=check.accepted&&rate<=1e-6;
info.phases.transient=toc(timer);
end

function [invalid,rate]=inputInvalid(previous,U,r)
D=diff([previous,U],1,2);force=max([reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1);0]);
rate=max([reshape(abs(D)/r.c.Ts-r.strict.forceRateLimit,[],1);0]);
invalid=~all(isfinite([U(:);previous]))||force>1e-6||rate>1e-6;
end
function [nodes,values,trace]=preview(x,U,k)
[nodes,values,trace]=k.preview(x,U);nodes=full(nodes);values=full(values);trace=full(trace);
end
function [finite,ua,steady,ratio]=terminal(nodes,values,trace,U,r,k)
finite=all(isfinite([nodes(:);values(:);trace(:);U(:)]));
if ~finite,ua=nan(6,1);steady=nan(r.plant.n,1);ratio=inf;return;end
ua=full(k.equilibrium(nodes(1:6,end)));steady=[nodes(1:6,end);zeros(6,1)];
if r.plant.n>12,steady=[steady;repmat(ua,(r.plant.n-12)/6,1)];end
tol=[repmat(r.c.terminal_position_numerical_tolerance,3,1); ...
    repmat(deg2rad(r.c.terminal_rotation_numerical_tolerance_degrees),3,1); ...
    repmat(r.c.terminal_linear_speed_numerical_tolerance,3,1); ...
    repmat(deg2rad(r.c.terminal_angular_speed_numerical_tolerance_degrees),3,1); ...
    repmat(r.c.terminal_actuator_numerical_tolerance,r.plant.n-12,1)];
ratio=max(abs(nodes(:,end)-steady)./tol);
end
