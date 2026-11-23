function [check,trajectory,cost]=ch5ReducedAuditV4(x,previous,U,goal,o,r,k,prior)
% Same physical mesh and terminal bands as the production executable gate.
if nargin<8,prior=zeros(size(U));end
[nodes,values,trace]=k.preview(x,U);nodes=full(nodes);values=full(values);trace=full(trace);
D=diff([previous,U],1,2);finite=all(isfinite([nodes(:);values(:);trace(:);U(:)]));
if finite
    ua=full(k.equilibrium(nodes(1:6,end)));steady=[nodes(1:6,end);zeros(6,1)];
    if r.plant.n>12,steady=[steady;repmat(ua,(r.plant.n-12)/6,1)];end
    tol=[repmat(r.c.terminal_position_numerical_tolerance,3,1); ...
        repmat(deg2rad(r.c.terminal_rotation_numerical_tolerance_degrees),3,1); ...
        repmat(r.c.terminal_linear_speed_numerical_tolerance,3,1); ...
        repmat(deg2rad(r.c.terminal_angular_speed_numerical_tolerance_degrees),3,1); ...
        repmat(r.c.terminal_actuator_numerical_tolerance,r.plant.n-12,1)];
    terminalRatio=max(abs(nodes(:,end)-steady)./tol);
    internal=ch5CheckRollout(nodes,values,trace,U,o,r);
    sv=full(r.plant.mechanicalValues(steady,ua));
    lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2;r.strict.forceMin];
    hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf;r.strict.forceMax];
    inputErrors=[reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1); ...
        reshape(abs(D)-r.strict.forceRateLimit*r.c.Ts,[],1);abs(ua-U(:,end))-r.strict.forceRateLimit*r.c.Ts;lo-sv;sv-hi;0];
    inputViolation=max(inputErrors);
    physicalViolation=max(inputViolation,internal.maximum_violation);
else
    ua=nan(6,1);terminalRatio=inf;physicalViolation=inf;internal=struct('accepted',false);
end
accepted=finite&&terminalRatio<=1&&physicalViolation<=1e-6;
check=struct('accepted',accepted,'finite',finite,'equality_tolerance_ratio',terminalRatio, ...
    'inequality_violation',physicalViolation,'internal_check',internal,'terminal_velocity',nodes(7:12,end));
trajectory=struct('X',nodes,'U',U,'delta',D,'artificial_pose',nodes(1:6,end),'artificial_force',ua);
ps=[.1;.1;.1;deg2rad(10)*ones(3,1)];vs=[.1;.1;.1;.2;.2;.2];qa=nodes(1:6,end);
cost=r.c.Ts*(sum(((nodes(1:6,2:end)-qa)./ps).^2,'all')+.1*sum((nodes(7:12,2:end)./vs).^2,'all')+ ...
    .05*sum(((nodes(1:6,2:end)-goal)./ps).^2,'all')+.01*sum((D/160).^2,'all')+.001*sum((U/2000).^2,'all'))+ ...
    .05*sum(((D-prior)/160).^2,'all')+100*sum(((qa-goal)./ps).^2);
end
