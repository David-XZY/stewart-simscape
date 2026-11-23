function [check,traj,cost]=ch5ForceAuditV62(x,previous,U,goal,o,r,k,qs,prior)
% The shooting mesh, internal mesh, actuator transients and terminal step share one gate.
if nargin<8||isempty(qs),qs=[];end
if nargin<9||isempty(prior),prior=U;end
D=diff([previous,U],1,2);[nodes,values,trace]=k.preview(x,U);nodes=full(nodes);values=full(values);trace=full(trace);
finite=all(isfinite([nodes(:);values(:);trace(:);U(:)]));
violation=inf;ratio=inf;cost=inf;term=[];tail=[];internal=[];extra=[];
if finite
    internal=ch5ReducedCheckRolloutV22(nodes,values,trace,U,o,r);
    violation=max([internal.maximum_violation;reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1); ...
        reshape(abs(D)-r.strict.forceRateLimit*r.c.Ts,[],1);0]);
end
if ~finite||violation>1e-6
    if isempty(qs),qs=nodes(1:6,end);end
    check=struct('accepted',false,'finite',finite,'inequality_violation',violation,'equality_tolerance_ratio',inf, ...
        'terminal_region_ratio',inf,'terminal_descent_residual',inf,'terminal_checked',false,'terminal_certified',false, ...
        'scope','full_horizon_physical_rejection','internal_check',internal,'terminal_tail',[],'transient',[], ...
        'command_rate_violation',max([reshape(abs(D)/r.c.Ts-r.strict.forceRateLimit,[],1);0]), ...
        'rejection_reason','physical_gate_failed_before_terminal_design');
    traj=struct('X',nodes,'U',U,'delta',D,'artificial_pose',qs,'artificial_force',[],'terminal',[]);return;
end
if finite
    if isempty(qs),qs=nodes(1:6,end);end
    term=ch5ForceTerminalV62(qs,r,k);xi=[nodes(:,end);U(:,end)];e=(xi-term.xi)./term.scale;
    V=e'*term.P*e;ratio=V/max(term.rho,realmin);
    % Reuse the identical physical result already computed above.
    % Receding-horizon terminal feedback must pass the same physical interval audit.
    ot=o;ot(1:3,:)=ot(1:3,:)+o(16:18,:)*(r.c.H*r.c.Ts);
    ut=U(:,end)-term.Du*term.K*e;
    [tail,tailTrajectory]=ch5ReducedHoldAuditV22(nodes(:,end),U(:,end),ut,ot,r);
    en=([tailTrajectory.X(:,end);ut]-term.xi)./term.scale;
    descent=en'*term.P*en-V+e'*e;
    terminalAccepted=term.numerically_valid&&ratio<=1+1e-6&&tail.accepted&&descent<=1e-7;
    ps=[.1*ones(3,1);deg2rad(10)*ones(3,1)];vs=[.1*ones(3,1);.2*ones(3,1)];
    cost=r.c.Ts*(sum(((nodes(1:6,2:end)-qs)./ps).^2,'all')+sum((nodes(7:12,2:end)./vs).^2,'all')+ ...
        .05*sum(((nodes(1:6,2:end)-goal)./ps).^2,'all')+.01*sum((D/160).^2,'all')+ ...
        .001*sum((U/2000).^2,'all'))+100*sum(((qs-goal)./ps).^2)+V+ ...
        .05*r.c.Ts*sum(((U-prior)/20).^2,'all');
else,terminalAccepted=false;descent=inf;end
accepted=finite&&violation<=1e-6&&terminalAccepted;
if accepted
    extra=ch5ReducedTransient(nodes,U,o,r);accepted=extra.accepted;violation=max(violation,extra.maximum_violation);
end
check=struct('accepted',accepted,'finite',finite,'inequality_violation',violation,'equality_tolerance_ratio',ratio, ...
    'terminal_region_ratio',ratio,'terminal_descent_residual',descent,'terminal_checked',true,'terminal_certified',false, ...
    'scope','full_horizon_and_local_terminal_step','internal_check',internal,'terminal_tail',tail,'transient',extra, ...
    'command_rate_violation',max([reshape(abs(D)/r.c.Ts-r.strict.forceRateLimit,[],1);0]));
traj=struct('X',nodes,'U',U,'delta',D,'artificial_pose',qs,'artificial_force',[], 'terminal',term);
if ~isempty(term),traj.artificial_force=term.u;end
end
