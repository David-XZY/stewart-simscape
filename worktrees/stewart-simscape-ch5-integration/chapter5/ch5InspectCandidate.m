function [decision,check,trajectory,cost]=ch5InspectCandidate(decision,parameter,x,previous,o,r,optimizer,preview)
% Eliminate approximate shooting variables and audit the actually executable
% control sequence. Finite optimization cannot bypass this acceptance gate.
H=r.c.H;U=reshape(decision(6*H+(1:6*H)),6,H);D=diff([previous,U],1,2);
nodesPreview=ch5NativeNodes(r);nodes=full(nodesPreview(x,U));values=[];trace=[];
if ~all(isfinite(nodes),'all')||~all(isfinite(U),'all')
    [check,trajectory,cost]=rejected(nodes,U,D,'nonfinite_rollout');return;
end
qa=nodes(1:6,end);[d,G]=r.ad.dynamics(qa,zeros(6,1));G=full(G);d=full(d);
if ~all(isfinite(G),'all')||~all(isfinite(d))||rcond(G)<1e-12
    [check,trajectory,cost]=rejected(nodes,U,D,'singular_terminal_map');return;
end
ua=G\(-d);
ns=optimizer.shooting_n;
mismatch=norm(reshape(decision(12*H+(1:ns*H)),ns,H)-nodes(optimizer.shooting_rows,2:end),'inf');
decision=[D(:);U(:);reshape(nodes(optimizer.shooting_rows,2:end),[],1);qa;ua];
[g,cost]=optimizer.checkExecutable(decision,parameter);g=full(g);cost=full(cost);
eq=optimizer.lower==optimizer.upper;ineq=~eq;
eqRatio=max([abs(g(eq)-optimizer.lower(eq))./optimizer.equality_tolerance(eq);0]);
ineqViolation=max([optimizer.lower(ineq)-g(ineq);g(ineq)-optimizer.upper(ineq);0]);
ineqScore=max([(optimizer.lower(ineq)-g(ineq))./optimizer.g_scale(ineq); ...
    (g(ineq)-optimizer.upper(ineq))./optimizer.g_scale(ineq);0])/1e-6;
finite=all(isfinite(nodes),'all')&&all(isfinite(g))&&isfinite(cost);
meritViolation=sum(abs(g(eq)-optimizer.lower(eq))./optimizer.g_scale(eq))+ ...
    sum(max(0,optimizer.lower(ineq)-g(ineq))./optimizer.g_scale(ineq))+ ...
    sum(max(0,g(ineq)-optimizer.upper(ineq))./optimizer.g_scale(ineq));
internal=struct('accepted',false,'evaluated',false,'maximum_violation',inf);
if finite&&eqRatio<=1&&ineqViolation<=1e-6
    [auditedNodes,values,trace]=preview(x,U);auditedNodes=full(auditedNodes);values=full(values);trace=full(trace);
    assert(norm(nodes-auditedNodes,'inf')<=1e-9,'ch5:PreviewMismatch','State-only and full-mesh rollout disagree.');
    internal=ch5CheckRollout(nodes,values,trace,U,o,r);internal.evaluated=true;
end
check=struct('accepted',finite&&eqRatio<=1&&ineqViolation<=1e-6&&internal.accepted, ...
    'violation',max([abs(g(eq)-optimizer.lower(eq));ineqViolation;0]), ...
    'equality_tolerance_ratio',eqRatio,'inequality_violation',ineqViolation, ...
    'constraint_score',max(eqRatio,ineqScore),'merit_violation',meritViolation,'internal_check',internal,'finite',finite, ...
    'rollout_mismatch',mismatch,'terminal_velocity',nodes(7:12,end));
trajectory=struct('X',nodes,'U',U,'delta',D,'artificial_pose',qa,'artificial_force',ua, ...
    'interval_value_minimum',min(values,[],2),'interval_value_maximum',max(values,[],2), ...
    'interval_value_names',[compose("length_%d",(1:6)');compose("speed_%d",(1:6)'); ...
        compose("acceleration_%d",(1:6)');"sigma_lower_squared";compose("actual_force_%d",(1:6)')]);
end
function [check,trajectory,cost]=rejected(nodes,U,D,reason)
cost=inf;check=struct('accepted',false,'finite',false,'rejection_reason',reason, ...
    'violation',inf,'merit_violation',inf,'equality_tolerance_ratio',inf, ...
    'inequality_violation',inf,'constraint_score',inf,'rollout_mismatch',inf, ...
    'internal_check',struct('accepted',false,'evaluated',false,'maximum_violation',inf));
trajectory=struct('X',nodes,'U',U,'delta',D,'artificial_pose',[],'artificial_force',[]);
end
