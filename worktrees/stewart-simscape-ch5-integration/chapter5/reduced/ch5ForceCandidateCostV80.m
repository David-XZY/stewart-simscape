function [cost,info]=ch5ForceCandidateCostV80(x,previous,U,goal,r,k)
% Exact objective and a necessary terminal-region test, without physical admission.
% All executed candidates still require the full horizon/transient/terminal audit.
cost=inf;info=struct('finite',false,'terminal_region_possible',false,'terminal_region_ratio',inf, ...
    'scope','objective_and_necessary_terminal_condition_only');
if ~all(isfinite(U),'all'),return;end
nodes=full(k.nodes(x,U));if ~all(isfinite(nodes),'all'),return;end
qs=nodes(1:6,end);term=ch5ForceTerminalV62(qs,r,k);e=([nodes(:,end);U(:,end)]-term.xi)./term.scale;
V=e'*term.P*e;D=diff([previous,U],1,2);ps=[.1*ones(3,1);deg2rad(10)*ones(3,1)];vs=[.1*ones(3,1);.2*ones(3,1)];
cost=r.c.Ts*(sum(((nodes(1:6,2:end)-qs)./ps).^2,'all')+sum((nodes(7:12,2:end)./vs).^2,'all')+ ...
    .05*sum(((nodes(1:6,2:end)-goal)./ps).^2,'all')+.01*sum((D/160).^2,'all')+ ...
    .001*sum(((U-term.u)/2000).^2,'all'))+100*sum(((qs-goal)./ps).^2)+V;
info.finite=isfinite(cost);info.terminal_region_ratio=V/max(term.rho,realmin);
info.terminal_region_possible=term.numerically_valid&&info.terminal_region_ratio<=1+1e-6;
end
