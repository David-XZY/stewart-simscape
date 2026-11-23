function [U,nodes,information]=ch5ControlInitialGuess(x,previous,warm,r,previewNodes)
% A shifted former plan remains an optimizer initial guess after a failed
% sample. It is never executed without a new full physical acceptance check.
H=r.c.H;information=struct('source','damped_dynamics','shift_steps',0);
if ~isempty(warm)
    shift=warm.shift_steps;
    U=[warm.U(:,shift+1:end),repmat(warm.artificial_force,1,shift)];
    U=boundCommands(U,previous,r);nodes=full(previewNodes(x,U));
    if all(isfinite(nodes),'all')
        information.source='shifted_plan';information.shift_steps=shift;information.finite=true;return;
    end
end
% Repeating a previously accelerating force for 1.2 s can leave the model's
% domain. Build a braking guess from current mechanics instead. This is a
% numerical initial guess, not a separate reference tracker or execution law.
U=zeros(6,H);state=x;last=previous;
for j=1:H
    [drift,G]=r.ad.dynamics(state(1:6),state(7:12));G=full(G);drift=full(drift);
    if ~all(isfinite(G),'all')||rcond(G)<1e-12||~all(isfinite(drift))
        U(:,j:end)=repmat(last,1,H-j+1);break;
    end
    desired=-10*state(7:12);target=G\(desired-drift);
    command=min(max(target,max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)), ...
        min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));
    U(:,j)=command;state=full(r.plant.fd(state,command,zeros(6,1)));last=command;
    if ~all(isfinite(state))
        U(:,j+1:end)=repmat(last,1,H-j);break;
    end
end
nodes=full(previewNodes(x,U));
information.finite=all(isfinite(nodes),'all');
end
function U=boundCommands(U,previous,r)
last=previous;
for j=1:size(U,2)
    U(:,j)=min(max(U(:,j),max(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)), ...
        min(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));last=U(:,j);
end
end
