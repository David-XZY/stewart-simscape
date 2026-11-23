function backup=ch5SpeedTailBackup(r)
% Retain the next planned force, then rebuild a braking continuation. This
% constructs a candidate only; the unchanged complete gate decides execution.
persistent cache cacheKey
key=char(r.mode+"|"+r.c.version);
if ~isempty(cacheKey)&&strcmp(cacheKey,key),backup=cache;return;end
import casadi.*
x=MX.sym('backup_initial',r.plant.n);first=MX.sym('backup_first',6);
state=r.plant.fd(x,first,zeros(6,1));last=first;commands=first;
for j=2:r.c.H
    [drift,G]=r.ad.dynamics(state(1:6),state(7:12));
    target=solve(G,-10*state(7:12)-drift);
    command=fmin(fmax(target,fmax(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)), ...
        fmin(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));
    commands=[commands,command];state=r.plant.fd(state,command,zeros(6,1));last=command; %#ok<AGROW>
end
backup=Function('ch5_speed_braking_continuation',{x,first},{commands},ch5ProjectionJitOptions());cache=backup;cacheKey=key;
end
