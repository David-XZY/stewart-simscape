function [warm,repair]=ch5ReducedWarmV23(x,previous,saved,r,k,time)
% Propagate the numerical iterate, as v9 did, without claiming its admission.
old=saved.trajectory;shift=round((time-saved.trajectory_time)/r.c.Ts);
assert(shift>=1&&shift<=r.c.H,'ch5:WarmAge','Warm iterate must cover the current time.');
base=[old.U(:,shift+1:end),repmat(old.artificial_force,1,shift)];
steady=[old.artificial_pose;zeros(6,1)];
if r.plant.n>12,steady=[steady;repmat(old.artificial_force,(r.plant.n-12)/6,1)];end
guess=[old.X(:,shift+1:end),repmat(steady,1,shift)];
feedback=ch5ReducedFeedbackRollout(r);gain=ch5ReducedFeedbackGain(x,r,k);
[U,nodes]=feedback(x,guess,base,gain,zeros(6,r.c.H/5));U=full(U);nodes=full(nodes);
ua=full(k.equilibrium(nodes(1:6,end)));repair=[];
tolerance=[repmat(r.c.terminal_position_numerical_tolerance,3,1); ...
    repmat(deg2rad(r.c.terminal_rotation_numerical_tolerance_degrees),3,1); ...
    repmat(r.c.terminal_linear_speed_numerical_tolerance,3,1); ...
    repmat(deg2rad(r.c.terminal_angular_speed_numerical_tolerance_degrees),3,1); ...
    repmat(r.c.terminal_actuator_numerical_tolerance,r.plant.n-12,1)];
target=[nodes(1:6,end);zeros(6,1)];if r.plant.n>12,target=[target;repmat(ua,(r.plant.n-12)/6,1)];end
if all(isfinite(nodes),'all')&&max(abs(nodes(:,end)-target)./tolerance)>1
    [U,repair,nodes]=ch5ReducedTerminalV23(x,U,r,k,nodes);ua=full(k.equilibrium(nodes(1:6,end)));
end
warm=struct('X',nodes,'U',U,'delta',diff([previous,U],1,2),'artificial_pose',nodes(1:6,end), ...
    'artificial_force',ua,'scope','numerical_warm_iterate');
end
