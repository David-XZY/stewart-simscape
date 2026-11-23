function [used,u,d,prediction,saved]=ch5ForceTerminalExecuteV77(x,previous,goal,o,r,saved,time)
% Execute the existing optimized local terminal law only after tight arrival.
% No moving obstacle is admitted to this mode. Every executed interval is
% checked again; numerical Lyapunov/region checks are not a stability proof.
used=false;u=[];d=[];prediction=[];
if isempty(saved)||isempty(saved.trajectory)||~isfield(saved.trajectory,'terminal')||isempty(saved.trajectory.terminal),return;end
if any(o(22,:)>.5)||norm(goal-saved.goal)>1e-10,return;end
term=saved.trajectory.terminal;if ~term.numerically_valid,return;end
rotation=acos(max(-1,min(1,(trace(ch5Rotation(goal(4:6))'*ch5Rotation(x(4:6)))-1)/2)));
omega=rpyRateMapZYX(x(4:6))*x(10:12);
if norm(x(1:3)-goal(1:3))>.001||rotation>deg2rad(.1)||norm(x(7:9))>.001||norm(omega)>deg2rad(.1),return;end
if norm(saved.trajectory.artificial_pose(1:3)-goal(1:3))>.001,return;end
steadyRotation=acos(max(-1,min(1,(trace(ch5Rotation(goal(4:6))'*ch5Rotation(saved.trajectory.artificial_pose(4:6)))-1)/2)));
if steadyRotation>deg2rad(.1),return;end
e=([x;previous]-term.xi)./term.scale;V=e'*term.P*e;
if V>term.rho*(1+1e-6),return;end
% The Riccati model augments the state with the previous command and its
% decision is a command increment: u(k)=u(k-1)+Delta u(k).
timer=tic;command=previous-term.Du*term.K*e;
[check,trajectory]=ch5ReducedHoldAuditV77(x,previous,command,o,r);
en=([trajectory.X(:,end);command]-term.xi)./term.scale;descent=en'*term.P*en-V+e'*e;
if ~check.accepted||descent>1e-7,return;end
used=true;u=command;d=struct('feasible',true,'usedFallback',false,'exitflag',1,'solver_failure',false);
saved.terminal_active=true;saved.observed=o;saved.observation_time=time;saved.goal=goal;
prediction=struct('refresh',false,'chosen',1,'candidates',{{trajectory}},'checks',{{check}},'raw_checks',{{}}, ...
    'raw_audited',0,'raw_accepted',0,'projected_accepted',0,'denoise_attempts',{{}},'final_attempts',{{}},'restoration_attempts',{{}}, ...
    'goal',goal,'observed',o,'cost',V,'phases',struct('warm_start',0,'prior',0,'incumbent_check',toc(timer),'prediction_step',0,'trial_checks',0,'recovery',0), ...
    'full_horizon_checked',false,'validation_scope','local_terminal_feedback_executed_interval','full_horizon_failure_active',false,'recovery_active',false, ...
    'terminal_active',true,'terminal_region_ratio',V/term.rho,'terminal_descent_residual',descent,'terminal_certified',false,'algorithm_revision',r.algorithm_revision);
end
