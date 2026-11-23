function [proposed,info]=ch5ReducedRestoreV12(x,previous,U,goal,o,prior,r,k)
% One elastic feasibility QP. Its slack never relaxes the execution gate.
timer=tic;[~,linear]=ch5ReducedStepV9(x,previous,U,goal,o,prior,r,k,true);linearSeconds=toc(timer);
nv=size(linear.Q,1);stageRows=size(linear.A,1)/r.c.H;
assert(stageRows==fix(stageRows),'ch5:RestorationRows','Invalid stage row mapping.');
hard=mod((0:size(linear.A,1)-1)',stageRows)<24;
% The first 24 rows in each stage are force and force-rate constraints.
A=[linear.A,-double(~hard)];b=linear.b;E=[linear.E,zeros(size(linear.E,1),1)];
Q=blkdiag(2e-5*eye(nv),0);f=[zeros(nv,1);1];
rho=max([0;-b(~hard)]);initial=[zeros(nv,1);rho];
persistent options
if isempty(options),options=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',400,'ConstraintTolerance',1e-7,'OptimalityTolerance',1e-6);end
timer=tic;[solution,~,flag,output]=quadprog(Q,f,A,b,E,-linear.e,[-ones(nv,1);0],[ones(nv,1);inf],initial,options);
info=struct('purpose','feasibility_restoration','status',sprintf('restoration_quadprog_flag_%d',flag), ...
    'exitflag',flag,'solver_success',flag>0,'finite',~isempty(solution)&&all(isfinite(solution)), ...
    'iterations',output.iterations,'linearization_seconds',linearSeconds,'solver_seconds',toc(timer), ...
    'variables',nv+1,'inequalities',numel(b),'elastic_rows',nnz(~hard),'hard_input_rows',nnz(hard), ...
    'elastic_slack',nan,'linear_residual',inf,'equality_residual',inf,'execution_gate_relaxed',false);
proposed=U;
if info.finite
    info.elastic_slack=solution(end);info.linear_residual=max([A*solution-b;0]);
    info.equality_residual=norm(E*solution+linear.e,inf);
    gain=ch5ReducedFeedbackGain(x,r,k);rollout=ch5ReducedFeedbackRollout(r);
    proposed=full(rollout(x,linear.nodes,U,gain,reshape(solution(1:nv),6,r.c.H/5)));
end
end
