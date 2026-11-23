function solution=ch5SolveProjection(optimizer,x0,parameter,lambda)
% Solver coordinates are dimensionless; all public data use SI units.
if nargin<4,lambda=[];end
if optimizer.kind=="sqp_osqp"
    scaled=x0./optimizer.x_scale;
    phaseTimer=tic;[g,baseCost,gradientF,jacobianG]=optimizer.linearization(scaled,parameter);linearizationSeconds=toc(phaseTimer);
    boundsLower=-inf(size(x0));boundsUpper=inf(size(x0));rows=6*optimizer.H+(1:6*optimizer.H);
    boundsLower(rows)=-optimizer.input_trust_region./optimizer.x_scale(rows);
    boundsUpper(rows)=optimizer.input_trust_region./optimizer.x_scale(rows);
    % OSQP 0.6.3 uses +/-1e30 as its infinity sentinel. This installed
    % CasADi bridge can leave IEEE infinities in the bound products used to
    % detect infeasibility (0*Inf), causing an infeasible QP to time out.
    % Encode only unbounded entries; every finite physical bound is intact.
    lba=osqpBounds(full(optimizer.lower./optimizer.g_scale-g));
    uba=osqpBounds(full(optimizer.upper./optimizer.g_scale-g));
    args={'h',optimizer.quadratic_hessian,'g',gradientF,'a',jacobianG, ...
        'lba',lba,'uba',uba,'lbx',osqpBounds(boundsLower),'ubx',osqpBounds(boundsUpper),'x0',zeros(size(x0))};
    if ~isempty(lambda),args=[args,{'lam_a0',lambda}];end
    phaseTimer=tic;raw=optimizer.solver(args{:});solverSeconds=toc(phaseTimer);stats=optimizer.solver.stats();step=raw.x;
    % Infeasibility certificates are not primal control updates. A finite
    % iteration-limited iterate can still be audited by the nonlinear gate.
    unusable=~all(isfinite(full(step)))||any(contains(lower(string(stats.return_status)),["infeasible","non convex","unsolved"]));
    if any(unusable),step=casadi.DM.zeros(numel(x0),1);raw.lam_a=casadi.DM.zeros(numel(optimizer.lower),1);end
    decision=(scaled+full(step)).*optimizer.x_scale;
    % Record the QP's linearized constraint residual explicitly. The exact
    % nonlinear residual is supplied by the subsequent rollout check.
    physicalG=(g+jacobianG*step).*optimizer.g_scale;
    cost=baseCost+dot(gradientF,step)+.5*mtimes(step',mtimes(optimizer.quadratic_hessian,step));
    solution=struct('x',decision,'lam_g',full(raw.lam_a),'f',full(cost), ...
        'g_before_increment_canonicalization',full(physicalG),'stats',stats,'constraint_evaluation','linearized_sqp', ...
        'rejected_solver_output',any(unusable),'linearization_seconds',linearizationSeconds,'solver_seconds',solverSeconds);
else
args={'x0',x0./optimizer.x_scale,'p',parameter, ...
    'lbg',optimizer.lower./optimizer.g_scale,'ubg',optimizer.upper./optimizer.g_scale};
if ~isempty(lambda),args=[args,{'lam_g0',lambda}];end
if ~optimizer.polish
    % A bounded local step keeps a one-iteration denoising projection near
    % the dynamically consistent point used to linearize the prediction.
    boundsLower=-inf(size(x0));boundsUpper=inf(size(x0));rows=6*optimizer.H+(1:6*optimizer.H);
    boundsLower(rows)=x0(rows)-optimizer.input_trust_region;boundsUpper(rows)=x0(rows)+optimizer.input_trust_region;
    args=[args,{'lbx',boundsLower./optimizer.x_scale,'ubx',boundsUpper./optimizer.x_scale}];
end
phaseTimer=tic;raw=optimizer.solver(args{:});solverSeconds=toc(phaseTimer);
solution=struct('x',full(raw.x).*optimizer.x_scale,'lam_g',full(raw.lam_g),'f',full(raw.f), ...
    'g_before_increment_canonicalization',full(raw.g).*optimizer.g_scale,'stats',optimizer.solver.stats(), ...
    'constraint_evaluation','nonlinear_ipopt','linearization_seconds',0,'solver_seconds',solverSeconds);
end
% Enforce the exact affine input identity in physical coordinates. The
% optimized U is the input used by all shooting dynamics; finite NLP defects
% in its lifted increment variable must not alter the executed sequence.
H=optimizer.H;U=reshape(solution.x(6*H+(1:6*H)),6,H);previous=parameter(optimizer.n+(1:6));
exact=reshape(diff([previous,U],1,2),[],1);
solution.increment_correction_inf=norm(solution.x(1:6*H)-exact,inf);solution.x(1:6*H)=exact;
end
function bounds=osqpBounds(bounds)
bounds(isinf(bounds)&bounds>0)=1e30;
bounds(isinf(bounds)&bounds<0)=-1e30;
end
