function solution=ch5SolveProjection(optimizer,x0,parameter,lambda)
% Solver coordinates are dimensionless; all public data use SI units.
if nargin<4,lambda=[];end
if optimizer.kind=="sqp_osqp"
    scaled=x0./optimizer.x_scale;
    [g,baseCost,gradientF,jacobianG]=optimizer.linearization(scaled,parameter);
    boundsLower=-inf(size(x0));boundsUpper=inf(size(x0));rows=6*optimizer.H+(1:6*optimizer.H);
    boundsLower(rows)=-optimizer.input_trust_region./optimizer.x_scale(rows);
    boundsUpper(rows)=optimizer.input_trust_region./optimizer.x_scale(rows);
    args={'h',optimizer.quadratic_hessian,'g',gradientF,'a',jacobianG, ...
        'lba',optimizer.lower./optimizer.g_scale-g,'uba',optimizer.upper./optimizer.g_scale-g, ...
        'lbx',boundsLower,'ubx',boundsUpper,'x0',zeros(size(x0))};
    if ~isempty(lambda),args=[args,{'lam_a0',lambda}];end
    raw=optimizer.solver(args{:});step=raw.x;decision=(scaled+full(step)).*optimizer.x_scale;
    % Record the QP's linearized constraint residual explicitly. The exact
    % nonlinear residual is supplied by the subsequent rollout check.
    physicalG=(g+jacobianG*step).*optimizer.g_scale;
    cost=baseCost+dot(gradientF,step)+.5*mtimes(step',mtimes(optimizer.quadratic_hessian,step));
    solution=struct('x',decision,'lam_g',full(raw.lam_a),'f',full(cost), ...
        'g_before_increment_canonicalization',full(physicalG),'stats',optimizer.solver.stats(),'constraint_evaluation','linearized_sqp');
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
raw=optimizer.solver(args{:});solution=struct('x',full(raw.x).*optimizer.x_scale,'lam_g',full(raw.lam_g),'f',full(raw.f), ...
    'g_before_increment_canonicalization',full(raw.g).*optimizer.g_scale,'stats',optimizer.solver.stats(),'constraint_evaluation','nonlinear_ipopt');
end
% Enforce the exact affine input identity in physical coordinates. The
% optimized U is the input used by all shooting dynamics; finite NLP defects
% in its lifted increment variable must not alter the executed sequence.
H=optimizer.H;U=reshape(solution.x(6*H+(1:6*H)),6,H);previous=parameter(optimizer.n+(1:6));
exact=reshape(diff([previous,U],1,2),[],1);
solution.increment_correction_inf=norm(solution.x(1:6*H)-exact,inf);solution.x(1:6*H)=exact;
end
