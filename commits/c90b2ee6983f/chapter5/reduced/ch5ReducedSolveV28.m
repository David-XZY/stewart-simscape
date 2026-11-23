function [delta,info]=ch5ReducedSolveV28(linear,r)
% Hessian whitening plus equivalent positive row scaling; check original units.
timer=tic;Q=(linear.Q+linear.Q')/2;f=linear.f;A=linear.A;b=linear.b;E=linear.E;e=linear.e;n=numel(f);
radius=option(r,'trust_radius',1);nc=option(r,'control_variables',n);
keep=b<radius*sum(abs(A(:,1:nc)),2)+1e-10|any(abs(A(:,nc+1:end))>0,2);A=A(keep,:);b=b(keep);phase=tic;
L=chol(Q,'lower');T=L'\eye(n);W=T'*Q*T;whiteningError=norm(W-eye(n),inf);
assert(whiteningError<1e-6,'ch5:Whitening','Whitening lost precision.');qpQ=(W+W')/2;qpF=T'*f;
bounds=[eye(nc),zeros(nc,n-nc)];originalA=[A;bounds;-bounds];originalB=[b;radius*ones(2*nc,1)];qpA=originalA*T;qpB=originalB;qpE=E*T;qpBE=-e;
rowScale=ones(size(qpB));equalityScale=ones(size(e));
if option(r,'qp_row_scaling',true)
    rowScale=max(vecnorm(qpA,2,2),1e-12);equalityScale=max(vecnorm(qpE,2,2),1e-12);
    qpA=qpA./rowScale;qpB=qpB./rowScale;qpE=qpE./equalityScale;qpBE=qpBE./equalityScale;
end
transformSeconds=toc(phase);
options=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',option(r,'qp_max_iterations',400), ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-8);
phase=tic;[z,~,flag,output,lambda]=quadprog(qpQ,qpF,qpA,qpB,qpE,qpBE,[],[],zeros(n,1),options);solveSeconds=toc(phase);
delta=[];finite=~isempty(z)&&all(isfinite(z));violation=inf;eqError=inf;objective=inf;kkt=inf;dualViolation=inf;complementarity=inf;
if finite
    delta=T*z;violation=max([originalA*delta-originalB;0]);eqError=norm(E*delta+e,inf);objective=.5*delta'*Q*delta+f'*delta;
    dual=lambda.ineqlin./rowScale;eqDual=lambda.eqlin./equalityScale;
    gradient=Q*delta+f;kkt=norm(gradient+originalA'*dual+E'*eqDual,inf)/max(1,norm(gradient,inf));
    dualViolation=max([-dual;0]);complementarity=norm(dual.*(originalA*delta-originalB),inf);
end
info=struct('backend','whiten-row-scaled-active','status',sprintf('whiten_row_scaled_flag_%d',flag),'exitflag',flag,'solver_success',flag>0, ...
    'finite',finite,'iterations',output.iterations,'variables',n,'inequalities',numel(qpB),'solver_seconds',solveSeconds, ...
    'transform_seconds',transformSeconds,'total_seconds',toc(timer),'whitening_error',whiteningError,'original_inequality_violation',violation, ...
    'original_equality_error',eqError,'objective',objective,'original_linear_feasible',finite&&violation<=1e-6&&eqError<=1e-6, ...
    'relative_stationarity',kkt,'dual_violation',dualViolation,'complementarity',complementarity,'trust_radius',radius,'row_scale_range',[min(rowScale),max(rowScale)]);
end
function value=option(r,name,default)
value=default;if isfield(r.reduced,name),value=r.reduced.(name);end
end
