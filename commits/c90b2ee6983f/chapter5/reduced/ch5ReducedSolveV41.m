function [delta,info]=ch5ReducedSolveV41(linear,r)
% Bound the original fast solve; retry the SAME problem with row scaling.
timer=tic;Q=(linear.Q+linear.Q')/2;f=linear.f;A=linear.A;b=linear.b;E=linear.E;e=linear.e;n=numel(f);
radius=1;assert(~isfield(r.reduced,'trust_radius')||r.reduced.trust_radius==1,'ch5:HybridDomain','Frozen probe has unit trust radius.');
keep=b<sum(abs(A),2)+1e-10;A=A(keep,:);b=b(keep);phase=tic;
L=chol(Q,'lower');T=L'\eye(n);W=T'*Q*T;error=norm(W-eye(n),inf);
assert(error<1e-6,'ch5:Whitening','Loss of coordinate precision.');
qpQ=(W+W')/2;qpF=T'*f;qpA=[A*T;T;-T];qpB=[b;ones(2*n,1)];qpE=E*T;
transformSeconds=toc(phase);persistent options
if isempty(options),options=optimoptions('quadprog','Display','off','Algorithm','active-set', ...
        'MaxIterations',40,'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-8);end
phase=tic;[z,~,flag,output]=quadprog(qpQ,qpF,qpA,qpB,qpE,-e,[],[],zeros(n,1),options);firstSeconds=toc(phase);
delta=[];finite=~isempty(z)&&all(isfinite(z));violation=inf;eqError=inf;objective=inf;
if finite
    delta=T*z;violation=max([A*delta-b;abs(delta)-radius;0]);eqError=norm(E*delta+e,inf);objective=.5*delta'*Q*delta+f'*delta;
end
first=struct('exitflag',flag,'iterations',output.iterations,'solver_seconds',firstSeconds,'finite',finite, ...
    'original_inequality_violation',violation,'original_equality_error',eqError,'objective',objective);
retry=flag<=0||~finite||violation>1e-6||eqError>1e-6;
if retry
    [delta,info]=ch5ReducedSolveV36(linear,r);info.row_scaled_retry=info;info.solver_seconds=info.solver_seconds+firstSeconds;
else
    info=struct('exitflag',flag,'solver_success',true,'finite',finite,'iterations',output.iterations, ...
        'variables',n,'inequalities',numel(qpB),'solver_seconds',firstSeconds,'transform_seconds',transformSeconds, ...
        'whitening_error',error,'original_inequality_violation',violation,'original_equality_error',eqError, ...
        'objective',objective,'original_linear_feasible',true,'dual_diagnostics','native convergence flag; independently checked against full KKT reference in frozen-case audit');
end
info.backend='bounded-unscaled-then-scaled';info.first_attempt=first;info.used_row_scaled_retry=retry;
info.first_transform_seconds=transformSeconds;info.total_seconds=toc(timer);info.status=sprintf('hybrid_final_flag_%d',info.exitflag);
end
