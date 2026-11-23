function [delta,info,warm]=ch5ForceQpSolveV62(p,f,warm)
% Reuse preparation and a feasible primal start; no infeasible point is executed.
if nargin<3||isempty(warm)||numel(warm)~=p.n||any(~isfinite(warm)),warm=zeros(p.n,1);end
timer=tic;[z,~,flag,output,lambda]=quadprog(p.qpQ,p.T'*f,p.qpA,p.qpB,p.qpE,p.qpBE,[],[],warm,p.options);seconds=toc(timer);
delta=[];finite=~isempty(z)&&all(isfinite(z));violation=inf;eqError=inf;objective=inf;kkt=inf;dualViolation=inf;complementarity=inf;
if finite
    delta=p.T*z;violation=max([p.A*delta-p.b;0]);eqError=norm(p.E*delta+p.e,inf);
    objective=.5*delta'*p.Q*delta+f'*delta;dual=lambda.ineqlin./p.rowScale;eqDual=lambda.eqlin./p.equalityScale;
    gradient=p.Q*delta+f;kkt=norm(gradient+p.A'*dual+p.E'*eqDual,inf)/max(1,norm(gradient,inf));
    dualViolation=max([-dual;0]);complementarity=norm(dual.*(p.A*delta-p.b),inf);
    if violation<=1e-6&&eqError<=1e-6,warm=z;else,warm=zeros(p.n,1);end
end
info=struct('backend','shared-whiten-row-scaled-active','status',sprintf('shared_whiten_flag_%d',flag),'exitflag',flag,'solver_success',flag>0, ...
    'finite',finite,'iterations',output.iterations,'variables',p.n,'inequalities',numel(p.qpB),'solver_seconds',seconds, ...
    'transform_seconds',0,'shared_transform_seconds',p.transformSeconds,'total_seconds',toc(timer),'whitening_error',p.whiteningError, ...
    'original_inequality_violation',violation,'original_equality_error',eqError,'objective',objective, ...
    'original_linear_feasible',finite&&violation<=1e-6&&eqError<=1e-6,'relative_stationarity',kkt, ...
    'dual_violation',dualViolation,'complementarity',complementarity,'trust_radius',p.radius, ...
    'row_scale_range',[min(p.rowScale),max(p.rowScale)]);
end
