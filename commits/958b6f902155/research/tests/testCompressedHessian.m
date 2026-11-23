function report=testCompressedHessian()
import casadi.*
x=MX.sym('x',30,1); sigma=MX.sym('sigma'); lambda=MX.sym('lambda',11,1);
objective=sumsqr(x(2:end)-sin(x(1:end-1)))+.1*sumsqr(x);
constraints=[x(1:3:end).^2+x(2:3:end).*x(3:3:end);sum(x)];
L=sigma*objective+dot(lambda,constraints);
[compressed,structure,grad]=buildAbfdCompressedHessian(x,L,sigma,lambda);
direct=Function('direct_hessian_contract',{x,sigma,lambda},{hessian(L,x)});
maxError=0; maxDirectionError=0;
for k=1:3
    point=.2*sin((1:30).'+k); multipliers=.3*cos((1:11).'+k); weight=.5+k;
    observed=full(compressed(point,weight,multipliers)); expected=full(direct(point,weight,multipliers));
    maxError=max(maxError,norm(observed-expected,'fro')/max(1,norm(expected,'fro')));
    direction=cos((1:30).'*sqrt(2)); direction=direction/norm(direction); h=1e-6;
    finite=full(grad(point+h*direction,weight,multipliers)-grad(point-h*direction,weight,multipliers))/(2*h);
    maxDirectionError=max(maxDirectionError,norm(observed*direction-finite)/max(1,norm(finite)));
end
assert(structure.colors<30 && maxError<1e-12 && maxDirectionError<1e-7, ...
    'ABFD:CompressedHessianContract','Compressed Hessian differs from direct exact AD.');
% Verify the real IPOPT custom-Hessian interface on an independent problem.
y=MX.sym('y',2,1); sigma2=MX.sym('sigma2'); lambda2=MX.sym('lambda2'); parameter=MX.sym('p',0,1);
f=sumsqr(y-[3;-1]); g=dot(y,y);
[~,~,~,H]=buildAbfdCompressedHessian(y,sigma2*f+lambda2*g,sigma2,lambda2);
callback=Function('custom_exact_hessian',{y,parameter,sigma2,lambda2},{triu(H)}, ...
    {'x','p','lam_f','lam_g'},{'hess_gamma_x_x'});
options=struct('print_time',false,'hess_lag',callback, ...
    'ipopt',struct('print_level',0,'tol',1e-9,'acceptable_iter',0,'max_iter',100));
solver=nlpsol('compressed_hessian_solver_contract','ipopt',struct('x',y,'f',f,'g',g),options);
sol=solver('x0',[.1;.1],'lbx',[-4;-4],'ubx',[4;4],'lbg',-inf,'ubg',4);
stats=solver.stats(); optimum=2*[3;-1]/sqrt(10); error=norm(full(sol.x)-optimum);
assert(stats.success && error<1e-7,'ABFD:CompressedHessianSolver','Custom Hessian solver did not converge to the known optimum.');
report=struct('passed',true,'structure',structure,'matrixRelativeError',maxError, ...
    'directionalRelativeError',maxDirectionError,'solverStatus',stats.return_status, ...
    'solverIterations',stats.iter_count,'optimumError',error, ...
    'scope','Three nonlinear derivative points and actual IPOPT custom-Hessian solve; full Stewart scale remains separate');
end
