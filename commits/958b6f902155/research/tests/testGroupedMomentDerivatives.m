function report=testGroupedMomentDerivatives()
% Noncontiguous variable groups, mixed nonlinear dependencies and PSD inputs.
import casadi.*
x=MX.sym('mixed_variables',7); meanIds=[1;4;7];
M=[sin(x(1))+2,x(4);x(7),cos(x(1))+2];
P=[x(2),x(3);x(3),x(5)]; S=M*P*M.'+x(6)^2*eye(2);
g=[sqrt(S(1,1));sqrt(S(2,2));S(1,2);sin(x(1)*x(7))+x(3)*x(4)];
f=trace(S)+x(4)^2+exp(x(7));
[callbacks,assembly]=buildAbfdGroupedMomentDerivatives(x,f,g,meanIds,true);
sigma=MX.sym('sigma'); lambda=MX.sym('lambda',4);
ordinaryJ=Function('ordinary_group_test_jacobian',{x},{jacobian(g,x)});
ordinaryH=Function('ordinary_group_test_hessian',{x,sigma,lambda},{hessian(sigma*f+dot(lambda,g),x)});
gradientF=Function('ordinary_group_test_gradient',{x,sigma,lambda},{gradient(sigma*f+dot(lambda,g),x)});
maxJ=0;maxH=0;maxSymmetry=0;maxFD=0;
for k=1:3
    point=[.1;2;.2;.15;1.5;.4;-.1]+k*.01;
    weight=sin((1:4).'+k); coefficient=.7+k/10;
    [~,J]=callbacks.jacobian(point,[]); expectedJ=full(ordinaryJ(point));
    H=full(callbacks.fullHessian(point,coefficient,weight)); expectedH=full(ordinaryH(point,coefficient,weight));
    maxJ=max(maxJ,norm(full(J)-expectedJ,'fro')/max(1,norm(expectedJ,'fro')));
    maxH=max(maxH,norm(H-expectedH,'fro')/max(1,norm(expectedH,'fro')));
    maxSymmetry=max(maxSymmetry,norm(H-H.','fro')/max(1,norm(H,'fro')));
    direction=cos((1:7).'+k);direction=direction/norm(direction);h=1e-6;
    finite=full(gradientF(point+h*direction,coefficient,weight)-gradientF(point-h*direction,coefficient,weight))/(2*h);
    maxFD=max(maxFD,norm(H*direction-finite)/max(1,norm(finite)));
end
assert(maxJ<1e-12 && maxH<1e-12 && maxSymmetry<1e-12 && maxFD<1e-7, ...
    'ABFD:GroupedDerivatives','Grouped AD differs from the full derivatives.');
% Verify the actual Ipopt callback contract with a small well-defined NLP.
objective=sumsqr(x-[.2;2;.1;.3;1;.4;.1]); constraints=[x(1)+x(4);x(7)+x(3)^2];
[cb,~]=buildAbfdGroupedMomentDerivatives(x,objective,constraints,meanIds,true);
options=struct('print_time',false,'jac_g',cb.jacobian,'hess_lag',cb.hessian, ...
    'ipopt',struct('print_level',0,'max_iter',50,'tol',1e-10,'linear_solver','ma27'));
solver=nlpsol('grouped_callback_contract','ipopt',struct('x',x,'f',objective,'g',constraints),options);
solution=solver('x0',ones(7,1),'lbg',[-inf;-inf],'ubg',[.8;.8]); stats=solver.stats();
assert(stats.success && norm(full(solution.x)-[.2;2;.1;.3;1;.4;.1])<1e-6, ...
    'ABFD:GroupedCallbackContract','Exact Ipopt callback interface failed.');
report=struct('passed',true,'jacobianRelativeError',maxJ,'hessianRelativeError',maxH, ...
    'hessianSymmetryError',maxSymmetry,'finiteDifferenceRelativeError',maxFD, ...
    'callbackSolverStatus',stats.return_status,'assembly',assembly, ...
    'scope','Mixed covariance-like nonlinear toy and real Ipopt callback contract; not Stewart optimization acceptance');
end
