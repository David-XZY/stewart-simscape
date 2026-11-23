function report=testRootMomentDerivatives()
% Exact chain rule with signed multipliers and mean-dependent affine roots.
import casadi.*
x=MX.sym('root_chain_state',8); meanIds=[1;4;8];
eta=x([2,3,5,6,7]); theta=x(meanIds);
B=[cos(theta(1))+2,theta(2)^2,.1,theta(3),0; ...
    0,1,cos(theta(3))+1,theta(1),2;1,1,0,0,1];
a=B*eta+20+[sin(theta(1));cos(theta(2));exp(theta(3))];
base=[eta(1)*theta(1)+eta(2);sin(theta(2))+eta(3);eta(4)+eta(5)*theta(3)^2;eta(2)+eta(3)];
map=sparse([1,0,1;0,2,0;-1,0,0;0,3,2]); g=base+map*sqrt(a);
f=sum(eta)+dot(theta,theta)+theta(1)*eta(3);
roots=struct('arguments',a,'affineConstraints',base,'constraintMap',map);
[cb,assembly]=buildAbfdGroupedMomentDerivatives(x,f,g,meanIds,true,@(~)[],1,roots);
sigma=MX.sym('objective_multiplier'); lambda=MX.sym('constraint_multipliers',4);
reference=Function('root_chain_reference',{x,sigma,lambda}, ...
    {jacobian(g,x),hessian(sigma*f+dot(lambda,g),x),gradient(sigma*f+dot(lambda,g),x),g});
maxJ=0;maxH=0;maxFD=0;maxConstraint=0;
for k=1:4
    point=.1*sin((1:8).'+k); multipliers=cos((1:4).'+k); coefficient=.7+k/10;
    [J,H,~,value]=reference(point,coefficient,multipliers);
    [~,actualJ]=cb.jacobian(point,[]); actualH=full(cb.fullHessian(point,coefficient,multipliers));
    maxJ=max(maxJ,norm(full(J-actualJ),'fro')/max(1,norm(full(J),'fro')));
    maxH=max(maxH,norm(full(H)-actualH,'fro')/max(1,norm(full(H),'fro')));
    maxConstraint=max(maxConstraint,norm(full(cb.reconstructedConstraints(point)-value),inf));
    direction=cos((1:8).'+k); direction=direction/norm(direction); step=1e-6;
    [~,~,plus]=reference(point+step*direction,coefficient,multipliers);
    [~,~,minus]=reference(point-step*direction,coefficient,multipliers);
    finite=full(plus-minus)/(2*step);
    maxFD=max(maxFD,norm(actualH*direction-finite)/max(1,norm(finite)));
end
assert(maxJ<1e-12 && maxH<1e-12 && maxFD<1e-8 && maxConstraint<1e-12, ...
    'ABFD:RootChainDerivative','Root factorization lost a covariance or mixed derivative.');
rejected=0;
for k=1:3
    bad=roots; objective=f;
    if k==1, bad.arguments=[roots.arguments(1)+eta(1)^2;roots.arguments(2:end)]; end
    if k==2, bad.affineConstraints=[roots.affineConstraints(1)+sin(eta(1));roots.affineConstraints(2:end)]; end
    if k==3, objective=objective+eta(1)^2; end
    try
        buildAbfdGroupedMomentDerivatives(x,objective,g,meanIds,true,@(~)[],1,bad);
    catch e
        if strcmp(e.identifier,'ABFD:NonAffineCovariance'), rejected=rejected+1; else, rethrow(e); end
    end
end
assert(rejected==3,'ABFD:RootChainGuard','Nonaffine covariance expressions must reject the shortcut.');
% Real Stewart tightening has a covariance-independent mean and one root
% per row, often repeated in upper/lower constraints with shared variance.
pure=roots;pure.affineConstraints=[sum(theta);cos(theta(2));theta(1)^2;exp(theta(3))];
pure.constraintMap=sparse([2,0,0;0,3,0;0,0,4;5,0,0]);
pureG=pure.affineConstraints+pure.constraintMap*sqrt(a);
[reuse,reuseReport]=buildAbfdGroupedMomentDerivatives(x,f,pureG,meanIds,true,@(~)[],1,pure);
ordinaryPure=Function('pure_root_hessian',{x,sigma,lambda},{hessian(sigma*f+dot(lambda,pureG),x)});
reusedH=full(reuse.fullHessian(point,coefficient,multipliers)); expectedH=full(ordinaryPure(point,coefficient,multipliers));
reuseError=norm(reusedH-expectedH,'fro')/max(1,norm(expectedH,'fro'));
assert(strcmp(reuseReport.covarianceDerivativeRecovery,'single-root constraint rows') && reuseError<1e-12, ...
    'ABFD:RootJacobianReuse','Reusing root constraint derivatives changed the full Hessian.');
report=struct('passed',true,'jacobianRelativeError',maxJ,'hessianRelativeError',maxH, ...
    'finiteDifferenceRelativeError',maxFD,'constraintReconstructionError',maxConstraint, ...
    'nonaffineRejections',rejected,'assembly',assembly,'reusedJacobianHessianError',reuseError, ...
    'reusedJacobianAssembly',reuseReport, ...
    'scope','Exact root-chain derivatives with mixed terms and rejected nonlinear covariance inputs; not Stewart performance acceptance');
end
