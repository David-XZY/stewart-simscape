function p=ch5ForceQpPrepareV62(linear,r)
% Prepare an identical Hessian/constraint system once for a candidate group.
timer=tic;Q=(linear.Q+linear.Q')/2;A=linear.A;b=linear.b;E=linear.E;e=linear.e;n=size(Q,1);
radius=option(r,'trust_radius',1);keep=b<radius*sum(abs(A),2)+1e-10;A=A(keep,:);b=b(keep);
L=chol(Q,'lower');T=L'\eye(n);W=T'*Q*T;err=norm(W-eye(n),inf);
assert(err<1e-6,'ch5:Whitening','Whitening lost precision.');
originalA=[A;eye(n);-eye(n)];originalB=[b;radius*ones(2*n,1)];qpA=originalA*T;qpE=E*T;
rowScale=ones(size(originalB));equalityScale=ones(size(e));
if option(r,'qp_row_scaling',true)
    rowScale=max(vecnorm(qpA,2,2),1e-12);equalityScale=max(vecnorm(qpE,2,2),1e-12);
    qpA=qpA./rowScale;qpE=qpE./equalityScale;
end
p=struct('Q',Q,'T',T,'qpQ',(W+W')/2,'qpA',qpA,'qpB',originalB./rowScale,'qpE',qpE,'qpBE',-e./equalityScale, ...
    'A',originalA,'b',originalB,'E',E,'e',e,'rowScale',rowScale,'equalityScale',equalityScale, ...
    'whiteningError',err,'radius',radius,'transformSeconds',toc(timer),'n',n);
p.options=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',option(r,'qp_max_iterations',400), ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-8);
end
function value=option(r,name,default)
value=default;if isfield(r.reduced,name),value=r.reduced.(name);end
end
