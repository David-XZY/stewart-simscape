function [delta,info]=ch5ReducedSolveV18(linear,backend)
% Solve exactly the same bounded convex QP, optionally eliminating equalities.
timer=tic;Q=(linear.Q+linear.Q')/2;f=linear.f;A=linear.A;b=linear.b;E=linear.E;e=linear.e;n=numel(f);
keep=b<sum(abs(A),2)+1e-10;A=A(keep,:);b=b(keep);
originalA=A;originalB=b;
nullMethod=startsWith(string(backend),"null");transformSeconds=0;d0=zeros(n,1);Z=eye(n);
if nullMethod
    phase=tic;[left,s,right]=svd(E);sv=diag(s);rankE=sum(sv>max(size(E))*eps(max(sv)));
    assert(rankE==size(E,1),'ch5:EqualityRank','Do not discard dependent terminal equations.');
    d0=-right(:,1:rankE)*((left(:,1:rankE)'*e)./sv(1:rankE));Z=right(:,rankE+1:end);
    assert(norm(E*d0+e,inf)<1e-8&&norm(E*Z,inf)<1e-8,'ch5:EqualityTransform','Equality transform failed.');
    qpQ=Z'*Q*Z;qpF=Z'*(Q*d0+f);qpA=[A*Z;Z;-Z];qpB=[b-A*d0;ones(n,1)-d0;ones(n,1)+d0];
    qpE=[];qpBE=[];lb=[];ub=[];initial=zeros(size(Z,2),1);transformSeconds=toc(phase);
else
    qpQ=Q;qpF=f;qpA=A;qpB=b;qpE=E;qpBE=-e;lb=-ones(n,1);ub=ones(n,1);initial=zeros(n,1);
end
beforeCount=numel(b);
if contains(string(backend),"dedup")
    [rows,first,group]=unique(A,'rows');bounds=accumarray(group,b,[],@min);[~,order]=sort(first);A=rows(order,:);b=bounds(order);
    nontrivial=any(A~=0,2)|b<0;A=A(nontrivial,:);b=b(nontrivial);
    qpA=A;qpB=b;
end
optimality=1e-6;if contains(string(backend),"loose"),optimality=1e-4;end
active=true;algorithm='active-set';
options=optimoptions('quadprog','Display','off','Algorithm',algorithm,'MaxIterations',400, ...
    'ConstraintTolerance',1e-7,'OptimalityTolerance',optimality);
if ~active,options.ScaleProblem=true;options.LinearSolver='dense';end
phase=tic;[z,~,flag,output]=quadprog((qpQ+qpQ')/2,qpF,qpA,qpB,qpE,qpBE,lb,ub,initial,options);solveSeconds=toc(phase);
delta=[];finite=~isempty(z)&&all(isfinite(z));if finite,delta=d0+Z*z;end
violation=inf;eqError=inf;objective=inf;
if finite
    violation=max([originalA*delta-originalB;abs(delta)-1;0]);eqError=norm(E*delta+e,inf);objective=.5*delta'*Q*delta+f'*delta;
end
info=struct('backend',char(backend),'status',sprintf('%s_flag_%d',backend,flag),'exitflag',flag, ...
    'solver_success',flag>0,'finite',finite,'iterations',output.iterations,'variables',size(qpQ,1), ...
    'inequalities',numel(qpB),'original_inequalities',beforeCount,'optimality_tolerance',optimality,'solver_seconds',solveSeconds,'transform_seconds',transformSeconds,'total_seconds',toc(timer), ...
    'original_inequality_violation',violation,'original_equality_error',eqError,'objective',objective, ...
    'original_linear_feasible',finite&&violation<=1e-6&&eqError<=1e-6);
end
