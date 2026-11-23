function [delta,info]=ch5ReducedSolveV19(linear,backend)
% Exact change of coordinates: delta=T*z, T'*Q*T=I. No regularization/slack.
timer=tic;Q=(linear.Q+linear.Q')/2;f=linear.f;A=linear.A;b=linear.b;E=linear.E;e=linear.e;n=numel(f);
keep=b<sum(abs(A),2)+1e-10;A=A(keep,:);b=b(keep);phase=tic;
L=chol(Q,'lower');T=L'\eye(n);W=T'*Q*T;whiteningError=norm(W-eye(n),inf);
assert(whiteningError<1e-6,'ch5:Whitening','The exact coordinate transform lost precision.');
qpQ=(W+W')/2;qpF=T'*f;qpA=[A*T;T;-T];qpB=[b;ones(2*n,1)];qpE=E*T;qpBE=-e;
d0=zeros(n,1);Z=eye(n);
if string(backend)=="whiten-null-active"
    [left,s,right]=svd(qpE);sv=diag(s);rankE=sum(sv>max(size(qpE))*eps(max(sv)));
    assert(rankE==size(E,1),'ch5:EqualityRank','Do not drop terminal equations.');
    d0=-right(:,1:rankE)*((left(:,1:rankE)'*e)./sv(1:rankE));Z=right(:,rankE+1:end);
    qpF=Z'*(qpQ*d0+qpF);qpQ=Z'*qpQ*Z;qpB=qpB-qpA*d0;qpA=qpA*Z;qpE=[];qpBE=[];
end
transformSeconds=toc(phase);persistent options
if isempty(options),options=optimoptions('quadprog','Display','off','Algorithm','active-set','MaxIterations',400, ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-8);end
phase=tic;[z,~,flag,output]=quadprog((qpQ+qpQ')/2,qpF,qpA,qpB,qpE,qpBE,[],[],zeros(size(qpQ,1),1),options);solveSeconds=toc(phase);
delta=[];finite=~isempty(z)&&all(isfinite(z));if finite,delta=T*(d0+Z*z);end
violation=inf;eqError=inf;objective=inf;
if finite,violation=max([A*delta-b;abs(delta)-1;0]);eqError=norm(E*delta+e,inf);objective=.5*delta'*Q*delta+f'*delta;end
info=struct('backend',char(backend),'status',sprintf('%s_flag_%d',backend,flag),'exitflag',flag, ...
    'solver_success',flag>0,'finite',finite,'iterations',output.iterations,'variables',size(qpQ,1),'inequalities',numel(qpB), ...
    'solver_seconds',solveSeconds,'transform_seconds',transformSeconds,'total_seconds',toc(timer),'whitening_error',whiteningError, ...
    'original_inequality_violation',violation,'original_equality_error',eqError,'objective',objective, ...
    'original_linear_feasible',finite&&violation<=1e-6&&eqError<=1e-6);
end
