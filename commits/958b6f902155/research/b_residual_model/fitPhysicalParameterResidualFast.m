function fit=fitPhysicalParameterResidualFast(data,model)
% Same physical parameterization and loss as the original fit, evaluated using
% exact rigid-body inertial sufficient statistics rather than a sample loop
% on every optimizer evaluation. No validation/test data are admitted.
assert(all(strcmp(string(data.split),'train')),'ABFD:TrainingLeakage','Physical fit may only see training groups.');
n=size(data.wrench,1);
assert(n>0 && isequal(size(data.x),[n,12]) && isequal(size(data.acceleration),[n,6]), ...
    'ABFD:DatasetShape','Invalid inertial regression data.');
I=model.dynamics.inertiaAtCOM_P; S=trace(I)/2*eye(3)-I;
L=chol(S+1e-10*eye(3),'lower');
theta=[log(model.dynamics.totalMass);model.dynamics.comP;log(diag(L));L(2,1);L(3,1);L(3,2)];
lower=[log(model.dynamics.totalMass*.25);model.dynamics.comP-.25;-10*ones(3,1);-100*ones(3,1)];
upper=[log(model.dynamics.totalMass*4);model.dynamics.comP+.25;10*ones(3,1);100*ones(3,1)];
scale=max(std(data.wrench,0,1),1).'; Y=zeros(6*n,10); target=zeros(6*n,1); timer=tic;
for j=1:n
    ids=(j-1)*6+(1:6);
    Y(ids,:)=buildInertialWrenchRegressor(data.x(j,1:6).',data.x(j,7:12).',data.acceleration(j,:).',model)./scale;
    target(ids)=data.wrench(j,:).'./scale;
end
assert(all(isfinite([Y(:);target])),'ABFD:DatasetNonfinite','Invalid inertial regression values.');
% Thin QR retains the residual exactly without cancellation in Y''Y normal equations.
[~,T]=qr([Y,target],0);
T=T/sqrt(6*n); design=T(:,1:10); observed=T(:,11);
preparationSeconds=toc(timer);
options=optimoptions('fmincon','Display','off','MaxFunctionEvaluations',5000,'MaxIterations',300);
timer=tic; [best,value,flag,info]=fmincon(@loss,theta,[],[],[],[],lower,upper,[],options);
assert(flag>0,'ABFD:PhysicalFitFailed','Constrained physical fit did not converge.');
fit=decode(best); fit.loss=value; fit.trainingGroups=unique(data.groupId);
fit.identification=struct('method','exact inertial regressor with QR sufficient statistics', ...
    'samples',n,'preparationSeconds',preparationSeconds,'optimizationSeconds',toc(timer), ...
    'iterations',info.iterations,'functionCount',info.funcCount,'exitFlag',flag, ...
    'initialLoss',loss(theta),'dataLoss',value-1e-5*sum((best-theta).^2),'regressorRank',rank(design));
    function value=loss(t)
        p=decode(t); shift=(p.com.'*p.com)*eye(3)-p.com*p.com.';
        origin=p.inertia+p.mass*shift;
        beta=[p.mass;p.mass*p.com;diag(origin);origin(1,2);origin(1,3);origin(2,3)];
        residual=design*beta-observed;
        value=sum(residual.^2)+1e-5*sum((t-theta).^2);
    end
end
function p=decode(t)
L=[exp(t(5)),0,0;t(8),exp(t(6)),0;t(9),t(10),exp(t(7))]; S=L*L.';
p=struct('mass',exp(t(1)),'com',t(2:4),'inertia',trace(S)*eye(3)-S);
end
