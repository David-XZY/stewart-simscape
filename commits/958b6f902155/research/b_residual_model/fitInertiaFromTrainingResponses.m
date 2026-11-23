function fit=fitInertiaFromTrainingResponses(measurements,model)
% Equal task weighting and physically realizable inertia. No motion residual
% is allowed to masquerade as inertia; only train force-response probes enter.
assert(~isempty(measurements) && all(strcmp({measurements.split},'train')), ...
    'ABFD:TrainingLeakage','Only frozen training-group response probes may fit inertia.');
groups={measurements.taskId}; assert(numel(unique(groups))==numel(groups),'ABFD:DuplicateInertiaGroup');
Y=vertcat(measurements.design); target=vertcat(measurements.target);
normalizer=model.dynamics.totalMass*sqrt(numel(target)); Y=Y/normalizer; target=target/normalizer;
I=model.dynamics.inertiaAtCOM_P; S=trace(I)/2*eye(3)-I;
L=chol(S,'lower'); theta=[log(model.dynamics.totalMass);model.dynamics.comP;log(diag(L));L(2,1);L(3,1);L(3,2)];
lower=[log(model.dynamics.totalMass*.25);model.dynamics.comP-.25;-10*ones(3,1);-100*ones(3,1)];
upper=[log(model.dynamics.totalMass*4);model.dynamics.comP+.25;10*ones(3,1);100*ones(3,1)];
options=optimoptions('fmincon','Display','off','MaxFunctionEvaluations',5000,'MaxIterations',300);
timer=tic; [best,value,flag,info]=fmincon(@loss,theta,[],[],[],[],lower,upper,[],options);
assert(flag>0,'ABFD:PhysicalFitFailed'); fit=decode(best); fit.loss=value; fit.trainingGroups=groups;
fit.identification=struct('method','Constrained inertia from measured force-to-spatial-acceleration responses', ...
    'groups',{groups},'responseHashes',{{measurements.calibrationHash}},'taskWeighting','equal per training task', ...
    'rows',numel(target),'regressorRank',rank(Y),'optimizationSeconds',toc(timer),'iterations',info.iterations, ...
    'functionCount',info.funcCount,'exitFlag',flag,'sourceHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
    'scope','Train-only independent physical identification; motion residual and held-out groups excluded');
    function value=loss(t)
        p=decode(t); origin=p.inertia+p.mass*((p.com.'*p.com)*eye(3)-p.com*p.com.');
        beta=[p.mass;p.mass*p.com;diag(origin);origin(1,2);origin(1,3);origin(2,3)];
        value=sum((Y*beta-target).^2)+1e-5*sum((t-theta).^2);
    end
end
function p=decode(t)
L=[exp(t(5)),0,0;t(8),exp(t(6)),0;t(9),t(10),exp(t(7))]; S=L*L.';
p=struct('mass',exp(t(1)),'com',t(2:4),'inertia',trace(S)*eye(3)-S);
end
