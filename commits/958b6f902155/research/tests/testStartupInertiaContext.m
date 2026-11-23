function report=testStartupInertiaContext(datasetFile,auditFile,oldAsset,originalRoot)
% Reuse logged startup observations with deliberately marked network fixture.
% Tests the new protocol; the fixture never becomes a deployable trained asset.
data=jsondecode(fileread(datasetFile)); audit=jsondecode(fileread(auditFile));
net=importResidualNetworkWeights(oldAsset); net.status='test-fixture';
net.physical=data.physical; net.meanAnchoring=data.meanAnchoring;
net.datasetHash=abfdFileHash(datasetFile); net.datasetAudit=audit; net.datasetAuditHash=abfdFileHash(auditFile);
context=data.inertiaContexts(1); source=load(fullfile(originalRoot,'B0',context.taskId,'result.mat'),'result');
bundle=source.result.planning.bundle; profile=source.result.profile; q0=context.initialPose(:);
measurement=readMeasuredInertiaRegression(context.calibrationFile,bundle,profile);
missing=reject(@()buildDynamicsProvider(bundle.model,'residual-B',net,true,q0),'ABFD:StartupInertiaContextRequired');
foreign=measurement; foreign.nominalModelHash=repmat('0',1,64);
wrong=reject(@()buildDynamicsProvider(bundle.model,'residual-B',net,true,q0,foreign),'ABFD:StartupInertiaContextRequired');
base=buildDynamicsProvider(bundle.model,'residual-B',net,true,q0,measurement);
unobservedAd=reject(@()buildCasadiCompositeDynamics(base),'ABFD:StartupBalanceRequired');
unobservedControl=reject(@()base.inverse(q0,zeros(6,1),zeros(6,1)),'ABFD:StartupBalanceRequired');
p=conditionResidualMeanOnStaticBalance(base,bundle,profile,context.calibrationFile);
ad=buildCasadiCompositeDynamics(p); rigid=buildDynamicsProvider(p.model);
[anchor,~]=evaluateResidualMlp([q0;zeros(6,1)],net);
staticError=norm(p.inverse(q0,zeros(6,1),zeros(6,1))-context.staticForce(:),inf);
formulaError=0; valueError=0; jacobianError=0;
for k=1:4
    q=q0+.005*sin((1:6).'+k); v=.02*cos((1:6).'+k); a=.04*sin((1:6).'-k);
    [mean,~]=evaluateResidualMlp([q;v],net); kin=sgpIK(q,bundle.model); J=sgpJacobian(q,bundle.model);
    passive=J.Jv.'*(p.elasticity.stiffness*(kin.L-p.elasticity.referenceLength)+p.elasticity.damping*J.Jq*v);
    expected=rigid.evaluate(q,v,a); expected=expected.W+passive+context.staticOffset(:)+mean-anchor;
    actual=p.evaluate(q,v,a); formulaError=max(formulaError,norm(actual.W-expected,inf));
    [symbolic,~]=ad.wrench(q,v,a); valueError=max(valueError,norm(actual.W-full(symbolic),inf));
    force=p.inverse(q,v,a); [A,~]=ad.linearize([q;v],force); finite=zeros(12); h=1e-6;
    for j=1:12
        delta=zeros(12,1); delta(j)=h;
        finite(:,j)=(p.forward([q;v]+delta,force)-p.forward([q;v]-delta,force))/(2*h);
    end
    jacobianError=max(jacobianError,norm(full(A)-finite,'fro')/max(1,norm(finite,'fro')));
end
assert(missing && wrong && unobservedAd && unobservedControl && staticError<1e-8 && ...
    formulaError<1e-8 && valueError<1e-8 && jacobianError<1e-7,'ABFD:StartupInertiaTest');
report=struct('passed',true,'staticForceError',staticError,'anchoredFormulaError',formulaError, ...
    'numericCasadiError',valueError,'jacobianRelativeError',jacobianError, ...
    'missingObservationRejected',missing,'foreignObservationRejected',wrong, ...
    'unconditionedAdRejected',unobservedAd,'unconditionedControlRejected',unobservedControl, ...
    'scope','Actual startup observations with explicitly synthetic network fixture; protocol test only, not trained model acceptance');
end
function yes=reject(call,identifier)
yes=false;
try
    call();
catch e
    assert(strcmp(e.identifier,identifier),'ABFD:UnexpectedRejection','Unexpected rejection: %s',e.identifier);
    yes=true;
end
end
