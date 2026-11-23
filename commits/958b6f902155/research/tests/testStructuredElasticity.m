function report=testStructuredElasticity()
% Synthetic mechanics contract only. No fixture is a trained residual model.
m=buildOptModelCustom(); n=struct('schemaVersion',1,'kind','residual-wrench-tanh','status','test-fixture', ...
    'physical',struct('mass',m.dynamics.totalMass,'com',m.dynamics.comP,'inertia',m.dynamics.inertiaAtCOM_P, ...
        'trainingGroups',{{'mechanics-contract'}}), ...
    'inputMean',zeros(12,1),'inputScale',ones(12,1),'outputScale',ones(6,1),'varianceCalibration',1, ...
    'layers',struct('W',zeros(12),'b',zeros(12,1)));
n.physical.elasticity=struct('reference','initial-leg-length-v1','stiffness',500,'damping',30, ...
    'trainingGroups',{{'mechanics-contract'}});
rejected=false;
try
    buildDynamicsProvider(m,'residual-B',n,true);
catch e
    rejected=strcmp(e.identifier,'ABFD:ResidualContextRequired');
end
assert(rejected,'ABFD:ElasticityTest','Missing initial pose must be rejected.');
q0=m.qHome+[.01;-.02;.01;.02;-.01;.03]; p=buildDynamicsProvider(m,'residual-B',n,true,q0);
other=buildDynamicsProvider(m,'residual-B',n,true,q0+[.01;zeros(5,1)]);
assert(~strcmp(p.hash,other.hash),'ABFD:ElasticityTest','Rest context must be included in the provider hash.');
bad=n; bad.physical.elasticity.damping=-1;
negativeRejected=throws(@()validateResidualNetwork(bad,true));
bad=n; bad.physical.elasticity.trainingGroups={'held-out'};
leakageRejected=throws(@()validateResidualNetwork(bad,true));
legacyRejected=throws(@()conditionResidualOnMeasuredInertia(p,struct(),struct(),''));
assert(negativeRejected && leakageRejected && legacyRejected,'ABFD:ElasticityTest', ...
    'Invalid passive parameters, held-out fit or unsupported conditioning must be rejected.');
plain=buildDynamicsProvider(m); ad=buildCasadiCompositeDynamics(p);
valueError=0; jacobianError=0; energyError=0; minimumDissipation=inf;
for k=1:6
    q=q0+.02*sin((1:6).'+k); v=.1*cos((1:6).'+k); x=[q;v]; a=.2*sin((1:6).'-k);
    F=p.inverse(q,v,a); numeric=p.forward(x,F); valueError=max(valueError,norm(numeric-full(ad.f(x,F)),inf));
    [A,~]=ad.linearize(x,F); finite=zeros(12); h=1e-6;
    for j=1:12
        delta=zeros(12,1); delta(j)=h;
        finite(:,j)=(p.forward(x+delta,F)-p.forward(x-delta,F))/(2*h);
    end
    jacobianError=max(jacobianError,norm(full(A)-finite,'fro')/max(1,norm(finite,'fro')));
    rest=p.elasticity.referenceLength; J=sgpJacobian(q,m);
    springW=p.evaluate(q,zeros(6,1),zeros(6,1)); rigidW=plain.evaluate(q,zeros(6,1),zeros(6,1));
    [E,~]=rpyRateMapZYX(q(4:6),v(4:6)); T=blkdiag(eye(3),E); gradient=zeros(6,1);
    for j=1:6
        delta=zeros(6,1); delta(j)=h;
        plus=sgpIK(q+delta,m); minus=sgpIK(q-delta,m);
        gradient(j)=250*(sum((plus.L-rest).^2)-sum((minus.L-rest).^2))/(2*h);
    end
    energyError=max(energyError,norm(T.'*(springW.W-rigidW.W)-gradient,inf));
    dampingW=J.Jv.'*(30*J.Jq*v); minimumDissipation=min(minimumDissipation,dampingW.'*T*v);
end
assert(valueError<1e-9 && jacobianError<1e-7 && energyError<1e-6 && minimumDissipation>=0, ...
    'ABFD:ElasticityTest','Passive term mechanics or full state derivatives differ.');
report=struct('passed',true,'maxDynamicsError',valueError,'maxJacobianRelativeError',jacobianError, ...
    'maxPotentialGradientError',energyError,'minimumDampingPower',minimumDissipation, ...
    'missingContextRejected',rejected,'negativeDampingRejected',negativeRejected,'heldOutFitRejected',leakageRejected, ...
    'legacyConditioningRejected',legacyRejected,'scope','Synthetic mechanics, context binding and numerical/CasADi derivative contract only');
end
function rejected=throws(call)
rejected=false;
try
    call();
catch
    rejected=true;
end
end
