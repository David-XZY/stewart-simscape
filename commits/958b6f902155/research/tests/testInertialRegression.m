function report=testInertialRegression()
% Independent comparison with existing nonlinear COM wrench implementation.
m=buildOptModelCustom(); stream=RandStream('mt19937ar','Seed',417);
largest=0; data=struct('x',zeros(64,12),'acceleration',zeros(64,6),'wrench',zeros(64,6), ...
    'split',repmat("train",64,1),'groupId',repmat("synthetic-contract-only",64,1));
for k=1:64
    q=m.qHome+.3*randn(stream,6,1); v=.2*randn(stream,6,1); a=.7*randn(stream,6,1);
    physical=m; physical.dynamics.totalMass=m.dynamics.totalMass*(.8+.4*rand(stream));
    physical.dynamics.comP=m.dynamics.comP+.1*randn(stream,3,1);
    L=randn(stream,3); S=L*L.'; physical.dynamics.inertiaAtCOM_P=trace(S)*eye(3)-S;
    physical.dynamics.includeGravity=mod(k,2)==0;
    p=physical.dynamics; origin=p.inertiaAtCOM_P+p.totalMass*((p.comP.'*p.comP)*eye(3)-p.comP*p.comP.');
    beta=[p.totalMass;p.totalMass*p.comP;diag(origin);origin(1,2);origin(1,3);origin(2,3)];
    expected=computeCompositeRequiredWrench(q,v,a,physical);
    actual=buildInertialWrenchRegressor(q,v,a,physical)*beta;
    largest=max(largest,max(abs(expected-actual)));
    assert(norm(expected-actual)<1e-9*max(1,norm(expected)));
    data.x(k,:)=[q;v].'; data.acceleration(k,:)=a.';
    data.wrench(k,:)=computeCompositeRequiredWrench(q,v,a,m).';
end
f=fitPhysicalParameterResidual(data,m);
assert(f.loss<1e-12 && abs(f.mass-m.dynamics.totalMass)<1e-4);
assert(min(eig(f.inertia))>0 && max(eig(f.inertia))<=trace(f.inertia)/2+1e-10);
bad=data; bad.split(1)="test"; rejected=false;
try
    fitPhysicalParameterResidualFast(bad,m);
catch e
    rejected=strcmp(e.identifier,'ABFD:TrainingLeakage');
end
assert(rejected);
report=struct('maxWrenchError',largest,'fit',f,'passed',true,'scope','synthetic mathematical contract only; no trained asset');
end
