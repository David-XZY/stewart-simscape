function report=testTubeMeanMetrics()
% Changing the prediction center must change coverage, not tracking metrics.
m=buildOptModelCustom(); p=buildDynamicsProvider(m); c=makeAbfdConfig(); t=[0,.01,.02];
q=repmat(m.qHome,1,3); q(1,:)=q(1,:)+.003; z=zeros(6,3);
F=p.inverse(q(:,1),zeros(6,1),zeros(6,1));
log=struct('t',t,'q',q,'qd',z,'qdd',z,'actualForce',repmat(F,1,3), ...
    'feedbackForce',z,'stepSeconds',.001*ones(3,1));
bundle=struct('model',m,'scene',buildCylinderBoxTransferScene(m), ...
    'refs',struct('t',t,'q',repmat(m.qHome,1,3),'qd',z));
tube=struct('t',t,'P',repmat(1e-10*eye(12),1,1,3));
original=evaluateUnifiedMetrics(log,bundle,c,tube);
tube.mean=[q;z]; centered=evaluateUnifiedMetrics(log,bundle,c,tube);
assert(original.tubeCoverage95==0 && centered.tubeCoverage95==1);
assert(isequal(original.translationRms,centered.translationRms));
bad=tube; bad.mean=bad.mean(1:11,:); rejected=false;
try
    evaluateUnifiedMetrics(log,bundle,c,bad);
catch e
    rejected=strcmp(e.identifier,'ABFD:TubeMean');
end
assert(rejected,'ABFD:TubeMeanTest','Malformed mean accepted.');
bad=tube; bad.t=t+.01; rejected=false;
try
    evaluateUnifiedMetrics(log,bundle,c,bad);
catch e
    rejected=strcmp(e.identifier,'ABFD:TubeMeanCoverage');
end
assert(rejected,'ABFD:TubeMeanTest','Uncovered time range accepted.');
report=struct('passed',true,'referenceCenteredCoverage',original.tubeCoverage95, ...
    'meanCenteredCoverage',centered.tubeCoverage95,'scope','metric semantics only; synthetic fixture');
end
