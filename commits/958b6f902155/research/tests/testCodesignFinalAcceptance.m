function report=testCodesignFinalAcceptance()
m=buildOptModelCustom(); p=buildDynamicsProvider(m); t=0:.01:.03;
q=repmat(m.qHome,1,numel(t)); v=zeros(size(q)); f=p.inverse(q(:,1),v(:,1),v(:,1));
refs=struct('t',t,'q',q,'qd',v,'qdd',v,'Fleg',repmat(f,1,numel(t)));
scene=struct('q0',m.qHome,'hood',struct('obstacles',[]));
bundle=struct('refs',refs,'scene',scene);
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','controllerMode','fullstate-tvlqr', ...
    'designMode','codesign-A','initialCovariance',1e-12*eye(12),'processSpectralDensity',zeros(12)));
good=evaluateAbfdCodesignFeasibility(bundle,p,c); assert(good.passed);
c.initialCovariance=.1*eye(12);
bad=evaluateAbfdCodesignFeasibility(bundle,p,c); assert(~bad.passed && bad.maxIneq>0);
[~,accept]=updateCodesignTrustRegion(.1,1,2,true,bad.maxIneq,0,c.constraintTolerance);
assert(accept,'ABFD:FeasibilityPriority','A feasible pair must not be rejected just because its nominal cost increased.');
[~,accept]=updateCodesignTrustRegion(.1,2,1,true,0,bad.maxIneq,c.constraintTolerance);
assert(~accept,'ABFD:FeasibilityRegression','A cheaper infeasible pair must not replace a feasible pair.');
c.outputDir=tempname(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))),'results','abfd'));
rejected=false;
try
    run_A1_alternating_codesign(m,scene,p,c);
catch exception
    rejected=strcmp(exception.identifier,'ABFD:InitialTubeInfeasible');
end
assert(rejected && isfile(fullfile(c.outputDir,'codesign_initial_rejection.mat')));
fallback=struct('kind','physical sequential reference');
unchanged=finalizeAbfdCodesignAcceptance(struct('passed',true),good,good,struct('accepted',false),fallback);
assert(~unchanged.passed && unchanged.codesign.pairPassed && isequal(unchanged.lastFeasible,fallback), ...
    'ABFD:UnrefinedSeedAccepted','A zero-iteration/failed optimization must not masquerade as accepted A.');
refined=finalizeAbfdCodesignAcceptance(struct('passed',true),good,good,struct('accepted',true),fallback);
unsafe=finalizeAbfdCodesignAcceptance(struct('passed',true),good,bad,struct('accepted',true),fallback);
assert(refined.passed && ~unsafe.passed,'ABFD:RefinedPairAcceptance');
report=struct('passed',true,'lowUncertaintyAccepted',good.passed,'highUncertaintyRejected',~bad.passed, ...
    'infeasibleInitialCheckpoint',c.outputDir,'feasibilityPriorityPassed',true,'unrefinedSeedRejected',~unchanged.passed);
end
