function report=testGuardedCodesignSolver(outputDir,formulation,hessian,continuations,primalStartFile,assembly,reductionMode,coordinates,derivativeAssembly)
% Exercise IPOPT creation and one real iteration, not trajectory acceptance.
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
s.q0=m.qHome; s.qWaypoint=m.qHome; s.qGoal=m.qHome;
s.qd0=zeros(6,1); s.qdGoal=zeros(6,1);
s.phase.durationApproach=.03; s.phase.durationInsertion=.015;
s.phase.numIntervalsApproach=2; s.phase.numIntervalsInsertion=1;
d=buildTwoPhaseIHSDiscretization(s,2,1);
[~,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d); p=buildDynamicsProvider(m);
initial.Xnode=repmat([m.qHome;zeros(6,1)],1,d.numNodes);
initial.XmidIHS=repmat([m.qHome;zeros(6,1)],1,d.numMidpoints);
initial.Anode=zeros(6,d.numNodes); initial.Amid=zeros(6,d.numMidpoints);
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2', ...
    'plannerProfile','guarded-exact-v1','controllerMode','geometric-D', ...
    'designMode','codesign-A','solverMaxIterations',1,'solverMaxCpuTime',30,'outputDir',outputDir));
if nargin>1
    c.solverFormulation=validatestring(formulation, ...
        {'full-space','hs-reduced','covariance-shooting','covariance-shooting-hs-reduced'});
    if any(strcmp(formulation,{'covariance-shooting','covariance-shooting-hs-reduced'})), c.covarianceShootingInterval=.02; end
end
if nargin<3, hessian='limited-memory'; else, c.solverHessianApproximation=hessian; end
if nargin<4, continuations=0; else, c.solverContinuationCount=continuations; end
if nargin>5 && ~isempty(assembly), c.solverHessianAssembly=assembly; end
if nargin>6 && ~isempty(reductionMode), c.solverReductionMode=reductionMode; end
if nargin>7, c.covarianceCoordinates=coordinates; end
if nargin>8, c.solverDerivativeAssembly=derivativeAssembly; end
c=makeAbfdConfig(c);
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS, ...
    'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime);
design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
primalStart=[];
if nargin>4 && ~isempty(primalStartFile)
    saved=load(primalStartFile,'result'); primalStart=saved.result.z;
end
result=solveAbfdIhsid(m,s,p,c,initial,[],design,primalStart);
save(fullfile(outputDir,'solver_contract.mat'),'result','c');
if ~isempty(primalStart)
    assert(strcmp(result.seedReport.primalStart.hash,abfdHash(primalStart)), ...
        'ABFD:PrimalStartContract','Explicit physical iterate was not used as the numerical starting point.');
end
assert(strcmp(result.numerics.hessian,hessian) && all(isfinite(result.z)) && ...
    result.iterations==1+continuations,'ABFD:GuardedSolver','Expected one finite IPOPT iteration per configured segment.');
progress=jsondecode(fileread(fullfile(outputDir,'solver_progress.json')));
assert(strcmp(progress.stage,'complete') && strcmp(progress.hessian,hessian));
report=struct('passed',true,'solverStatus',result.solverStatus,'hessian',result.numerics.hessian, ...
    'preparationSeconds',result.preparationTime,'solveSeconds',result.solveTime, ...
    'iterations',result.iterations,'solverCalls',result.solverContinuation.solverCalls,'trajectoryAccepted',result.passed, ...
    'scope','Actual guarded A solver construction and bounded iteration(s) on a deliberately unaccepted short fixture');
if ~isempty(primalStart), report.primalStart=result.seedReport.primalStart; end
if isfield(result.numerics,'hessianAssembly'), report.hessianAssembly=result.numerics.hessianAssembly; end
if isfield(c,'solverDerivativeAssembly') && strcmp(c.solverDerivativeAssembly,'grouped-moments')
    report.derivativeAssembly=result.numerics.derivativeAssembly;
    assert(report.derivativeAssembly.variables==numel(result.solverState.scaledDecision) && ...
        report.derivativeAssembly.covarianceVariables==numel(result.auxiliaryDecision) && ...
        report.derivativeAssembly.meanVariables+report.derivativeAssembly.covarianceVariables== ...
        report.derivativeAssembly.variables,'ABFD:GroupedSampledLayout', ...
        'Every retained physical and covariance coordinate must be differentiated.');
end
if isfield(c,'solverFormulation') && strcmp(c.solverFormulation,'covariance-shooting-hs-reduced')
    assert(isfield(result.numerics,'reduction') && isfield(result.numerics,'covarianceShooting') && ...
        numel(result.auxiliaryDecision)==156,'ABFD:CombinedSolverLayout','Both covariance cuts and the exact HS elimination must remain active.');
    report.reduction=result.numerics.reduction; report.covarianceShooting=result.numerics.covarianceShooting;
end
if isfield(c,'solverReductionMode')
    assert(strcmp(result.numerics.reduction.mode,c.solverReductionMode), ...
        'ABFD:SolverReductionMode','The production solver did not use the selected HS elimination mode.');
end
end
