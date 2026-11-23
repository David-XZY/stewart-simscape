function report=testGuardedActuatorMomentSolver(outputDir,asset,temporalFile,bundle,calibrationFile,plain,hessian,assembly,directions,derivativeAssembly)
% Exercise the complete physical NLP, moment variables, bounds and IPOPT.
% One bounded iteration of an intentionally infeasible short fixture is not
% claimed as trajectory convergence, dense acceptance or full A+B execution.
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
if nargin<7, hessian='exact'; end
hessian=validatestring(hessian,{'exact','limited-memory'});
m=bundle.model; s=buildCylinderBoxTransferScene(m);
s.q0=bundle.refs.q(:,1); s.qWaypoint=s.q0; s.qGoal=s.q0;
s.qd0=zeros(6,1); s.qdGoal=zeros(6,1);
s.phase.durationApproach=.03; s.phase.durationInsertion=.015;
s.phase.numIntervalsApproach=2; s.phase.numIntervalsInsertion=1;
d=buildTwoPhaseIHSDiscretization(s,2,1);
[~,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d);
initial.Xnode=repmat([s.q0;zeros(6,1)],1,d.numNodes);
initial.XmidIHS=repmat([s.q0;zeros(6,1)],1,d.numMidpoints);
initial.Anode=zeros(6,d.numNodes); initial.Amid=zeros(6,d.numMidpoints);
c=makeAbfdConfig(struct('modelMode','residual-B','residualFile',asset, ...
    'staticBalanceFile',calibrationFile,'inertiaCalibrationFile',calibrationFile, ...
    'predictionProfile','actuator-var-v3','temporalFile',temporalFile,'allowPilotTemporal',true, ...
    'plantProfile','high-fidelity','plantOverrides',plain,'controllerMode','geometric-D', ...
    'designMode','codesign-A','plannerProfile','guarded-exact-v1', ...
    'solverFormulation','covariance-shooting','covarianceShootingInterval',.02, ...
    'solverMaxIterations',1,'solverMaxCpuTime',60,'solverHessianApproximation',hessian, ...
    'outputDir',outputDir));
if nargin>7 && ~isempty(assembly), c.solverHessianAssembly=validatestring(assembly,{'direct','compressed'}); end
if nargin>8, c.solverMaxDerivativeDirections=directions; c.solverMaxCpuTime=180; end
if nargin>9, c.solverDerivativeAssembly=derivativeAssembly; end
c=makeAbfdConfig(c);
[p,c]=buildAbfdProviderForTask(m,c,plain,s.q0);
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS, ...
    'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime);
design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
result=solveAbfdIhsid(m,s,p,c,initial,[],design);
save(fullfile(outputDir,'solver_contract.mat'),'result','c');
assert(result.iterations==1 && all(isfinite(result.z)) && strcmp(result.numerics.hessian,hessian) && ...
    result.seedReport.covarianceShooting.maxNormalizedContinuityResidual<1e-9, ...
    'ABFD:GuardedMomentSolver','Expected one finite full actuator/VAR NLP iteration with the explicitly selected Hessian.');
report=struct('passed',true,'solverStatus',result.solverStatus,'iterations',result.iterations, ...
    'preparationSeconds',result.preparationTime,'solveSeconds',result.solveTime, ...
    'trajectoryAccepted',result.passed,'seed',result.seedReport.covarianceShooting,'hessian',hessian, ...
    'scope','Actual full physical NLP with full B/D/actuator/VAR moment continuity; one bounded iteration, no performance acceptance');
if nargin>8, report.maxDerivativeDirections=directions; end
if nargin>9, report.derivativeAssembly=derivativeAssembly; end
end
