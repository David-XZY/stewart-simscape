function result=solveAbfdIhsid(model,scene,provider,c,initial,center,design,primalStart)
if nargin<5, initial=[]; end
if nargin<6, center=[]; end
if nargin<7, design=[]; end
if nargin<8, primalStart=[]; end
timer=tic; profile='legacy';
if isfield(c,'plannerProfile'), profile=c.plannerProfile; end
guards=.0005;
if strcmp(profile,'guarded-adaptive-v2'), guards=[.0005,.0025,.005]; end
history=struct('guard',{},'passed',{},'reason',{},'seconds',{},'solveTime',{},'iterations',{},'objective',{});
lastFeasible=[]; solveSeconds=0; prepareSeconds=0; iterations=0;
for attempt=1:numel(guards)
    local=c;
    if numel(guards)>1, local.outputDir=fullfile(c.outputDir,sprintf('guard_%02d',attempt)); end
    result=solveAttempt(model,scene,provider,local,initial,center,design,guards(attempt),attempt>1,primalStart);
    result.failureReason=abfdPlannerFailureReason(result);
    history(end+1)=struct('guard',guards(attempt),'passed',result.passed, ...
        'reason',result.failureReason,'seconds',result.totalTime,'solveTime',result.solveTime, ...
        'iterations',result.iterations,'objective',result.objective); %#ok<AGROW>
    solveSeconds=solveSeconds+result.solveTime; prepareSeconds=prepareSeconds+result.preparationTime;
    iterations=iterations+result.iterations;
    if ~isempty(result.lastFeasible), lastFeasible=result.lastFeasible; end
    if numel(guards)>1
        if ~isfolder(local.outputDir), mkdir(local.outputDir); end
        save(fullfile(local.outputDir,'attempt.mat'),'result');
        writeAbfdJson(fullfile(c.outputDir,'guard_attempts.json'),history);
    end
    % Only an independently identified extra-obstacle clearance failure
    % triggers a tighter solve. Failed solver status is never called success.
    if result.passed || ~isfield(result,'denseReport') || ...
            result.denseReport.maxAdditionalObstacleViolation<=c.constraintTolerance
        break;
    end
    initial=result.initial;
    primalStart=[];
end
result.lastFeasible=lastFeasible; result.retryHistory=history; result.retryCount=numel(history)-1;
result.finalSolveTime=result.solveTime; result.finalIterations=result.iterations;
result.solveTime=solveSeconds; result.preparationTime=prepareSeconds; result.iterations=iterations;
result.totalTime=toc(timer);
end
function result=solveAttempt(model,scene,provider,c,initial,center,design,obstacleGuard,isRetry,primalStart)
import casadi.*
totalTimer=tic; profile='legacy';
if isfield(c,'plannerProfile'), profile=validatestring(c.plannerProfile,{'legacy','guarded-exact-v1','guarded-adaptive-v2'}); end
guarded=any(strcmp(profile,{'guarded-exact-v1','guarded-adaptive-v2'}));
disc=buildTwoPhaseIHSDiscretization(scene,scene.phase.numIntervalsApproach,scene.phase.numIntervalsInsertion);
seedReport=struct('method','supplied HS initial');
if isempty(initial)
    if guarded
        [cp,seedReport]=buildObstacleAwareSplineSeed(struct('model',model,'scene',scene),1);
        assert(seedReport.passed,'ABFD:GeometricSeedRejected', ...
            'Geometric seed rejected: %s, violation=%g.',seedReport.status,seedReport.maxViolation);
        [~,initial]=buildLearnedInitialGuessIHSID(cp,model,scene,disc,provider);
    else
        [~,initial]=buildInitialGuessTwoPhaseIHSImplicit(model,scene,disc);
        seedReport.method='nominal initial';
    end
end
if guarded && isempty(center) && ~isRetry
    [initial,~,projection]=projectAbfdHsSeed(initial,scene,disc,provider);
    seedReport.projection=projection;
end
% All force guesses use the selected provider, even for a sequential B solve.
for k=1:disc.numNodes, initial.Fnode(:,k)=provider.inverse(initial.Xnode(1:6,k),initial.Xnode(7:12,k),initial.Anode(:,k)); end
for k=1:disc.numMidpoints, initial.Fmid(:,k)=provider.inverse(initial.XmidIHS(1:6,k),initial.XmidIHS(7:12,k),initial.Amid(:,k)); end
solverScene=scene;
if guarded
    solverScene.collision.smoothingEps=max(1e-3,scene.collision.smoothingEps);
    [initial.separator,extra]=buildAbfdSeparators(initial.Xnode(1:6,:),initial.XmidIHS(1:6,:),solverScene,disc);
else
    [~,extra]=buildAbfdSeparators(initial.Xnode(1:6,:),initial.XmidIHS(1:6,:),scene,disc);
end
z0=packIHSDecisionImplicit(initial.Xnode(:,2:end-1),initial.XmidIHS, ...
    initial.Anode,initial.Amid,initial.Fnode,initial.Fmid,disc,initial.separator);
z0=[z0;extra(:)];
research=struct('provider',provider,'config',c,'center',center,'design',design);
useReduction=isfield(c,'solverFormulation') && ...
    any(strcmp(c.solverFormulation,{'hs-reduced','covariance-shooting-hs-reduced'}));
if useReduction
    assert(guarded,'ABFD:ReducedSolverProfile'); research.prepareReduction=true;
end
opts=struct('solverBackend','ipopt','hessianApproximation','limited-memory', ...
    'maxIter',c.solverMaxIterations);
if guarded
    research.enforceInsertionDerivatives=true;
    research.additionalObstacleGuard=obstacleGuard;
    endpoints=[scene.q0,scene.qWaypoint,scene.qGoal]; slack=inf(6,1);
    for k=1:size(endpoints,2)
        kin=sgpIK(endpoints(:,k),model);
        slack=min(slack,min(kin.L-model.lmin,model.lmax-kin.L));
    end
    research.lengthGuard=max(0,min(.00025,.5*slack));
    research.buildOnly=true; opts.hessianApproximation='exact';
    % A differentiates a long feedback/covariance rollout. Keep its complete
    % first derivatives; build a limited-memory Hessian inside IPOPT.
    if strcmp(c.designMode,'codesign-A'), opts.hessianApproximation='limited-memory'; end
    if isempty(center)
        research.center=z0; research.config.trustRadius=min(.05,c.trustRadius);
    end
end
if isfield(c,'solverHessianApproximation'), opts.hessianApproximation=c.solverHessianApproximation; end
continuationCount=0;
if isfield(c,'solverContinuationCount'), continuationCount=c.solverContinuationCount; end
publishProgress(c,'building-nlp',opts.hessianApproximation,toc(totalTimer));
nlp=buildCasadiTrackableIHSNLP(model,solverScene,disc,initial,opts,research);
if ~isempty(center), z0=center; end
if ~isempty(primalStart)
    validateattributes(primalStart,{'double'},{'size',[nlp.physicalNumZ,1],'real','finite'});
    assert(all(primalStart>=nlp.lbz(1:nlp.physicalNumZ)-1e-9) && ...
        all(primalStart<=nlp.ubz(1:nlp.physicalNumZ)+1e-9), ...
        'ABFD:PrimalStartBounds','Explicit solver iterate violates the current decision bounds; it was not clipped.');
    z0=primalStart;
    seedReport.primalStart=struct('hash',abfdHash(primalStart), ...
        'scope','Explicit numerical starting point only; trust-region center and accepted trajectory unchanged');
end
useShooting=isfield(nlp.tube,'lifting');
if useShooting
    [z0,shootingSeed]=buildAbfdCovarianceShootingSeed(nlp,z0); seedReport.covarianceShooting=shootingSeed;
    writeAbfdJson(fullfile(c.outputDir,'covariance_shooting.json'),shootingSeed);
end
solverData=nlp; solverInitial=z0; reduction=[];
if useReduction
    reductionMode='global-qr';
    if isfield(c,'solverReductionMode'), reductionMode=c.solverReductionMode; end
    reduction=buildAbfdReducedHsNlp(nlp,disc,z0,reductionMode); solverData=reduction; solverInitial=reduction.x0;
    z0=full(reduction.lift(solverInitial)); seedReport.reduction=reduction.report;
    writeAbfdJson(fullfile(c.outputDir,'hs_reduction.json'),reduction.report);
end
scale=ones(size(solverInitial)); solver=nlp.solver; hessianAssembly=[]; derivativeAssembly=[];
if guarded
    finite=isfinite(solverData.lbz) & isfinite(solverData.ubz);
    scale(finite)=(solverData.ubz(finite)-solverData.lbz(finite))/2;
    assert(all(isfinite(scale) & scale>0),'ABFD:InvalidScaling','Nonpositive decision range.');
    y=MX.sym('scaled_decision',numel(solverInitial),1);
    if useReduction
        expressions=substitute([reduction.nlp.f;reduction.nlp.g],reduction.nlp.x,scale.*y);
        objective=expressions(1); constraint=expressions(2:end);
    elseif useShooting
        expressions=substitute([nlp.J;nlp.gEq;nlp.cIneq],nlp.z,scale.*y);
        objective=expressions(1); constraint=expressions(2:end);
    else
        [objective,equality,inequality]=solverData.eval(scale.*y); constraint=[equality;inequality];
    end
    options=nlp.opts; options.ipopt.mu_strategy='monotone'; options.ipopt.mu_init=1e-6;
    if continuationCount>0
        options.ipopt.warm_start_init_point='yes';
        options.ipopt.warm_start_bound_push=1e-9;
        options.ipopt.warm_start_bound_frac=1e-9;
        options.ipopt.warm_start_slack_bound_push=1e-9;
        options.ipopt.warm_start_slack_bound_frac=1e-9;
        options.ipopt.warm_start_mult_bound_push=1e-9;
    end
    if isfield(c,'solverHessianAssembly') && strcmp(c.solverHessianAssembly,'compressed')
        assert(strcmp(opts.hessianApproximation,'exact'),'ABFD:HessianAssemblyConfig');
        sigmaH=MX.sym('lam_f'); lambdaH=MX.sym('lam_g',numel(constraint),1);
        parameterH=MX.sym('p',0,1);
        progress=@(stage)publishProgress(c,stage,opts.hessianApproximation,toc(totalTimer));
        [~,hessianAssembly,~,H]=buildAbfdCompressedHessian(y, ...
            sigmaH*objective+dot(lambdaH,constraint),sigmaH,lambdaH,progress);
        options.hess_lag=Function('abfd_guarded_compressed_hessian', ...
            {y,parameterH,sigmaH,lambdaH},{triu(H)}, ...
            {'x','p','lam_f','lam_g'},{'hess_gamma_x_x'});
    end
    if isfield(c,'solverDerivativeAssembly') && strcmp(c.solverDerivativeAssembly,'grouped-moments')
        assert(useShooting && isfield(nlp.tube.lifting,'kind') && ...
            any(strcmp(nlp.tube.lifting.kind,{'actuator-moment-shooting-v1','sampled-symmetric-shooting-v1'})), ...
            'ABFD:GroupedDerivativeLayout','Expected complete explicit mean and covariance coordinates.');
        lifting=nlp.tube.lifting; roots=[];
        if strcmp(lifting.kind,'actuator-moment-shooting-v1')
            assert(~useReduction,'ABFD:GroupedActuatorReduction','Actuator moments require their full mean layout.');
            meanIds=nlp.physicalNumZ+(0:numel(lifting.cutTimes)-1)*lifting.factorSize+(1:30).';
            meanIds=[(1:nlp.physicalNumZ).';meanIds(:)];
            roots=nlp.tube.rootCurvature;
            roots.arguments=substitute(roots.arguments,nlp.z,scale.*y);
            roots.affineConstraints=substitute(roots.affineConstraints,nlp.z,scale.*y);
        elseif useReduction
            meanIds=find(reduction.keepIndices<=nlp.physicalNumZ);
        else
            meanIds=(1:nlp.physicalNumZ).';
        end
        progress=@(stage)publishProgress(c,stage,opts.hessianApproximation,toc(totalTimer));
        directions=1;
        if isfield(c,'solverMaxDerivativeDirections'), directions=c.solverMaxDerivativeDirections; end
        [derivatives,derivativeAssembly]=buildAbfdGroupedMomentDerivatives( ...
            y,objective,constraint,meanIds,strcmp(opts.hessianApproximation,'exact'),progress,directions,roots);
        options.jac_g=derivatives.jacobian;
        if ~isempty(derivatives.hessian), options.hess_lag=derivatives.hessian; end
        writeAbfdJson(fullfile(c.outputDir,'derivative_assembly.json'),derivativeAssembly);
    end
    publishProgress(c,'building-solver',opts.hessianApproximation,toc(totalTimer));
    solver=nlpsol('abfd_guarded_ihsid_solver','ipopt', ...
        struct('x',y,'f',objective,'g',constraint),options);
end
lastFeasible=[];
if guarded
    publishProgress(c,'validating-initial',opts.hessianApproximation,toc(totalTimer));
    % Preserve a feasible seed without treating it as successful refinement.
    [initialCost,initialEq,initialIneq]=nlp.eval(z0);
    seedReport.objective=full(initialCost);
    seedReport.maxEq=max(abs(full(initialEq))); seedReport.maxIneq=max([0;full(initialIneq)]);
    raw=trajectory(z0,nlp,scene,disc);
    refs=reconstructIhsidContinuousReference(raw,provider,c.sampleTime);
    seedReport.dense=validateAbfdReference(refs,model,scene,c);
    if seedReport.dense.passed, lastFeasible=buildAbfdReferenceBundle(raw,model,scene,disc,provider,c); end
end
preparationSeconds=toc(totalTimer); timer=tic;
publishProgress(c,'solving',opts.hessianApproximation,preparationSeconds);
try
    bounds=struct('lbx',solverData.lbz./scale,'ubx',solverData.ubz./scale, ...
        'lbg',solverData.lbg,'ubg',solverData.ubg);
    segmentDirectory='';
    if strcmp(c.designMode,'codesign-A') || continuationCount>0
        segmentDirectory=fullfile(c.outputDir,'solver_segments');
    end
    [sol,stats,solverContinuation]=solveAbfdNlpWithContinuation( ...
        solver,bounds,solverInitial./scale,continuationCount,segmentDirectory);
catch exception
    if ~isfolder(c.outputDir), mkdir(c.outputDir); end
    failure=struct('identifier',exception.identifier,'message',exception.message);
    save(fullfile(c.outputDir,'solver_failure_checkpoint.mat'),'z0','initial','lastFeasible','seedReport','failure','c');
    rethrow(exception);
end
elapsed=toc(timer); z=full(sol.x).*scale;
if useReduction, z=full(reduction.lift(z)); end
coordinateCheck=[];
if useShooting && isfield(nlp.tube.lifting,'kind') && ...
        strcmp(nlp.tube.lifting.kind,'sampled-symmetric-shooting-v1') && stats.success
    [~,coordinateCheck]=convertAbfdSampledCovarianceCoordinates(nlp.tube.lifting,nlp.physicalNumZ,z,'cholesky');
end
publishProgress(c,'validating-solution',opts.hessianApproximation,toc(totalTimer));
[objective,eq,ineq]=nlp.eval(z); eq=full(eq); ineq=full(ineq);
[traj,separator]=trajectory(z,nlp,scene,disc);
auxiliary=[];
if useShooting, auxiliary=z(nlp.physicalNumZ+1:end); z=z(1:nlp.physicalNumZ); end
result=struct('z',z,'traj',traj,'disc',disc,'initial',initial,'objective',full(objective), ...
    'solveTime',elapsed,'solverStatus',stats.return_status,'iterations',solverContinuation.iterations, ...
    'maxEq',max(abs(eq)),'maxIneq',max([0;ineq]),'passed',false,'bundle',[], ...
    'preparationTime',preparationSeconds,'plannerProfile',profile, ...
    'implementationHash',abfdPlannerImplementationHash(),'seedReport',seedReport,'lastFeasible',lastFeasible);
result.solverContinuation=solverContinuation;
result.solverState=struct('scaledDecision',full(sol.x),'decisionScale',scale, ...
    'scaledBoundMultipliers',full(sol.lam_x),'constraintMultipliers',full(sol.lam_g),'stats',stats);
if guarded
    result.numerics=struct('collisionSmoothing',solverScene.collision.smoothingEps, ...
        'lengthGuard',research.lengthGuard,'additionalObstacleGuard',research.additionalObstacleGuard, ...
        'trustRadius',research.config.trustRadius,'scaledVariables',sum(scale~=1), ...
        'barrier','monotone','hessian',opts.hessianApproximation);
    if useReduction, result.numerics.reduction=reduction.report; end
    if ~isempty(hessianAssembly), result.numerics.hessianAssembly=hessianAssembly; end
    if ~isempty(derivativeAssembly), result.numerics.derivativeAssembly=derivativeAssembly; end
    if useShooting
        result.numerics.covarianceShooting=shootingSeed; result.auxiliaryDecision=auxiliary;
        if ~isempty(coordinateCheck), result.numerics.covarianceCoordinates=coordinateCheck; end
    end
end
result.initial.Xnode=traj.Xnode; result.initial.XmidIHS=traj.Xmid;
result.initial.Anode=traj.Anode; result.initial.Amid=traj.Amid;
result.initial.Fnode=traj.Unode; result.initial.Fmid=traj.Umid; result.initial.separator=separator;
if stats.success && result.maxEq<=c.constraintTolerance && result.maxIneq<=c.constraintTolerance
    refs=reconstructIhsidContinuousReference(traj,provider,c.sampleTime);
    report=validateAbfdReference(refs,model,scene,c); result.denseReport=report;
    if report.passed
        result.bundle=buildAbfdReferenceBundle(traj,model,scene,disc,provider,c); result.passed=true;
    end
end
result.totalTime=toc(totalTimer);
publishProgress(c,'complete',opts.hessianApproximation,result.totalTime);
end
function publishProgress(c,stage,hessian,seconds)
if strcmp(c.designMode,'codesign-A')
    if ~isfolder(c.outputDir), mkdir(c.outputDir); end
    writeAbfdJson(fullfile(c.outputDir,'solver_progress.json'), ...
        struct('stage',stage,'hessian',hessian,'elapsedSeconds',seconds,'configHash',abfdHash(c)));
end
end
function [traj,separator]=trajectory(z,nlp,scene,disc)
[X,Xm,An,Am,Fn,Fm,~,separator]=unpackIHSDecisionImplicit(z(1:nlp.baseNumZ),scene,disc);
traj=struct('t',disc.tNode,'tc',disc.tMid,'Xnode',X,'Xmid',Xm,'Anode',An,'Amid',Am,'Unode',Fn,'Umid',Fm);
end
