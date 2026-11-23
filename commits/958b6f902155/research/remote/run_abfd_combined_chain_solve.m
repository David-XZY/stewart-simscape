function report=run_abfd_combined_chain_solve(reducedDir,derivativeDir,oracleDir,checkpointFile,outputDir,equalityAuditDir)
% Solve the complete original A subproblem through its audited exact reduction.
% Map original multipliers into retained variables and eliminated bound rows.
import casadi.*
if nargin<6, equalityAuditDir=''; end
assert(~isfolder(outputDir),'ABFD:ChainSolveExists','Use a fresh solve directory.');
mkdir(outputDir); timer=tic;
audit=readAbfdJsonUtf8(fullfile(derivativeDir,'status.json'));
assert(strcmp(audit.state,'complete') && audit.passed,'ABFD:ChainSolveAudit','The full derivative audit must pass.');
for k=1:numel(audit.inputFiles)
    assert(strcmp(abfdFileHash(fullfile(reducedDir,audit.inputFiles{k})),audit.inputHashes{k}), ...
        'ABFD:ChainSolveInput','The audited full reduced problem changed.');
end
data=load(fullfile(reducedDir,'problem.mat')); kit=data.kit; mapping=data.mapping; bounds=data.bounds;
directMoments=isfield(data.c,'covarianceCoordinates') && strcmp(data.c.covarianceCoordinates,'symmetric');
if directMoments
    assert(isfield(audit,'assembly') && any(strcmp(audit.assembly,{'direct-reduced-ad-v1','grouped-reduced-ad-v1'})) && ...
        isfield(data,'covarianceLifting'),'ABFD:ChainCovarianceCoordinates', ...
        'Symmetric moments require their own audited full derivatives and conversion metadata.');
end
assert(strcmp(data.c.solverFormulation,'covariance-shooting-hs-reduced') && ...
    strcmp(audit.oracleHash,kit.oracleHash),'ABFD:ChainSolveScope','Expected the original complete reduced covariance problem.');
saved=load(checkpointFile,'checkpoint'); point=saved.checkpoint.iterate;
originalAudit=readAbfdJsonUtf8(fullfile(oracleDir,'status.json'));
if iscell(originalAudit.segments), segments=[originalAudit.segments{:}]; else, segments=originalAudit.segments; end
assert(strcmp(originalAudit.state,'complete') && isequal(saved.checkpoint.bounds,kit.bounds) && ...
    any(strcmp({segments.checkpointHash},abfdFileHash(checkpointFile))), ...
    'ABFD:ChainSolveCheckpoint','Use a complete checkpoint audited against the original problem and bounds.');
originalFile=fullfile(oracleDir,'original_nlp.casadi');
assert(strcmp(abfdFileHash(originalFile),kit.oracleHash),'ABFD:ChainSolveOracle','Original oracle changed.');
oracle=Function.load(originalFile); [fp,gp]=oracle(point.x);
assert(max(abs([full(fp);full(gp)]-[point.f;point.g]))<1e-8, ...
    'ABFD:ChainSolvePoint','Saved original point does not match its audited oracle.');
publish('loading-checked-chain-callbacks');
jacobianFile=fullfile(derivativeDir,'jac_g.casadi'); gradientFile=fullfile(derivativeDir,'grad_f.casadi');
assert(strcmp(abfdFileHash(jacobianFile),audit.jacobianFileHash) && ...
    strcmp(abfdFileHash(gradientFile),audit.gradientFileHash), ...
    'ABFD:ChainSolveDerivative','Audited callbacks changed.');
J=Function.load(jacobianFile); G=Function.load(gradientFile);
assert(J.n_in()==2 && J.n_out()==2 && G.n_in()==2 && G.n_out()==2, ...
    'ABFD:ChainSolveSignature','Expected Ipopt value-and-derivative callback signatures.');
reference=Function.load(fullfile(reducedDir,'values.casadi')); lift=Function.load(fullfile(reducedDir,'lift.casadi'));
x=MX.sym('decision',numel(data.x0),1); [f,g]=reference(x);
[f0,g0]=reference(data.x0); [gj,J0]=J(data.x0,[]); [fg,gf0]=G(data.x0,[]);
assert(isequal(size(J0),[numel(g),numel(x)]) && isequal(size(gf0),[numel(x),1]) && ...
    max(abs([full(f0-fg);full(g0-gj)]))<1e-8,'ABFD:ChainSolveCallbacks','Callback values or dimensions changed.');
eqCount=sum(kit.bounds.lbg==kit.bounds.ubg);
retainedRows=[mapping.retainedEquality;eqCount+(1:mapping.originalInequalityCount).'];
lower=mapping.lowerBoundIndices; upper=mapping.upperBoundIndices;
lamG=[point.lam_g(retainedRows);max(0,-point.lam_x(lower))./kit.scale(lower); ...
    max(0,point.lam_x(upper))./kit.scale(upper)];
lamX=point.lam_x(mapping.keepIndices).*data.scale./kit.scale(mapping.keepIndices);
momentBoundRestart=struct('enabled',false);
if directMoments
    momentRows=find(mapping.keepIndices>kit.physicalNumZ);
    momentBoundRestart=struct('enabled',true,'coordinates',numel(momentRows), ...
        'oldBoundMultiplierInfinity',norm(lamX(momentRows),inf), ...
        'scope','Original factor-diagonal bounds have different interior derivatives; reset their warm-start multipliers to zero while retaining every original constraint multiplier');
    lamX(momentRows)=0;
end
assert(numel(lamG)==numel(g) && numel(lamX)==numel(x) && all(isfinite([lamG;lamX])), ...
    'ABFD:ChainSolveMultipliers','Invalid original-to-reduced multiplier layout.');
initialStationarity=norm(full(gf0)+sparse(J0).'*lamG+lamX,inf);
solverRows=(1:numel(g)).'; solverBounds=bounds; solverG=g; solverJ=J;
equalityProjection=struct('enabled',false);
if ~isempty(equalityAuditDir)
    publish('verifying-independent-equality-representation');
    projection=verifyAbfdEqualityProjection(oracle,kit,point,equalityAuditDir,abfdFileHash(checkpointFile));
    removed=find(ismember(mapping.retainedEquality,projection.dependentOriginalRows));
    assert(~isempty(removed),'ABFD:EqualityProjectionEmpty','No certified rows remain in this reduced formulation.');
    solverRows=setdiff(solverRows,removed); solverG=g(solverRows);
    mapped=[projection.mappedMultipliers(retainedRows);lamG(numel(retainedRows)+1:end)];
    oldGradient=sparse(J0).'*lamG;
    lamG=mapped(solverRows);
    mappedGradient=sparse(J0(solverRows,:)).'*lamG;
    difference=norm(oldGradient-mappedGradient,inf);
    assert(difference<1e-8,'ABFD:ReducedEqualityMultipliers','Projected reduced multipliers changed the complete Lagrangian gradient.');
    solverBounds.lbg=bounds.lbg(solverRows); solverBounds.ubg=bounds.ubg(solverRows);
    parameter=MX.sym('p',0,1); [callbackG,callbackJ]=J(x,parameter);
    solverJ=Function('abfd_independent_equality_jac_g',{x,parameter}, ...
        {callbackG(solverRows),callbackJ(solverRows,:)},{'x','p'},{'g','jac_g'});
    equalityProjection=projection.report; equalityProjection.removedReducedRows=removed;
    equalityProjection.reducedMultiplierGradientDifference=difference;
end
initialPhysical=full(lift(data.x0));
if directMoments
    initialPhysical=convertAbfdSampledCovarianceCoordinates(data.covarianceLifting,kit.physicalNumZ,initialPhysical,'cholesky');
end
initialOriginal=initialPhysical./kit.scale;
initialCorrection=norm(initialOriginal-point.x,inf);
clear J0 gf0 gj fg
options=kit.options; options.ipopt.hessian_approximation='limited-memory';
options.oracle_options=struct('max_num_dir',1);
options.ipopt.mu_strategy='monotone'; options.ipopt.mu_init=1e-10;
options.ipopt.max_iter=300; options.ipopt.max_cpu_time=1800; options.ipopt.print_level=5;
options.ipopt.warm_start_init_point='yes'; options.ipopt.alpha_for_y='primal';
for name={'bound_push','bound_frac','slack_bound_push','slack_bound_frac','mult_bound_push'}
    options.ipopt.(['warm_start_',name{1}])=1e-9;
end
assert(options.ipopt.tol==kit.c.solverTolerance && options.ipopt.constr_viol_tol==kit.c.solverTolerance && ...
    options.ipopt.acceptable_iter==0,'ABFD:ChainSolveTolerance','Keep the original strict termination conditions.');
manifest=struct('oracleHash',kit.oracleHash,'checkpointHash',abfdFileHash(checkpointFile), ...
    'derivativeAuditHash',abfdFileHash(fullfile(derivativeDir,'status.json')), ...
    'implementationHash',abfdFileHash([mfilename('fullpath'),'.m']),'options',options, ...
    'variables',numel(x),'constraints',numel(solverG),'completeReducedConstraints',numel(g), ...
    'equalityProjection',equalityProjection,'initialStationarityInfinity',initialStationarity, ...
    'symmetricCovarianceCoordinates',directMoments,'momentBoundRestart',momentBoundRestart, ...
    'initialScaledCorrection',initialCorrection,'derivative',audit, ...
    'scope','Same full frozen-gain A feasible set and objective after exact HS/force elimination; all covariance cuts, inequalities and bounds retained; mapped primal and dual starts are not accepted solutions');
writeAbfdJson(fullfile(outputDir,'manifest.json'),manifest);
options.jac_g=solverJ; options.grad_f=G;
publish('building-solver'); buildTimer=tic;
solver=nlpsol('abfd_full_chain_solve','ipopt',struct('x',x,'f',f,'g',solverG),options); buildSeconds=toc(buildTimer);
publish('solving'); solveTimer=tic;
sol=solver('x0',data.x0,'lam_x0',lamX,'lam_g0',lamG, ...
    'lbx',solverBounds.lbx,'ubx',solverBounds.ubx,'lbg',solverBounds.lbg,'ubg',solverBounds.ubg);
solveSeconds=toc(solveTimer); stats=solver.stats();
iterate=struct('x',full(sol.x),'f',full(sol.f),'g',full(sol.g),'lam_x',full(sol.lam_x),'lam_g',full(sol.lam_g));
assert(all(isfinite([iterate.x;iterate.f;iterate.g;iterate.lam_x;iterate.lam_g])), ...
    'ABFD:ChainSolveNonfinite','The complete solve returned nonfinite values.');
z=full(lift(iterate.x)); originalPhysical=z; coordinateConversion=[];
if directMoments
    % Preserve the native iterate even if an intermediate moment matrix is
    % indefinite. Such a point cannot be accepted as an original factor NLP.
    save(fullfile(outputDir,'native_moment_result.mat'),'iterate','stats','manifest','bounds','solverBounds','solverRows','z');
    [originalPhysical,coordinateConversion]=convertAbfdSampledCovarianceCoordinates( ...
        data.covarianceLifting,kit.physicalNumZ,z,'cholesky');
end
originalDecision=originalPhysical./kit.scale;
save(fullfile(outputDir,'solver_result.mat'),'iterate','stats','kit','manifest','bounds','solverBounds','solverRows','originalDecision','z');
publish('checking-original-constraints');
[fOriginal,gOriginal]=oracle(originalDecision); gOriginal=full(gOriginal);
eq=kit.bounds.lbg==kit.bounds.ubg;
maxEq=max(abs(gOriginal(eq)-kit.bounds.lbg(eq)));
maxIneq=max([0;kit.bounds.lbg(~eq)-gOriginal(~eq);gOriginal(~eq)-kit.bounds.ubg(~eq)]);
maxBound=max([0;kit.bounds.lbx-originalDecision;originalDecision-kit.bounds.ubx]);
[~,completeG]=reference(iterate.x); completeG=full(completeG);
reducedViolation=max([0;bounds.lbg-completeG;completeG-bounds.ubg;bounds.lbx-iterate.x;iterate.x-bounds.ubx]);
[~,Jfinal]=J(iterate.x,[]); [~,gfFinal]=G(iterate.x,[]);
completeMultipliers=zeros(numel(g),1); completeMultipliers(solverRows)=iterate.lam_g;
stationarity=norm(full(gfFinal)+sparse(Jfinal).'*completeMultipliers+iterate.lam_x,inf);
assert(abs(full(fOriginal)-iterate.f)<1e-8,'ABFD:ChainSolveObjective','Final reduced and original objectives disagree.');
report=struct('state','complete','solverStatus',stats.return_status,'solverSuccess',logical(stats.success), ...
    'iterations',stats.iter_count,'buildSeconds',buildSeconds,'solveSeconds',solveSeconds,'objective',iterate.f, ...
    'originalMaxEq',maxEq,'originalMaxIneq',maxIneq,'originalMaxBound',maxBound, ...
    'reducedMaxViolation',reducedViolation,'stationarityInfinity',stationarity, ...
    'equalityProjection',equalityProjection,'physicalAccepted',false,'finalGainAccepted',false,'seconds',toc(timer), ...
    'symmetricCovarianceCoordinates',directMoments,'coordinateConversion',coordinateConversion, ...
    'scope','Full original frozen-gain A subproblem; trajectory/gain outer-loop and Simscape acceptance remain separate');
requiredResidual=kit.c.constraintTolerance;
if equalityProjection.enabled || directMoments, requiredResidual=kit.c.solverTolerance; end
report.requiredOriginalResidual=requiredResidual;
report.originalConstraintsPassed=max([maxEq,maxIneq,maxBound,reducedViolation])<=requiredResidual;
if stats.success && strcmp(stats.return_status,'Solve_Succeeded') && report.originalConstraintsPassed
    publish('validating-trajectory-and-new-gains');
    seed=kit.seed; scene=seed.bundle.scene; model=seed.bundle.model; disc=seed.disc;
    [X,Xm,An,Am,Fn,Fm]=unpackIHSDecisionImplicit(z(1:kit.baseNumZ),scene,disc);
    traj=struct('t',disc.tNode,'tc',disc.tMid,'Xnode',X,'Xmid',Xm,'Anode',An,'Amid',Am,'Unode',Fn,'Umid',Fm);
    provider=buildDynamicsProvider(model); refs=reconstructIhsidContinuousReference(traj,provider,kit.c.sampleTime);
    dense=validateAbfdReference(refs,model,scene,kit.c); report.dense=dense; report.physicalAccepted=dense.passed;
    if dense.passed
        bundle=buildAbfdReferenceBundle(traj,model,scene,disc,provider,kit.c);
        feasibility=evaluateAbfdCodesignFeasibility(bundle,provider,kit.c);
        report.finalGainAccepted=feasibility.passed; report.finalGainFeasibility=feasibility;
        save(fullfile(outputDir,'candidate_bundle.mat'),'bundle','feasibility','traj','z');
    end
end
report.seconds=toc(timer); writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
