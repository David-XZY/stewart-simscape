function report=run_abfd_shooting_hessian_probe(checkpointFile,outputDirectory,cutInterval,assembly)
% Full-horizon derivative experiment only; no trajectory is accepted here.
% checkpointFile is the frozen physical seed from the original A solve.
import casadi.*
if nargin<3, cutInterval=.1; end
if nargin<4, assembly='direct'; end
assembly=validatestring(assembly,{'direct','compressed'});
validateattributes(cutInterval,{'double'},{'scalar','positive','finite'});
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
timer=tic;
try
    loaded=load(checkpointFile,'physicalSeed','c');
    seed=loaded.physicalSeed; model=seed.bundle.model; scene=seed.bundle.scene;
    disc=seed.disc; initial=seed.initial; provider=buildDynamicsProvider(model);
    c=loaded.c; c.outputDir=outputDirectory; c.trustRadius=.1;
    c.solverFormulation='covariance-shooting'; c.covarianceShootingInterval=cutInterval;
    assert(strcmp(c.predictionProfile,'sampled-hold-v2'), ...
        'ABFD:HessianProbeProfile','This diagnostic requires the original sampled-hold-v2 seed.');
    design=buildAbfdSchedule(seed.bundle.refs,provider,c);
    [time,order]=sort([disc.tNode,disc.tMid]);
    q=[initial.Xnode(1:6,:),initial.XmidIHS(1:6,:)];
    design.nominal=struct('q',q(:,order),'t',time);
    solverScene=scene;
    solverScene.collision.smoothingEps=max(1e-3,scene.collision.smoothingEps);
    [initial.separator,~]=buildAbfdSeparators(initial.Xnode(1:6,:), ...
        initial.XmidIHS(1:6,:),solverScene,disc);
    endpoints=[scene.q0,scene.qWaypoint,scene.qGoal]; slack=inf(6,1);
    for k=1:size(endpoints,2)
        kin=sgpIK(endpoints(:,k),model);
        slack=min(slack,min(kin.L-model.lmin,model.lmax-kin.L));
    end
    research=struct('provider',provider,'config',c,'design',design, ...
        'center',seed.z,'buildOnly',true,'enforceInsertionDerivatives',true, ...
        'additionalObstacleGuard',.0005,'lengthGuard',max(0,min(.00025,.5*slack)));
    publish('building-nlp'); buildTimer=tic;
    nlp=buildCasadiTrackableIHSNLP(model,solverScene,disc,initial, ...
        struct('hessianApproximation','exact','maxIter',1500),research);
    nlpBuildSeconds=toc(buildTimer);
    [z,seedReport]=buildAbfdCovarianceShootingSeed(nlp,seed.z);
    [cost,equality,inequality]=nlp.eval(z);
    seedValues=struct('objective',full(cost),'maxEqualityResidual',max(abs(full(equality))), ...
        'maxInequalityViolation',max([0;full(inequality)]),'covariance',seedReport);
    writeAbfdJson(fullfile(outputDirectory,'initial.json'),seedValues);

    % Reproduce the guarded solver's variable scaling. Multipliers stay
    % symbolic so this exercises the general Lagrangian, not only the cost.
    scale=ones(size(z)); finite=isfinite(nlp.lbz) & isfinite(nlp.ubz);
    scale(finite)=(nlp.ubz(finite)-nlp.lbz(finite))/2;
    assert(all(isfinite(scale) & scale>0),'ABFD:HessianProbeScaling','Invalid decision scaling.');
    x=MX.sym('scaled_decision',numel(z),1);
    expressions=substitute([nlp.J;nlp.gEq;nlp.cIneq],nlp.z,scale.*x);
    lambda=MX.sym('constraint_multipliers',numel(expressions)-1,1);
    sigma=MX.sym('objective_multiplier',1,1);
    lagrangian=sigma*expressions(1)+dot(lambda,expressions(2:end));
    publish('building-lagrangian-hessian'); buildTimer=tic;
    compressedReport=[];
    if strcmp(assembly,'compressed')
        [hessianFunction,compressedReport,gradientFunction,H]= ...
            buildAbfdCompressedHessian(x,lagrangian,sigma,lambda,@publish);
    else
        [H,gradient]=hessian(lagrangian,x);
        hessianFunction=Function('abfd_full_shooting_hessian',{x,sigma,lambda},{H});
        gradientFunction=Function('abfd_full_shooting_gradient',{x,sigma,lambda},{gradient});
    end
    hessianBuildSeconds=toc(buildTimer);
    shape=struct('variables',numel(z),'physicalVariables',nlp.physicalNumZ, ...
        'constraints',numel(lambda),'hessianStructuralNnz',H.nnz(), ...
        'covarianceCutInterval',cutInterval,'covarianceCuts',numel(nlp.tube.lifting.cutTimes), ...
        'assembly',assembly,'compressed',compressedReport, ...
        'nlpBuildSeconds',nlpBuildSeconds,'hessianBuildSeconds',hessianBuildSeconds);
    writeAbfdJson(fullfile(outputDirectory,'structure.json'),shape);
    publish('evaluating-lagrangian-hessian'); evaluateTimer=tic;
    x0=z./scale; lambda0=.01*sin((1:numel(lambda)).'); sigma0=1;
    value=sparse(hessianFunction(x0,sigma0,lambda0));
    hessianEvaluationSeconds=toc(evaluateTimer);
    assert(all(isfinite(nonzeros(value))),'ABFD:HessianProbeFinite','Nonfinite full Hessian.');
    symmetryError=norm(value-value.','fro')/max(1,norm(value,'fro'));
    assert(symmetryError<1e-10,'ABFD:HessianProbeSymmetry','Full Hessian is not symmetric.');
    publish('checking-hessian-directions');
    steps=[1e-5,1e-6,1e-7]; errors=zeros(2,numel(steps));
    for directionIndex=1:2
        direction=sin((1:numel(z)).'*sqrt(directionIndex+1));
        direction=direction/norm(direction); exact=value*direction;
        for stepIndex=1:numel(steps)
            h=steps(stepIndex);
            plus=full(gradientFunction(x0+h*direction,sigma0,lambda0));
            minus=full(gradientFunction(x0-h*direction,sigma0,lambda0));
            difference=(plus-minus)/(2*h);
            errors(directionIndex,stepIndex)=norm(exact-difference)/max(1,norm(exact));
        end
    end
    % Keep the full convergence curves; a single finite-difference step
    % cannot establish a nonsmooth/roundoff-sensitive derivative contract.
    passed=all(min(errors,[],2)<1e-4) && all(isfinite(errors(:)));
    report=struct('state','complete','passed',passed,'structure',shape, ...
        'hessianEvaluationSeconds',hessianEvaluationSeconds,'symmetryError',symmetryError, ...
        'differenceSteps',steps,'directionalRelativeErrors',errors, ...
        'checkpointHash',abfdFileHash(checkpointFile),'implementationHash',abfdPlannerImplementationHash(), ...
        'totalSeconds',toc(timer),'seed',seedValues, ...
        'scope','Full original horizon and constraints, sampled-hold-v2 covariance shooting; derivative feasibility only, no A refinement or Simscape acceptance');
    save(fullfile(outputDirectory,'hessian_diagnostics.mat'), ...
        'value','scale','x0','lambda0','sigma0','report','-v7.3');
    writeAbfdJson(fullfile(outputDirectory,'status.json'),report);
    assert(passed,'ABFD:HessianProbeDerivative','Full Hessian directional derivative check failed.');
catch exception
    failure=struct('state','failed','identifier',exception.identifier, ...
        'message',exception.message,'seconds',toc(timer), ...
        'report',getReport(exception,'extended','hyperlinks','off'));
    writeAbfdJson(fullfile(outputDirectory,'failure.json'),failure);
    rethrow(exception);
end
    function publish(stage)
        writeAbfdJson(fullfile(outputDirectory,'status.json'), ...
            struct('state','running','stage',stage,'elapsedSeconds',toc(timer)));
    end
end
