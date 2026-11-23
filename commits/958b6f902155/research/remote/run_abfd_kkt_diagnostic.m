function report=run_abfd_kkt_diagnostic(seedFile,attemptFile,segmentFiles,outputDir,exportOracle)
% Reconstruct the frozen sampled A NLP and audit actual primal/dual iterates.
% No solve, multiplier replacement, tolerance change or trajectory acceptance.
import casadi.*
if nargin<5, exportOracle=false; end
assert(~isfolder(outputDir),'ABFD:DiagnosticExists','Use a fresh diagnostic directory.');
mkdir(outputDir); timer=tic;
source=load(seedFile,'physicalSeed','c'); loaded=load(attemptFile,'result');
seed=source.physicalSeed; attempt=loaded.result; c=source.c; c.outputDir=outputDir;
assert(strcmp(c.predictionProfile,'sampled-hold-v2') && strcmp(c.modelMode,'analytic'), ...
    'ABFD:KktScope','This audit reconstructs the original analytic sampled A solve.');
c.trustRadius=attempt.numerics.trustRadius;
model=seed.bundle.model; scene=seed.bundle.scene; disc=seed.disc; initial=seed.initial;
p=buildDynamicsProvider(model); design=buildAbfdSchedule(seed.bundle.refs,p,c);
[times,order]=sort([seed.traj.t,seed.traj.tc]);
q=[seed.traj.Xnode(1:6,:),seed.traj.Xmid(1:6,:)];
design.nominal=struct('q',q(:,order),'t',times);
solverScene=scene; solverScene.collision.smoothingEps=attempt.numerics.collisionSmoothing;
[initial.separator,~]=buildAbfdSeparators(initial.Xnode(1:6,:),initial.XmidIHS(1:6,:),solverScene,disc);
research=struct('provider',p,'config',c,'center',seed.z,'design',design,'buildOnly',true, ...
    'enforceInsertionDerivatives',true,'additionalObstacleGuard',attempt.numerics.additionalObstacleGuard, ...
    'lengthGuard',attempt.numerics.lengthGuard);
publish('building-original-nlp');
nlp=buildCasadiTrackableIHSNLP(model,solverScene,disc,initial, ...
    struct('hessianApproximation','exact','maxIter',c.solverMaxIterations),research);
scale=attempt.solverState.decisionScale; x=MX.sym('scaled_decision',numel(scale),1);
assert(numel(nlp.z)==numel(x),'ABFD:KktLayout','Rebuilt layout differs from the saved solve.');
expr=substitute([nlp.J;nlp.gEq;nlp.cIneq],nlp.z,scale.*x);
lambda=MX.sym('lambda',numel(expr)-1,1);
gradientL=gradient(expr(1)+dot(lambda,expr(2:end)),x,struct('helper_options',struct('max_num_dir',1)));
evaluate=Function('abfd_kkt_values',{x},{expr});
lagrangeGradient=Function('abfd_kkt_gradient',{x,lambda},{gradientL});
if exportOracle
    oracle=Function('abfd_reconstructed_nlp',{x},{expr(1),expr(2:end)});
    oracleFile=fullfile(outputDir,'original_nlp.casadi'); oracle.save(oracleFile);
    kit=struct('seed',seed,'c',c,'scale',scale,'baseNumZ',nlp.baseNumZ, ...
        'physicalNumZ',nlp.physicalNumZ,'options',nlp.opts,'oracleHash',abfdFileHash(oracleFile), ...
        'bounds',struct('lbx',nlp.lbz./scale,'ubx',nlp.ubz./scale,'lbg',nlp.lbg,'ubg',nlp.ubg), ...
        'attemptHash',abfdFileHash(attemptFile),'seedHash',abfdFileHash(seedFile), ...
        'implementationHash',abfdPlannerImplementationHash(),'matlab',version,'casadi',CasadiMeta.version());
    save(fullfile(outputDir,'oracle_kit.mat'),'kit');
end
writeAbfdJson(fullfile(outputDir,'structure.json'),struct('variables',numel(x),'physicalVariables',nlp.physicalNumZ, ...
    'constraints',numel(lambda),'buildSeconds',toc(timer)));
rows=cell(1,numel(segmentFiles)); detailed=cell(size(rows));
for k=1:numel(segmentFiles)
    publish(sprintf('evaluating-segment-%d',k)); saved=load(segmentFiles{k},'checkpoint');
    point=saved.checkpoint.iterate; bounds=saved.checkpoint.bounds;
    assert(isequal(bounds.lbx,nlp.lbz./scale) && isequal(bounds.ubx,nlp.ubz./scale) && ...
        isequal(bounds.lbg,nlp.lbg) && isequal(bounds.ubg,nlp.ubg),'ABFD:KktBounds','Rebuilt NLP bounds changed.');
    value=full(evaluate(point.x)); delta=max(abs(value-[point.f;point.g]));
    assert(delta<1e-8,'ABFD:KktReconstruction','Reconstructed values differ from the actual solver checkpoint.');
    derivative=full(lagrangeGradient(point.x,point.lam_g)); stationarity=derivative+point.lam_x;
    [~,ids]=sort(abs(stationarity),'descend'); ids=ids(1:min(12,numel(ids)));
    pairs=struct('variable',{},'covarianceCoordinate',{},'residual',{},'lagrangianGradient',{}, ...
        'boundMultiplier',{},'value',{},'lowerSlack',{},'upperSlack',{},'decisionScale',{});
    for j=1:numel(ids)
        id=ids(j);
        pairs(j)=struct('variable',id,'covarianceCoordinate',id>nlp.physicalNumZ, ...
            'residual',stationarity(id),'lagrangianGradient',derivative(id),'boundMultiplier',point.lam_x(id), ...
            'value',point.x(id),'lowerSlack',point.x(id)-bounds.lbx(id), ...
            'upperSlack',bounds.ubx(id)-point.x(id),'decisionScale',scale(id));
    end
    lowerIds=isfinite(bounds.lbx) & point.lam_x<0; upperIds=isfinite(bounds.ubx) & point.lam_x>0;
    variableComplementarity=max([0;abs(point.lam_x(lowerIds).*(point.x(lowerIds)-bounds.lbx(lowerIds))); ...
        abs(point.lam_x(upperIds).*(bounds.ubx(upperIds)-point.x(upperIds)))]);
    equality=bounds.lbg==bounds.ubg;
    lowerRows=~equality & isfinite(bounds.lbg) & point.lam_g<0;
    upperRows=~equality & isfinite(bounds.ubg) & point.lam_g>0;
    constraintComplementarity=max([0;abs(point.lam_g(lowerRows).*(point.g(lowerRows)-bounds.lbg(lowerRows))); ...
        abs(point.lam_g(upperRows).*(bounds.ubg(upperRows)-point.g(upperRows)))]);
    steps=[1e-4,1e-5,1e-6]; differences=zeros(3,numel(steps));
    for j=1:3
        direction=zeros(numel(x),1); direction(ids(j))=1;
        for hIndex=1:numel(steps)
            h=steps(hIndex); plus=full(evaluate(point.x+h*direction)); minus=full(evaluate(point.x-h*direction));
            fd=(plus(1)-minus(1)+point.lam_g.'*(plus(2:end)-minus(2:end)))/(2*h)+point.lam_x(ids(j));
            differences(j,hIndex)=fd-stationarity(ids(j));
        end
    end
    rows{k}=struct('segment',k,'checkpointHash',abfdFileHash(segmentFiles{k}),'valueMaximumDifference',delta, ...
        'stationarityInfinity',norm(stationarity,inf),'physicalStationarityInfinity',norm(stationarity(1:nlp.physicalNumZ),inf), ...
        'covarianceStationarityInfinity',norm(stationarity(nlp.physicalNumZ+1:end),inf), ...
        'maximumBoundComplementarity',variableComplementarity,'maximumConstraintComplementarity',constraintComplementarity, ...
        'maximumAbsMultiplier',max(abs([point.lam_x;point.lam_g])),'largestCoordinates',pairs, ...
        'finiteDifferenceSteps',steps,'topThreeStationarityFiniteDifferenceError',differences, ...
        'solverFinalInfDu',saved.checkpoint.stats.iterations.inf_du(end));
    detailed{k}=struct('stationarity',stationarity,'gradient',derivative,'iterate',point,'bounds',bounds);
    writeAbfdJson(fullfile(outputDir,sprintf('segment_%03d.json',k)),rows{k});
end
report=struct('state','complete','seedHash',abfdFileHash(seedFile),'attemptHash',abfdFileHash(attemptFile), ...
    'implementationHash',abfdPlannerImplementationHash(),'segments',{rows},'seconds',toc(timer), ...
    'scope','Original objective, constraints and actual saved multipliers; unscaled constraint multipliers and scaled decision coordinates; no optimizer or trajectory acceptance');
save(fullfile(outputDir,'diagnostic.mat'),'report','detailed','scale','c');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
