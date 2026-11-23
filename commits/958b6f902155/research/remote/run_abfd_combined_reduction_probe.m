function report=run_abfd_combined_reduction_probe(oracleDir,attemptFile,checkpointFile,outputDir,mode,auditDerivative,coordinates)
% Audit the complete frozen A problem after equivalent HS/force elimination.
% Every covariance cut, original inequality and eliminated-variable bound stays.
import casadi.*
if nargin<5, mode='global-qr'; end
if nargin<6, auditDerivative=true; end
if nargin<7, coordinates='cholesky'; end
coordinates=validatestring(coordinates,{'cholesky','symmetric'});
mode=validatestring(mode,{'global-qr','local-midpoint'});
validateattributes(auditDerivative,{'logical'},{'scalar'});
assert(~isfolder(outputDir),'ABFD:CombinedProbeExists','Use a fresh probe directory.');
mkdir(outputDir); timer=tic;
loaded=load(fullfile(oracleDir,'oracle_kit.mat'),'kit'); kit=loaded.kit;
assert(strcmp(abfdFileHash(attemptFile),kit.attemptHash),'ABFD:CombinedAttempt','Use the original audited solver attempt.');
loaded=load(attemptFile,'result'); attempt=loaded.result;
saved=load(checkpointFile,'checkpoint'); point=saved.checkpoint.iterate;
assert(isequal(saved.checkpoint.bounds,kit.bounds),'ABFD:CombinedBounds','Checkpoint bounds differ.');
oracleFile=fullfile(oracleDir,'original_nlp.casadi');
assert(strcmp(abfdFileHash(oracleFile),kit.oracleHash),'ABFD:CombinedOracle','Original oracle changed.');
oracle=Function.load(oracleFile); [fSaved,gSaved]=oracle(point.x);
assert(max(abs([full(fSaved);full(gSaved)]-[point.f;point.g]))<1e-8, ...
    'ABFD:CombinedCheckpoint','Checkpoint values do not match the frozen oracle.');
seed=kit.seed; c=kit.c;
assert(strcmp(c.predictionProfile,'sampled-hold-v2') && strcmp(c.modelMode,'analytic') && ...
    strcmp(c.solverFormulation,'covariance-shooting'),'ABFD:CombinedScope','Expected the complete sampled analytic A problem.');
c.solverFormulation='covariance-shooting-hs-reduced'; c.solverReductionMode=mode; c=makeAbfdConfig(c);
if strcmp(coordinates,'symmetric'), c.covarianceCoordinates=coordinates; c=makeAbfdConfig(c); end
model=seed.bundle.model; scene=seed.bundle.scene; disc=seed.disc; initial=seed.initial;
p=buildDynamicsProvider(model); design=buildAbfdSchedule(seed.bundle.refs,p,c);
[times,order]=sort([seed.traj.t,seed.traj.tc]); q=[seed.traj.Xnode(1:6,:),seed.traj.Xmid(1:6,:)];
design.nominal=struct('q',q(:,order),'t',times);
solverScene=scene; solverScene.collision.smoothingEps=attempt.numerics.collisionSmoothing;
[initial.separator,~]=buildAbfdSeparators(initial.Xnode(1:6,:),initial.XmidIHS(1:6,:),solverScene,disc);
r=struct('provider',p,'config',c,'center',seed.z,'design',design,'buildOnly',true,'prepareReduction',true, ...
    'enforceInsertionDerivatives',true,'additionalObstacleGuard',attempt.numerics.additionalObstacleGuard, ...
    'lengthGuard',attempt.numerics.lengthGuard);
publish('reconstructing-complete-original-problem');
original=buildCasadiTrackableIHSNLP(model,solverScene,disc,initial,struct(),r);
assert(numel(original.z)==numel(point.x) && original.physicalNumZ==kit.physicalNumZ && ...
    isequal(original.lbz./kit.scale,kit.bounds.lbx) && isequal(original.ubz./kit.scale,kit.bounds.ubx) && ...
    isequal(original.lbg,kit.bounds.lbg) && isequal(original.ubg,kit.bounds.ubg), ...
    'ABFD:CombinedOriginalLayout','Full decision space, covariance cuts or bounds changed.');
z=kit.scale.*point.x; coordinateReport=[];
if strcmp(coordinates,'symmetric')
    factorLifting=rmfield(original.tube.lifting,'kind');
    [z,coordinateReport]=convertAbfdSampledCovarianceCoordinates(factorLifting,kit.physicalNumZ,z,'symmetric');
    back=convertAbfdSampledCovarianceCoordinates(original.tube.lifting,kit.physicalNumZ,z,'cholesky');
    assert(norm(back-kit.scale.*point.x,inf)<1e-8,'ABFD:CombinedCoordinateRoundTrip','Original factor point did not survive covariance conversion.');
end
[f,g,h]=original.eval(z);
originalDifference=max(abs([full(f);full(g);full(h)]-[point.f;point.g]));
assert(originalDifference<1e-8,'ABFD:CombinedOriginalValues','Reconstruction changed the original objective or constraints.');
publish('eliminating-hs-and-leg-force-variables'); reduced=buildAbfdReducedHsNlp(original,disc,z,mode);
lifted=full(reduced.lift(reduced.x0)); [f,g,h]=original.eval(lifted); [fr,gr,hr]=reduced.eval(reduced.x0);
g=full(g); h=full(h); expected=[h;original.lbz(reduced.lowerBoundIndices)-lifted(reduced.lowerBoundIndices); ...
    lifted(reduced.upperBoundIndices)-original.ubz(reduced.upperBoundIndices)];
comparison=max([abs(full(f-fr));abs(g(reduced.retainedEquality)-full(gr));abs(expected-full(hr))]);
removedResidual=norm(g(reduced.removedEquality),inf);
assert(comparison<1e-8 && removedResidual<1e-8,'ABFD:CombinedEquivalence','Full reduced values, removed equations or original bounds disagree.');
scale=ones(size(reduced.x0)); finite=isfinite(reduced.lbz) & isfinite(reduced.ubz);
scale(finite)=(reduced.ubz(finite)-reduced.lbz(finite))/2;
assert(all(isfinite(scale) & scale>0),'ABFD:CombinedScaling','Invalid reduced decision scaling.');
x=MX.sym('scaled_reduced',numel(scale),1); value=substitute([reduced.nlp.f;reduced.nlp.g],reduced.nlp.x,scale.*x);
x0=reduced.x0./scale; evaluation=Function('abfd_combined_values',{x},{value(1),value(2:end)});
lifting=Function('abfd_combined_lift',{x},{reduced.lift(scale.*x)});
bounds=struct('lbx',reduced.lbz./scale,'ubx',reduced.ubz./scale,'lbg',reduced.lbg,'ubg',reduced.ubg);
mapping=struct('keepIndices',reduced.keepIndices,'removedEquality',reduced.removedEquality, ...
    'retainedEquality',reduced.retainedEquality,'lowerBoundIndices',reduced.lowerBoundIndices, ...
    'upperBoundIndices',reduced.upperBoundIndices,'originalInequalityCount',reduced.originalInequalityCount);
covarianceLifting=[];
if strcmp(coordinates,'symmetric')
    covarianceLifting=struct('kind',original.tube.lifting.kind,'factorSize',original.tube.lifting.factorSize, ...
        'lowerIndices',original.tube.lifting.lowerIndices,'cutTimes',original.tube.lifting.cutTimes);
end
save(fullfile(outputDir,'problem.mat'),'kit','c','x0','scale','bounds','mapping','covarianceLifting');
evaluation.save(fullfile(outputDir,'values.casadi')); lifting.save(fullfile(outputDir,'lift.casadi'));
structure=struct('fullVariables',numel(z),'reducedVariables',numel(x),'constraints',numel(value)-1, ...
    'covarianceCuts',numel(original.tube.lifting.cutTimes),'auxiliaryVariables',numel(original.tube.lifting.variables), ...
    'reduction',reduced.report,'originalValueMaximumDifference',originalDifference, ...
    'reducedValueMaximumDifference',comparison,'removedEqualityResidual',removedResidual, ...
    'covarianceCoordinates',coordinates,'coordinateConversion',coordinateReport);
writeAbfdJson(fullfile(outputDir,'structure.json'),structure);
if ~auditDerivative
    report=struct('state','complete','passed',true,'derivativeAudited',false,'structure',structure, ...
        'oracleHash',kit.oracleHash,'checkpointHash',abfdFileHash(checkpointFile),'seconds',toc(timer), ...
        'scope','Complete original/reduced values, eliminated equations and bounds only; derivative and optimization acceptance remain separate');
    writeAbfdJson(fullfile(outputDir,'status.json'),report); return
end
publish('building-complete-reduced-jacobian'); started=tic;
settings=struct('helper_options',struct('max_num_dir',1));
J=jacobian(value(2:end),x,settings); gf=gradient(value(1),x,settings);
derivative=Function('abfd_combined_derivatives',{x},{J,gf}); buildSeconds=toc(started);
publish('evaluating-complete-reduced-jacobian'); started=tic;
[matrix,gfValue]=derivative(x0); matrix=sparse(matrix); gfValue=full(gfValue); evaluationSeconds=toc(started);
steps=[1e-5,1e-6]; errors=zeros(2,2);
for k=1:2
    direction=sin(sqrt(k+1)*(1:numel(x)).'); direction=direction/norm(direction);
    exact=[gfValue.'*direction;matrix*direction];
    for j=1:2
        step=steps(j); [fp,gp]=evaluation(x0+step*direction); [fm,gm]=evaluation(x0-step*direction);
        fd=[full(fp-fm);full(gp-gm)]/(2*step); errors(k,j)=norm(exact-fd)/max(1,norm(exact));
    end
end
assert(all(min(errors,[],2)<1e-5),'ABFD:CombinedDerivative','Reduced Jacobian disagrees with finite differences.');
lambda=cos((1:size(matrix,1)).'); lambda=lambda/norm(lambda);
adjoint=Function('abfd_combined_adjoint',{x},{gradient(value(1)+dot(lambda,value(2:end)),x,settings)});
adjointError=norm(gfValue+matrix.'*lambda-full(adjoint(x0)),inf);
assert(adjointError<1e-8,'ABFD:CombinedAdjoint','Sparse derivatives disagree with the independent scalar adjoint.');
parameter=MX.sym('p',0,1); callback=Function('abfd_combined_jacobian',{x,parameter},{J});
publish('saving-audited-derivative'); callback.save(fullfile(outputDir,'jac_g.casadi'));
report=struct('state','complete','passed',true,'structure',structure,'buildSeconds',buildSeconds, ...
    'evaluationSeconds',evaluationSeconds,'structuralNnz',J.nnz(),'directionalRelativeErrors',errors, ...
    'adjointMaximumDifference',adjointError,'oracleHash',kit.oracleHash,'checkpointHash',abfdFileHash(checkpointFile), ...
    'jacobianFileHash',abfdFileHash(fullfile(outputDir,'jac_g.casadi')),'seconds',toc(timer), ...
    'scope','Same full original A feasible set after exact HS/force elimination, all covariance cuts and bounds retained; derivative audit only, no optimization acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
