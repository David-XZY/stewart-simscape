function report=run_abfd_direct_reduced_probe(reducedDir,outputDir,assembly)
% Audit full direct AD callbacks in the declared covariance coordinates.
import casadi.*
if nargin<3, assembly='direct'; end
assert(any(strcmp(assembly,{'direct','grouped-moments'})), ...
    'ABFD:ReducedDerivativeAssembly','Unknown reduced derivative assembly.');
assert(~isfolder(outputDir),'ABFD:DirectReducedExists','Use a fresh derivative directory.');
mkdir(outputDir); timer=tic;
names={'problem.mat','values.casadi','lift.casadi','structure.json'};
hashes=cellfun(@(name)abfdFileHash(fullfile(reducedDir,name)),names,'UniformOutput',false);
valueAudit=readAbfdJsonUtf8(fullfile(reducedDir,'status.json'));
assert(strcmp(valueAudit.state,'complete') && valueAudit.passed, ...
    'ABFD:DirectReducedValueAudit','Complete original-problem equivalence must pass before differentiating.');
data=load(fullfile(reducedDir,'problem.mat')); structure=readAbfdJsonUtf8(fullfile(reducedDir,'structure.json'));
assert(strcmp(data.c.solverFormulation,'covariance-shooting-hs-reduced') && ...
    structure.originalValueMaximumDifference<1e-8 && structure.reducedValueMaximumDifference<1e-8 && ...
    structure.removedEqualityResidual<1e-8,'ABFD:DirectReducedScope','Complete reduced/original values and bounds must agree.');
reference=Function.load(fullfile(reducedDir,'values.casadi'));
x=MX.sym('decision',numel(data.x0),1); p=MX.sym('p',0,1);
expression=reference.call({x},true,false); f=expression{1}; g=expression{2};
settings=struct('helper_options',struct('max_num_dir',1));
publish('building-full-direct-derivatives'); started=tic;
grouping=struct('enabled',false); assemblyId='direct-reduced-ad-v1';
if strcmp(assembly,'grouped-moments')
    assert(isfield(data.c,'covarianceCoordinates') && strcmp(data.c.covarianceCoordinates,'symmetric') && ...
        numel(data.mapping.keepIndices)==numel(x),'ABFD:ReducedMomentPartition', ...
        'Grouped covariance derivatives require the complete symmetric-coordinate mapping.');
    meanIndices=find(data.mapping.keepIndices<=data.kit.physicalNumZ);
    [callbacks,grouping]=buildAbfdGroupedMomentDerivatives(x,f,g,meanIndices,false,@publish,1);
    callback=callbacks.jacobian; structuralNnz=grouping.jacobianStructuralNnz;
    grouping.enabled=true; assemblyId='grouped-reduced-ad-v1';
else
    J=jacobian(g,x,settings); structuralNnz=J.nnz();
    callback=Function('abfd_direct_reduced_jac_g',{x,p},{g,J},{'x','p'},{'g','jac_g'});
end
gf=gradient(f,x,settings);
gradientCallback=Function('abfd_direct_reduced_grad_f',{x,p},{f,gf},{'x','p'},{'f','grad_f'});
buildSeconds=toc(started); publish('evaluating-full-direct-derivatives'); started=tic;
[gj,matrix]=callback(data.x0,[]); [fj,costGradient]=gradientCallback(data.x0,[]);
[f0,g0]=reference(data.x0); matrix=sparse(matrix); costGradient=full(costGradient);
assert(max(abs([full(gj-g0);full(fj-f0)]))<1e-8,'ABFD:DirectReducedCallback','Derivative callback values differ.');
evaluationSeconds=toc(started); steps=[1e-5,1e-6]; errors=zeros(2,2);
for k=1:2
    direction=sin(sqrt(k+1)*(1:numel(x)).'); direction=direction/norm(direction);
    expected=[costGradient.'*direction;matrix*direction];
    for j=1:2
        h=steps(j); [fp,gp]=reference(data.x0+h*direction); [fm,gm]=reference(data.x0-h*direction);
        finite=[full(fp-fm);full(gp-gm)]/(2*h);
        errors(k,j)=norm(expected-finite)/max(1,norm(expected));
    end
end
assert(all(min(errors,[],2)<1e-5),'ABFD:DirectReducedFiniteDifference','Full derivatives disagree with independent finite differences.');
lambda=cos((1:numel(g)).'); lambda=lambda/norm(lambda);
adjoint=Function('abfd_direct_reduced_adjoint',{x},{gradient(f+dot(lambda,g),x,settings)});
adjointError=norm(costGradient+matrix.'*lambda-full(adjoint(data.x0)),inf);
assert(adjointError<1e-8,'ABFD:DirectReducedAdjoint','Matrix derivatives disagree with the scalar Lagrangian derivative.');
publish('saving-audited-full-derivatives');
callback.save(fullfile(outputDir,'jac_g.casadi')); gradientCallback.save(fullfile(outputDir,'grad_f.casadi'));
for k=1:numel(names)
    assert(strcmp(abfdFileHash(fullfile(reducedDir,names{k})),hashes{k}), ...
        'ABFD:DirectReducedChanged','Audited inputs changed during differentiation.');
end
report=struct('state','complete','passed',true,'assembly',assemblyId,'grouping',grouping, ...
    'variables',numel(x),'constraints',numel(g),'structuralNnz',structuralNnz, ...
    'buildSeconds',buildSeconds,'evaluationSeconds',evaluationSeconds, ...
    'directionalRelativeErrors',errors,'adjointMaximumDifference',adjointError, ...
    'oracleHash',data.kit.oracleHash,'inputFiles',{names},'inputHashes',{hashes}, ...
    'reducedDirectory',reducedDir,'valueAuditHash',abfdFileHash(fullfile(reducedDir,'status.json')), ...
    'jacobianFileHash',abfdFileHash(fullfile(outputDir,'jac_g.casadi')), ...
    'gradientFileHash',abfdFileHash(fullfile(outputDir,'grad_f.casadi')), ...
    'implementationHash',abfdFileHash([mfilename('fullpath'),'.m']),'seconds',toc(timer), ...
    'scope','Full original horizon, constraints, covariance cuts and bounds in declared coordinates; derivative audit, not optimization acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
