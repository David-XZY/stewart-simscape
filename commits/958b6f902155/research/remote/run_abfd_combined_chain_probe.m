function report=run_abfd_combined_chain_probe(reducedDir,oracleDir,derivativeDir,outputDir)
% Exact chain-rule callbacks reuse the audited original sparse Jacobian.
% This changes derivative assembly, not the reduced problem or its bounds.
import casadi.*
assert(~isfolder(outputDir),'ABFD:ChainProbeExists','Use a fresh chain-rule probe directory.');
mkdir(outputDir); timer=tic;
names={'problem.mat','values.casadi','lift.casadi','structure.json'};
hashes=cellfun(@(name)abfdFileHash(fullfile(reducedDir,name)),names,'UniformOutput',false);
data=load(fullfile(reducedDir,'problem.mat')); kit=data.kit; mapping=data.mapping;
assert(~isfield(data.c,'covarianceCoordinates') || strcmp(data.c.covarianceCoordinates,'cholesky'), ...
    'ABFD:ChainCovarianceCoordinates','This original-factor derivative bridge requires Cholesky coordinates.');
structure=readAbfdJsonUtf8(fullfile(reducedDir,'structure.json'));
assert(structure.originalValueMaximumDifference<1e-8 && structure.reducedValueMaximumDifference<1e-8 && ...
    structure.removedEqualityResidual<1e-8 && numel(data.x0)==structure.reducedVariables, ...
    'ABFD:ChainEquivalence','The complete reduced problem has not passed value and elimination checks.');
audit=readAbfdJsonUtf8(fullfile(derivativeDir,'status.json'));
jacobianFile=fullfile(derivativeDir,'jac_g.casadi'); oracleFile=fullfile(oracleDir,'original_nlp.casadi');
assert(strcmp(audit.state,'complete') && audit.passed && strcmp(audit.oracleHash,kit.oracleHash) && ...
    strcmp(abfdFileHash(oracleFile),kit.oracleHash) && strcmp(abfdFileHash(jacobianFile),audit.jacobianFileHash), ...
    'ABFD:ChainOriginalDerivative','Original sparse derivative and oracle must be audited and unchanged.');
publish('loading-original-jacobian-and-reduced-map');
oracle=Function.load(oracleFile); originalJacobian=Function.load(jacobianFile);
lift=Function.load(fullfile(reducedDir,'lift.casadi')); reference=Function.load(fullfile(reducedDir,'values.casadi'));
x=MX.sym('reduced',numel(data.x0),1); parameter=MX.sym('p',0,1);
z=lift(x); u=z./kit.scale; [f,gOriginal]=oracle(u);
equalityCount=sum(kit.bounds.lbg==kit.bounds.ubg);
rows=[mapping.retainedEquality;equalityCount+(1:mapping.originalInequalityCount).'];
lower=mapping.lowerBoundIndices; upper=mapping.upperBoundIndices;
g=[gOriginal(rows);kit.bounds.lbx(lower).*kit.scale(lower)-z(lower); ...
    z(upper)-kit.bounds.ubx(upper).*kit.scale(upper)];
assert(numel(g)==numel(data.bounds.lbg),'ABFD:ChainRows','The full reduced constraint layout changed.');
values=Function('abfd_chain_values',{x},{f,g});
maximumValueDifference=0;
for k=0:2
    direction=sin(sqrt(k+1)*(1:numel(x)).'); direction=direction/norm(direction);
    point=data.x0+k*1e-6*direction;
    [fr,gr]=reference(point); [fc,gc]=values(point);
    maximumValueDifference=max(maximumValueDifference,max(abs([full(fr-fc);full(gr-gc)])));
end
assert(maximumValueDifference<1e-8,'ABFD:ChainValues','Chain assembly changed the original reduced objective or constraints.');
publish('building-map-jacobian-and-sparse-products'); started=tic;
settings=struct('helper_options',struct('max_num_dir',1)); D=jacobian(u,x,settings);
Jfull=originalJacobian(u,parameter);
J=[Jfull(rows,:)*D;-diag(MX(kit.scale(lower)))*D(lower,:);diag(MX(kit.scale(upper)))*D(upper,:)];
u0=MX.sym('original',numel(kit.scale),1); originalExpressions=oracle.call({u0},true,false);
originalGradient=Function('abfd_original_cost_gradient',{u0},{gradient(originalExpressions{1},u0,settings)});
gf=D.'*originalGradient(u);
% CasADi 3.7.2 Ipopt callbacks have two inputs and two outputs:
% grad_f returns (f, grad_f); jac_g returns (g, jac_g).
jacobianCallback=Function('abfd_chain_jac_g',{x,parameter},{g,J},{'x','p'},{'g','jac_g'});
gradientCallback=Function('abfd_chain_grad_f',{x,parameter},{f,gf},{'x','p'},{'f','grad_f'});
buildSeconds=toc(started); publish('evaluating-chain-derivatives'); started=tic;
[~,matrix]=jacobianCallback(data.x0,[]); [~,costGradient]=gradientCallback(data.x0,[]);
matrix=sparse(matrix); costGradient=full(costGradient); evaluationSeconds=toc(started);
steps=[1e-5,1e-6]; errors=zeros(2,2);
for k=1:2
    direction=sin(sqrt(k+1)*(1:numel(x)).'); direction=direction/norm(direction);
    exact=[costGradient.'*direction;matrix*direction];
    for j=1:2
        h=steps(j); [fp,gp]=reference(data.x0+h*direction); [fm,gm]=reference(data.x0-h*direction);
        fd=[full(fp-fm);full(gp-gm)]/(2*h); errors(k,j)=norm(exact-fd)/max(1,norm(exact));
    end
end
assert(all(min(errors,[],2)<1e-5),'ABFD:ChainFiniteDifference','Chain derivatives disagree with the separately saved reduced problem.');
publish('checking-independent-scalar-adjoint');
lambda=cos((1:size(matrix,1)).'); lambda=lambda/norm(lambda);
independent=reference.call({x},true,false);
adjoint=Function('abfd_independent_reduced_adjoint',{x}, ...
    {gradient(independent{1}+dot(lambda,independent{2}),x,settings)});
adjointError=norm(costGradient+matrix.'*lambda-full(adjoint(data.x0)),inf);
assert(adjointError<1e-8,'ABFD:ChainAdjoint','Chain derivatives disagree with the independent scalar adjoint.');
publish('saving-checked-chain-callbacks');
jacobianCallback.save(fullfile(outputDir,'jac_g.casadi'));
gradientCallback.save(fullfile(outputDir,'grad_f.casadi')); values.save(fullfile(outputDir,'values.casadi'));
for k=1:numel(names)
    assert(strcmp(abfdFileHash(fullfile(reducedDir,names{k})),hashes{k}), ...
        'ABFD:ChainInputChanged','Reduced input artifacts changed during the audit.');
end
report=struct('state','complete','passed',true,'variables',numel(x),'constraints',numel(g), ...
    'structuralNnz',J.nnz(),'mapNnz',D.nnz(),'buildSeconds',buildSeconds,'evaluationSeconds',evaluationSeconds, ...
    'maximumValueDifference',maximumValueDifference,'directionalRelativeErrors',errors,'adjointMaximumDifference',adjointError, ...
    'reducedDirectory',reducedDir,'inputFiles',{names},'inputHashes',{hashes},'oracleHash',kit.oracleHash, ...
    'originalJacobianHash',audit.jacobianFileHash,'jacobianFileHash',abfdFileHash(fullfile(outputDir,'jac_g.casadi')), ...
    'gradientFileHash',abfdFileHash(fullfile(outputDir,'grad_f.casadi')),'seconds',toc(timer), ...
    'scope','Exact chain-rule derivatives of the full equivalent reduced A problem; original constraints, covariance cuts and bounds retained; no optimization acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
