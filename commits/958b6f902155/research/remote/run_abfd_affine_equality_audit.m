function report=run_abfd_affine_equality_audit(oracleDir,checkpointFile,outputDir)
% Diagnose constant and linearly dependent equalities without altering the NLP.
import casadi.*
assert(~isfolder(outputDir),'ABFD:AffineAuditExists','Use a fresh audit directory.');
mkdir(outputDir); timer=tic;
v=MX.sym('fixture',3); fixture=[2*v(1)+3;v(1)*v(2);sin(v(3));MX(0);MX(1e-15)];
firstOrder=logical(which_depends(fixture,v,1,true)); secondOrder=logical(which_depends(fixture,v,2,true));
assert(isequal(firstOrder(:),logical([1;1;1;0;0])) && ...
    isequal(secondOrder(:),logical([0;1;1;0;0])), ...
    'ABFD:DependenceApi','The dependence query must identify output order correctly.');
loaded=load(fullfile(oracleDir,'oracle_kit.mat'),'kit'); kit=loaded.kit;
checkpoint=load(checkpointFile,'checkpoint'); point=checkpoint.checkpoint.iterate;
assert(isequal(checkpoint.checkpoint.bounds,kit.bounds),'ABFD:AffineAuditBounds','Checkpoint and original bounds differ.');
file=fullfile(oracleDir,'original_nlp.casadi');
assert(strcmp(abfdFileHash(file),kit.oracleHash),'ABFD:AffineAuditOracle','Original oracle changed.');
oracle=Function.load(file); [f0,g0]=oracle(point.x);
assert(max(abs([full(f0);full(g0)]-[point.f;point.g]))<1e-8, ...
    'ABFD:AffineAuditPoint','Checkpoint values differ from the original problem.');
x=MX.sym('original',numel(point.x)); expressions=oracle.call({x},true,false);
equalityRows=find(kit.bounds.lbg==kit.bounds.ubg); equality=expressions{2}(equalityRows)-kit.bounds.lbg(equalityRows);
publish('classifying-affine-equalities');
nonlinear=logical(which_depends(equality,x,2,true)); nonlinear=nonlinear(:);
assert(numel(nonlinear)==numel(equalityRows),'ABFD:AffineAuditLayout','Dependence output must describe every equality.');
affineIndices=find(~nonlinear); affineRows=equalityRows(affineIndices); affine=equality(affineIndices);
settings=struct('helper_options',struct('max_num_dir',1)); A=jacobian(affine,x,settings);
assert(~depends_on(A,x),'ABFD:AffineAuditNonlinear','Detected affine coefficients must be independent of every decision.');
evaluate=Function('abfd_affine_coefficients',{x},{A,affine}); [matrix,value]=evaluate(point.x);
matrix=sparse(matrix); offset=full(value)-matrix*point.x;
maximumReconstruction=0;
for k=1:3
    trial=point.x+1e-3*sin(k*(1:numel(x)).'); [~,trialValue]=evaluate(trial);
    maximumReconstruction=max(maximumReconstruction,norm(full(trialValue)-matrix*trial-offset,inf));
end
assert(maximumReconstruction<1e-10,'ABFD:AffineAuditReconstruction','Affine reconstruction differs off the checkpoint.');
constantLocal=find(sum(spones(matrix),2)==0);
activeColumns=find(any(matrix,1));
assert(all(activeColumns<=kit.physicalNumZ) && numel(activeColumns)*numel(affineRows)<=2e7, ...
    'ABFD:AffineAuditSize','Unexpected large affine covariance dependence; no dense rank factorization attempted.');
publish('factoring-only-active-affine-columns');
[~,R,permutation]=qr(full(matrix(:,activeColumns)).','vector');
diagonal=abs(diag(R)); scale=max([1;diagonal]); thresholds=[1e-8,1e-10,1e-12,1e-14];
ranks=arrayfun(@(threshold)sum(diagonal>threshold*scale),thresholds); numericalRank=ranks(3);
kept=permutation(1:numericalRank); dependent=permutation(numericalRank+1:end);
coefficients=(R(1:numericalRank,1:numericalRank)\R(1:numericalRank,numericalRank+1:end)).';
coefficientResidual=matrix(dependent,:)-coefficients*matrix(kept,:);
offsetResidual=offset(dependent)-coefficients*offset(kept);
support=find(any(coefficientResidual~=0,1)); magnitudes=max(abs([kit.bounds.lbx,kit.bounds.ubx]),[],2);
if any(~isfinite(magnitudes(support)))
    boundedResidual=inf;
else
    boundedResidual=max([0;abs(offsetResidual)+abs(coefficientResidual(:,support))*magnitudes(support)]);
end
details=struct('affineRows',affineRows,'activeColumns',activeColumns,'matrix',matrix,'offset',offset, ...
    'keptRows',affineRows(kept),'dependentRows',affineRows(dependent),'coefficients',coefficients, ...
    'coefficientResidual',coefficientResidual,'offsetResidual',offsetResidual,'diagonal',diagonal);
save(fullfile(outputDir,'affine_equality_audit.mat'),'details','kit');
report=struct('state','complete','passed',true,'equalities',numel(equalityRows),'affineEqualities',numel(affineRows), ...
    'nonlinearEqualities',sum(nonlinear),'activeAffineColumns',numel(activeColumns), ...
    'constantRows',affineRows(constantLocal),'constantValues',offset(constantLocal), ...
    'rankThresholds',thresholds,'numericalRanks',ranks,'candidateDependentRows',affineRows(dependent), ...
    'maximumReconstruction',maximumReconstruction,'coefficientResidualInfinity',norm(coefficientResidual,inf), ...
    'offsetResidualInfinity',norm(offsetResidual,inf),'boundedDependenceResidual',boundedResidual, ...
    'dependenceBoundFinite',isfinite(boundedResidual), ...
    'withinRoundoffBudget',isfinite(boundedResidual) && boundedResidual<=kit.c.solverTolerance/100, ...
    'originalSolverTolerance',kit.c.solverTolerance,'oracleHash',kit.oracleHash, ...
    'checkpointHash',abfdFileHash(checkpointFile),'implementationHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
    'seconds',toc(timer),'scope','Read-only structural and bounded numerical dependence audit; no constraints removed, no solve or performance acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
