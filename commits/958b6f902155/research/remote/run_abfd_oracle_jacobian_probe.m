function report=run_abfd_oracle_jacobian_probe(oracleDir,checkpointFile,outputDir)
% Unwrap only the exported outer MX call; keep all original model operations.
import casadi.*
assert(~isfolder(outputDir),'ABFD:JacobianProbeExists','Use a fresh probe directory.');
mkdir(outputDir); timer=tic;
loaded=load(fullfile(oracleDir,'oracle_kit.mat'),'kit'); kit=loaded.kit;
saved=load(checkpointFile,'checkpoint'); point=saved.checkpoint.iterate;
auditStatus=readAbfdJsonUtf8(fullfile(oracleDir,'status.json'));
assert(strcmp(auditStatus.state,'complete'),'ABFD:OracleAudit','Original oracle audit has not completed.');
assert(isequal(saved.checkpoint.bounds,kit.bounds),'ABFD:OracleBounds','Checkpoint bounds differ from the frozen oracle.');
oracleFile=fullfile(oracleDir,'original_nlp.casadi');
assert(strcmp(abfdFileHash(oracleFile),kit.oracleHash),'ABFD:OracleHash','Oracle changed.');
oracle=Function.load(oracleFile); x=MX.sym('x',numel(point.x),1);
publish('inlining-original-graph'); values=oracle.call({x},true,false); f=values{1}; g=values{2};
evaluation=Function('abfd_inlined_values',{x},{f,g}); [fv,gv]=evaluation(point.x);
delta=max(abs([full(fv);full(gv)]-[point.f;point.g]));
assert(delta<1e-8,'ABFD:InlineValues','Inlining changed checkpoint values.');
publish('building-jacobian'); buildTimer=tic;
settings=struct('helper_options',struct('max_num_dir',1));
J=jacobian(g,x,settings); gf=gradient(f,x,settings);
fun=Function('abfd_inlined_jacobian',{x},{J,gf}); buildSeconds=toc(buildTimer);
publish('evaluating-jacobian'); evalTimer=tic; [matrix,costGradient]=fun(point.x);
evalSeconds=toc(evalTimer); matrix=sparse(matrix); costGradient=full(costGradient);
publish('checking-directional-derivatives'); steps=[1e-5,1e-6]; errors=zeros(2,2);
for k=1:2
    direction=sin(sqrt(k+1)*(1:numel(x)).'); direction=direction/norm(direction);
    expected=[costGradient.'*direction;matrix*direction];
    for j=1:2
        h=steps(j); [fp,gp]=evaluation(point.x+h*direction); [fm,gm]=evaluation(point.x-h*direction);
        fd=[full(fp-fm);full(gp-gm)]/(2*h);
        errors(k,j)=norm(expected-fd)/max(1,norm(expected));
    end
end
assert(all(min(errors,[],2)<1e-5),'ABFD:InlineDerivative','Inlined derivatives failed finite differences.');
audit=load(fullfile(oracleDir,'diagnostic.mat'),'detailed'); expected=[];
for k=1:numel(audit.detailed)
    item=audit.detailed{k};
    if isequal(item.iterate.x,point.x) && isequal(item.iterate.lam_g,point.lam_g), expected=item.gradient; break; end
end
assert(~isempty(expected),'ABFD:InlineCheckpoint','No independently audited gradient for this iterate.');
reconstructed=costGradient+matrix.'*point.lam_g;
gradientError=norm(reconstructed-expected,inf);
assert(gradientError<1e-8,'ABFD:InlineAdjoint','Sparse Jacobian disagrees with independent scalar adjoint.');
parameter=MX.sym('p',0,1);
jacobianCallback=Function('abfd_inlined_ipopt_jacobian',{x,parameter},{J},{'x','p'},{'jac_g_x'});
publish('saving-derivatives'); jacobianCallback.save(fullfile(outputDir,'jac_g.casadi'));
evaluation.save(fullfile(outputDir,'values.casadi'));
report=struct('state','complete','passed',true,'oracleHash',kit.oracleHash,'checkpointHash',abfdFileHash(checkpointFile), ...
    'implementationHash',abfdFileHash([mfilename('fullpath'),'.m']),'matlabVersion',version, ...
    'variables',numel(x),'constraints',numel(g),'structuralNnz',J.nnz(),'numericalNnz',nnz(matrix), ...
    'buildSeconds',buildSeconds,'evaluationSeconds',evalSeconds,'valueMaximumDifference',delta, ...
    'finiteDifferenceSteps',steps,'directionalRelativeErrors',errors,'adjointMaximumDifference',gradientError, ...
    'jacobianFileHash',abfdFileHash(fullfile(outputDir,'jac_g.casadi')),'seconds',toc(timer), ...
    'scope','Same complete frozen NLP; direct sparse Jacobian agrees with scalar adjoint and finite differences; no solve or trajectory acceptance');
save(fullfile(outputDir,'derivative.mat'),'matrix','costGradient','report','-v7.3');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
    function publish(stage)
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage',stage,'seconds',toc(timer),'pid',feature('getpid')));
    end
end
