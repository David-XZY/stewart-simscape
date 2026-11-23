function report=run_abfd_frozen_oracle_restart(oracleDir,checkpointFile,outputDir,variant,derivativeDir,resume)
% Controlled numerical restart of an exported, fully checked original NLP.
% A saved point is only an initial guess; all physical/gain checks run again.
import casadi.*
if nargin<5, derivativeDir=''; end
if nargin<6, resume=[]; end
variant=validatestring(variant,{'baseline','dual-step','adaptive-dual-step','near-central','bound-dual-step','bound-near-central'});
assert(~isfolder(outputDir),'ABFD:RestartExists','Use a fresh restart directory.');
mkdir(outputDir); timer=tic;
loaded=load(fullfile(oracleDir,'oracle_kit.mat'),'kit'); kit=loaded.kit;
saved=load(checkpointFile,'checkpoint'); checkpoint=saved.checkpoint; point=checkpoint.iterate;
audit=readAbfdJsonUtf8(fullfile(oracleDir,'status.json'));
assert(strcmp(audit.state,'complete'),'ABFD:OracleAudit','Original oracle audit has not completed.');
if iscell(audit.segments), audits=[audit.segments{:}]; else, audits=audit.segments; end
assert(any(strcmp({audits.checkpointHash},abfdFileHash(checkpointFile))), ...
    'ABFD:OracleCheckpoint','Checkpoint was not audited against this oracle.');
assert(isequal(checkpoint.bounds,kit.bounds),'ABFD:OracleBounds','Saved bounds differ.');
file=fullfile(oracleDir,'original_nlp.casadi');
assert(strcmp(abfdFileHash(file),kit.oracleHash),'ABFD:OracleHash','Serialized NLP changed.');
oracle=Function.load(file); [f0,g0]=oracle(point.x);
reconstruction=max(abs([full(f0);full(g0)]-[point.f;point.g]));
assert(reconstruction<1e-8,'ABFD:OracleValues','Original checkpoint values changed.');
continuation=struct('used',false);
if ~isempty(resume)
    assert(isstruct(resume) && isscalar(resume) && ...
        isequal(sort(fieldnames(resume)),sort({'file';'sha256';'maxCpuTime';'maxIterations'})), ...
        'ABFD:ResumeConfig','Provide the saved result, its byte hash and explicit compute limits.');
    validateattributes(resume.maxCpuTime,{'double'},{'scalar','real','finite','positive'});
    validateattributes(resume.maxIterations,{'double'},{'scalar','real','finite','positive','integer'});
    assert(strcmp(abfdFileHash(resume.file),resume.sha256),'ABFD:ResumeHash','Continuation artifact changed.');
    prior=load(resume.file,'iterate','stats','kit','manifest');
    assert(isequaln(prior.kit,kit) && strcmp(prior.manifest.oracleHash,kit.oracleHash) && ...
        strcmp(prior.manifest.checkpointHash,abfdFileHash(checkpointFile)) && strcmp(prior.manifest.variant,variant), ...
        'ABFD:ResumeProblem','Saved iterate must belong to this identical frozen problem and numerical variant.');
    point=prior.iterate;
    assert(numel(point.x)==numel(kit.bounds.lbx) && numel(point.g)==numel(kit.bounds.lbg) && ...
        numel(point.lam_x)==numel(point.x) && numel(point.lam_g)==numel(point.g) && ...
        all(isfinite([point.x;point.f;point.g;point.lam_x;point.lam_g])), ...
        'ABFD:ResumeIterate','Saved primal and dual values must have the complete finite layout.');
    [fResume,gResume]=oracle(point.x);
    resumeDifference=max(abs([full(fResume);full(gResume)]-[point.f;point.g]));
    assert(resumeDifference<1e-8,'ABFD:ResumeValues','Saved iterate values differ from the original oracle.');
    continuation=struct('used',true,'sourceFile',resume.file,'sourceHash',resume.sha256, ...
        'previousStatus',prior.stats.return_status,'previousIterations',prior.stats.iter_count, ...
        'valueMaximumDifference',resumeDifference,'initialObjective',point.f);
end
x=MX.sym('decision',numel(point.x),1); [f,g]=oracle(x);
options=kit.options; options.ipopt.hessian_approximation='limited-memory';
% Bound AD direction batching without changing the derivative or sparsity.
options.oracle_options=struct('max_num_dir',1);
options.ipopt.mu_strategy='monotone'; options.ipopt.mu_init=1e-6;
options.ipopt.max_iter=300; options.ipopt.max_cpu_time=600; options.ipopt.print_level=5;
options.ipopt.warm_start_init_point='yes';
for name={'bound_push','bound_frac','slack_bound_push','slack_bound_frac','mult_bound_push'}
    options.ipopt.(['warm_start_',name{1}])=1e-9;
end
options.ipopt.alpha_for_y='primal';
if any(strcmp(variant,{'dual-step','adaptive-dual-step'})), options.ipopt.alpha_for_y='min-dual-infeas'; end
if any(strcmp(variant,{'bound-dual-step','bound-near-central'})), options.ipopt.alpha_for_y='bound-mult'; end
if strcmp(variant,'adaptive-dual-step'), options.ipopt.mu_strategy='adaptive'; end
if any(strcmp(variant,{'near-central','bound-near-central'})), options.ipopt.mu_init=1e-10; end
if ~isempty(resume)
    % The only changed numerical options are the explicit run limits.
    priorOptions=prior.manifest.options; compared=options;
    compared.ipopt.max_cpu_time=priorOptions.ipopt.max_cpu_time;
    compared.ipopt.max_iter=priorOptions.ipopt.max_iter;
    assert(isequaln(compared,priorOptions),'ABFD:ResumeOptions','Continuation cannot silently alter solver settings.');
    options.ipopt.max_cpu_time=resume.maxCpuTime; options.ipopt.max_iter=resume.maxIterations;
end
assert(options.ipopt.tol==kit.c.solverTolerance && options.ipopt.constr_viol_tol==kit.c.solverTolerance && ...
    options.ipopt.acceptable_iter==0,'ABFD:RestartTolerance','Original strict termination tests must remain.');
derivative=struct('kind','automatic'); callback=[];
if ~isempty(derivativeDir)
    publish('loading-audited-jacobian');
    derivative=readAbfdJsonUtf8(fullfile(derivativeDir,'status.json'));
    assert(strcmp(derivative.state,'complete') && derivative.passed && ...
        strcmp(derivative.oracleHash,kit.oracleHash) && strcmp(derivative.checkpointHash,abfdFileHash(checkpointFile)), ...
        'ABFD:RestartDerivativeAudit','Derivative audit must match this complete original problem and checkpoint.');
    jacobianFile=fullfile(derivativeDir,'jac_g.casadi');
    assert(strcmp(derivative.jacobianFileHash,abfdFileHash(jacobianFile)), ...
        'ABFD:RestartDerivativeHash','Audited sparse derivative changed.');
    jacobianFunction=Function.load(jacobianFile); parameter=MX.sym('p',0,1);
    assert(jacobianFunction.n_in()==2 && jacobianFunction.n_out()==1, ...
        'ABFD:RestartDerivativeSignature','Expected the audited matrix-only derivative function.');
    J=jacobianFunction(x,parameter);
    assert(isequal(size(J),[numel(g),numel(x)]),'ABFD:RestartDerivativeSize','Sparse derivative dimensions changed.');
    callback=Function('abfd_checked_sparse_jac_g',{x,parameter},{g,J},{'x','p'},{'g','jac_g'});
    derivative.kind='explicit-audited-sparse-jacobian';
end
manifest=struct('variant',variant,'oracleHash',kit.oracleHash,'checkpointHash',abfdFileHash(checkpointFile), ...
    'implementationHash',abfdFileHash([mfilename('fullpath'),'.m']),'derivative',derivative, ...
    'options',options,'continuation',continuation,'valueMaximumDifference',reconstruction,'variables',numel(x),'constraints',numel(g), ...
    'scope','Same full horizon, frozen gains, objective, nonlinear constraints, covariance variables, scaling and bounds; no acceptance transferred from the checkpoint');
writeAbfdJson(fullfile(outputDir,'manifest.json'),manifest);
if ~isempty(callback), options.jac_g=callback; end
publish('building-solver'); solver=nlpsol('abfd_frozen_restart','ipopt',struct('x',x,'f',f,'g',g),options);
publish('solving'); solveTimer=tic;
sol=solver('x0',point.x,'lam_x0',point.lam_x,'lam_g0',point.lam_g, ...
    'lbx',kit.bounds.lbx,'ubx',kit.bounds.ubx,'lbg',kit.bounds.lbg,'ubg',kit.bounds.ubg);
solveSeconds=toc(solveTimer); stats=solver.stats();
iterate=struct('x',full(sol.x),'f',full(sol.f),'g',full(sol.g),'lam_x',full(sol.lam_x),'lam_g',full(sol.lam_g));
assert(all(isfinite([iterate.x;iterate.f;iterate.g;iterate.lam_x;iterate.lam_g])), ...
    'ABFD:RestartNonfinite','Restart returned nonfinite values.');
save(fullfile(outputDir,'solver_result.mat'),'iterate','stats','kit','manifest');
equality=kit.bounds.lbg==kit.bounds.ubg;
maxEq=max(abs(iterate.g(equality)-kit.bounds.lbg(equality)));
maxIneq=max([0;kit.bounds.lbg(~equality)-iterate.g(~equality);iterate.g(~equality)-kit.bounds.ubg(~equality)]);
maxBound=max([0;kit.bounds.lbx-iterate.x;iterate.x-kit.bounds.ubx]);
lambda=MX.sym('lambda',numel(g),1);
grad=Function('abfd_restart_kkt',{x,lambda},{gradient(f+dot(lambda,g),x)});
stationarity=norm(full(grad(iterate.x,iterate.lam_g))+iterate.lam_x,inf);
report=struct('state','complete','variant',variant,'solverStatus',stats.return_status,'solverSuccess',logical(stats.success), ...
    'iterations',stats.iter_count,'solveSeconds',solveSeconds,'objective',iterate.f, ...
    'maxEq',maxEq,'maxIneq',maxIneq,'maxBound',maxBound,'stationarityInfinity',stationarity, ...
    'physicalAccepted',false,'finalGainAccepted',false,'seconds',toc(timer), ...
    'scope','Controlled numerical restart; full A outer-loop and Simscape acceptance are separate');
if stats.success && max([maxEq,maxIneq,maxBound])<=kit.c.constraintTolerance
    publish('validating-trajectory-and-gains');
    seed=kit.seed; scene=seed.bundle.scene; model=seed.bundle.model; disc=seed.disc;
    z=kit.scale.*iterate.x;
    [X,Xm,An,Am,Fn,Fm]=unpackIHSDecisionImplicit(z(1:kit.baseNumZ),scene,disc);
    traj=struct('t',disc.tNode,'tc',disc.tMid,'Xnode',X,'Xmid',Xm,'Anode',An,'Amid',Am,'Unode',Fn,'Umid',Fm);
    provider=buildDynamicsProvider(model); refs=reconstructIhsidContinuousReference(traj,provider,kit.c.sampleTime);
    dense=validateAbfdReference(refs,model,scene,kit.c); report.dense=dense;
    report.physicalAccepted=dense.passed;
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
