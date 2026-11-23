function report=run_abfd_observer_simscape_validation(inputDir,referenceDir,outputDir,comparison)
% Fixed training-side trajectory, matched physical object, direct/observer pair.
% Optional 'kernel' comparison uses the same observer with two execution kernels.
if nargin<4, comparison='state-source'; end
assert(any(strcmp(comparison,{'state-source','kernel'})), ...
    'ABFD:ObserverComparison','Expected state-source or kernel comparison.');
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
assert(~isfolder(outputDir),'ABFD:ValidationExists','Use a fresh validation directory.'); mkdir(outputDir);
timer=tic; writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage','reference-audit'));
try
    transfer=readAbfdJsonUtf8(fullfile(referenceDir,'transfer_manifest.json'));
    assert(strcmp(transfer.state,'complete') && strcmp(transfer.split,'train') && transfer.start==1, ...
        'ABFD:ValidationSplit','Use the preselected first training reference.');
    for k=1:numel(transfer.files)
        assert(strcmp(abfdFileHash(fullfile(referenceDir,transfer.files(k).name)),transfer.files(k).sha256), ...
            'ABFD:ValidationHash','Training reference changed.');
    end
    frozen=load(fullfile(inputDir,'frozen.mat')); saved=load(fullfile(referenceDir,'start_01.mat'));
    ids=find(strcmp({frozen.tasks.taskId},transfer.taskId)); assert(isscalar(ids),'ABFD:ValidationTask','Unknown task.');
    task=frozen.tasks(ids); assert(strcmp(task.split,'train'),'ABFD:ValidationSplit','Test data cannot select this repair.');
    status=readAbfdJsonUtf8(fullfile(referenceDir,'task_status.json'));
    assert(strcmp(status.state,'complete') && strcmp(status.taskHash,abfdHash(task)), ...
        'ABFD:ValidationTask','Native task freeze differs.');
    attempts=readAbfdJsonUtf8(fullfile(referenceDir,'proposal_attempts.json'));
    assert(attempts(1).start==1 && attempts(1).passed,'ABFD:ValidationStart','First original start was not accepted.');
    p=buildDynamicsProvider(task.model); c=frozen.c;
    expected=abfdHash(struct('task',task,'config',rmfield(c,'outputDir'),'start',1, ...
        'providerHash',p.hash,'implementationHash',frozen.manifest.generatorHash));
    assert(strcmp(saved.inputHash,expected),'ABFD:ValidationInput','Original planner checkpoint provenance differs.');
    planning=saved.result;
    % Original F1 checkpoints predate solverState; their persisted contract
    % requires successful termination, constraint residuals and a dense certificate.
    assert(planning.passed && strcmp(planning.solverStatus,'Solve_Succeeded') && ...
        all(isfinite([planning.maxEq,planning.maxIneq])) && ...
        planning.maxEq<=c.constraintTolerance && planning.maxIneq<=c.constraintTolerance && ...
        planning.denseReport.passed && planning.bundle.publishPassed && ...
        isequaln(planning.bundle.model,task.model) && isequaln(planning.bundle.scene,task.scene), ...
        'ABFD:ValidationReference','Saved training trajectory is not accepted for this task.');
    if isfield(planning,'solverState')
        assert(planning.solverState.stats.success,'ABFD:ValidationSolver','Saved solver state contradicts successful termination.');
    end
    bundle=buildAbfdReferenceBundle(planning.bundle.refs.hs,task.model,task.scene,planning.disc,p,c);
    save(fullfile(outputDir,'reference_bundle.mat'),'bundle');
    modelFile=fullfile(root,'matlab','stewart_platform_model.slx'); originalModelHash=abfdFileHash(modelFile);
    files={'research/integration/geometricTvlqrSimscapeSfun.m','research/integration/stepAbfdIntervalObserver.m', ...
        'research/integration/installAbfdSimscapeRuntime.m','research/integration/readAbfdSimscapeLog.m', ...
        'research/integration/stepAbfdController.m','research/integration/makeAbfdRuntime.m', ...
        'research/integration/stepAbfdIntervalObserverFused.m','research/integration/buildAbfdIntervalObserverKernel.m', ...
        'research/integration/prepareAbfdSimscape.m','research/common/makeAbfdConfig.m'};
    writeAbfdJson(fullfile(outputDir,'freeze.json'),struct('taskId',task.taskId,'taskHash',abfdHash(task), ...
        'referenceHash',abfdFileHash(fullfile(outputDir,'reference_bundle.mat')), ...
        'modelFileHash',originalModelHash,'sourceFiles',{files}, ...
        'sourceHashes',{cellfun(@(file)abfdFileHash(fullfile(root,file)),files,'UniformOutput',false)}, ...
        'comparison',comparison,'scope','Training-side repair validation; same reference, nominal LQI, matched object and force budget; no test tuning'));
    names={'direct','external-observer'};
    if strcmp(comparison,'kernel'), names={'interval-rk4-v1','fused-rk4-v1'}; end
    records=struct('stateSource',{},'observerKernel',{},'state',{},'passed',{},'metrics',{},'peakEstimationError',{},'reason',{});
    logs=cell(1,2);
    for index=1:numel(names)
        local=c; local.modelMode='analytic'; local.controllerMode='baseline-lqi'; local.designMode='sequential';
        local.plannerMode='ihsid'; local.stateSource=names{index}; local.plantProfile='matched'; local.plantOverrides=struct();
        kernel='interval-rk4-v1';
        if isfield(local,'observerKernel'), local=rmfield(local,'observerKernel'); end
        if strcmp(comparison,'kernel')
            kernel=names{index}; local.stateSource='external-observer'; local.observerKernel=kernel;
        end
        local.outputDir=fullfile(outputDir,names{index}); local=makeAbfdConfig(local); mkdir(local.outputDir);
        profile=makeHighFidelityPlantProfile('matched');
        try
            result=runAbfdSimscapeCase(bundle,p,local,profile); save(fullfile(local.outputDir,'result.mat'),'result','local');
            logs{index}=result.log;
            error=result.log.estimatedState-[result.log.q;result.log.qd];
            peak=max(abs(error),[],2);
            records(end+1)=struct('stateSource',local.stateSource,'observerKernel',kernel,'state','complete','passed',result.passed, ...
                'metrics',result.metrics,'peakEstimationError',peak,'reason',strjoin(result.acceptance.reasons,', ')); %#ok<AGROW>
        catch e
            if bdIsLoaded('stewart_platform_model'), close_system('stewart_platform_model',0); end
            records(end+1)=struct('stateSource',local.stateSource,'observerKernel',kernel,'state','failed','passed',false, ...
                'metrics',struct(),'peakEstimationError',[],'reason',getReport(e,'extended','hyperlinks','off')); %#ok<AGROW>
            writeAbfdJson(fullfile(local.outputDir,'failure.json'),struct('identifier',e.identifier,'report',records(end).reason));
        end
        assert(strcmp(originalModelHash,abfdFileHash(modelFile)) && ~bdIsLoaded('stewart_platform_model'), ...
            'ABFD:ModelRestore','The source model must remain unchanged and closed.');
        writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','records',records,'seconds',toc(timer)));
    end
    equivalence=struct('applicable',false,'passed',true);
    if strcmp(comparison,'kernel'), equivalence=compareKernels(logs); end
    report=struct('state','complete','passed',all([records.passed]) && equivalence.passed, ...
        'records',records,'seconds',toc(timer),'comparison',comparison,'equivalence',equivalence, ...
        'modelFileUnchanged',true,'scope','Full-duration original training reference and matched Simscape; declared state-source or kernel pair; not the full experiment matrix');
    writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    failure=struct('state','failed','identifier',e.identifier,'report',getReport(e,'extended','hyperlinks','off'));
    writeAbfdJson(fullfile(outputDir,'failure.json'),failure);
    writeAbfdJson(fullfile(outputDir,'status.json'),failure);
    rethrow(e);
end
end
function report=compareKernels(logs)
report=struct('applicable',true,'passed',false,'reason','Both complete simulation logs are required.');
if any(cellfun(@isempty,logs)), return; end
a=logs{1}; b=logs{2};
if ~isequal(size(a.t),size(b.t)) || max(abs(a.t-b.t))>1e-12
    report.reason='Simulation time grids differ; do not interpolate away a discrepancy.'; return;
end
% Numerical equivalence tolerances only; original physical and timing gates
% independently apply to BOTH runs. Values are not selected from test results.
fields={'q','qd','estimatedState','actualForce','commandForce','feedbackForce','measuredForceIntegral'};
limits=[1e-8,1e-8,1e-8,1e-5,1e-5,1e-5,1e-5]; differences=zeros(size(limits));
for k=1:numel(fields)
    left=a.(fields{k}); right=b.(fields{k});
    if ~isequal(size(left),size(right)) || ~all(isfinite([left(:);right(:)]))
        report.reason=['Missing, nonfinite or mismatched log: ',fields{k}]; return;
    end
    differences(k)=max(abs(left(:)-right(:)));
end
report=struct('applicable',true,'passed',all(differences<=limits), ...
    'fields',{fields},'maxAbsoluteDifferences',differences,'tolerances',limits, ...
    'timeSamples',numel(a.t),'reason','Full logged trajectories on identical time grids; each run also retains the original independent acceptance.');
end
