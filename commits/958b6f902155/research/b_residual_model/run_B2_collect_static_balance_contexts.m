function report=run_B2_collect_static_balance_contexts(originalRoot,replayRoot,inertiaRoot,dataRoot,assetFile,outputDir)
% Pre-task observations for a separately declared conditional model. The
% same fixed observation rule applies to every split; no evaluation motion
% sample or held-out parameter fit is used to estimate the starting bias.
setupAbfdResearch();
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:BalanceContextExists','Use a fresh output directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
dataFile=fullfile(dataRoot,'residual_dataset.json'); data=jsondecode(fileread(dataFile));
verified=jsondecode(fileread(fullfile(dataRoot,'delivery','delivery_verification.json')));
assetHash=abfdFileHash(assetFile);
assert(verified.passed && strcmp(verified.datasetSha256,abfdFileHash(dataFile)) && ...
    strcmp(verified.assetSha256,assetHash),'ABFD:BalanceContextAsset','Use the independently verified trained asset and dataset.');
tasksFile=fullfile(originalRoot,'B_tasks_frozen.json'); tasks=jsondecode(fileread(tasksFile));
rows=struct([]); cases=data.pairedSourceCases;
report=struct('state','running','passed',false,'completed',0,'expected',numel(cases), ...
    'staticAssetHash',assetHash,'baseDatasetHash',abfdFileHash(dataFile),'tasksHash',abfdFileHash(tasksFile), ...
    'protocol','independent-initial-static-balance-v1', ...
    'scope','Observable pre-task static balance context; weights and physical coefficients frozen; no held-out motion used to estimate bias');
writeAbfdJson(fullfile(outputDir,'status.json'),report);
try
    for k=1:numel(cases)
        source=cases(k); task=tasks(strcmp({tasks.taskId},source.taskId));
        assert(isscalar(task) && strcmp(task.split,source.split),'ABFD:BalanceContextSplit','Frozen split differs.');
        originalFolder=fullfile(originalRoot,'B0',source.taskId);
        originalFile=fullfile(originalFolder,'result.mat');
        assert(strcmp(abfdFileHash(originalFile),source.sourceHash),'ABFD:BalanceContextSource','Original simulation source changed.');
        original=load(originalFile,'result'); bundle=original.result.planning.bundle; profile=original.result.profile;
        manifest=jsondecode(fileread(fullfile(originalFolder,'manifest.json'))); c=makeAbfdConfig(manifest.config);
        c.outputDir=fullfile(outputDir,source.taskId); timer=tic;
        calibrationFile=fullfile(inertiaRoot,source.taskId,'status.json');
        reused=strcmp(task.split,'train') && isfile(calibrationFile);
        if ~reused
            calibrateAbfdStaticPreload(bundle,c,profile,c.outputDir);
            calibrationFile=fullfile(c.outputDir,'status.json');
        end
        applyAbfdStaticPreload(profile,bundle,calibrationFile);
        calibration=jsondecode(fileread(calibrationFile));
        % Only the measured equilibrium force and initial pose are context.
        % Do not identify inertia from held-out response matrices here.
        base=buildDynamicsProvider(task.model,'residual-B',assetFile,false,calibration.pose(:));
        conditioned=conditionResidualMeanOnStaticBalance(base,bundle,profile,calibrationFile);
        replayFile=fullfile(replayRoot,source.taskId,'result.mat');
        assert(strcmp(abfdFileHash(replayFile),source.resultHash),'ABFD:BalanceContextSource','Frozen replay changed.');
        replay=load(replayFile,'result');
        assert(norm(replay.result.log.q(:,1)-calibration.pose(:),inf)<1e-10, ...
            'ABFD:BalanceContextPose','Independent context and trajectory start at different poses.');
        row=struct('taskId',source.taskId,'split',source.split,'initialPose',calibration.pose(:), ...
            'staticForce',calibration.equilibriumForce(:),'wrenchOffset',conditioned.biasCalibration.wrenchOffset, ...
            'calibrationFile',calibrationFile,'calibrationHash',abfdFileHash(calibrationFile), ...
            'plantHash',calibration.plantHash,'sourceHash',source.resultHash,'baseProviderHash',base.hash, ...
            'conditionedProviderHash',conditioned.hash,'collectionSeconds',toc(timer),'reusedObservation',reused);
        if isempty(rows), rows=row; else, rows(k)=row; end
        report.completed=k; report.cases=rows;
        save(fullfile(outputDir,'contexts_checkpoint.mat'),'rows','report');
        writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    assert(strcmp(assetHash,abfdFileHash(assetFile)),'ABFD:BalanceContextAsset','Weights changed during context collection.');
    data.initialBalanceContexts=rows;
    data.conditioning=struct('kind',report.protocol,'staticAssetHash',assetHash, ...
        'baseDatasetHash',report.baseDatasetHash,'sourceHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
        'observation','Independent initial static equilibrium, before the task motion; no future-state fitting', ...
        'scope',report.scope);
    writeAbfdJson(fullfile(outputDir,'context_dataset.json'),data);
    report.state='complete'; report.passed=true;
    report.contextDatasetHash=abfdFileHash(fullfile(outputDir,'context_dataset.json'));
    writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
