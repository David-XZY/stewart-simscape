function report=run_B0_replay_direct_acceleration(sourceRoot,outputDir,resume)
% Replay only the original accepted collection cohort with a direct sensor.
% Freeze trajectories, controllers, physical plants and all task splits.
% Original rejected tasks remain in the manifest and are never replaced.
setupAbfdResearch();
if nargin<3, resume=false; end
validateattributes(resume,{'logical'},{'scalar'});
statusFile=fullfile(outputDir,'status.json');
assert(~isfile(statusFile) || resume,'ABFD:ReplayExists','Use a fresh run or explicitly resume checked inputs.');
if ~isfolder(outputDir), mkdir(outputDir); end
tasks=jsondecode(fileread(fullfile(sourceRoot,'B_tasks_frozen.json')));
original=jsondecode(fileread(fullfile(sourceRoot,'B0','collection_status.json')));
accepted=strcmp({original.status},'collected'); selected=original(accepted);
assert(numel(original)==numel(tasks) && all(ismember({selected.taskId},{tasks.taskId})), ...
    'ABFD:ReplayManifest','Original collection must match the frozen task manifest.');
report=struct('state','running','passed',false,'completed',0,'expected',numel(selected), ...
    'sourceDatasetHash',abfdFileHash(fullfile(sourceRoot,'B0','residual_dataset.json')), ...
    'frozenTasksHash',abfdFileHash(fullfile(sourceRoot,'B_tasks_frozen.json')), ...
    'originalCollection',original,'cases',[],'scope', ...
    'Paired direct-acceleration replay of the original accepted cohort; original failures remain in the denominator');
if isfile(statusFile)
    previous=jsondecode(fileread(statusFile));
    assert(strcmp(previous.sourceDatasetHash,report.sourceDatasetHash) && ...
        strcmp(previous.frozenTasksHash,report.frozenTasksHash) && strcmp(previous.state,'failed'), ...
        'ABFD:ReplayResume','Only a failed replay with identical frozen inputs may resume.');
    stamp=char(datetime('now','TimeZone','UTC','Format','yyyyMMdd''T''HHmmssSSS'));
    writeAbfdJson(fullfile(outputDir,['previous_status_',stamp,'.json']),previous);
end
writeAbfdJson(fullfile(outputDir,'status.json'),report);
sessions=cell(1,numel(selected)); splits=struct('taskId',{},'split',{}); rows=struct([]);
try
    for k=1:numel(selected)
        id=selected(k).taskId; index=find(strcmp({tasks.taskId},id)); assert(isscalar(index));
        task=tasks(index); source=fullfile(sourceRoot,'B0',id);
        saved=load(fullfile(source,'result.mat'),'result'); old=saved.result;
        manifest=jsondecode(fileread(fullfile(source,'manifest.json')));
        c=makeAbfdConfig(manifest.config); c.outputDir=fullfile(outputDir,id);
        assert(strcmp(c.modelMode,'analytic') && strcmp(c.controllerMode,'fullstate-tvlqr') && ...
            strcmp(c.designMode,'sequential'),'ABFD:ReplayControl','Expected original analytic TVLQR data collection.');
        bundle=old.planning.bundle; provider=buildDynamicsProvider(task.model);
        assert(strcmp(abfdHash(bundle.model),abfdHash(task.model)), ...
            'ABFD:ReplayModel','Frozen task and saved reference model differ.');
        validateResidualCollectionRun(old,task.model,c.constraintTolerance);
        checkpoint=fullfile(c.outputDir,'result.mat');
        if resume && isfile(checkpoint)
            replay=load(checkpoint);
            assert(strcmp(abfdHash(replay.c),abfdHash(c)) && strcmp(abfdHash(replay.bundle),abfdHash(bundle)) && ...
                strcmp(abfdHash(replay.result.profile),abfdHash(old.profile)), ...
                'ABFD:ReplayCheckpoint','Saved configuration, reference or plant differs.');
            result=replay.result; log=result.log; historical=replay.historical;
        else
            setup=prepareAbfdSimscape(bundle,provider,c,old.profile);
            cleanup=onCleanup(@()close_system(setup.modelName,0));
            interface=installAbfdAccelerationProbe(setup.modelName);
            simulation=sim(setup.modelName,'ReturnWorkspaceOutputs','on');
            historical=readAbfdSimscapeLog(simulation.get('simout'),setup);
            log=readAbfdDirectAcceleration(simulation,historical);
            log.taskId=old.log.taskId; log.loadId=old.log.loadId; log.trajectoryId=old.log.trajectoryId;
            metrics=evaluateUnifiedMetrics(log,bundle,c,[]);
            result=struct('log',log,'metrics',metrics,'profile',old.profile,'interface',interface, ...
                'status','experiment-completed','passed',metrics.engineeringPassed && metrics.budgetPassed);
            if ~isfolder(c.outputDir), mkdir(c.outputDir); end
            save(checkpoint,'result','bundle','c','historical');
            clear cleanup;
        end
        assert(isequal(log.t,old.log.t),'ABFD:ReplayClock','Replay changed the source sample clock.');
        poseDifference=max(abs(log.q(:)-old.log.q(:)));
        velocityDifference=max(abs(log.qd(:)-old.log.qd(:)));
        forceDifference=max(abs(log.actualForce(:)-old.log.actualForce(:)));
        assert(poseDifference<=1e-6 && velocityDifference<=1e-5 && forceDifference<=.01, ...
            'ABFD:ReplayMotion','Replay changed the original motion/force beyond declared numerical tolerances.');
        quality=validateResidualCollectionRun(result,task.model,c.constraintTolerance);
        sessions{k}=log; splits(k)=struct('taskId',id,'split',task.split);
        row=struct('taskId',id,'split',task.split,'samples',numel(log.t), ...
            'sourceHash',abfdFileHash(fullfile(source,'result.mat')), ...
            'resultHash',abfdFileHash(fullfile(c.outputDir,'result.mat')), ...
            'poseDifference',poseDifference,'velocityDifference',velocityDifference,'forceDifference',forceDifference, ...
            'accelerationDifferenceRms',sqrt(mean((log.qdd-historical.qdd).^2,2)), ...
            'collectionPassed',quality.passed,'engineeringPassed',quality.engineeringPassed,'budgetPassed',quality.budgetPassed);
        if isempty(rows), rows=row; else, rows(k)=row; end
        report.cases=rows; report.completed=k;
        writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    assert(all(ismember({'train','validation','test'},{splits.split})), ...
        'ABFD:MissingDatasetSplit','All original splits are required for export.');
    report.state='fitting-physical'; writeAbfdJson(fullfile(outputDir,'status.json'),report);
    rawFile=fullfile(outputDir,'residual_dataset_without_time.json');
    data=exportResidualDataset(sessions,tasks(1).model,rawFile,splits);
    previous=jsondecode(fileread(fullfile(sourceRoot,'B0','residual_dataset.json')));
    assert(isequal(string(data.groups),string(previous.groups)) && isequal(string(data.split),string(previous.split)), ...
        'ABFD:ReplayGroups','Replay changed original rows, group order or split assignment.');
    data.collectionAcceptance=original;
    data.accelerationSource='Independent Simscape world linear/angular acceleration transformed to generalized coordinates';
    data.collectionPolicy='Original accepted cohort, fixed reference/controller/plant, direct acceleration replay; no replacement of failed tasks';
    data.pairedSourceDatasetHash=report.sourceDatasetHash; data.pairedSourceCases=rows;
    writeAbfdJson(rawFile,data);
    timed=attachResidualDatasetTimes(rawFile,outputDir,fullfile(outputDir,'residual_dataset.json'));
    report.state='complete'; report.passed=true; report.datasetHash=timed.timedDatasetHash;
    report.samples=timed.rows; writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    report.detail=getReport(e,'extended','hyperlinks','off'); writeAbfdJson(fullfile(outputDir,'status.json'),report);
    rethrow(e);
end
end
