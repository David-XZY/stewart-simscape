function dataset=run_B0_collect_dataset(tasks,c)
sessions=cell(1,numel(tasks)); splits=struct('taskId',{},'split',{});
attempts=struct('taskId',{},'status',{},'reason',{},'engineeringPassed',{},'budgetPassed',{});
hashes=arrayfun(@(t)string(abfdHash(t.model)),tasks);
assert(isscalar(unique(hashes)),'ABFD:MixedNominalModels', ...
    'Collect B tasks with one nominal model; use makeAbfdTaskManifest(''residual'').');
for k=1:numel(tasks)
    try
    local=c; local.modelMode='analytic'; local.plannerMode='ihsid'; local.designMode='sequential';
    local.plantProfile='high-fidelity'; local.outputDir=fullfile(c.outputDir,tasks(k).taskId);
    if isfield(tasks(k),'plantOverrides'), local.plantOverrides=tasks(k).plantOverrides; end
    savedFile=fullfile(local.outputDir,'result.mat'); manifestFile=fullfile(local.outputDir,'manifest.json');
    if isfile(savedFile) && isfile(manifestFile)
        manifest=jsondecode(fileread(manifestFile));
        assert(strcmp(manifest.details.fingerprint,abfdRunFingerprint(tasks(k),local)), ...
            'ABFD:CheckpointMismatch','B0 checkpoint inputs changed.');
        saved=load(savedFile,'result'); result=saved.result;
    else
        result=runAbfdExperiment(tasks(k),local);
    end
    quality=validateResidualCollectionRun(result,tasks(k).model,c.constraintTolerance);
    sessions{k}=result.log;
    splits(k)=struct('taskId',tasks(k).taskId,'split',tasks(k).split);
    attempts(k)=struct('taskId',tasks(k).taskId,'status','collected','reason','', ...
        'engineeringPassed',quality.engineeringPassed,'budgetPassed',quality.budgetPassed);
    catch e
        attempts(k)=struct('taskId',tasks(k).taskId,'status','failed','reason',e.message, ...
            'engineeringPassed',false,'budgetPassed',false);
        splits(k)=struct('taskId',tasks(k).taskId,'split',tasks(k).split);
    end
    writeAbfdJson(fullfile(c.outputDir,'collection_status.json'),attempts);
end
valid=~cellfun(@isempty,sessions);
assert(any(valid),'ABFD:NoResidualSessions','No valid actual-force sessions were collected.');
assert(all(ismember({'train','validation','test'},{splits(valid).split})), ...
    'ABFD:MissingDatasetSplit','Collected sessions must cover all three frozen splits.');
dataset=exportResidualDataset(sessions(valid),tasks(1).model,fullfile(c.outputDir,'residual_dataset.json'),splits(valid));
dataset.collectionAcceptance=attempts;
dataset.collectionPolicy='Measured states must be collision-free, in travel and force range, and nonsingular. Control engineering and feedback-budget results are retained separately.';
writeAbfdJson(fullfile(c.outputDir,'residual_dataset.json'),dataset);
end
