function report=run_B0_identify_training_inertia(originalRoot,replayRoot,outputDir,resume)
% Preserve the original 20 motion logs/splits; add identification probes only
% for the 11 collected train groups. Never calibrate a validation/test plant.
if nargin<4, resume=false; end
setupAbfdResearch();
assert(resume || ~isfile(fullfile(outputDir,'status.json')),'ABFD:IdentificationExists', ...
    'Use a new output directory or explicitly resume a failed checkpoint.');
if ~isfolder(outputDir), mkdir(outputDir); end
datasetFile=fullfile(replayRoot,'residual_dataset.json'); old=jsondecode(fileread(datasetFile));
tasks=jsondecode(fileread(fullfile(originalRoot,'B_tasks_frozen.json')));
cases=old.pairedSourceCases; training=cases(strcmp({cases.split},'train'));
assert(numel(training)==numel(unique(string(old.groups(strcmp(string(old.split),'train'))))), ...
    'ABFD:InertiaTrainingGroups');
report=struct('state','running','passed',false,'completed',0,'expected',numel(training), ...
    'datasetHash',abfdFileHash(datasetFile),'tasksHash',abfdFileHash(fullfile(originalRoot,'B_tasks_frozen.json')), ...
    'cases',[],'scope','Train-only independent inertia identification; no new held-out calibration data');
measurements=struct([]); rows=struct([]);
if resume
    previous=jsondecode(fileread(fullfile(outputDir,'status.json')));
    assert(strcmp(previous.state,'failed') && strcmp(previous.datasetHash,report.datasetHash) && ...
        strcmp(previous.tasksHash,report.tasksHash),'ABFD:InertiaResumeSource', ...
        'Resume requires a failed run with exactly the same frozen inputs.');
    checkpoint=load(fullfile(outputDir,'responses_checkpoint.mat'),'measurements','rows');
    measurements=checkpoint.measurements; rows=checkpoint.rows;
    assert(numel(measurements)==previous.completed && numel(rows)==previous.completed, ...
        'ABFD:InertiaResumeLayout','Checkpoint and terminal state counts differ.');
    for k=1:numel(measurements)
        assert(strcmp(measurements(k).taskId,training(k).taskId) && strcmp(measurements(k).split,'train') && ...
            strcmp(rows(k).sourceHash,training(k).sourceHash) && rows(k).passed && ...
            strcmp(measurements(k).calibrationHash,abfdFileHash(fullfile(outputDir,training(k).taskId,'status.json'))), ...
            'ABFD:InertiaResumeSource','A saved training response or source binding has changed.');
    end
    failureFile=fullfile(outputDir,['failure_before_resume_',abfdHash(previous),'.json']);
    if ~isfile(failureFile), writeAbfdJson(failureFile,previous); end
    report.completed=numel(measurements); report.cases=rows;
end
writeAbfdJson(fullfile(outputDir,'status.json'),report);
try
    for k=(report.completed+1):numel(training)
        id=training(k).taskId; task=tasks(strcmp({tasks.taskId},id)); assert(isscalar(task) && strcmp(task.split,'train'));
        source=fullfile(originalRoot,'B0',id); saved=load(fullfile(source,'result.mat'),'result'); original=saved.result;
        assert(strcmp(abfdFileHash(fullfile(source,'result.mat')),training(k).sourceHash),'ABFD:InertiaSourceChanged');
        manifest=jsondecode(fileread(fullfile(source,'manifest.json'))); c=makeAbfdConfig(manifest.config);
        folder=fullfile(outputDir,id); c.outputDir=folder; profile=original.profile; bundle=original.planning.bundle;
        calibration=calibrateAbfdStaticPreload(bundle,c,profile,folder);
        measurement=readMeasuredInertiaRegression(fullfile(folder,'status.json'),bundle,profile);
        measurement.taskId=id; measurement.split='train';
        if isempty(measurements), measurements=measurement; else, measurements(k)=measurement; end
        row=struct('taskId',id,'split','train','mass',measurement.mass,'rigidResidual',measurement.relativeRigidResidual, ...
            'calibrationHash',measurement.calibrationHash,'passed',calibration.passed,'sourceHash',training(k).sourceHash);
        if isempty(rows), rows=row; else, rows(k)=row; end
        save(fullfile(outputDir,'responses_checkpoint.mat'),'measurements','rows');
        report.completed=k; report.cases=rows; writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    model=tasks(1).model; fit=fitInertiaFromTrainingResponses(measurements,model);
    writeAbfdJson(fullfile(outputDir,'physical_fit.json'),fit);
    % Keep every original motion row and time. Only the fitted physical part
    % and consequently the residual training targets change.
    effective=model; effective.dynamics.totalMass=fit.mass; effective.dynamics.comP=fit.com;
    effective.dynamics.inertiaAtCOM_P=fit.inertia;
    output=old; output.physical=fit; output.y=zeros(size(old.y)); cursor=0;
    for k=1:numel(cases)
        id=cases(k).taskId; file=fullfile(replayRoot,id,'result.mat');
        assert(strcmp(abfdFileHash(file),cases(k).resultHash),'ABFD:InertiaReplayChanged');
        saved=load(file,'result'); data=buildResidualWrenchTarget(saved.result.log,model); n=size(data.x,1); ids=cursor+(1:n);
        assert(isequal(data.x,old.x(ids,:)) && all(strcmp(string(old.groups(ids)),id)),'ABFD:InertiaRows');
        for j=1:n
            output.y(ids(j),:)=data.wrench(j,:)-computeCompositeRequiredWrench(data.x(j,1:6).', ...
                data.x(j,7:12).',data.acceleration(j,:).',effective).';
        end
        cursor=cursor+n;
    end
    assert(cursor==size(old.x,1),'ABFD:InertiaRowCount');
    output.parentDatasetHash=report.datasetHash; output.physicalIdentificationProtocol=report.scope;
    output.targetDefinition='Actual-force world wrench minus independently identified train-only inertial wrench';
    writeAbfdJson(fullfile(outputDir,'residual_dataset.json'),output);
    report.state='complete'; report.passed=true; report.physical=fit; report.rows=cursor;
    report.newDatasetHash=abfdFileHash(fullfile(outputDir,'residual_dataset.json'));
    writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
