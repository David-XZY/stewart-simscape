function report=verifyInertiaRelabeling(parentFile,replayRoot,outputDir,tasksFile)
% Independently reproduce every target change using the linear inertial
% regressor, rather than the composite wrench routine used by relabeling.
parent=jsondecode(fileread(parentFile)); file=fullfile(outputDir,'residual_dataset.json');
data=jsondecode(fileread(file)); state=jsondecode(fileread(fullfile(outputDir,'status.json')));
tasks=jsondecode(fileread(tasksFile)); model=tasks(1).model;
assert(strcmp(abfdFileHash(tasksFile),state.tasksHash),'ABFD:InertiaDatasetHash','Frozen task manifest differs.');
assert(strcmp(state.state,'complete') && state.passed && state.completed==state.expected, ...
    'ABFD:InertiaDatasetIncomplete','Physical identification has no accepted terminal dataset.');
assert(strcmp(state.datasetHash,abfdFileHash(parentFile)) && strcmp(state.newDatasetHash,abfdFileHash(file)) && ...
    strcmp(data.parentDatasetHash,state.datasetHash),'ABFD:InertiaDatasetHash','Dataset provenance differs.');
fields={'x','groups','split','time','pairedSourceCases','collectionAcceptance','referencePhysical'};
for k=1:numel(fields)
    assert(isequal(parent.(fields{k}),data.(fields{k})),'ABFD:InertiaDatasetRows', ...
        'Relabeling changed a frozen field: %s.',fields{k});
end
groups=unique(string(data.groups(strcmp(string(data.split),'train'))));
assert(isequal(sort(groups),sort(string(data.physical.trainingGroups(:)))) && ...
    isequal(sort(groups),sort(string({state.cases.taskId}).')) && all(strcmp({state.cases.split},'train')), ...
    'ABFD:TrainingLeakage','Physical identification groups must exactly match the frozen training groups.');
betaChange=parameters(parent.physical)-parameters(data.physical);
maxError=0; cursor=0; cases=parent.pairedSourceCases;
for k=1:numel(cases)
    source=fullfile(replayRoot,cases(k).taskId,'result.mat');
    assert(strcmp(abfdFileHash(source),cases(k).resultHash),'ABFD:InertiaReplayChanged','Replay source changed.');
    saved=load(source,'result');
    samples=buildResidualWrenchTarget(saved.result.log,model); n=size(samples.x,1); ids=cursor+(1:n);
    assert(isequal(samples.x,data.x(ids,:)),'ABFD:InertiaDatasetRows','Replay state rows differ.');
    for j=1:n
        x=samples.x(j,:); Y=buildInertialWrenchRegressor(x(1:6).',x(7:12).',samples.acceleration(j,:).',model);
        expected=parent.y(ids(j),:)+(Y*betaChange).';
        maxError=max(maxError,max(abs(expected-data.y(ids(j),:))));
    end
    cursor=cursor+n;
end
assert(cursor==size(data.y,1) && all(isfinite(data.y(:))) && maxError<1e-7, ...
    'ABFD:InertiaRelabeling','Independent inertial target changes differ.');
report=struct('passed',true,'rows',cursor,'trainingGroups',numel(groups), ...
    'maxIndependentTargetError',maxError,'parentDatasetHash',state.datasetHash, ...
    'datasetHash',state.newDatasetHash,'sourceHash',abfdFileHash([mfilename('fullpath'),'.m']), ...
    'scope','Frozen rows and train-only physical provenance; independent target algebra, not model performance acceptance');
end
function beta=parameters(p)
c=p.com(:); origin=p.inertia+p.mass*((c.'*c)*eye(3)-c*c.');
beta=[p.mass;p.mass*c;diag(origin);origin(1,2);origin(1,3);origin(2,3)];
end
