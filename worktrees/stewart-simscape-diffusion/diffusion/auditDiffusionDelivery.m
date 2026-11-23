function audit=auditDiffusionDelivery(runName)
% Artifact-based completion, independent from the process exit status.
c=setupDiffusion(); folder=fullfile(c.output,runName);
s=load(fullfile(folder,'manifest.mat')); manifest=s.manifest;
tasks=manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
modelFile=fullfile(folder,'training','model.json');
asset=jsondecode(fileread(modelFile)); checkNetworkExport(modelFile);
assert(strcmp(asset.status,'trained') && strcmp(asset.manifestHash,manifest.hash),'DP:Audit','Model provenance mismatch.');
methods={'traditional','diffusion','guided_diffusion'};
success=struct('simple',false,'hood',false); complete=true; paired=[];
offlineSeconds=0; attempts=0; acceptedLabels=0;
for i=1:numel(tasks)
    task=tasks{i};
    for k=1:8
        file=fullfile(folder,'attempts',task.taskId,sprintf('start_%02d.mat',k));
        if ~isfile(file), complete=false; continue; end
        v=load(file,'record'); attempts=attempts+1; acceptedLabels=acceptedLabels+v.record.passed;
        offlineSeconds=offlineSeconds+v.record.seconds;
    end
    if ~strcmp(task.split,'test'), continue; end
    for j=1:3
        file=fullfile(folder,'benchmark',task.taskId,methods{j},'benchmark.mat');
        if ~isfile(file), complete=false; continue; end
        v=load(file);
        assert(strcmp(v.task.hash,task.hash),'DP:Audit','Benchmark task mismatch.');
        if j==3 && v.row.passed
            assert(v.outcome.best.passed && v.outcome.best.report.passed,'DP:Audit','Success flag has no accepted trajectory.');
            success.(task.family)=true;
        end
    end
    plainFile=fullfile(folder,'benchmark',task.taskId,'diffusion','benchmark.mat');
    guidedFile=fullfile(folder,'benchmark',task.taskId,'guided_diffusion','benchmark.mat');
    if ~isfile(plainFile) || ~isfile(guidedFile), continue; end
    plain=load(plainFile); guided=load(guidedFile);
    valid=~isempty(plain.generation) && ~isempty(guided.generation) && size(guided.candidates,3)==8;
    if ~valid, complete=false; continue; end
    sameNoise=isequal(plain.generation.initialNoise,guided.generation.initialNoise);
    sameWeights=strcmp(plain.generation.assetHash,guided.generation.assetHash);
    delta=max(abs(plain.candidates(:)-guided.candidates(:)));
    updates=guided.generation.updates; nAccepted=sum([updates.accepted]);
    monotone=all([updates([updates.accepted]).after]<[updates([updates.accepted]).before]);
    row=struct('taskId',task.taskId,'family',task.family,'sameInitialNoise',sameNoise, ...
        'sameWeights',sameWeights,'acceptedGuidanceUpdates',nAccepted,'acceptedCostsDecrease',monotone, ...
        'maxControlDifference',delta,'maxPositionRmsM',guided.diversity.maxPositionRmsM, ...
        'maxAttitudeRmsRad',guided.diversity.maxAttitudeRmsRad);
    if isempty(paired), paired=row; else, paired(end+1)=row; end %#ok<AGROW>
end
guidanceVerified=~isempty(paired) && all([paired.sameInitialNoise]) && all([paired.sameWeights]) && ...
    all([paired.acceptedCostsDecrease]) && any([paired.acceptedGuidanceUpdates]>0 & [paired.maxControlDifference]>1e-12);
media=true;
for family={'simple','hood'}
    media=media && isfile(fullfile(folder,'figures',[family{1},'_candidates.png'])) && ...
        isfile(fullfile(folder,'figures',[family{1},'_constraints.png'])) && isfile(fullfile(folder,'figures',[family{1},'.mp4']));
end
audit=struct('schemaVersion',1,'runName',runName,'manifestHash',manifest.hash,'trainedWeights',true, ...
    'attempts',attempts,'acceptedLabels',acceptedLabels,'offlineAttemptWallSecondsSum',offlineSeconds, ...
    'allScheduledRecordsExist',complete,'guidedHeldoutSuccess',success,'pairedGuidance',paired, ...
    'guidanceVerified',guidanceVerified,'mediaFilesExist',media, ...
    'numericalDeliveryPassed',complete && success.simple && success.hood && guidanceVerified && media, ...
    'visualReviewRequired',true,'scope','Dense numerical checks; candidate curve distinction requires visual review.');
dpWriteJson(fullfile(folder,'delivery_audit.json'),audit);
if ~isempty(paired), writetable(struct2table(paired),fullfile(folder,'paired_guidance.csv')); end
fprintf('DELIVERY numerical=%d labels=%d/%d simple=%d hood=%d guidance=%d\n', ...
    audit.numericalDeliveryPassed,acceptedLabels,attempts,success.simple,success.hood,guidanceVerified);
end
