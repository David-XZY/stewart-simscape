function records=run_abfd_matrix(tasks,base,caseOverrides,taskOverrides)
% Explicit opt-in formal run; pending, execution failure and rejection differ.
if nargin<3, caseOverrides=struct(); end
if nargin<4, taskOverrides=struct('taskId',{},'caseOverrides',{}); end
assert(isstruct(tasks) && ~isempty(tasks) && isfield(tasks,'taskId'),'ABFD:MatrixTasks','A nonempty task list is required.');
assert(numel(unique({tasks.taskId}))==numel(tasks),'ABFD:MatrixTasks','Duplicate task identifiers.');
assert(isstruct(taskOverrides) && all(isfield(taskOverrides,{'taskId','caseOverrides'})), ...
    'ABFD:MatrixTaskOverrides','Task overrides require taskId and caseOverrides.');
assert(all(ismember(fieldnames(taskOverrides),{'taskId','caseOverrides','plantInitializationFile'})), ...
    'ABFD:MatrixTaskOverrides','Unknown task-specific setting.');
assert(numel(unique({taskOverrides.taskId}))==numel(taskOverrides) && ...
    all(ismember({taskOverrides.taskId},{tasks.taskId})),'ABFD:MatrixTaskOverrides','Unknown or duplicate task overrides.');
matrix=buildAbfdExperimentMatrix(base,caseOverrides); matrices=cell(1,numel(tasks));
records=struct('taskId',{},'id',{},'status',{},'passed',{},'reason',{},'acceptance',{});
% Validate every case before starting any expensive experiment.
for i=1:numel(tasks)
    changes=caseOverrides; taskBase=base; selected=find(strcmp({taskOverrides.taskId},tasks(i).taskId));
    if ~isempty(selected)
        if isfield(taskOverrides,'plantInitializationFile')
            taskBase.plantInitializationFile=taskOverrides(selected).plantInitializationFile;
        end
        specific=taskOverrides(selected).caseOverrides;
        assert(isstruct(specific) && isscalar(specific) && ...
            all(ismember(fieldnames(specific),{matrix.id})),'ABFD:MatrixTaskOverrides','Unknown task-specific experiment.');
        ids=fieldnames(specific);
        for k=1:numel(ids)
            id=ids{k}; extra=specific.(id);
            assert(isstruct(extra) && isscalar(extra),'ABFD:MatrixTaskOverrides','Each task-specific override must be scalar.');
            if ~isfield(changes,id), changes.(id)=struct(); end
            names=fieldnames(extra);
            for n=1:numel(names), changes.(id).(names{n})=extra.(names{n}); end
        end
    end
    matrices{i}=buildAbfdExperimentMatrix(taskBase,changes);
    for j=1:numel(matrix)
        records(end+1)=struct('taskId',tasks(i).taskId,'id',matrix(j).id, ...
            'status','pending','passed',false,'reason','','acceptance',struct()); %#ok<AGROW>
    end
end
writeAbfdJson(fullfile(base.outputDir,'matrix_status.json'),records);
for i=1:numel(tasks)
    baselineFile=''; baselineProfileHash=''; matrix=matrices{i};
    for j=1:numel(matrix)
        row=(i-1)*numel(matrix)+j; c=matrix(j).config;
        c.outputDir=fullfile(c.outputDir,tasks(i).taskId);
        file=fullfile(c.outputDir,'result.mat'); manifestFile=fullfile(c.outputDir,'manifest.json');
        records(row).status='running'; writeAbfdJson(fullfile(base.outputDir,'matrix_status.json'),records);
        try
            if strcmp(matrix(j).id,'EB')
                assert(~isempty(baselineFile),'ABFD:MissingBaseline','EB requires the same accepted E0 trajectory.');
                c.trajectoryFile=baselineFile;
            end
            profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
            [~,c,profile]=buildAbfdProviderForTask(tasks(i).model,c,profile,tasks(i).scene.q0);
            if strcmp(matrix(j).id,'EB')
                assert(strcmp(abfdHash(profile),baselineProfileHash),'ABFD:MatrixPlantChanged', ...
                    'B conditioning changed the actual object or initial actuator force. Supply the same pre-task physical initialization to E0 and EB.');
            end
            if isfile(file) || isfile(manifestFile)
                assert(isfile(file) && isfile(manifestFile),'ABFD:IncompleteCheckpoint', ...
                    'Incomplete prior run retained; use a fresh output directory.');
                previous=readAbfdJsonUtf8(manifestFile);
                assert(strcmp(previous.status,'experiment-completed') && strcmp(previous.configHash,abfdHash(c)) && ...
                    isfield(previous.details,'fingerprint') && strcmp(previous.details.fingerprint,abfdRunFingerprint(tasks(i),c)), ...
                    'ABFD:CheckpointMismatch','Existing result has different configuration or incomplete execution.');
                assert(isfield(previous.details,'outputHashes') && ...
                    strcmp(previous.details.outputHashes.result,abfdFileHash(file)) && ...
                    strcmp(previous.details.outputHashes.reference,abfdFileHash(fullfile(c.outputDir,'reference_bundle.mat'))), ...
                    'ABFD:CheckpointArtifacts','Result and accepted reference must match their recorded hashes.');
                saved=load(file,'result'); result=saved.result;
                assert(isfield(previous.details,'metrics') && strcmp(abfdHash(previous.details.metrics),abfdHash(result.metrics)), ...
                    'ABFD:CheckpointMetrics','Saved metrics differ from the completion manifest.');
                status='resumed';
            else
                result=runAbfdExperiment(tasks(i),c); status='experiment-completed';
            end
            acceptance=assessAbfdExperimentMetrics(result.metrics,c);
            records(row).status=status; records(row).passed=acceptance.passed;
            records(row).acceptance=acceptance; records(row).reason=strjoin(acceptance.reasons,', ');
            if strcmp(matrix(j).id,'E0')
                baselineFile=fullfile(c.outputDir,'reference_bundle.mat');
                assert(isfile(baselineFile),'ABFD:MissingBaseline','E0 completion has no accepted reference bundle.');
                baselineProfileHash=abfdHash(result.profile);
            end
        catch e
            records(row).status='failed'; records(row).passed=false;
            records(row).reason=[e.identifier,': ',e.message];
        end
        writeAbfdJson(fullfile(base.outputDir,'matrix_status.json'),records);
    end
end
end
