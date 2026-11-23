function dataset=run_B1_export_safe_prefix_dataset(tasks,sourceRoots,c,allowPartial)
% Reuse immutable actual Simscape logs; never rerun or reclassify the full run.
% Source roots are searched in explicit priority order within each frozen task.
if nargin<4, allowPartial=false; end
assert(iscell(sourceRoots) && ~isempty(sourceRoots),'ABFD:SourceRoots','Specify ordered source roots.');
hashes=arrayfun(@(t)string(abfdHash(t.model)),tasks);
assert(isscalar(unique(hashes)),'ABFD:MixedNominalModels','B1 requires one frozen nominal model.');
sessions={}; splits=struct('taskId',{},'split',{});
records=struct('taskId',{},'split',{},'status',{},'reason',{},'resultHash',{},'prefix',{});
pending=false;
for k=1:numel(tasks)
    task=tasks(k); file=''; manifest=[]; terminal=false;
    row=struct('taskId',task.taskId,'split',task.split,'status','missing','reason','', ...
        'resultHash','','prefix',[]);
    for j=1:numel(sourceRoots)
        folder=fullfile(sourceRoots{j},task.taskId); mf=fullfile(folder,'manifest.json');
        if ~isfile(mf), continue; end
        candidate=jsondecode(fileread(mf));
        if strcmp(candidate.status,'running'), continue; end
        terminal=true;
        if strcmp(candidate.status,'experiment-completed') && isfile(fullfile(folder,'result.mat'))
            file=fullfile(folder,'result.mat'); manifest=candidate; break;
        end
        if isfield(candidate.details,'message'), row.reason=candidate.details.message; end
    end
    try
        if isempty(file)
            if terminal, row.status='no-session'; else, pending=true; end
        else
            assert(strcmp(manifest.config.modelMode,'analytic') && strcmp(manifest.config.plantProfile,'high-fidelity'), ...
                'ABFD:CollectionConfiguration','B1 expects the declared analytic/high-fidelity collection protocol.');
            assert(strcmp(manifest.config.controllerMode,c.controllerMode) && strcmp(manifest.config.stateSource,c.stateSource), ...
                'ABFD:CollectionConfiguration','Source controller/state protocol differs.');
            assert(strcmp(manifest.details.fingerprint,abfdRunFingerprint(task,manifest.config)), ...
                'ABFD:CheckpointMismatch','Source run does not match the frozen task/configuration.');
            saved=load(file,'result'); result=saved.result;
            assert(strcmp(result.log.taskId,task.taskId) && strcmp(abfdHash(result.planning.bundle.model),abfdHash(task.model)), ...
                'ABFD:CheckpointMismatch','Source model or task identity differs.');
            [session,prefix]=extractResidualSafePrefix(result,result.planning.bundle,c);
            row.resultHash=abfdFileHash(file);
            sessions{end+1}=session; %#ok<AGROW>
            splits(end+1)=struct('taskId',task.taskId,'split',task.split); %#ok<AGROW>
            row.status='prefix-collected'; row.prefix=prefix;
        end
    catch e
        row.status='rejected'; row.reason=[e.identifier,': ',e.message];
    end
    records(end+1)=row; %#ok<AGROW>
    writeAbfdJson(fullfile(c.outputDir,'prefix_collection_status.json'),records);
end
assert(allowPartial || ~pending,'ABFD:CollectionPending','Frozen tasks are still running or have not been attempted.');
assert(~isempty(sessions) && all(ismember({'train','validation','test'},{splits.split})), ...
    'ABFD:MissingDatasetSplit','Prefix data must cover all three frozen group splits.');
dataset=exportResidualDataset(sessions,tasks(1).model,fullfile(c.outputDir,'residual_dataset.json'),splits);
dataset.scope='initial safe intervals of actual high-fidelity runs; full-run acceptance remains separate';
dataset.partial=allowPartial; dataset.frozenTaskCount=numel(tasks);
dataset.collectionPolicy='initial-safe-prefix-v1'; dataset.collectionSessions=records;
% Preserve the original clock for temporal residual calibration. These rows
% follow exactly the same session order used by exportResidualDataset.
times=cellfun(@(session)session.t(:),sessions,'UniformOutput',false);
dataset.time=vertcat(times{:});
assert(numel(dataset.time)==size(dataset.x,1),'ABFD:DatasetTimeShape','Timestamp row count differs.');
dataset.timeDefinition='original measured seconds within each grouped source trajectory; no resampling';
dataset.prefixImplementationHash=abfdFileHash(fullfile(fileparts(mfilename('fullpath')),'extractResidualSafePrefix.m'));
writeAbfdJson(fullfile(c.outputDir,'residual_dataset.json'),dataset);
end
