function report=pgAdmissionMaterialize(oldDatasetFile,auditFolder,selectionFile,runName)
% Materialize only independently rechecked cached candidates, in a new dataset.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
addpath(fullfile(root,'chapter3','payload_generalization')); c=pgConfig(runName);
assert(~isfile(fullfile(c.output,'dataset.json')),'Admission:Immutable','Dataset already exists.');
old=jsondecode(readUtf8(oldDatasetFile)); choice=jsondecode(readUtf8(selectionFile));
audit=jsondecode(fileread(fullfile(auditFolder,'audit_summary.json')));
assert(strcmp(audit.status,'completed'),'Admission:Incomplete','Physical recheck incomplete.');
assert(strcmp(bdFileHash(oldDatasetFile),choice.parentDatasetSha256),'Admission:Identity','Parent dataset changed.');
assert(strcmp(bdFileHash(fullfile(auditFolder,'candidates.json')),choice.candidatesSha256),'Admission:Identity','Candidate census changed.');
ids=choice.selectedLabels; assert(numel(unique(ids))==numel(ids),'Admission:Duplicate','Repeated candidate ID.');
rows=cell(numel(ids),1); library=cell(numel(ids),1); indexRows=cell(numel(ids),1);
for i=1:numel(ids)
    file=fullfile(auditFolder,'candidate_cache',[ids{i},'.mat']); s=load(file,'record','entry');
    record=s.record; e=s.entry;
    assert(e.physicalRecheckPassed && record.fittedPassed,'Admission:Physical','Unqualified candidate.');
    assert(strcmp(e.sourceSha256,bdFileHash(record.source)),'Admission:Identity','Source changed.');
    index=find(strcmp({old.rows.taskId},record.task.taskId),1); assert(~isempty(index),'Admission:Coverage','Unknown task.');
    row=old.rows(index); row.labelId=ids{i}; row.controls=e.controls; row.objective=record.objective;
    % Cache numerical checks remain valid; preserve the original UTF-8 metadata.
    record.task=row.task;
    row.rawPassed=true; row.fittedPassed=true; rows{i}=row;
    save(fullfile(c.output,[ids{i},'.mat']),'record','-v7');
    library{i}=struct('labelId',ids{i},'taskId',row.taskId,'split',row.split,'file',[ids{i},'.mat'], ...
        'rawPassed',true,'fittedPassed',true,'source',record.source);
    indexRows{i}=struct('labelId',ids{i},'taskId',row.taskId,'split',row.split,'source',record.source,'sourceSha256',e.sourceSha256);
end
data=rmfield(old,'contentHash'); data.rows=rows;
data.parentContentHash=old.contentHash; data.admissionPolicy=choice.policy;
data.admissionAudit=struct('identity',audit.identity,'selectionSha256',bdFileHash(selectionFile));
data.contentHash=dpHash(data); dpWriteJson(fullfile(c.output,'dataset.json'),data);
dpWriteJson(fullfile(c.output,'library.json'),struct('version','pg_library_v1','controlCount',data.controlCount, ...
    'records',{library},'datasetHash',data.contentHash));
for name={'quality_caps.json','quality_reference_rows.json'}
    src=fullfile(fileparts(oldDatasetFile),name{1}); if isfile(src), copyfile(src,fullfile(c.output,name{1})); end
end
splits=cellfun(@(r)r.split,rows,'UniformOutput',false);
trainIds=cellfun(@(r)r.taskId,rows(strcmp(splits,'train')),'UniformOutput',false);
valIds=cellfun(@(r)r.taskId,rows(strcmp(splits,'validation')),'UniformOutput',false);
assert(numel(unique(trainIds))==175 && numel(unique(valIds))==53,'Admission:Coverage','Condition coverage changed.');
assert(sum(strcmp(splits,'validation'))==53,'Admission:Validation','Original validation labels must remain one per task.');
report=struct('status','completed','parentDataset',oldDatasetFile,'parentDatasetSha256',choice.parentDatasetSha256, ...
    'rawQualified',numel(old.rawAudit),'availableFittedQualified',audit.candidateCount,'commonQualified',numel(rows), ...
    'trainLabels',numel(trainIds),'validationLabels',numel(valIds),'trainConditions',numel(unique(trainIds)), ...
    'validationConditions',numel(unique(valIds)),'validationEvaluationConditions',numel(old.validationTasks), ...
    'controlCount',data.controlCount,'fitDamageRate',mean(~[old.fitAudit.passed]),'trainingAllowed',true, ...
    'modelRetrained',false,'physicalGateUnchanged',true,'policy',choice.policy,'datasetHash',data.contentHash);
dpWriteJson(fullfile(c.output,'dataset_report.json'),report);
writetable(struct2table([indexRows{:}]),fullfile(c.output,'label_sources.csv'));
fprintf('ADMISSION_DATASET train=%d validation=%d conditions=%d/%d\n',numel(trainIds),numel(valIds),numel(unique(trainIds)),numel(unique(valIds)));
end

function text=readUtf8(file)
fid=fopen(file,'r','n','UTF-8'); assert(fid>=0,'Admission:IO','Cannot open %s',file);
cleanup=onCleanup(@()fclose(fid)); text=fread(fid,inf,'*char').';
end
