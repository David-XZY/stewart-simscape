function bdImportLegacy(auditFile,runName)
% Development-only import. Retained training labels only; no old test/val rows.
c=bdConfig(runName); audit=jsondecode(fileread(auditFile)); rows=audit.rows;
if iscell(rows), rows=[rows{:}]; end
labels={}; folder=fullfile(c.output,'legacy_library'); if ~isfolder(folder), mkdir(folder); end
for k=1:numel(rows)
    row=rows(k); if ~strcmp(row.split,'train')||~row.retained, continue; end
    source=row.source;
    if ~isfile(source)
        suffix=strfind(strrep(source,'\','/'),'/results/'); assert(~isempty(suffix),'BD:Source','Cannot relocate legacy source.');
        normalized=strrep(source,'\','/'); source=fullfile(c.root,normalized(suffix(end)+1:end));
    end
    z=load(source); assert(z.record.passed&&row.teacherPassed&&row.physicalPassed,'BD:LegacyAudit','Invalid retained audit row.');
    task=z.task; assert(strcmp(task.split,'train'),'BD:Leakage','Legacy source is not training.');
    result=z.result; result.passed=row.teacherPassed;
    fitted=struct('kind','spline','cp',z.record.cp,'T',task.T);
    record=struct('fitPassed',row.physicalPassed,'legacySource',source,'legacyAuditLabelId',row.labelId, ...
        'objectiveScope','Legacy seed-dependent teacher objective; diagnostic history only');
    file=fullfile(folder,[row.labelId,'.mat']);
    if ~isfile(file), save(file,'task','result','fitted','record','-v7'); end
    labels{end+1}=struct('labelId',row.labelId,'taskHash',task.hash,'split','train','file',file,'fileHash',bdFileHash(file),'modeId',0,'fitPassed',true);
end
dpWriteJson(fullfile(c.output,'legacy_library.json'),struct('purpose','legacy_development_diagnostic_only','labels',{labels},'sourceAudit',auditFile,'datasetHash','legacy_not_new_training'));
fprintf('BD_IMPORTED_LEGACY_TRAIN %d\n',numel(labels));
end
