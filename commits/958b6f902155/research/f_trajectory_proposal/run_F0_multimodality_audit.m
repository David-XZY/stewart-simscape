function audit=run_F0_multimodality_audit(datasetFile,outputFile)
data=jsondecode(fileread(datasetFile)); groups=string(data.groups); modes=string(data.routeMode);
splits=string(data.split); ids=unique(groups(splits=="train")); stable=zeros(numel(ids),1);
successfulTrainingTasks=numel(ids);
if isfield(data,'frozenTasks')
    frozen=data.frozenTasks; train=strcmp({frozen.split},'train');
    ids=string({frozen(train).taskId}); stable=zeros(numel(ids),1);
    assert(numel(unique(ids))==numel(ids),'ABFD:DuplicateTask','Frozen task IDs must be unique.');
end
for i=1:numel(ids)
    counts=groups==ids(i); values=unique(modes(counts));
    for k=1:numel(values), stable(i)=stable(i)+double(sum(counts & modes==values(k))>=2); end
end
fraction=mean(stable>=2);
fid=fopen(datasetFile,'rb'); cleanup=onCleanup(@()fclose(fid)); bytes=fread(fid,inf,'*uint8'); clear cleanup;
digest=java.security.MessageDigest.getInstance('SHA-256'); digest.update(bytes);
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
audit=struct('schemaVersion',1,'datasetHash',hash,'trainingTaskCount',numel(ids), ...
    'trainingTasksWithFeasibleRows',successfulTrainingTasks,'trainingTaskIds',ids, ...
    'multimodalTaskFraction',fraction,'stableModesPerTask',stable, ...
    'diffusionApproved',numel(ids)>=10 && fraction>=0.30, ...
    'criterion','At least 10 train tasks; >=30% have >=2 route classes each recovered by >=2 successful starts.', ...
    'limitation','Geometric route classification, not a homotopy proof; test groups excluded.');
writeAbfdJson(outputFile,audit);
end
