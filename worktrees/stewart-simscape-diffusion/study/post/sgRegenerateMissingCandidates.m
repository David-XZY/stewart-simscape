function sgRegenerateMissingCandidates(runName)
% Reproduce unpersisted candidates after process failures; never alter outcomes.
c=sgSetup(runName); m=load(fullfile(c.output,'manifest.mat')); j=load(fullfile(c.output,'jobs.mat'));
assert(strcmp(sgImplementationHash(c),m.manifest.implementationHash));
asset=sgAssets(c); assert(strcmp(asset.modelHash,m.manifest.modelHash));
folder=fullfile(c.output,'regenerated_candidates'); if ~isfolder(folder), mkdir(folder); end
records={};
for k=1:numel(j.jobs)
    job=j.jobs{k}; original=fullfile(c.output,'jobs',job.jobId);
    if isfile(fullfile(original,'result.mat')) || isfile(fullfile(original,'online.mat')), continue; end
    if ~isfile(fullfile(original,'result.json')), continue; end
    if job.probe, item=m.manifest.probes{job.caseIndex}; else, item=m.manifest.cases{job.caseIndex}; end
    if ~item.generationPassed || ~item.planningTask.endpointPassed, continue; end
    task=item.planningTask;
    if strncmp(job.method,'geo',3), kind='geometric'; elseif strncmp(job.method,'retrieve',8), kind='retrieval'; else, kind='diffusion'; end
    [candidates,generation]=sgCandidates(kind,task,job.count,job.seed,asset);
    metadata=struct('jobId',job.jobId,'generatedOffline',true,'planningTaskHash',task.hash, ...
        'modelHash',asset.modelHash,'implementationHash',m.manifest.implementationHash, ...
        'note','Deterministic regeneration from frozen planning conditions and seed. Not the original saved output, timing, validation or success.');
    save(fullfile(folder,[job.jobId,'.mat']),'candidates','generation','metadata','job','-v7');
    records{end+1}=metadata; %#ok<AGROW>
end
dpWriteJson(fullfile(folder,'manifest.json'),records);
fprintf('REGENERATED_MISSING_CANDIDATES %d\n',numel(records));
end
