function sgReferenceRun(runName,jobIndex)
% Separate reference-only persistence revision; main study remains immutable.
c=sgSetup(runName); source=c.output;
m=load(fullfile(source,'manifest.mat')); j=load(fullfile(source,'jobs.mat'));
job=j.jobs{jobIndex}; assert(strcmp(job.purpose,'reference'),'Only the frozen eight references are allowed.');
assert(strcmp(sgImplementationHash(c),m.manifest.implementationHash),'Frozen main implementation changed.');
task=m.manifest.cases{job.caseIndex}.planningTask;
asset=sgAssets(c); assert(strcmp(asset.modelHash,m.manifest.modelHash),'Frozen weights changed.');
sgPhysics(buildOptModelCustom());
folder=fullfile(source,'reference_checkpointed_v1',job.jobId);
if ~isfolder(folder), mkdir(folder); end
c.referenceFingerprint=dpHash(struct('main',m.manifest.implementationHash,'manifest',m.manifest.hash, ...
    'job',job,'planner',fileread(which('sgPlanCheckpointed')),'entry',fileread(which('sgReferenceRun'))));
c.referenceCheckpointFile=fullfile(folder,'checkpoint.mat');
c.progressFile=fullfile(folder,'online_progress.json');
metadata=struct('job',job,'taskHash',task.hash,'fingerprint',c.referenceFingerprint, ...
    'mainImplementationHash',m.manifest.implementationHash,'manifestHash',m.manifest.hash, ...
    'modelHash',asset.modelHash,'matlab',version, ...
    'note','Separate 600s reference repeat. Same eight tasks, candidates, objective and budgets. Do not pool best values with the original reference run.');
dpWriteJson(fullfile(folder,'metadata.json'),metadata);
dpWriteJson(fullfile(folder,'active.json'),struct('startedEpoch',posixtime(datetime('now','TimeZone','UTC')), ...
    'budget',job.budget,'jobId',job.jobId,'fingerprint',c.referenceFingerprint));
try
    out=sgPlanCheckpointed(task,job,c,asset);
    save(fullfile(folder,'completed.mat'),'out','metadata','-v7');
    dpWriteJson(fullfile(folder,'completion.json'),struct('status','returned', ...
        'onlineSeconds',out.onlineSeconds,'reportedPassed',out.passed, ...
        'withinBudget',out.onlineSeconds<=job.budget,'fingerprint',c.referenceFingerprint));
catch e
    dpWriteJson(fullfile(folder,'completion.json'),struct('status','exception','reason',e.message, ...
        'identifier',e.identifier,'fingerprint',c.referenceFingerprint));
    rethrow(e);
end
end
