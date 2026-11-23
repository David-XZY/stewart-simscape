function sgWorker(runName,worker,workers)
startup=tic; c=sgSetup(runName); s=load(fullfile(c.output,'manifest.mat')); manifest=s.manifest;
j=load(fullfile(c.output,'jobs.mat')); jobs=j.jobs;
implementation=sgImplementationHash(c); assert(strcmp(implementation,manifest.implementationHash),'Frozen implementation changed.');
asset=sgAssets(c); assert(strcmp(asset.modelHash,manifest.modelHash),'Model changed.');
sgPhysics(buildOptModelCustom()); % Task-independent nominal runtime warmup.
startupSeconds=toc(startup); activeFile=fullfile(c.output,sprintf('active_%d.json',worker));
dpWriteJson(fullfile(c.output,sprintf('startup_%d.json',worker)),struct('seconds',startupSeconds,'matlab',version));
for index=worker:workers:numel(jobs)
    if isfile(fullfile(c.output,'safety_stop.json')), error('SG:SafetyStop','Audit mismatch stopped study.'); end
    job=jobs{index}; folder=fullfile(c.output,'jobs',job.jobId); file=fullfile(folder,'result.json');
    fingerprint=dpHash(struct('manifest',manifest.hash,'implementation',implementation,'job',job));
    if isfile(file)
        old=jsondecode(fileread(file)); assert(strcmp(old.fingerprint,fingerprint),'Job resume fingerprint changed.'); continue;
    end
    controlFile=fullfile(c.output,'run_control.json');
    if isfile(controlFile)
        control=jsondecode(fileread(controlFile));
        if posixtime(datetime('now','TimeZone','UTC'))>=control.computeDeadlineEpoch, break; end
    end
    if job.probe, item=manifest.probes{job.caseIndex}; else, item=manifest.cases{job.caseIndex}; end
    active=struct('jobId',job.jobId,'caseId',item.caseId,'fingerprint',fingerprint, ...
        'phase','online','startedEpoch',posixtime(datetime('now','TimeZone','UTC')), ...
        'budget',job.budget,'job',job,'metadata',struct('family',item.family,'mode',item.mode,'category',item.category,'groupId',item.groupId));
    dpWriteJson(activeFile,active);
    try
        sgRunJob(item,job,c,asset,fingerprint,activeFile);
    catch e
        failure=struct('jobId',job.jobId,'caseId',item.caseId,'family',item.family,'mode',item.mode, ...
            'category',item.category,'parameter',item.parameter,'value',item.value,'groupId',item.groupId, ...
            'method',job.method,'count',job.count,'repeat',job.repeat,'seed',job.seed,'purpose',job.purpose, ...
            'fingerprint',fingerprint,'completed',true,'passed',false,'status','exception', ...
            'reason',[e.identifier,': ',e.message],'stack',e.stack);
        dpWriteJson(file,failure); fprintf('STUDY_EXCEPTION %s %s\n',job.jobId,e.message);
    end
    active.phase='finished'; dpWriteJson(activeFile,active);
end
dpWriteJson(fullfile(c.output,sprintf('worker_%d_complete.json',worker)), ...
    struct('finished',true,'worker',worker,'epoch',posixtime(datetime('now','TimeZone','UTC'))));
end
