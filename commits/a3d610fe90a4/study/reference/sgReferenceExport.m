function sgReferenceExport(runName)
% Independent audit of best checkpoints; runtime failures remain documented.
c=sgSetup(runName); source=c.output; m=load(fullfile(source,'manifest.mat'));
assert(strcmp(sgImplementationHash(c),m.manifest.implementationHash),'Frozen main implementation changed.');
j=load(fullfile(source,'jobs.mat')); rows={};
for k=1:numel(j.jobs)
    job=j.jobs{k}; if ~strcmp(job.purpose,'reference'), continue; end
    folder=fullfile(source,'reference_checkpointed_v1',job.jobId);
    checkpointFile=fullfile(folder,'checkpoint.mat');
    row=struct('caseId',job.caseId,'jobId',job.jobId,'budget',job.budget, ...
        'checkpointAvailable',false,'auditPassed',false,'status','unfinished');
    if isfile(fullfile(folder,'completion.json'))
        completion=jsondecode(fileread(fullfile(folder,'completion.json'))); row.status=completion.status;
    end
    if isfile(checkpointFile)
        s=load(checkpointFile); metadata=jsondecode(fileread(fullfile(folder,'metadata.json')));
        assert(strcmp(s.referenceFingerprint,metadata.fingerprint),'Checkpoint fingerprint mismatch.');
        assert(strcmp(metadata.mainImplementationHash,m.manifest.implementationHash),'Main implementation mismatch.');
        assert(strcmp(metadata.manifestHash,m.manifest.hash),'Manifest mismatch.');
        out=s.checkpoint; task=m.manifest.cases{job.caseIndex}.truthTask;
        assert(out.passed && all([out.events.seconds]<=job.budget),'Checkpoint was not accepted within budget.');
        audit=sgAudit(out.bundle,task,c); quality=sgQuality(out.bundle,task,c);
        row.checkpointAvailable=true; row.auditPassed=audit.passed;
        row.bestFoundSeconds=out.events(end).seconds; row.firstFeasibleSeconds=out.firstFeasibleSeconds;
        row.quality=quality; row.minClearance=audit.minClearance;
        row.minSigma=audit.minSigma; row.maxCondition=audit.maxCondition;
        row.fingerprint=s.referenceFingerprint;
        save(fullfile(folder,'independent_audit.mat'),'audit','quality','row','-v7');
        if ~audit.passed
            dpWriteJson(fullfile(source,'safety_stop_reference.json'),struct('jobId',job.jobId, ...
                'reason','Accepted known-model reference checkpoint failed independent audit.'));
            error('SG:ReferenceAudit','Accepted reference failed independent audit: %s',job.jobId);
        end
    end
    rows{end+1}=row; %#ok<AGROW>
end
dpWriteJson(fullfile(source,'reference_checkpointed_v1','audited_results.json'),rows);
fprintf('REFERENCE_AUDIT %d records\n',numel(rows));
end
