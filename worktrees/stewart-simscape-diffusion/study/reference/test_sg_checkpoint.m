function test_sg_checkpoint()
% Persistence test on the OLD development set; no new study tasks are used.
c=sgSetup('reference_checkpoint_development'); asset=sgAssets(c);
s=load(fullfile(c.source,'benchmark','hood_20262041','diffusion','benchmark.mat'));
c.referenceCheckpointFile=fullfile(c.output,'checkpoint.mat');
c.referenceFingerprint='development_persistence_test';
job=struct('method','diff_fast','count',4,'seed',12345,'budget',30);
out=sgPlanCheckpointed(s.task,job,c,asset);
assert(out.passed && isfile(c.referenceCheckpointFile),'No accepted checkpoint was saved.');
saved=load(c.referenceCheckpointFile);
assert(strcmp(saved.referenceFingerprint,c.referenceFingerprint));
assert(strcmp(dpHash(saved.checkpoint.bundle),dpHash(out.bundle)),'Latest accepted output was not retained.');
assert(all([saved.checkpoint.events.seconds]<=job.budget));
audit=sgAudit(saved.checkpoint.bundle,s.task,c); assert(audit.passed);
dpWriteJson(fullfile(c.output,'test_result.json'),struct('passed',true, ...
    'oldDevelopmentTask','hood_20262041','checkpointSurvivesFunctionReturn',true));
fprintf('REFERENCE_CHECKPOINT_TEST_PASSED\n');
end
