function test_sg_integration()
c=sgSetup('integration_development'); a=sgAssets(c);
s=load(fullfile(c.source,'benchmark','hood_20262041','diffusion','benchmark.mat'));
item=struct('caseId','development_hood','category','development','parameter','none','value',0, ...
    'family','hood','mode','unknown','groupId','development','generationPassed',true, ...
    'generationReason','','planningTask',s.task,'truthTask',s.task);
job=struct('jobId','nominal_first','caseId',item.caseId,'method','diff_fast','count',4, ...
    'repeat',1,'seed',12345,'purpose','development','budget',30);
active=fullfile(c.output,'integration_active.json');
first=sgRunJob(item,job,c,a,'development',active);
assert(first.passed,'Development integration output should pass.');
item.truthTask.model.actuator.forceMin(:)=-1; item.truthTask.model.actuator.forceMax(:)=1;
job.jobId='unknown_force_one'; second=sgRunJob(item,job,c,a,'development',active);
assert(second.planningReused && second.nominalPassed && ~second.truthPassed && ~second.passed);
assert(second.onlineSeconds==first.onlineSeconds && second.onlineSeconds>0);
assert(second.rawCandidatePasses==0 && second.forceMismatchPeak<1e-8);
% Changed limits affect acceptance, not inverse-dynamics demand itself.
item.truthTask.model=sgModel(s.task.model,s.task.scene,struct('payloadMass',150));
job.jobId='unknown_mass'; third=sgRunJob(item,job,c,a,'development',active);
assert(third.planningReused && third.forceMismatchPeak>1);
dpWriteJson(fullfile(c.output,'integration_tests.json'),struct('passed',true, ...
    'nominalReuseTimePreserved',true,'unknownTruthNeverFiltersPlanner',true));
fprintf('SG_INTEGRATION_PASSED\n');
end
