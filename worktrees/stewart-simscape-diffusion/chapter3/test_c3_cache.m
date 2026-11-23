function test_c3_cache(runName)
% Real worker test: identical conditions in adjacent methods must each build.
source=c3Setup(runName); signature=c3Fingerprint(source); testName=[runName,'_cache_',signature(1:8)]; c=c3Setup(testName);
copyfile(source.datasetFile,c.datasetFile);
if ~isfolder(fullfile(c.output,'training')), mkdir(fullfile(c.output,'training')); end
copyfile(source.modelFile,c.modelFile); copyfile(fullfile(source.output,'standard.mat'),fullfile(c.output,'standard.mat'));
z=load(fullfile(c.source,'manifest.mat')); tasks=z.manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
k=find(cellfun(@(t)strcmp(t.family,'simple')&&strcmp(t.split,'validation'),tasks),1); task=tasks{k};
item=struct('caseId','cache_fixture','family',task.family,'category','sequence','groupId','cache_fixture', ...
    'rootGroupId','cache_fixture','sequence','cache_fixture','step',0,'parameter','none','value',0,'task',task);
manifest=struct('cases',{{item}}); save(fullfile(c.output,'manifest.mat'),'manifest');
jobs={}; methods={'geo_ihsid','retrieve_ihsid'};
for i=1:2
    jobs{i}=struct('jobId',methods{i},'caseId',item.caseId,'caseIndex',1,'method',methods{i}, ...
        'repeat',1,'purpose','cache_test','count',8,'seed',c.seed,'budget',180);
end
save(fullfile(c.output,'jobs.mat'),'jobs');
z=load(fullfile(c.output,'standard.mat')); frozen=struct('implementation',c3Fingerprint(c),'model',dpHash(fileread(c.modelFile)),'standard',z.standard.hash);
dpWriteJson(fullfile(c.output,'frozen.json'),frozen); c3Worker(testName);
for i=1:2
    z=load(fullfile(c.output,'jobs',methods{i},'result.mat')); assert(z.row.passed,'C3:CacheTest','Development solve failed.');
    first=z.out.attempts{1}; assert(isfield(first,'solver') && ~first.solver.problemCacheHit,'C3:CacheLeak','Cross-method cache hit.');
end
changed=task; changed.T=task.T+.1; changed.qGoal(1)=task.qGoal(1)+.002; changed=sgFinalizeTask(changed);
asset=c3Assets(c); standard=load(fullfile(c.output,'standard.mat')); warmJob=jobs{2}; warmJob.method='previous_ihsid'; warmJob.budget=180;
clear c3Solve sgPhysics sgBasis sgRepairPoint
warm=c3Plan(changed,warmJob,c,asset,standard.standard,z.out.warm);
assert(warm.passed && warm.attempts{1}.solver.warmInfo.endpointAdapted && warm.attempts{1}.solver.warmInfo.dual);
save(fullfile(c.output,'warm_transport.mat'),'warm','changed','-v7');
dpWriteJson(fullfile(source.output,'cache_tests.json'),struct('passed',true,'actualWorkerCalls',2,'warmTransportCall',true,'firstSolverCacheHit',[false,false],'implementation',signature));
fprintf('C3_CACHE_OWNERSHIP_PASSED\n');
end
