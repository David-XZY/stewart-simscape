function report=bdIntegrationTests(runName)
% Deliberately wide fixture layout for infrastructure verification only.
% Not a mechanism experiment and not training data.
if nargin<1, runName='boundary_integration_v1'; end
c=bdConfig(runName); task=bdMakeTask([],[0,0,0,7.5],c);
for j=1:2, task.scene.hood.obstacles(j).center_S(1)=10*(-1)^j; end
task=sgFinalizeTask(task); task.hash=dpHash(rmfield(task,'hash')); checks={};
b=struct('kind','spline','T',task.T,'cp',bdBaseline(task,16));
[ok,r]=bdAccept(b,task,'physical',tic,30,c); assert(ok,r.reason); checks{end+1}='wide_layout_physical_acceptance';
h=struct('bundle',b,'fitted',b,'task',task,'labelId','infrastructure_test_only');
a=struct('library',{{h}},'trainX',task.context(:).','inputMean',task.context(:).','inputScale',ones(1,93), ...
    'trainNormalized',zeros(1,93),'trainY',zeros(1,60),'modeIds',1,'distanceVariant','equal_blocks','localNeighbors',16,'ridge',.01);
for method={'M1','M2_H','M3_L'}
    pool=bdCandidates(method{1},task,1,a,tic,20,c,10); assert(numel(pool)==1);
    [ok,r]=bdAccept(pool{1}.bundle,task,'physical',tic,20,c); assert(ok,r.reason);
end
checks{end+1}='retrieval_adapters_local_fallback';
multi=a; multi.library={h,h,h}; multi.trainX=repmat(a.trainX,3,1); multi.trainX(:,13)=task.T+[0;.1;-.1];
multi.trainNormalized=zeros(3,93); multi.trainNormalized(:,13)=[0;.1;-.1]; multi.trainY=zeros(3,60); multi.modeIds=[1,1,1];
regressed=bdCandidates('M3_L',task,1,multi,tic,20,c,10);
assert(max(abs(regressed{1}.bundle.cp(:)-b.cp(:)))<1e-8); checks{end+1}='mode_local_ridge_regression';
clock=tic; [timed,info]=bdRetime(b,task,clock,30,c); assert(info.solverSuccess,info.status);
[ok,r]=bdAccept(timed,task,'physical',tic,30,c); assert(ok,r.reason); checks{end+1}='nonuniform_retime_full_acceptance';
q=bdQuality(b,task,c); qm=bdQuality(struct('kind','path_time_map','path',b,'pathHash',dpHash(b),'T',task.T,'timeControls',[]),task,c);
assert(abs(q.positionLengthM-qm.positionLengthM)<1e-8&&abs(q.legAccelRms-qm.legAccelRms)<1e-8); checks{end+1}='representation_metric_equivalence';
% Learned network arithmetic/prefix checks use a zero predictor, not trained output claims.
model=struct('inputMean',zeros(93,1),'inputScale',ones(93,1),'outputMean',zeros(60,1),'outputScale',ones(60,1), ...
    'beta',linspace(1e-4,.02,100),'layers',struct('W',zeros(60,155),'b',zeros(60,1)));
a.model=model; one=bdCandidates('M4',task,1,a,tic,30,c,123); many=bdCandidates('M4',task,1:4,a,tic,30,c,123);
assert(isequal(one{1}.bundle.cp,many{1}.bundle.cp)); checks{end+1}='candidate_random_prefix';
out=bdPlan(task,'M1',10,123,a,c); assert(out.passed&&out.candidateChecked==1&&strcmp(out.returnStage,'direct'));
assert(out.otherSeconds>=0); checks{end+1}='true_early_return_and_timing_closure';
c.batchTotals=4; out=bdPlan(task,'M1',10,123,multi,c);
assert(out.passed&&out.candidateGenerated==3&&out.candidateChecked==1&&out.selectedCandidateId==1);
checks{end+1}='complete_batch_count_with_early_return'; c.batchTotals=[1,4,8,16,32,64];
out=bdPlan(task,'M1',0,123,a,c); assert(~out.passed&&~out.returnedWithinBudget); checks{end+1}='expired_call_never_success';
c.maxCpuSeconds=10; bdBuildProblem(c);
c.clockHandle=tic; c.wallBudget=10; sol=bdSolve(task,b.cp,c); assert(sol.problemCacheHit,'Expected parameter-only graph reuse.');
checks{end+1}='cross_task_graph_cache';
report=struct('passed',true,'checks',{checks},'solverPassed',sol.passed,'solverStatus',sol.solverStatus,'scope','Infrastructure tests, not mechanism evidence');
save(fullfile(c.output,'integration.mat'),'report','task','sol','-v7'); dpWriteJson(fullfile(c.output,'integration.json'),report);
fprintf('BD_INTEGRATION_PASSED %d solverPassed=%d\n',numel(checks),sol.passed);
end
