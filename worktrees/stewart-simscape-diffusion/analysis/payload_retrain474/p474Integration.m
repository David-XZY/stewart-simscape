function p474Integration(dataset,library,output)
c=pgConfig('payload_retrain474_v1');c.returnPolicy='physical';c.maxCpuSeconds=1e9;c.maxIter=2000;
d=jsondecode(fileread(dataset));rows=d.rows;if ~iscell(rows),rows=num2cell(rows);end
index=find(cellfun(@(r)strcmp(r.split,'train')&&strcmp(r.task.family,'open'),rows),1);row=rows{index};task=pgFinalizeTask(row.task,row.task.payload,c);
asset=pgAssets(library,'',true);asset.distanceVariant='dimensions_x2';assert(numel(asset.library)==474);
pgBuildProblem(c);records={};
for name={'ddpm_basic','ddpm_signature','regression','regression_signature','retrieval_signature','ihsid'}
    method=name{1};dim=99;schema='pg_basic99_v1';if contains(method,'signature'),dim=354;schema='pg_signature354_v1';end
    if ~any(strcmp(method,{'retrieval_signature','ihsid'}))
        input=dim;if startsWith(method,'ddpm'),input=input+62;end
        model=struct('method',method,'contextSchema',schema,'controlCount',16,'outputRepresentation','absolute_free_controls_column_major', ...
            'inputMean',zeros(dim,1),'inputScale',ones(dim,1),'outputMean',row.controls(:),'outputScale',ones(60,1), ...
            'layers',struct('W',zeros(60,input),'b',zeros(60,1)));
        if startsWith(method,'ddpm'),model.noiseSchedule=pgNoiseSchedule(100);model.cleanProjection=struct('lower',zeros(60,1),'upper',zeros(60,1));end
        asset.model=model;
    end
    [result,state]=p474Plan(task,method,10,20261001,asset,c,[]);
    assert(result.elapsed>=0&&isfinite(result.elapsed));
    if ~strcmp(method,'ihsid'),assert(isempty(state.warm),'P474:Fallback','Unexpected IHSID state');end
    if startsWith(method,'regression')||startsWith(method,'ddpm'),assert(result.passed&&strcmp(result.returnStage,'direct'));end
    if result.passed,audit=pgValidate(result.bundle,task,c);assert(audit.passed);end
    records{end+1}=struct('method',method,'passed',result.passed,'reason',result.reason,'seconds',result.elapsed);
end
% A zero-time interruption cannot mark the optimizer exhausted.
[r,s]=p474Plan(task,'ihsid',0,20261001,asset,c,[]);assert(~r.passed&&~r.searchExhausted);
[r,s]=p474Plan(task,'ihsid',10,20261001,asset,c,s);assert(s.cumulativeSeconds>=r.elapsed);
% Deterministic regression must keep its candidate cursor over a continuation.
model.method='regression';model.contextSchema='pg_basic99_v1';model.inputMean=zeros(99,1);model.inputScale=ones(99,1);model.layers=struct('W',zeros(60,99),'b',zeros(60,1));
model.outputMean=100*ones(60,1);asset.model=model;
[r,s]=p474Plan(task,'regression',.2,20261001,asset,c,[]);previous=s.nextCandidate;
[r,s]=p474Plan(task,'regression',.5,20261001,asset,c,s);assert(s.nextCandidate==max(previous,2)&&s.candidateGenerated<=1);
% Exercise outer preparation timing, audit, state/result commit, and resume.
folder=fullfile(c.output,'verification_assets');if ~isfolder(folder),mkdir(folder);end
model.outputMean=row.controls(:);modelPath=fullfile(folder,'constant_model.json');dpWriteJson(modelPath,model);
spec=struct('taskId',task.taskId,'baseSceneId',task.baseSceneId,'split',task.split,'taskClass',task.taskClass,'family',task.family, ...
    'generalizationType',task.generalizationType,'q0',task.q0,'qGoal',task.qGoal,'parameters',task.parameters, ...
    'shape',task.payload.shape,'dimensions',task.payload.dimensions,'mass',task.payload.mass);
job=struct('id','positive','unitId','positive','spec',spec,'panel','verification','method','regression','modelKey','constant', ...
    'trainingSeed',0,'samplingSeed',20261001,'repeat',0,'B',10,'sliceBudget',10,'previousState','');
zero=job;zero.id='zero';zero.unitId='zero';zero.B=0;zero.sliceBudget=0;
req=struct('identity','integration_v1','library',library,'output',fullfile(c.output,'verification_calls'), ...
    'models',struct('key','constant','path',modelPath),'jobs',[job,zero]);
request=fullfile(folder,'request.json');dpWriteJson(request,req);p474Batch(request);p474Batch(request);
good=jsondecode(fileread(fullfile(req.output,'positive','result.json')));bad=jsondecode(fileread(fullfile(req.output,'zero','result.json')));
assert(good.finalQualified&&~bad.finalQualified);
dpWriteJson(output,struct('passed',true,'records',{records},'noFallback',true,'singleRegressionCandidateAcrossContinuation',true,'scope','pipeline_integration_with_constant_output_test_models_not_performance'));
end
