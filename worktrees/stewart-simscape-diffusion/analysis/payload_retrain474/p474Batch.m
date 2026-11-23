function p474Batch(requestPath)
r=jsondecode(fileread(requestPath));c=pgConfig('payload_retrain474_v1');c.returnPolicy='physical';
% Identical options for task-independent warmup and every online IHSID slice.
c.maxCpuSeconds=1e9;c.maxIter=2000;
assets=pgAssets(r.library,'',true);assets.distanceVariant='dimensions_x2';assert(numel(assets.library)==474);
models=struct();mm=r.models;if ~iscell(mm),mm=num2cell(mm);end
for k=1:numel(mm),models.(mm{k}.key)=jsondecode(fileread(mm{k}.path));end
pgBuildProblem(c);
jobs=r.jobs;if ~iscell(jobs),jobs=num2cell(jobs);end
for j=1:numel(jobs)
    job=jobs{j};folder=fullfile(r.output,job.id);if ~isfolder(folder),mkdir(folder);end
    resultPath=fullfile(folder,'result.json');statePath=fullfile(folder,'state.mat');
    if isfile(resultPath)
        previous=jsondecode(fileread(resultPath));assert(strcmp(previous.identity,r.identity)&&isfile(statePath)&&isfile(fullfile(folder,'trajectory.mat')));continue;
    end
    state=[];
    if ~isempty(job.previousState)
        saved=load(job.previousState,'state','identity');assert(strcmp(saved.identity,r.identity));state=saved.state;
    end
    local=assets;if ~isempty(job.modelKey),local.model=models.(job.modelKey);end
    outer=tic;task=pgMakeTask(job.spec,c);prep=toc(outer);
    [result,state]=p474Plan(task,job.method,max(0,job.sliceBudget-prep),job.samplingSeed,local,c,state);
    result.taskId=task.taskId;result.baseSceneId=task.baseSceneId;result.method=job.method;result.panel=job.panel;
    result.B=job.B;result.trainingSeed=job.trainingSeed;result.samplingSeed=job.samplingSeed;result.repeat=job.repeat;
    result.family=task.family;result.taskClass=task.taskClass;result.shape=task.payload.shape;result.T=task.T;
    result.id=job.id;result.unitId=job.unitId;result.identity=r.identity;result.outerPreparationSeconds=prep;
    innerElapsed=result.elapsed;result.elapsed=toc(outer);state.cumulativeSeconds=state.cumulativeSeconds+result.elapsed-innerElapsed;
    result.cumulativeSeconds=state.cumulativeSeconds;
    result.passed=result.passed&&result.elapsed<=job.sliceBudget;
    if result.elapsed>job.sliceBudget,result.reason='timeout';end
    result.onlinePassed=result.passed;result.independentAuditPassed=false;result.finalQualified=false;
    offline=tic;
    if result.passed
        cc=c;cc.denseStep=.0025;audit=pgValidate(result.bundle,task,cc);
        result.independentAuditPassed=audit.passed;result.finalQualified=audit.passed;
        if ~audit.passed
            % Numerical audit disagreement invalidates the campaign, not an algorithm failure.
            save(fullfile(folder,'audit_disagreement.mat'),'task','result','audit');
            error('P474:AuditDisagreement','Returned trajectory failed independent audit: %s',task.taskId);
        end
    end
    result.offlineAuditSeconds=toc(offline);
    result.events=state.events;result.attempts=state.attempts;
    identity=r.identity;temp=fullfile(folder,'state.tmp.mat');save(temp,'state','identity','-v7');movefile(temp,statePath,'f');
    save(fullfile(folder,'trajectory.mat'),'task','result','-v7');
    compact=rmfield(result,{'bundle','report'});
    compact.statePath=statePath;compact.trajectoryPath=fullfile(folder,'trajectory.mat');
    temp=fullfile(folder,'result.tmp.json');dpWriteJson(temp,compact);movefile(temp,resultPath,'f');
    dpWriteJson(fullfile(c.output,'benchmark_progress.json'),struct('request',requestPath,'completed',j,'total',numel(jobs),'lastId',job.id,'lastPassed',result.finalQualified));
    fprintf('P474_CALL %d/%d %s B=%g passed=%d seconds=%.4f\n',j,numel(jobs),job.method,job.B,result.finalQualified,result.elapsed);
end
end
