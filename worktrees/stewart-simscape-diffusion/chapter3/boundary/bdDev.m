function summary=bdDev(maxAttempts,runName)
% Bounded Stage B only. Does not read any learned model or test dataset.
if nargin<1, maxAttempts=16; end
if nargin<2, runName='boundary_dev_v1'; end
assert(maxAttempts>=1&&maxAttempts<=280&&fix(maxAttempts)==maxAttempts);
c=bdConfig(runName); offsets=[-.02,-.01,0,.01,.02]; tasks=cell(25,1); idx=0;
for i=1:5
    for j=1:5
        idx=idx+1; spec=struct('taskId',sprintf('bd_dev_%02d',idx),'groupId',sprintf('dev_cell_%d_%d',i,j));
        tasks{idx}=bdMakeTask(spec,[offsets(i),offsets(j),0,7.5],c);
    end
end
manifest=struct('kind','boundary-development','tasks',{tasks},'seed',20260919,'modelSelectionUsed',false,'maxAttempts',maxAttempts);
file=fullfile(c.output,'tasks.json');
if isfile(file), old=jsondecode(fileread(file)); assert(strcmp(dpHash(old.tasks),dpHash(jsondecode(jsonencode(tasks)))),'BD:Manifest','Task manifest changed.');
else, dpWriteJson(file,manifest); end
attempts={}; passed=0; count=0;
for k=1:25
    task=tasks{k}; folder=fullfile(c.output,task.taskId); if ~isfolder(folder), mkdir(folder); end
    dpWriteJson(fullfile(folder,'task.json'),task);
    for seedId=1:4
        if count>=min(maxAttempts,100), break; end
        count=count+1; output=fullfile(folder,sprintf('seed_%02d.mat',seedId)); jsonfile=strrep(output,'.mat','.json');
        if isfile(jsonfile)
            record=jsondecode(fileread(jsonfile)); assert(isfield(record,'codeHash')&&strcmp(record.codeHash,c.codeHash),'BD:VersionChanged','Use a new development run after code changes.'); attempts{end+1}=record; passed=passed+double(record.passed); continue;
        end
        clock=tic; record=struct('taskId',task.taskId,'seedId',seedId,'codeHash',c.codeHash,'passed',false,'reason','boundary_rejected','seconds',0);
        result=[];
        try
            if task.endpointPassed
                c.clockHandle=clock; c.wallBudget=c.referenceBudget; c.maxCpuSeconds=120;
                result=bdSolve(task,bdSeed(task,seedId,c),c); record.passed=result.passed;
                record.reason=result.solverStatus; record.physicalReason=result.report.reason;
                record.buildSeconds=result.buildSeconds; record.solveSeconds=result.solveSeconds; record.validationSeconds=result.validationSeconds;
                record.cacheHit=result.problemCacheHit; record.maxEquality=result.maxEquality; record.maxInequality=result.maxInequality; record.maxBounds=result.maxBounds;
                if result.passed, record.quality=bdQuality(result.bundle,task,c); end
            end
        catch e
            record.reason=[e.identifier,': ',e.message]; record.stack=e.stack;
        end
        record.seconds=toc(clock); save(output,'task','result','record','-v7'); dpWriteJson(jsonfile,record);
        attempts{end+1}=record; passed=passed+double(record.passed);
        summary=struct('attemptsCompleted',count,'successes',passed,'trainingCalls',0,'diffusionCalls',0,'records',{attempts});
        dpWriteJson(fullfile(c.output,'progress.json'),summary);
        fprintf('BD_DEV %d/%d passed=%d %s\n',count,min(maxAttempts,100),record.passed,record.reason);
    end
    if count>=min(maxAttempts,100), break; end
end
summary=struct('attemptsCompleted',count,'successes',passed,'trainingCalls',0,'diffusionCalls',0,'records',{attempts}, ...
    'stage','B_initial_grid','scope',c.envelope.scope,'mechanismEstablished',false, ...
    'remainingStageB','Timing and counterfactual scans require inspection of independent references; not silently auto-expanded.');
dpWriteJson(fullfile(c.output,'summary.json'),summary);
end
