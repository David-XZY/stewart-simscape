function summary=generateDataset(manifest,c,worker,workers)
if nargin<3, worker=1; workers=1; end
tasks=manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
implementation=dpImplementationHash(c); summary=struct('taskId',{},'start',{},'passed',{},'reason',{},'file',{});
for i=worker:workers:numel(tasks)
    task=tasks{i};
    for k=1:c.candidateCount
        folder=fullfile(c.output,'attempts',task.taskId); if ~isfolder(folder), mkdir(folder); end
        file=fullfile(folder,sprintf('start_%02d.mat',k));
        config=dpRunConfig(c);
        hash=dpHash(struct('task',task.hash,'config',config,'start',k,'implementation',implementation));
        if isfile(file)
            old=load(file,'record'); record=old.record;
            assert(strcmp(record.hash,hash),'DP:Resume','Source/config/task changed: start a new run.');
        else
            timer=tic; result=[]; fitReport=[]; cp=[]; seedPreparation=[]; passed=false; reason='';
            try
                [initial,seedPreparation]=prepareGeometricSeed(task,k,c);
                result=dpSolve(task,initial,c);
                if result.passed
                    [cp,fitReport]=dpFitSpline(result,task,c); passed=fitReport.passed;
                    if ~passed, reason='IHSID passed; fitted spline failed independent acceptance.'; end
                else, reason=['IHSID failed: ',result.solverStatus]; end
            catch e, reason=[e.identifier,': ',e.message]; end
            record=struct('hash',hash,'taskId',task.taskId,'taskHash',task.hash,'start',k, ...
                'passed',passed,'reason',reason,'cp',cp,'seedPreparation',seedPreparation,'seconds',toc(timer), ...
                'config',config,'implementationHash',implementation);
            save(file,'record','result','fitReport','task','-v7');
        end
        summary(end+1)=struct('taskId',task.taskId,'start',k,'passed',record.passed,'reason',record.reason,'file',file); %#ok<AGROW>
        dpWriteJson(fullfile(c.output,sprintf('worker_%d.json',worker)),summary);
        fprintf('DATA %s start %d passed %d\n',task.taskId,k,record.passed);
    end
end
dpWriteJson(fullfile(c.output,sprintf('worker_%d_complete.json',worker)),struct('completed',true,'implementationHash',implementation));
end
