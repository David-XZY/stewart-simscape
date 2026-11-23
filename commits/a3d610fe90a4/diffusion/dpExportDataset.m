function dataset=dpExportDataset(manifest,c)
tasks=manifest.tasks; if ~iscell(tasks), tasks=num2cell(tasks); end
rows={}; attempts=struct('taskId',{},'start',{},'passed',{},'reason',{});
for i=1:numel(tasks)
    task=tasks{i}; base=dpBaseline(task);
    for k=1:c.candidateCount
        file=fullfile(c.output,'attempts',task.taskId,sprintf('start_%02d.mat',k));
        assert(isfile(file),'DP:Incomplete','Attempt missing: %s',file);
        loaded=load(file,'record'); r=loaded.record;
        attempts(end+1)=struct('taskId',task.taskId,'start',k,'passed',r.passed,'reason',r.reason); %#ok<AGROW>
        if r.passed
            delta=r.cp(:,4:13)-base(:,4:13);
            rows{end+1}=struct('x',task.context(:).','y',delta(:).','split',task.split,'group',task.groupId,'family',task.family); %#ok<AGROW>
        end
    end
end
assert(~isempty(rows),'DP:NoData','No accepted fitted trajectories.');
r=[rows{:}]; dataset=struct('schemaVersion',1,'kind','stewart-full-diffusion-dataset', ...
    'x',vertcat(r.x),'y',vertcat(r.y),'split',{{r.split}},'groups',{{r.group}},'family',{{r.family}}, ...
    'manifestHash',manifest.hash,'implementationHash',dpImplementationHash(c),'attempts',attempts, ...
    'controlCount',16,'freeControls',4:13,'clearance',.01);
dataset.hash=dpHash(dataset); dpWriteJson(fullfile(c.output,'dataset.json'),dataset);
end
