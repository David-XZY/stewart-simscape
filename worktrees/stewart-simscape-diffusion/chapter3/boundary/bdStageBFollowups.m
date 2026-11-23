function summary=bdStageBFollowups(runName,maxAttempts)
% Independent-reference-driven scheduling; never reads learned model outputs.
c=bdConfig(runName); progress=jsondecode(fileread(fullfile(c.output,'progress.json')));
rows=progress.records; if ~iscell(rows), rows=num2cell(rows); end
passed=rows(cellfun(@(r)r.passed,rows)); ids=unique(cellfun(@(r)r.taskId,passed,'UniformOutput',false),'stable');
if isempty(ids)
    summary=struct('attempts',0,'reason','No qualified IHSID anchor; timing/counterfactual expansion stopped.');
    dpWriteJson(fullfile(c.output,'followup_summary.json'),summary); return;
end
manifest=jsondecode(fileread(fullfile(c.output,'tasks.json'))); initial=manifest.tasks;
if iscell(initial), initial=[initial{:}]; end
selected=ids(1:min(12,numel(ids))); schedule={};
for k=1:numel(selected)
    old=initial(find(strcmp({initial.taskId},selected{k}),1));
    for T=[6,4,3]
        pars=old.parameters; pars(4)=T; spec=struct('taskId',sprintf('%s_T%g',old.taskId,T),'groupId',old.groupId,'split','development');
        schedule{end+1}=struct('task',bdMakeTask(spec,pars,c),'seedIds',1:4,'kind','duration_scan');
    end
end
for k=1:min(6,numel(selected))
    old=initial(find(strcmp({initial.taskId},selected{k}),1));
    for change=1:3
        pars=old.parameters;
        if change==1, pars(1)=pars(1)-.5; elseif change==2, pars(2)=pars(2)+.5; else, pars(4)=10; end
        spec=struct('taskId',sprintf('%s_cf%d',old.taskId,change),'groupId',old.groupId,'split','development');
        schedule{end+1}=struct('task',bdMakeTask(spec,pars,c),'seedIds',[1,2],'kind','counterfactual');
    end
end
dpWriteJson(fullfile(c.output,'followup_tasks.json'),struct('schedule',{schedule},'selection','Qualified independent IHSID anchors only','modelOutputsRead',false));
records={}; count=0;
for k=1:numel(schedule)
    item=schedule{k}; seeds=item.seedIds(1:min(numel(item.seedIds),maxAttempts-count)); if isempty(seeds), break; end
    r=bdGenerateReferences(item.task,seeds,c); records=[records,r]; count=count+numel(seeds);
end
summary=struct('attempts',count,'records',{records},'reason','Bounded reference-only followup complete; formal domain not automatically frozen.');
dpWriteJson(fullfile(c.output,'followup_summary.json'),summary);
end
