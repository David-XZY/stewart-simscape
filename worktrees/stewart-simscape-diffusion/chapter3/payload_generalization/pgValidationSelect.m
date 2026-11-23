function result=pgValidationSelect(libraryPath,manifestPath,runName)
c=pgConfig(runName); a=pgAssets(libraryPath); m=jsondecode(fileread(manifestPath)); specs=m.tasks;
c.controlCount=a.controlCount; warmup=pgBuildProblem(c); dpWriteJson(fullfile(c.output,'startup.json'),warmup);
configs={'equal_blocks','shape_x2','dimensions_x2','mass_x2','obstacle_x2','time_x2'}; rows={};
for k=1:numel(specs)
    if ~any(strcmp(specs(k).split,{'dev_validation','validation'})), continue; end
    task=pgMakeTask(specs(k),c);
    for j=1:numel(configs)
        % Rotated method order prevents always charging warmup to equal_blocks.
        index=mod(j+k-2,numel(configs))+1; aa=a; aa.distanceVariant=configs{index};
        r=pgPlan(task,'retrieval',2,20260919,aa,c);
        rows{end+1}=struct('taskId',task.taskId,'stratum',[task.family,'_',task.taskClass,'_',task.payload.shape], ...
            'configuration',configs{index},'passed',r.passed,'actualReturnTime',r.actualReturnTime,'reason',r.reason);
    end
end
scores=zeros(1,numel(configs));
for j=1:numel(configs)
    selected=rows(cellfun(@(r)strcmp(r.configuration,configs{j}),rows)); strata=unique(cellfun(@(r)r.stratum,selected,'UniformOutput',false)); rates=zeros(size(strata));
    for k=1:numel(strata), group=selected(cellfun(@(r)strcmp(r.stratum,strata{k}),selected)); rates(k)=mean(cellfun(@(r)r.passed,group)); end
    scores(j)=mean(rates);
end
[~,best]=max(scores); result=struct('split','validation','criterion','stratum_equal_physical_return_B2','configurations',{configs},'scores',scores,'selectedDistance',configs{best},'records',{rows});
dpWriteJson(fullfile(c.output,'validation_selection.json'),result);
end
