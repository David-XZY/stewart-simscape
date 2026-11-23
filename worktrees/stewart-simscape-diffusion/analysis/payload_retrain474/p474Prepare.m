function p474Prepare(requestPath,outputPath)
r=jsondecode(fileread(requestPath)); c=pgConfig('payload_retrain474_v1');
d=jsondecode(fileread(r.dataset)); old=d.validationTasks; if ~iscell(old),old=num2cell(old);end
checkpoint={}; tasks={}; selected={}; counts=containers.Map('KeyType','char','ValueType','double'); scenes={}; geom={};
for k=1:numel(old)
    row=old{k}; row.panel='legacy'; tasks{end+1}=row;
    if row.task.endpointPassed,checkpoint{end+1}=row;end
end
assert(numel(checkpoint)==74);
pool=r.pool;if ~iscell(pool),pool=num2cell(pool);end
for k=1:numel(pool)
    s=pool{k}; key=[s.family,'_',s.taskClass,'_',s.shape,'_',s.combinationClass];
    if isKey(counts,key)||any(strcmp(s.baseSceneId,scenes))||any(strcmp(s.geometryIdentity,geom)),continue;end
    task=pgMakeTask(s,c);if ~task.endpointPassed,continue;end
    counts(key)=1;scenes{end+1}=s.baseSceneId;geom{end+1}=s.geometryIdentity;
    for T=[5,7.5,10,12.5,15]
        ss=s;ss.parameters(4)=T;ss.taskId=sprintf('%s_T%03d',s.taskId,round(10*T));
        tt=pgMakeTask(ss,c);assert(tt.endpointPassed);
        row=struct('taskId',tt.taskId,'task',tt,'context',tt.context(:).','signature',pgPhysicsSignature(tt).','panel','longtime');
        tasks{end+1}=row;checkpoint{end+1}=row;selected{end+1}=ss;
    end
    if counts.Count==24,break;end
end
assert(counts.Count==24&&numel(tasks)==240&&numel(checkpoint)==194,'P474:Coverage','Insufficient independent endpoint-valid strata');
dpWriteJson(outputPath,struct('version','p474_validation_v1','tasks',{tasks},'checkpointRows',{checkpoint},'newSpecs',{selected},'baseSceneCount',24,'trainingLabels',474));
end
