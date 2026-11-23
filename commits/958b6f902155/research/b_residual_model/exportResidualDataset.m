function output=exportResidualDataset(sessions,model,file,taskSplits)
% taskSplits is a pre-frozen task manifest; never split adjacent samples.
all=cell(1,numel(sessions));
for k=1:numel(sessions)
    d=buildResidualWrenchTarget(sessions{k},model);
    match=find(strcmp({taskSplits.taskId},d.taskId));
    assert(isscalar(match),'ABFD:TaskSplit','Each task must have exactly one frozen split.');
    d.split=repmat(string(taskSplits(match).split),size(d.x,1),1);
    d.groupId=repmat(string(d.taskId),size(d.x,1),1); all{k}=d;
end
data=struct();
for key={'x','acceleration','wrench','residual','split','groupId'}
    name=key{1}; pieces=cellfun(@(d)d.(name),all,'UniformOutput',false);
    data.(name)=vertcat(pieces{:});
end
mask=data.split=="train"; training=data;
for key=fieldnames(data).', training.(key{1})=data.(key{1})(mask,:); end
physical=fitPhysicalParameterResidual(training,model);
m=model; m.dynamics.totalMass=physical.mass; m.dynamics.comP=physical.com; m.dynamics.inertiaAtCOM_P=physical.inertia;
y=zeros(size(data.wrench));
for k=1:size(y,1)
    y(k,:)=data.wrench(k,:)-computeCompositeRequiredWrench(data.x(k,1:6).',data.x(k,7:12).',data.acceleration(k,:).',m).';
end
output=struct('schemaVersion',1,'kind','residual-training-dataset','x',data.x,'y',y, ...
    'groups',data.groupId,'split',data.split,'physical',physical,'modelHash',abfdHash(model), ...
    'referencePhysical',struct('mass',model.dynamics.totalMass,'com',model.dynamics.comP,'inertia',model.dynamics.inertiaAtCOM_P), ...
    'targetDefinition','Jv(q)^T F_actual - physically fitted world wrench');
writeAbfdJson(file,output);
end
