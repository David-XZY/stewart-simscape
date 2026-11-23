function task=pgMakeTask(spec,c)
base=bdMakeTask([],spec.parameters,c); base.taskId=spec.taskId; base.baseSceneId=spec.baseSceneId;
base.split=spec.split; base.taskClass=spec.taskClass; base.family=spec.family;
base.generalizationType=spec.generalizationType; base.q0=spec.q0(:); base.qGoal=spec.qGoal(:);
if isfield(spec,'payloadRegionId'), base.payloadRegionId=spec.payloadRegionId; end
if strcmp(spec.family,'open'), base.scene.hood.obstacles=base.scene.hood.obstacles([]); end
p=pgMakePayload(spec.shape,spec.dimensions,spec.mass);
task=pgFinalizeTask(base,p,c);
end
