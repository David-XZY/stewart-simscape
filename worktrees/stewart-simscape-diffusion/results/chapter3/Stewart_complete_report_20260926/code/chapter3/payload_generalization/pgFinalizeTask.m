function task=pgFinalizeTask(base,payload,c)
% Always rederive inertia and COM; never trust imported derived fields.
payload=pgMakePayload(payload.shape,payload.dimensions,payload.mass,payload.mount);
task=base; task.payload=payload; task.derived=payload.derived;
task.schemaVersion='pg_task_v1'; task.contextSchema=c.contextSchema;
task.geometryVersion=c.geometryVersion; task.physicsVersion=c.physicsVersion;
task.model.dynamics.platformMass=50; task.model.dynamics.objectMass=payload.mass;
task.model.dynamics.totalMass=payload.derived.totalMass;
task.model.dynamics.comP=payload.derived.comP; task.model.dynamics.inertiaAtCOM_P=payload.derived.inertiaCOM;
% Remove the legacy payload field: an accidental cylinder-only call must fail.
if isfield(task.scene,'objectCylinder'), task.scene=rmfield(task.scene,'objectCylinder'); end
task.scene.payload=payload; task.scene.collision.safeDistance=.01; task.scene.collision.finalGap=.01;
task.context=pgEncodeContext(task,'basic'); task.modelHash=dpHash(task.model);
if isfield(task,'hash'), task=rmfield(task,'hash'); end
if isfield(task,'taskHash'), task=rmfield(task,'taskHash'); end
task.endpointPassed=false; task.endpointReason='not_checked';
r=pgEndpointCheck(task,c); task.endpointPassed=r.passed; task.endpointReason=r.reason;
task.hash=dpHash(task); task.taskHash=task.hash;
end
