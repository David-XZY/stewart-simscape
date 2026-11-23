function task=bdPayloadTask(base,mass,lengthM,radiusM,c)
% Independent development contract: centered uniform cylinders, fixed mount.
% No lateral eccentricity parameter. The existing vertical mount is retained.
assert(all(isfinite([mass,lengthM,radiusM]))&&all([mass,lengthM,radiusM]>0),'BD:Payload','Positive SI parameters required.');
task=base; old=base.scene.objectCylinder; dyn=base.model.dynamics;
assert(norm(old.axis_P-[1;0;0])<1e-12,'BD:PayloadAxis','This version assumes cylinder axis x.');
mp=dyn.platformMass; mo=dyn.objectMass; mount=old.center_P(:);
assert(norm(mount(1:2))<1e-12,'BD:PayloadOffset','No lateral eccentricity in this experiment.');
% Infer the fixed platform COM inertia from the verified nominal composite.
% Nominal values recover diag([3.125,3.125,6.25]) kg m^2 exactly.
oldI=diag([mo*old.radius^2/2,mo*(3*old.radius^2+old.length^2)/12,mo*(3*old.radius^2+old.length^2)/12]);
oldCOM=mo/(mp+mo)*mount;
assert(norm(dyn.comP-oldCOM)<1e-10,'BD:PayloadModel','Nominal composite COM does not match centered mounting assumption.');
platformI=dyn.inertiaAtCOM_P-oldI-shift(mp,-oldCOM)-shift(mo,mount-oldCOM);
assert(min(eig(platformI))>0,'BD:PayloadModel','Inferred platform inertia must be positive definite.');
objectI=diag([mass*radiusM^2/2,mass*(3*radiusM^2+lengthM^2)/12,mass*(3*radiusM^2+lengthM^2)/12]);
com=mass/(mp+mass)*mount;
task.model.dynamics.objectMass=mass; task.model.dynamics.totalMass=mp+mass;
task.model.dynamics.comP=com;
task.model.dynamics.inertiaAtCOM_P=platformI+objectI+shift(mp,-com)+shift(mass,mount-com);
task.scene.objectCylinder.mass=mass; task.scene.objectCylinder.length=lengthM; task.scene.objectCylinder.radius=radiusM;
task.payload=struct('schema','centered_uniform_cylinder_v1','mass',mass,'length',lengthM,'radius',radiusM, ...
    'mountCenterP',mount,'objectInertiaAtCOM',objectI,'platformInertiaAtCOM',platformI, ...
    'geometryScope','Existing idealized mounting; no actual CAD or contact/self-collision validation.');
task=sgFinalizeTask(task);
% 93 legacy task entries + 3 payload entries. Old checkpoints are incompatible.
task.context=[dpContext(task);mass;lengthM;radiusM]; task.contextSchema='legacy93_plus_cylinder_m_L_r';
task.taskId=['payload_',dpHash([base.parameters(:);mass;lengthM;radiusM])];
r=bdEndpointCheck(task,c); task.endpointPassed=r.passed; task.endpointReason=r.reason;
task=rmfield(task,'hash'); task.hash=dpHash(task);
end
function I=shift(m,r)
I=m*((r.'*r)*eye(3)-r*r.');
end
