function model=updateAbfdPayloadModel(model,scene)
mp=model.dynamics.platformMass; ml=model.dynamics.objectMass; c=scene.objectCylinder.center_P;
Ip=diag([.25,.25,.5])*mp*model.rB^2;
Il=diag([.5*ml*scene.objectCylinder.radius^2, ...
    ml/12*(3*scene.objectCylinder.radius^2+scene.objectCylinder.length^2), ...
    ml/12*(3*scene.objectCylinder.radius^2+scene.objectCylinder.length^2)]);
com=ml*c/(mp+ml); shift=@(r) ((r.'*r)*eye(3)-r*r.');
model.dynamics.totalMass=mp+ml; model.dynamics.comP=com;
model.dynamics.inertiaAtCOM_P=Ip+mp*shift(-com)+Il+ml*shift(c-com);
end
