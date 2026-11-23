function m=sgModel(base,scene,change)
% Fixed geometry, physically consistent platform/payload composite inertia.
p=struct('payloadMass',100,'platformMass',50,'comOffset',[0;0;0], ...
    'inertiaScale',1,'speedScale',1,'accelerationScale',1,'forceScale',1);
if nargin>2
    for name=fieldnames(change).', assert(isfield(p,name{1})); p.(name{1})=change.(name{1}); end
end
assert(all([p.payloadMass,p.platformMass,p.inertiaScale,p.speedScale,p.accelerationScale,p.forceScale]>0));
m=base; r=scene.objectCylinder.radius; len=scene.objectCylinder.length;
mp=p.platformMass; mo=p.payloadMass; rp=zeros(3,1); ro=scene.objectCylinder.center_P+p.comOffset(:);
Ip=mp*diag([base.rB^2/4,base.rB^2/4,base.rB^2/2]);
Io=mo*p.inertiaScale*diag([r^2/2,(3*r^2+len^2)/12,(3*r^2+len^2)/12]);
com=(mp*rp+mo*ro)/(mp+mo); a=rp-com; b=ro-com;
I=Ip+Io+mp*((a.'*a)*eye(3)-a*a.')+mo*((b.'*b)*eye(3)-b*b.');
ev=sort(eig(I)); assert(all(isfinite(ev)) && ev(1)>0 && ev(3)<=ev(1)+ev(2)+1e-10,'SG:Inertia','Nonphysical inertia.');
m.dynamics.platformMass=mp; m.dynamics.objectMass=mo; m.dynamics.totalMass=mp+mo;
m.dynamics.comP=com; m.dynamics.inertiaAtCOM_P=I;
m.actuator.ldotMax=base.actuator.ldotMax*p.speedScale;
m.actuator.lddotMax=base.actuator.lddotMax*p.accelerationScale;
m.actuator.forceMin=base.actuator.forceMin*p.forceScale;
m.actuator.forceMax=base.actuator.forceMax*p.forceScale;
m.studyParameters=p;
end
