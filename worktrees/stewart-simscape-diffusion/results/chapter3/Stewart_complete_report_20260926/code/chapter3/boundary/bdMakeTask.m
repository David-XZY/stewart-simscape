function task=bdMakeTask(baseSpec,params,c)
% params=[fixture A y offset, fixture B y offset, goal yaw, duration], SI.
if nargin<1 || isempty(baseSpec), baseSpec=struct(); end
assert(numel(params)==4 && all(isfinite(params)) && params(4)>0,'BD:Parameters','Invalid task parameters.');
m=buildOptModelCustom(); scene=buildCylinderBoxTransferScene(m);
scene.name='Industrial cylinder / independent dual fixtures / full horizon';
scene.collision.safeDistance=.01; scene.collision.finalGap=.01;
scene.collision.stage1ConstraintDistance=.01;
q0=[-.12;-.18;1.04;0;0;0]; qg=[.12;.18;1.08;0;0;params(3)];
A=scene.hood.obstacles(1); A.name='fixture_A_front';
A.center_S=[.60;-.17+params(1);1.30]; A.center_B=A.center_S;
A.size=[.10;.16;.12]; A.halfSize=A.size/2; A.R_S=eye(3); A.axes_S=eye(3);
B=A; B.name='fixture_B_rear'; B.center_S=[-.60;.17+params(2);1.30]; B.center_B=B.center_S;
scene.hood.obstacles=[A,B]; scene.qWaypoint=(q0+qg)/2;
scene.phasePurpose='No waypoint or insertion constraints';
scene.collisionGeometry.safeDistance=.01; scene.collisionGeometry.finalGap=.01;
task=struct('schemaVersion',1,'taskId',['bd_',dpHash(params)],'groupId','development', ...
    'baseSceneId','dual_fixture_01','family','dual_fixture','split','development', ...
    'seed',20260919,'T',params(4),'q0',q0,'qGoal',qg,'model',m,'scene',scene, ...
    'parameters',params(:).','envelope',c.envelope);
for name={'taskId','groupId','split','macroCellId','baseSceneId'}
    if isfield(baseSpec,name{1}), task.(name{1})=baseSpec.(name{1}); end
end
task=sgFinalizeTask(task);
r=bdEndpointCheck(task,c); task.endpointPassed=r.passed; task.endpointReason=r.reason;
task=rmfield(task,'hash'); task.hash=dpHash(task);
end
