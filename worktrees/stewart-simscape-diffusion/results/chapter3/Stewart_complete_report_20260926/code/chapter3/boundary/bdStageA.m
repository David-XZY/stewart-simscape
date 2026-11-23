function bdStageA()
% Bounded Stage-A audit only: no optimization, sampling or training.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'chapter3')); c=c3Setup('boundary_stageA_v2');
m=buildOptModelCustom(); scene=buildCylinderBoxTransferScene(m);
scene.name='Ordinary industrial cylinder transfer with two independently placed fixtures';
scene.collision.safeDistance=.01; scene.collision.finalGap=.01;
scene.collision.stage1ConstraintDistance=.01; scene.collision.smoothingEps=1e-10;
scene.q0=[-.12;-.18;1.04;0;0;0]; scene.qGoal=[.12;.18;1.08;0;0;0];
template=scene.hood.obstacles(1);
A=template; A.name='fixture_A_front'; A.center_S=[.60;-.17;1.30];
A.center_B=A.center_S; A.size=[.10;.16;.12]; A.halfSize=A.size/2;
A.R_S=eye(3); A.axes_S=eye(3);
B=A; B.name='fixture_B_rear'; B.center_S=[-.60;.17;1.30]; B.center_B=B.center_S;
scene.hood.obstacles=[A,B];
scene.qWaypoint=(scene.q0+scene.qGoal)/2;
scene.phasePurpose='Full-horizon endpoints only; qWaypoint is unused geometric metadata.';
scene.collisionGeometry.obstacles=scene.hood.obstacles;
scene.collisionGeometry.safeDistance=.01; scene.collisionGeometry.finalGap=.01;
task=struct('schemaVersion',1,'taskId','dual_fixture_stageA_001', ...
    'groupId','dual_fixture_stageA_001','family','dual_fixture','seed',20260919, ...
    'split','development_only','T',7.5,'q0',scene.q0,'qGoal',scene.qGoal,'model',m,'scene',scene);
task=sgFinalizeTask(task); p=sgPhysics(m); endpoints={};
for k=1:2
    if k==1, q=task.q0; else, q=task.qGoal; end
    [L,~,~,F,J]=p.point(q,zeros(6,1),zeros(6,1));
    sv=svd(full(J)*diag([1,1,1,2,2,2]));
    R=rpy2rotmZYX(q(4:6)); center=q(1:3)+R*scene.objectCylinder.center_P;
    axis=R*scene.objectCylinder.axis_P;
    endpoints{k}=struct('q',q,'legLength',full(L),'staticForce',full(F), ...
        'sigmaMin',min(sv),'condition',max(sv)/min(sv), ...
        'clearanceLowerBounds',dpClearance(q,task,true), ...
        'front',center+.5*scene.objectCylinder.length*axis, ...
        'rear',center-.5*scene.objectCylinder.length*axis);
end
% Probe mechanisms geometrically without reading any learned model output.
u=linspace(0,1,41); straight=task.q0+(task.qGoal-task.q0)*u;
coordinated=straight; coordinated(6,:)=deg2rad(20)*sin(pi*u).^2;
probe={};
for j=1:2
    if j==1, Q=straight; name='straight_pose_path'; else, Q=coordinated; name='yaw_bump_pose_path'; end
    gaps=zeros(2,numel(u)); robotPlaneGap=inf;
    for k=1:numel(u)
        gaps(:,k)=dpClearance(Q(:,k),task,true).';
        R=rpy2rotmZYX(Q(4:6,k)); attachments=Q(1:3,k)+R*m.B;
        % Declared example enclosure, not actual CAD dimensions: disk radius
        % 0.55 m, half-thickness 0.05 m; leg/joint capsule radius 0.06 m.
        diskTop=Q(3,k)+.55*norm(R(3,1:2))+.05*abs(R(3,3));
        legTop=max([m.A(3,:),attachments(3,:)])+.06;
        fixtureBottom=min([A.center_S(3)-A.halfSize(3),B.center_S(3)-B.halfSize(3)]);
        robotPlaneGap=min(robotPlaneGap,fixtureBottom-max(diskTop,legTop));
    end
    probe{j}=struct('name',name,'q',Q,'s',u,'clearanceLowerBounds',gaps, ...
        'minCertificate',min(gaps(:)),'declaredRobotEnclosureVerticalGap',robotPlaneGap, ...
        'status','Geometry probes only. A failed lower-bound certificate is not proof of collision.');
end
result=struct('stage','A','endpointPassed',task.endpointPassed,'endpoints',{endpoints}, ...
    'probes',{probe},'optimizationCalls',0,'trainingCalls',0,'diffusionCalls',0, ...
    'robotEnclosureStatus','Proposed conservative envelopes, not verified against actual body CAD; no whole-robot claim.', ...
    'mechanismStatus','Endpoint legality verified; dynamic feasibility, coupled necessity and historical boundary not established.', ...
    'taskHash',task.hash,'matlab',version);
save(fullfile(c.output,'stageA.mat'),'task','result','-v7');
dpWriteJson(fullfile(c.output,'stageA.json'),result);
dpWriteJson(fullfile(c.output,'task.json'),task);
assert(task.endpointPassed,'BD:Endpoint','Proposed prototype endpoints fail existing physical gate.');
fprintf('BD_STAGE_A_COMPLETE endpoints=%d; optimization=0; training=0\n',task.endpointPassed);
end
