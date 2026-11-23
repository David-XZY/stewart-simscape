function task=makeTask(family,seed,split,nominal)
if nargin<3, split='development'; end
if nargin<4, nominal=false; end
taskConfig=struct();
if isstruct(family)
    taskConfig=family; assert(isfield(taskConfig,'family'),'DP:TaskConfig','family is required.'); family=taskConfig.family;
    allowed={'family','duration','positionOffset','attitudeOffset','extraObstacleCount'};
    assert(all(ismember(fieldnames(taskConfig),allowed)),'DP:TaskConfig','Unknown task configuration field.');
end
validatestring(family,{'simple','hood'}); rngs=RandStream('mt19937ar','Seed',seed);
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
s.name='Full-horizon endpoint-conditioned cylinder planning';
s.collision.safeDistance=.01; s.collision.finalGap=.01; s.collision.stage1ConstraintDistance=.01; s.collision.smoothingEps=1e-10;
s.phase.pCylinderGoal_B(3)=s.hood.roof.center_B(3)-s.hood.roof.halfSize(3)-s.objectCylinder.radius-.01;
[~,nominalGoal]=computeCylinderWaypointAndGoal(s);
dp=zeros(3,1); dr=zeros(3,1); T=7.5;
if ~nominal, dp=.08*(rand(rngs,3,1)-.5); dr=deg2rad(16)*(rand(rngs,3,1)-.5); T=6+4*rand(rngs); end
if isfield(taskConfig,'duration'), T=taskConfig.duration; end
if isfield(taskConfig,'positionOffset'), dp=taskConfig.positionOffset(:); end
if isfield(taskConfig,'attitudeOffset'), dr=taskConfig.attitudeOffset(:); end
validateattributes(T,{'double'},{'scalar','positive','finite'});
validateattributes(dp,{'double'},{'numel',3,'finite','real'}); validateattributes(dr,{'double'},{'numel',3,'finite','real'});
s.phase.durationApproach=2*T/3; s.phase.durationInsertion=T/3;
s.phasePurpose='Geometric seed hint only; no intermediate or insertion constraint in the full-horizon NLP.';
s.box.rpy=s.box.rpy+dr; s.box.R_S=rpy2rotmZYX(s.box.rpy);
% Condition ranges refer to the platform endpoint, not the rotated box center.
s.box.center_S=nominalGoal(1:3)+dp-s.box.R_S*(s.phase.pCylinderGoal_B-s.objectCylinder.center_P);
names={'roof','leftSkirt','rightSkirt'};
for k=1:3
    o=s.hood.(names{k}); o.center_S=s.box.center_S+s.box.R_S*o.center_B;
    o.R_S=s.box.R_S; o.axes_S=o.R_S; s.hood.(names{k})=o;
end
s.hood.obstacles=[s.hood.roof,s.hood.leftSkirt,s.hood.rightSkirt];
[s.qWaypoint,qGoal,s.target]=computeCylinderWaypointAndGoal(s);
if strcmp(family,'hood')
    o=s.hood.roof; o.name='target-body'; o.center_B=zeros(3,1); o.center_S=s.box.center_S;
    o.size=s.box.size; o.halfSize=s.box.halfSize; s.hood.obstacles(end+1)=o;
else
    o=s.hood.roof; o.name='divider'; o.R_S=eye(3); o.axes_S=eye(3); o.size=[.08;.06;.08];
    if ~nominal, o.size=[.06;.04;.06]+.04*rand(rngs,3,1); o.R_S=rpy2rotmZYX(deg2rad(20)*(rand(rngs,3,1)-.5)); o.axes_S=o.R_S; end
    o.halfSize=o.size/2;
    probe=struct('scene',s); placed=false;
    for fraction=[.5,.4,.6]
        for axial=[.43,-.43,.25,-.25]
            for radial=[.14,.17,.20]
                for angle=[0,pi/2,pi,3*pi/2]
                    mid=(1-fraction)*m.qHome+fraction*qGoal; R=rpy2rotmZYX(mid(4:6));
                    o.center_S=mid(1:3)+R*(s.objectCylinder.center_P+[axial;radial*cos(angle);radial*sin(angle)]);
                    o.center_B=o.center_S; probe.scene.hood.obstacles=o;
                    if dpClearance(m.qHome,probe,false)>.012 && dpClearance(qGoal,probe,false)>.012 && dpClearance(mid,probe,false)<.008
                        placed=true; break;
                    end
                end
                if placed, break; end
            end
            if placed, break; end
        end
        if placed, break; end
    end
    assert(placed,'DP:Divider','Could not place an endpoint-safe obstructing divider.');
    s.hood.obstacles=o;
end
s.q0=m.qHome; s.qGoal=qGoal;
extraCount=0; if ~nominal, extraCount=mod(seed,3); end
if isfield(taskConfig,'extraObstacleCount'), extraCount=taskConfig.extraObstacleCount; end
validateattributes(extraCount,{'double'},{'scalar','integer','>=',0,'<=',2});
for j=1:extraCount
    o=s.hood.obstacles(1); o.name=sprintf('external_%d',j); o.size=.06+.08*rand(rngs,3,1); o.halfSize=o.size/2;
    o.R_S=rpy2rotmZYX(deg2rad(60)*(rand(rngs,3,1)-.5)); o.axes_S=o.R_S; placed=false;
    for retry=1:50
        o.center_S=.5*(s.q0(1:3)+qGoal(1:3))+[.9*(rand(rngs)-.5);.9*(rand(rngs)-.5);.1+.5*rand(rngs)];
        o.center_B=s.box.R_S.'*(o.center_S-s.box.center_S); probe=struct('scene',s); probe.scene.hood.obstacles=o;
        if dpClearance(s.q0,probe,true)>.025 && dpClearance(qGoal,probe,true)>.025, placed=true; break; end
    end
    assert(placed,'DP:ExternalObstacle','No endpoint-safe external obstacle position.');
    s.hood.obstacles(end+1)=o;
end
s.collisionGeometry=struct('obstacles',s.hood.obstacles,'obstacleNames',{{s.hood.obstacles.name}}, ...
    'objectCylinder',s.objectCylinder,'safeDistance',.01,'finalGap',.01);
task=struct('schemaVersion',1,'taskId',sprintf('%s_%08d',family,seed),'groupId',sprintf('%s_%08d',family,seed), ...
    'family',family,'seed',seed,'split',split,'T',T,'q0',m.qHome,'qGoal',qGoal,'model',m,'scene',s);
task.nominalGoal=nominalGoal; task.targetPositionOffset=dp; task.targetAttitudeOffset=dr;
task.context=dpContext(task); p=dpPhysics(m); task.endpointPassed=true; task.endpointReason='';
for q=[task.q0,task.qGoal]
    [L,~,~,F,J]=p.point(q,zeros(6,1),zeros(6,1)); L=full(L); F=full(F); J=full(J)*diag([1,1,1,2,2,2]);
    sv=svd(J); d=dpClearance(q,task);
    if any(L<m.lmin-1e-8 | L>m.lmax+1e-8) || any(abs(F)>2000+1e-6) || min(d)<.01-1e-8 || min(sv)<.1 || max(sv)/min(sv)>20
        task.endpointPassed=false; task.endpointReason='Endpoint violates length/force/clearance/singularity constraints.';
    end
end
task.hash=dpHash(task);
end
