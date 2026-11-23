function test_sg_edges()
c=sgSetup('development'); task=makeTask('simple',20262041,'development',false);
task.qGoal=task.q0; task.T=.02;
b=struct('kind','spline','cp',repmat(task.q0,1,16),'T',task.T);
R=rpy2rotmZYX(task.q0(4:6)); object=task.scene.objectCylinder;
center=task.q0(1:3)+R*object.center_P; tested=0;
for axisIndex=[1,2]
    direction=R(:,axisIndex); support=object.radius;
    if axisIndex==1, support=object.length/2; end
    for gap=[-.02,.01-2e-8,.01,.01+2e-8]
        o=task.scene.hood.obstacles(1); o.R_S=R; o.halfSize=[.1;.1;.1];
        o.center_S=center+direction*(support+.1+gap); task.scene.hood.obstacles=o;
        fast=sgValidate(b,task,c); gold=sgAudit(b,task,c);
        assert(fast.passed==gold.passed,'Cylinder endpoint/side threshold discrepancy.');
        assert(fast.passed==(gap>=.01-1e-8),'Unexpected clearance acceptance.'); tested=tested+1;
    end
end
s=load(fullfile(c.source,'benchmark','simple_20262041','diffusion','benchmark.mat'));
bundle=s.outcome.best.bundle; task=s.task;
for limit={'ldotMax','lddotMax','forceMax'}
    changed=task; changed.model.actuator.(limit{1})(:)=1e-7;
    f=sgValidate(bundle,changed,c); g=sgAudit(bundle,changed,c); assert(~f.passed && ~g.passed);
end
asset=sgAssets(c); [a,~]=sgSample(task,4,901,asset); [b,~]=sgSample(task,64,901,asset);
prefix=b(:,:,1:4); assert(max(abs(a(:)-prefix(:)))<1e-10);
changed=task; changed.model=sgModel(task.model,task.scene,struct('payloadMass',125));
[differentPhysics,~]=sgSample(changed,4,901,asset); assert(isequal(a,differentPhysics),'Frozen model unexpectedly sees physical parameters.');
d=sgDiversity(a,task); old=dpDiversity(a,task);
assert(max(abs(d.positionRmsPairsM(:)-old.positionRmsPairsM(:)))<1e-10);
assert(max(abs(d.attitudeRmsPairsRad(:)-old.attitudeRmsPairsRad(:)))<1e-10);
dpWriteJson(fullfile(c.output,'edge_tests.json'),struct('passed',true,'collisionCases',tested, ...
    'dynamicLimitsChecked',3,'prefix64',true,'physicalParametersInvisibleToModel',true));
fprintf('SG_EDGES_PASSED\n');
end
