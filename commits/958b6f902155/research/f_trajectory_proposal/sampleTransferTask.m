function task=sampleTransferTask(seed,split)
if nargin<2, split='train'; end
stream=RandStream('mt19937ar','Seed',seed); model=buildOptModelCustom();
scene=buildCylinderBoxTransferScene(model);
scene.q0=scene.q0+[.04*(rand(stream,3,1)-.5);deg2rad(6)*(rand(stream,3,1)-.5)];
scene.box.center_S=scene.box.center_S+.08*(rand(stream,3,1)-.5);
scene.box.rpy=scene.box.rpy+deg2rad(16)*(rand(stream,3,1)-.5); scene.box.R_S=rpy2rotmZYX(scene.box.rpy);
scene.phase.durationApproach=4+2*rand(stream); scene.phase.durationInsertion=scene.phase.durationApproach/2;
scene.phase.pCylinderGoal_B(1)=.02*(rand(stream)-.5);
scene.phase.insertionDirection_B=scene.phase.pCylinderGoal_B-scene.phase.pCylinderWaypoint_B;
names={'roof','leftSkirt','rightSkirt'};
for k=1:3
    obstacle=scene.hood.(names{k}); obstacle.center_S=scene.box.center_S+scene.box.R_S*obstacle.center_B;
    obstacle.R_S=scene.box.R_S; obstacle.axes_S=scene.box.R_S; scene.hood.(names{k})=obstacle;
end
scene.hood.obstacles=[scene.hood.roof,scene.hood.leftSkirt,scene.hood.rightSkirt];
[scene.qWaypoint,scene.qGoal,scene.target]=computeCylinderWaypointAndGoal(scene);
scene=sampleFeasibleStartPose(scene,model,seed);
scene=buildMultiObstacleScene(scene,mod(seed,2)==0);
scene.collisionGeometry=buildCylinderBoxCollisionModel(scene);
model.dynamics.objectMass=80+40*rand(stream); model=updateAbfdPayloadModel(model,scene);
task=struct('taskId',sprintf('task_%08d',seed),'split',split,'seed',seed, ...
    'loadId',sprintf('load_%08d',seed),'model',model,'scene',scene);
task.context=encodeTaskContext(task);
end
