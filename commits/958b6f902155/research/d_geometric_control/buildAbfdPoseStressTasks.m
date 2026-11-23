function tasks=buildAbfdPoseStressTasks(seed,count)
% Freeze wide-attitude/near-singular feasible endpoints before controller comparison.
if nargin<1, seed=20270905; end
if nargin<2, count=12; end
stream=RandStream('mt19937ar','Seed',seed); pool={};
for attempt=1:500
    task=sampleTransferTask(seed+attempt,'test'); s=task.scene;
    s.box.rpy=deg2rad([60*(rand(stream)-.5);20+60*(rand(stream)-.5);45+100*(rand(stream)-.5)]);
    s.box.R_S=rpy2rotmZYX(s.box.rpy);
    for name={'roof','leftSkirt','rightSkirt'}
        o=s.hood.(name{1}); o.center_S=s.box.center_S+s.box.R_S*o.center_B;
        o.R_S=s.box.R_S; o.axes_S=s.box.R_S; s.hood.(name{1})=o;
    end
    s.hood.obstacles=[s.hood.roof,s.hood.leftSkirt,s.hood.rightSkirt];
    [s.qWaypoint,s.qGoal,s.target]=computeCylinderWaypointAndGoal(s);
    s.collisionGeometry=buildCylinderBoxCollisionModel(s);
    valid=true; minimum=inf;
    for q=[s.q0,s.qWaypoint,s.qGoal]
        kin=sgpIK(q,task.model); J=sgpJacobian(q,task.model);
        valid=valid && all(kin.L>=task.model.lmin & kin.L<=task.model.lmax) && J.sigmaMin>=task.model.singularity.sigmaMinSafe;
        minimum=min(minimum,J.sigmaMin);
    end
    if valid
        task.scene=s; task.context=encodeTaskContext(task); task.endpointSigma=minimum;
        pool{end+1}=task; %#ok<AGROW>
    end
end
assert(numel(pool)>=count,'ABFD:StressTasksUnavailable','Not enough feasible endpoint tasks; no infeasible task is relabelled feasible.');
poolValues=[pool{:}]; [~,order]=sort([poolValues.endpointSigma]); tasks=poolValues(order(1:count));
% Their complete trajectories still require IHSID and dense acceptance.
end
