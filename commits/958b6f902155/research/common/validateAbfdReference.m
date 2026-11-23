function report=validateAbfdReference(refs,model,scene,config)
% Independent dense engineering checks, including phase-dependent roof gap.
assert(all(isfinite([refs.q(:);refs.qd(:);refs.qdd(:);refs.Fleg(:)])), ...
    'ABFD:NonfiniteReference','Nonfinite execution reference.');
report=struct('maxViolation',0,'minClearance',inf,'minSigma',inf, ...
    'maxInsertionError',0,'maxDynamicsResidual',refs.maxDynamicsResidual, ...
    'worstConstraintIndex',0,'worstTime',NaN,'maxAdditionalObstacleViolation',0);
waypoint=interp1(refs.t,refs.q.',scene.phase.durationApproach).';
report.maxBoundaryError=max(abs([refs.q(:,1)-scene.q0;refs.q(:,end)-scene.qGoal; ...
    refs.qd(:,1)-scene.qd0;refs.qd(:,end)-scene.qdGoal;waypoint-scene.qWaypoint]));
assert(isfinite(report.maxBoundaryError),'ABFD:ReferenceCoverage','Reference does not cover the waypoint time.');
for k=1:numel(refs.t)
    p=evaluatePathConstraintsAtPoint([refs.q(:,k);refs.qd(:,k)], ...
        refs.Fleg(:,k),model,scene,refs.qdd(:,k));
    required=scene.collision.safeDistance*ones(numel(p.collisionDistances),1);
    if refs.t(k)<=scene.phase.durationApproach+1e-10
        required(1)=scene.collision.stage1ConstraintDistance;
    else
        u=(refs.t(k)-scene.phase.durationApproach)/scene.phase.durationInsertion;
        % Roof clearance relaxes with insertion progress, not with wall-clock time.
        d=scene.phase.pCylinderGoal_B-scene.phase.pCylinderWaypoint_B;
        R=rpy2rotmZYX(refs.q(4:6,k));
        pc=scene.box.R_S.'*(refs.q(1:3,k)+R*scene.objectCylinder.center_P-scene.box.center_S);
        progress=(pc-scene.phase.pCylinderWaypoint_B).'*d/(d.'*d);
        required(1)=(1-progress)*scene.collision.stage2RoofStartGap+progress*scene.collision.finalGap;
        errorLine=norm(pc-scene.phase.pCylinderWaypoint_B-progress*d);
        omega=rpyRateMapZYX(refs.q(4:6,k))*refs.qd(4:6,k);
        vc=scene.box.R_S.'*(refs.qd(1:3,k)+cross(omega,R*scene.objectCylinder.center_P));
        errorLine=max([errorLine;norm(refs.q(4:6,k)-scene.box.rpy); ...
            -progress;progress-1;-(d.'*vc)/norm(d)]);
        report.maxInsertionError=max(report.maxInsertionError,errorLine);
        assert(u<=1+1e-8,'ABFD:ReferenceDuration','Reference exceeds task time.');
    end
    violations=[p.cPath;model.actuator.forceMin-refs.Fleg(:,k); ...
        refs.Fleg(:,k)-model.actuator.forceMax;required-p.collisionDistances; ...
        model.singularity.sigmaMinSafe-p.sigmaMin];
    [worst,index]=max(violations);
    report.maxAdditionalObstacleViolation=max([report.maxAdditionalObstacleViolation; ...
        required(4:end)-p.collisionDistances(4:end)]);
    if worst>report.maxViolation
        report.maxViolation=worst; report.worstConstraintIndex=index; report.worstTime=refs.t(k);
    end
    report.minClearance=min(report.minClearance,p.minClearance);
    report.minSigma=min(report.minSigma,p.sigmaMin);
end
report.passed=report.maxViolation<=config.constraintTolerance && ...
    report.maxBoundaryError<=config.constraintTolerance && ...
    report.maxInsertionError<=config.constraintTolerance && ...
    report.maxDynamicsResidual<=config.dynamicsTolerance;
end
