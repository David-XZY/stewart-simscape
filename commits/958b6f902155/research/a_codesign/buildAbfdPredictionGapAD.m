function fun=buildAbfdPredictionGapAD(scene,obstacle,index,insertion)
% Signed clearance minus the same phase-dependent requirement as dense checks.
import casadi.*
q=SX.sym('mean_gap_q',6); n=SX.sym('frozen_separator',3);
[R,~,~]=abfdRotationExpressions(q(4:6),SX.zeros(3,1));
axis=R*scene.objectCylinder.axis_P; center=q(1:3)+R*scene.objectCylinder.center_P;
epsilon=scene.collision.smoothingEps;
transverse=n-axis*(n.'*axis);
gap=n.'*(obstacle.center_S-center)-sum(obstacle.halfSize.*abs(obstacle.R_S.'*n))- ...
    scene.objectCylinder.length/2*sqrt((n.'*axis)^2+epsilon^2)- ...
    scene.objectCylinder.radius*sqrt(transverse.'*transverse+epsilon^2);
required=scene.collision.safeDistance;
if index==1
    required=scene.collision.stage1ConstraintDistance;
    if insertion
        d=scene.phase.pCylinderGoal_B-scene.phase.pCylinderWaypoint_B;
        pc=scene.box.R_S.'*(center-scene.box.center_S);
        progress=(pc-scene.phase.pCylinderWaypoint_B).'*d/(d.'*d);
        required=(1-progress)*scene.collision.stage2RoofStartGap+progress*scene.collision.finalGap;
    end
end
clearance=gap-required;
fun=Function('abfd_prediction_clearance',{q,n},{clearance,jacobian(clearance,q)});
end
