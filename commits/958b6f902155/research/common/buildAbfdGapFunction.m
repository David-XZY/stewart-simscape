function fun=buildAbfdGapFunction(scene,obstacle,n)
import casadi.*
q=SX.sym('gap_q',6); [R,~,~]=abfdRotationExpressions(q(4:6),SX.zeros(3,1));
axis=R*scene.objectCylinder.axis_P; center=q(1:3)+R*scene.objectCylinder.center_P;
epsi=scene.collision.smoothingEps;
gap=n.'*(obstacle.center_S-center)-sum(obstacle.halfSize.*abs(obstacle.R_S.'*n))- ...
    scene.objectCylinder.length/2*sqrt((n.'*axis)^2+epsi^2)- ...
    scene.objectCylinder.radius*sqrt(1-(n.'*axis)^2+epsi^2);
fun=Function('abfd_support_gap',{q},{gap,jacobian(gap,q)});
end
