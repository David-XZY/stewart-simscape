function task=sgFinalizeTask(task)
task.scene.q0=task.q0; task.scene.qGoal=task.qGoal;
task.scene.phase.durationApproach=2*task.T/3; task.scene.phase.durationInsertion=task.T/3;
task.scene.objectCylinder.mass=task.model.dynamics.objectMass;
task.scene.collisionGeometry.obstacles=task.scene.hood.obstacles;
task.scene.collisionGeometry.objectCylinder=task.scene.objectCylinder;
task.context=dpContext(task); m=task.model; p=sgPhysics(m);
task.endpointPassed=true; task.endpointReason='';
for q=[task.q0,task.qGoal]
    [L,~,~,F,J]=p.point(q,zeros(6,1),zeros(6,1)); L=full(L); F=full(F);
    sv=svd(full(J)*diag([1,1,1,2,2,2])); gap=dpClearance(q,task,true);
    if any(~isfinite([L;F;sv;gap(:)])) || any(L<m.lmin-1e-8 | L>m.lmax+1e-8) || ...
            any(F<m.actuator.forceMin-1e-6 | F>m.actuator.forceMax+1e-6) || ...
            min(gap)<.01-1e-8 || min(sv)<.1 || max(sv)/min(sv)>20
        task.endpointPassed=false; task.endpointReason='Boundary violates length, static force, clearance or singularity.';
    end
end
if isfield(task,'hash'), task=rmfield(task,'hash'); end
task.hash=dpHash(task);
end
