function context=encodeTaskContext(task)
s=task.scene; m=task.model; obstacles=zeros(12,4);
assert(numel(s.hood.obstacles)<=4,'ABFD:ContextObstacles','Context v1 supports up to four OBBs.');
for k=1:numel(s.hood.obstacles)
    o=s.hood.obstacles(k); obstacles(:,k)=[o.center_S;o.halfSize;o.R_S(:,1);o.R_S(:,2)];
end
context=[s.q0;s.qWaypoint;s.qGoal;s.phase.durationApproach;s.phase.durationInsertion; ...
    m.dynamics.totalMass;m.dynamics.comP;obstacles(:)];
end
