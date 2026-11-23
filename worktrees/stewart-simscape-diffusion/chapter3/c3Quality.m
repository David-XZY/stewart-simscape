function q=c3Quality(bundle,task,c)
q=sgQuality(bundle,task,c);
R0=rpy2rotmZYX(task.q0(4:6)); R1=rpy2rotmZYX(task.qGoal(4:6));
angle=acos(max(-1,min(1,(trace(R0.'*R1)-1)/2)));
q.positionDetour=q.positionLengthM/max(norm(task.qGoal(1:3)-task.q0(1:3)),c.detourFloors(1));
q.rotationDetour=q.rotationLengthRad/max(angle,c.detourFloors(2));
q.values=cellfun(@(n)q.(n),c.metricNames);
end
