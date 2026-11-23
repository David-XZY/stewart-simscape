function r=bdEndpointCheck(task,c)
t=sgFinalizeTask(task); r=struct('passed',t.endpointPassed,'reason',t.endpointReason);
g=bdRobotEnvelope(task,[task.q0,task.qGoal],c); r.envelope=g;
if ~g.passed, r.passed=false; r.reason='robot_envelope_not_certified'; end
end
