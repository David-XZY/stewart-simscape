function r=pgEndpointCheck(task,c)
% Static boundary samples use exactly the same physical gate as trajectories.
q=[task.q0,task.qGoal]; p=pgPhysics(task.model); m=task.model;
[L,~,~,F,J]=p.point(q,zeros(6,2),zeros(6,2)); L=full(L); F=full(F); J=full(J);
r=struct('passed',true,'reason','passed');
if any(~isfinite([q(:);L(:);F(:);J(:)])), r.passed=false; r.reason='nonfinite'; return; end
if any(any(L<m.lmin-1e-6|L>m.lmax+1e-6)), r.passed=false; r.reason='endpoint_leg'; return; end
if any(any(F<m.actuator.forceMin-1e-6|F>m.actuator.forceMax+1e-6)), r.passed=false; r.reason='endpoint_force'; return; end
for k=1:2
    s=svd(J(:,(k-1)*6+(1:6))*diag([1,1,1,2,2,2]));
    if min(s)<.1||max(s)/min(s)>20, r.passed=false; r.reason='endpoint_singularity'; return; end
    g=pgClearance(q(:,k),task,true);
    if any(~isfinite(g))||any(g<.01-1e-8), r.passed=false; r.reason='endpoint_clearance_not_certified'; return; end
end
e=bdRobotEnvelope(task,q,c); if ~e.passed, r.passed=false; r.reason='endpoint_envelope'; end
end
