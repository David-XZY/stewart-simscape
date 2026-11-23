function spline=fitTrajectorySpline(t,Q,scene)
T=scene.phase.durationApproach; use=t<=T+1e-10; t=t(use); Q=Q(:,use);
cp=zeros(6,10); cp(:,1:3)=repmat(scene.q0,1,3); cp(:,8:10)=repmat(scene.qWaypoint,1,3);
base=evaluateTrajectorySpline(cp,t,T); B=zeros(4,numel(t));
for k=1:4
    probe=zeros(6,10); probe(1,k+3)=1; val=evaluateTrajectorySpline(probe,t,T); B(k,:)=val(1,:);
end
cp(:,4:7)=(Q-base)*B.'/(B*B.'+1e-10*eye(4));
fit=evaluateTrajectorySpline(cp,t,T);
spline=struct('controlPoints',cp,'features',reshape(cp(:,4:7),24,1), ...
    'duration',T,'maxFitError',max(abs(fit-Q),[],'all'));
end
