function [u,d]=ch5ReducedRecovery(x,previous,o,r,k,time,anchor)
% Retained-pose restoring feedback, followed by the existing strict safety QP.
% Check the actual next hold interval. Neither QP nor this finite check is a
% continuous-time or robust safety certificate.
poseError=[anchor(1:3)-x(1:3);atan2(sin(anchor(4:6)-x(4:6)),cos(anchor(4:6)-x(4:6)))];
desired=16*poseError-8*x(7:12);
[drift,G,~,~,~,~,~,~,~,geometric,~,aG]=r.ad.dynamics(x(1:6),x(7:12));
G=full(G);drift=full(drift);aG=full(aG);geometric=full(geometric);
% Generalized desired acceleration is clipped in leg space, where the hard
% acceleration constraints are defined, rather than clipping Euler rates.
[~,~,~,~,~,Jq]=r.ad.dynamics(x(1:6),x(7:12));Jq=full(Jq);
legDesired=Jq*desired+geometric;limit=.90*r.physical.actuator.lddotMax;
legDesired=min(max(legDesired,-limit),limit);desired=Jq\(legDesired-geometric);
nominal=G\(desired-drift);best=[];bestD=[];attempts={};
for margin=[.97,.90,.80]
    tightened=r;tightened.strict.legAccelerationLimit=margin*r.strict.legAccelerationLimit;
    [candidate,diagnostic]=ch5SafetyQp(x(1:6),x(7:12),anchor,zeros(6,1),zeros(6,1),nominal,previous,o,tightened,time);
    [~,trace]=r.plant.fd(x,candidate,zeros(6,1));trace=full(trace);
    values=zeros(19,size(trace,2));minimumGap=inf;
    for j=1:size(trace,2)
        value=full(r.plant.mechanicalValues(trace(:,j),candidate));values(:,j)=value(1:19);
        ot=o;ot(1:3,:)=o(1:3,:)+r.plant.traceTime(j)*o(16:18,:);
        for oi=find(o(22,:)>.5),gap=full(k.collision(trace(1:6,j),ot(:,oi)));minimumGap=min(minimumGap,gap);end
    end
    lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];
    hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
    violation=max([lo-values;values-hi;zeros(1,size(values,2))],[],'all');
    violation=max(violation,r.c.safe_distance+r.c.sample_hold_margin-minimumGap);
    valid=diagnostic.feasible&&all(isfinite(values),'all')&&violation<=1e-6;
    attempts{end+1}=struct('margin',margin,'qp_feasible',diagnostic.feasible,'hold_violation',violation,'valid',valid);
    if isempty(best)||violation<bestD.hold_violation,best=candidate;bestD=diagnostic;bestD.hold_violation=violation;end
    if valid,best=candidate;bestD=diagnostic;bestD.hold_violation=violation;break;end
end
u=best;d=bestD;d.usedFallback=true;d.fallbackSafetyCertified=false;d.recovery_attempts=attempts;
d.interval_valid=attempts{end}.valid;d.recovery_anchor=anchor;
end
