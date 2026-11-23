function [bundle,log]=bdRepair(bundle,task,clockHandle,budget,c)
% Bounded shape repair with a fixed task reference, never learned score weights.
import casadi.*
timer=tic; n=c.controlCount; t=linspace(0,task.T,61); [B,D,DD]=bdBasis(t,task.T,n);
if strcmp(bundle.kind,'spline'), initial=bundle.cp;
else, tr=bdReconstruct(bundle,t); initial=bdBaseline(task,n); initial(:,4:end-3)=0; initial(:,4:end-3)=(tr.q-initial*B)/B(4:end-3,:); end
z=MX.sym('repair',(n-6)*6); scale=repmat([.1;.1;.1;.2;.2;.2],n-6,1);
cp=MX(initial); cp(:,4:end-3)=cp(:,4:end-3)+reshape(z.*scale,6,n-6);
Q=cp*B; V=cp*D; A=cp*DD; p=sgPhysics(task.model); m=task.model;
[L,Ld,Ldd,F,~,~,phi]=p.point(Q,V,A); ref=bdBaseline(task,n)*B;
penalty=sum2(sum1(fmax(0,(m.lmin-L)/.1).^2+fmax(0,(L-m.lmax)/.1).^2+fmax(0,abs(Ld)./m.actuator.ldotMax-1).^2+fmax(0,abs(Ldd)./m.actuator.lddotMax-1).^2+fmax(0,(m.actuator.forceMin-F)/2000).^2+fmax(0,(F-m.actuator.forceMax)/2000).^2));
for k=1:numel(t)
    R=p.pose(Q(:,k)); axis=R*task.scene.objectCylinder.axis_P; center=Q(1:3,k)+R*task.scene.objectCylinder.center_P;
    for j=1:numel(task.scene.hood.obstacles)
        o=task.scene.hood.obstacles(j); dirs=[eye(3),-eye(3),o.R_S,-o.R_S]; ax=axis.'*dirs;
        gap=mmax((o.center_S-center).'*dirs-o.halfSize.'*abs(o.R_S.'*dirs)-task.scene.objectCylinder.length/2*abs(ax)-task.scene.objectCylinder.radius*sqrt(fmax(0,1-ax.^2)+1e-12));
        robot=bdEnvelopeSymbolic(Q(:,k),o,m,c.envelope,p);
        penalty=penalty+10*fmax(0,(.011-gap)/.01)^2+sum1(fmax(0,(.011-robot)/.01).^2);
    end
end
cost=.1*sum2(sum1(((Q-ref)./[.1;.1;.1;.1745329252;.1745329252;.1745329252]).^2))+.3*sum2(sum1((Ldd/1.2).^2))+.4*sum2(phi)+.08*sum2(sum1((diff(F,1,2)/(t(2)-t(1))/1000).^2));
objective=100*penalty+cost;
if c.qualityFrozen && strcmp(c.returnPolicy,'quality')
    plen=(t(2)-t(1))*sum2(sqrt(sum1(V(1:3,:).^2)+1e-12));
    objective=objective+100*fmax(0,plen/max(norm(task.qGoal(1:3)-task.q0(1:3)),.01)-c.positionCap)^2;
    pitch=Q(5,:); yaw=Q(6,:); rd=V(4,:); pd=V(5,:); yd=V(6,:);
    omega=[cos(yaw).*cos(pitch).*rd-sin(yaw).*pd;sin(yaw).*cos(pitch).*rd+cos(yaw).*pd;-sin(pitch).*rd+yd];
    rlen=(t(2)-t(1))*sum2(sqrt(sum1(omega.^2)+1e-12)); R0=rpy2rotmZYX(task.q0(4:6)); Rg=rpy2rotmZYX(task.qGoal(4:6));
    angle=acos(max(-1,min(1,(trace(R0.'*Rg)-1)/2)));
    objective=objective+100*fmax(0,rlen/max(angle,pi/180)-c.rotationCap)^2;
end
fun=Function('bd_repair',{z},{objective,gradient(objective,z)}); value=zeros(size(scale)); accepted=0;
for it=1:c.repairIterations
    if toc(clockHandle)>=budget, break; end
    [cost,grad]=fun(value); cost=full(cost); grad=full(grad);
    if any(~isfinite([cost;grad])), break; end
    direction=grad/max(1e-12,sqrt(mean(grad.^2))); improved=false;
    for bt=0:6
        if toc(clockHandle)>=budget, break; end
        trial=value-.02/2^bt*direction; if sqrt(mean(trial.^2))>.5, continue; end
        trialCost=full(fun(trial));
        if trialCost<cost, value=trial; accepted=accepted+1; improved=true; break; end
    end
    if ~improved, break; end
end
cp=initial; cp(:,4:end-3)=cp(:,4:end-3)+reshape(value.*scale,6,n-6);
bundle=struct('kind','spline','T',task.T,'cp',cp);
log=struct('iterations',it,'acceptedSteps',accepted,'seconds',toc(timer),'shapeChanged',true,'withinBudget',toc(clockHandle)<=budget);
end
