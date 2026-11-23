function [val,grad]=pgDetourPenalty(z,task,c)
import casadi.*
assert(c.qualityFrozen,'PG:Quality','Frozen shape-specific caps required.');
persistent key fun;
k=dpHash(struct('task',task.hash,'caps',[c.positionCap,c.rotationCap],'n',c.controlCount));
if isempty(key)||~strcmp(k,key)
    x=MX.sym('free',numel(z)); cp=[repmat(task.q0,1,3),reshape(x,6,c.controlCount-6),repmat(task.qGoal,1,3)];
    [B,D,~]=bdBasis(linspace(0,task.T,33),task.T,c.controlCount); q=cp*B; v=cp*D;
    p=pgPhysics(task.model); [~,omega]=p.rotation(q,v,MX.zeros(6,33)); weights=[.5,ones(1,31),.5]*task.T/32;
    plen=sum2(sqrt(sum1(v(1:3,:).^2)+1e-12).*weights); rlen=sum2(sqrt(sum1(omega.^2)+1e-12).*weights);
    R0=rpy2rotmZYX(task.q0(4:6)); Rg=rpy2rotmZYX(task.qGoal(4:6)); angle=acos(max(-1,min(1,(trace(R0.'*Rg)-1)/2)));
    cost=fmax(plen/max(norm(task.qGoal(1:3)-task.q0(1:3)),.01)-c.positionCap,0)^2+fmax(rlen/max(angle,pi/180)-c.rotationCap,0)^2;
    fun=Function('pg_detour',{x},{cost,gradient(cost,x)}); key=k;
end
[val,grad]=fun(z); val=full(val); grad=full(grad);
end
