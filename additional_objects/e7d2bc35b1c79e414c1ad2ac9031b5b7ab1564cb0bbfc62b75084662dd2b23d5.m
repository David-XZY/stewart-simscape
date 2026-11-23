function fd=ch5DerivativeMap(r)
% Algebraically identical actuator-augmented fd for optimization derivatives.
% sign(delta)*min(h,abs(delta)/rate) has the wrong AD derivative at delta=0.
% Select the ramp direction and magnitude with the same piecewise branch so
% its unsaturated product differentiates to delta. The physical acceptance
% gate still executes the original shared plant.fd and full rollout.
% ch5CheckDerivativeMap checks the full state/input Jacobians and primal map.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);
if isKey(caches,key),fd=caches(key);return;end
import casadi.*
if r.mode=="ideal-force",fd=r.plant.fd;caches(key)=fd;return;end
p=r.plant.profile;c=r.c;n=r.plant.n;
x=MX.sym('smooth_x',n);u=MX.sym('smooth_u',6);state=MX(x);
memory=reshape(x(25:end),6,r.plant.delayCount);
integer=floor(p.delay/c.Ts);fraction=p.delay-integer*c.Ts;sequence=[u,memory];
if fraction>1e-14
    targets={sequence(:,integer+2),sequence(:,integer+1)};durations=[fraction,c.Ts-fraction];
else,targets={sequence(:,integer+1)};durations=c.Ts;end
for seg=1:numel(durations)
    duration=durations(seg);target=targets{seg};
    if p.deadzone>0,target=sign(target).*fmax(abs(target)-p.deadzone,0);end
    steps=max(1,ceil(duration/(c.Ts/r.plant.substeps)));h=duration/steps;
    for j=1:steps
        qv=state(1:12);slew=state(13:18);lag=state(19:24);
        [~,~,f0]=actuator(slew,lag,target,0,p);
        [~,~,fm]=actuator(slew,lag,target,h/2,p);
        [sn,ln,f1]=actuator(slew,lag,target,h,p);
        k1=r.plant.rhs(qv,f0,zeros(6,1));k2=r.plant.rhs(qv+h*k1/2,fm,zeros(6,1));
        k3=r.plant.rhs(qv+h*k2/2,fm,zeros(6,1));k4=r.plant.rhs(qv+h*k3,f1,zeros(6,1));
        state(1:12)=qv+h*(k1+2*k2+2*k3+k4)/6;state(13:24)=[sn;ln];
    end
end
state(25:end)=reshape([u,memory(:,1:end-1)],6*r.plant.delayCount,1);
fd=Function('ch5_differentiable_fd',{x,u},{state},struct('max_num_dir',4));caches(key)=fd;
end
function [slew,lag,F]=actuator(slew0,lag0,target,h,p)
import casadi.*
delta=target-slew0;direction=if_else(delta>=0,1,-1);
magnitude=if_else(delta>=0,delta,-delta);slope=direction.*p.slewRate;
ramp=magnitude./p.slewRate;active=fmin(h,ramp);
atRamp=slew0+slope.*(active-p.tau)+(lag0-slew0+slope*p.tau).*exp(-active/p.tau);
slew=slew0+fmin(fmax(delta,-p.slewRate*h),p.slewRate*h);
lag=target+(atRamp-target).*exp(-(h-active)/p.tau);
F=fmin(fmax(lag,p.forceMin),p.forceMax);
end
