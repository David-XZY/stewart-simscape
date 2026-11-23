function stage=ch5ReducedFactorStage(r)
% Exact chain rule through small RK4 maps; bound generated AD/compiler graphs.
% Physical acceptance still uses the shared original fd and its full mesh.
assert(r.mode=="nonideal-force"&&r.plant.n==30&&r.plant.delayCount==1&&all(r.plant.profile.deadzone==0), ...
    'ch5:FactorDomain','Factorized diagnostic supports only the registered nonideal profile.');
persistent cached keySaved
key=char(r.mode+"|"+r.c.version);
if ~isempty(keySaved)&&strcmp(keySaved,key),stage=cached;return;end
import casadi.*
p=r.plant.profile;z=MX.sym('factor_substate',24);target=MX.sym('factor_target',6);h=MX.sym('factor_h');
qv=z(1:12);slew=z(13:18);lag=z(19:24);
[~,~,f0]=actuator(slew,lag,target,0,p);[~,~,fm]=actuator(slew,lag,target,h/2,p);
[sn,ln,f1]=actuator(slew,lag,target,h,p);
k1=r.plant.rhs(qv,f0,MX.zeros(6,1));k2=r.plant.rhs(qv+h*k1/2,fm,MX.zeros(6,1));
k3=r.plant.rhs(qv+h*k2/2,fm,MX.zeros(6,1));k4=r.plant.rhs(qv+h*k3,f1,MX.zeros(6,1));
next=[qv+h*(k1+2*k2+2*k3+k4)/6;sn;ln];
local=Function('ch5_factor_substep',{z,target,h},{next,jacobian(next,z),jacobian(next,target)},ch5ProjectionJitOptions());
x=MX.sym('factor_state',30);u=MX.sym('factor_command',6);state=MX(x);A=MX.eye(30);B=MX.zeros(30,6);
durations=[p.delay,r.c.Ts-p.delay];targets={x(25:30),u};
for segment=1:2
    steps=max(1,ceil(durations(segment)/(r.c.Ts/r.plant.substeps)));dt=durations(segment)/steps;
    for j=1:steps
        [next,a,b]=local(state(1:24),targets{segment},dt);
        if segment==1
            fullA=[a,b;MX.zeros(6,24),MX.eye(6)];fullB=MX.zeros(30,6);
        else
            fullA=[a,MX.zeros(24,6);MX.zeros(6,24),MX.eye(6)];fullB=[b;MX.zeros(6,6)];
        end
        A=mtimes(fullA,A);B=mtimes(fullA,B)+fullB;state(1:24)=next;
    end
end
state(25:30)=u;A(25:30,:)=MX.zeros(6,30);B(25:30,:)=MX.eye(6);
stage=Function('ch5_factor_stage',{x,u},{state,A,B},ch5ProjectionJitOptions());cached=stage;keySaved=key;
end
function [slew,lag,F]=actuator(slew0,lag0,target,h,p)
import casadi.*
delta=target-slew0;direction=if_else(delta>=0,1,-1);magnitude=if_else(delta>=0,delta,-delta);
slope=direction.*p.slewRate;active=fmin(h,magnitude./p.slewRate);
atRamp=slew0+slope.*(active-p.tau)+(lag0-slew0+slope.*p.tau).*exp(-active./p.tau);
slew=slew0+fmin(fmax(delta,-p.slewRate*h),p.slewRate*h);
lag=target+(atRamp-target).*exp(-(h-active)./p.tau);F=fmin(fmax(lag,p.forceMin),p.forceMax);
end
