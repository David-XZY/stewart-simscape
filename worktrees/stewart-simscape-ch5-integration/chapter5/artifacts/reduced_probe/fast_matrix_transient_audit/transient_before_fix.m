function audit=ch5ReducedTransient(nodes,U,o,r,accelerationLimit)
% Extra physical points inside the <=200-us bounded actuator slew transition.
% Exact actuator ramp/lag formula + variable-time RK4 mechanical prediction.
% A finite refinement, explicitly not a continuous-time safety certificate.
if nargin<5,accelerationLimit=ch5PredictionAccelerationLimit(r);end
audit=struct('accepted',true,'maximum_violation',0,'samples',0,'minimum_gap',inf);
if r.mode=="ideal-force",return;end
p=r.plant.profile;
assert(r.plant.delayCount==1&&p.deadzone==0&&p.delay<.0025,'ch5:TransientDomain','Unsupported actuator profile.');
persistent point keySaved
key=char(r.mode+"|"+r.c.version);
if isempty(keySaved)||~strcmp(keySaved,key)
    import casadi.*
    x=MX.sym('transient_state',r.plant.n);u=MX.sym('transient_command',6);time=MX.sym('transient_time');state=x;
    durations={fmin(time,p.delay),fmax(time-p.delay,0)};targets={x(25:30),u};
    for seg=1:2
        h=durations{seg};target=targets{seg};qv=state(1:12);slew=state(13:18);lag=state(19:24);
        [~,~,f0]=actuator(slew,lag,target,0,p);[~,~,fm]=actuator(slew,lag,target,h/2,p);
        [sn,ln,f1]=actuator(slew,lag,target,h,p);
        k1=r.plant.rhs(qv,f0,MX.zeros(6,1));k2=r.plant.rhs(qv+h*k1/2,fm,MX.zeros(6,1));
        k3=r.plant.rhs(qv+h*k2/2,fm,MX.zeros(6,1));k4=r.plant.rhs(qv+h*k3,f1,MX.zeros(6,1));
        state(1:12)=qv+h*(k1+2*k2+2*k3+k4)/6;state(13:24)=[sn;ln];
    end
    point=Function('ch5_transient_point',{x,u,time},{state,r.plant.mechanicalValues(state,u)},ch5ProjectionJitOptions());keySaved=key;
end
H=size(U,2);grid=[25;50;75;100;150;200]*1e-6;
ramp=abs(U-nodes(13:18,1:H))./p.slewRate;
times=[repmat(grid+p.delay,1,H);fmin(ramp+p.delay,.0025)];count=size(times,1);
times=min(max(times,0),.0025);
mapped=point.map(count*H);[states,values]=mapped(repelem(nodes(:,1:H),1,count),repelem(U,1,count),reshape(times,1,[]));
states=full(states);values=full(values);
lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-accelerationLimit;r.strict.sigmaSafe^2;r.strict.forceMin];
hi=[r.physical.lmax;r.physical.actuator.ldotMax;accelerationLimit;inf;r.strict.forceMax];
violation=max([lo-values;values-hi;zeros(1,size(values,2))],[],'all');gap=inf;
if any(o(22,:)>.5)
    absoluteTimes=reshape(times+(0:H-1)*r.c.Ts,1,[]);collision=r.collision.single.map(count*H);
    for oi=find(o(22,:)>.5)
        ot=repmat(o(:,oi),1,count*H);ot(1:3,:)=ot(1:3,:)+o(16:18,oi)*absoluteTimes;
        gaps=full(collision(states(1:6,:),ot));gap=min(gap,min(gaps));
    end
    violation=max(violation,r.c.safe_distance+r.c.sample_hold_margin-gap);
end
audit=struct('accepted',all(isfinite(values),'all')&&all(isfinite(states),'all')&&violation<=1e-6, ...
    'maximum_violation',violation,'samples',count*H,'minimum_gap',gap,'maximum_acceleration',max(abs(values(13:18,:)),[],'all'));
end
function [slew,lag,F]=actuator(slew0,lag0,target,h,p)
import casadi.*
slope=sign(target-slew0).*p.slewRate;ramp=abs(target-slew0)./p.slewRate;active=fmin(h,ramp);
atRamp=slew0+slope.*(active-p.tau)+(lag0-slew0+slope*p.tau).*exp(-active/p.tau);
slew=slew0+slope.*active;lag=target+(atRamp-target).*exp(-(h-active)/p.tau);
F=fmin(fmax(lag,p.forceMin),p.forceMax);
end
