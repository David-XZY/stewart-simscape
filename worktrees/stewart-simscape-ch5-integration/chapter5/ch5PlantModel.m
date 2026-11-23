function model = ch5PlantModel(physical,scene,p,c)
% Exact existing composite mechanics, with a differentiated RK4 ZOH map.
% This explicit scene adapter supports the existing cylinder/hood scene only.
assert(isfield(scene,'hood') && isfield(scene,'objectCylinder'), ...
    'TrackingMpc:UnsupportedScene','No collision adapter for this scene; no template substitution.');
strict=makeStrictClfCbfQpConfig(physical);
[~,~,ad]=ch5Physics(c);
import casadi.*
nonideal=p.mode=="nonideal-force";
nd=ceil(p.delay/c.Ts); if ~nonideal, nd=0; end
n=12+double(nonideal)*(12+6*nd);
z=MX.sym('z',n); u=MX.sym('u',6); w=MX.sym('w',6);
x=MX.sym('x',12); force=MX.sym('force',6);
f=Function('ch5_rhs',{x,force,w},{ad.motion(x(1:6),x(7:12),force,w)});
state=MX(z); trace=MX(z); traceTime=0; timeOffset=0;
if nonideal
    delays=reshape(z(25:end),6,nd);
    integer=floor(p.delay/c.Ts); fraction=p.delay-integer*c.Ts;
    sequence=[u,delays];
    if fraction>1e-14
        targets={sequence(:,integer+2),sequence(:,integer+1)};
        durations=[fraction,c.Ts-fraction];
    else
        targets={sequence(:,integer+1)}; durations=c.Ts;
    end
else
    targets={u}; durations=c.Ts;
end
for seg=1:numel(durations)
    duration=durations(seg); target=targets{seg};
    steps=max(1,ceil(duration/(c.Ts/c.substeps))); h=duration/steps;
    if nonideal
        target=sign(target).*fmax(abs(target)-p.deadzone,0);
    end
    for j=1:steps
        current=state(1:12);
        if nonideal
            slew=state(13:18); lag=state(19:24);
            [~,~,f0]=act(slew,lag,target,0,p);
            [~,~,fm]=act(slew,lag,target,h/2,p);
            [sn,ln,f1]=act(slew,lag,target,h,p);
        else
            f0=u; fm=u; f1=u;
        end
        k1=f(current,f0,w); k2=f(current+h*k1/2,fm,w);
        k3=f(current+h*k2/2,fm,w); k4=f(current+h*k3,f1,w);
        state(1:12)=current+h*(k1+2*k2+2*k3+k4)/6;
        if nonideal, state(13:24)=[sn;ln]; end
        trace=[trace,state]; %#ok<AGROW>
        traceTime=[traceTime,timeOffset+j*h]; %#ok<AGROW>
    end
    timeOffset=timeOffset+duration;
end
if nonideal && nd>0, state(25:end)=reshape([u,delays(:,1:end-1)],6*nd,1); end
if nonideal
    F=fmin(fmax(z(19:24),p.forceMin),p.forceMax);
    rate=(z(13:18)-z(19:24))/p.tau;
    rate=if_else((z(19:24)>=p.forceMax & rate>0) | (z(19:24)<=p.forceMin & rate<0),0,rate);
else
    F=u; rate=MX.zeros(6,1); % ideal rate means sampled command slew, checked separately
end
[d,G,~,~,~,~,~,L,Ld,~,ldd0,lddG]=ad.dynamics(z(1:6),z(7:12)); %#ok<ASGLU>
[gaps]=ad.collision(z(1:6));
[sigma2]=ad.singularity(z(1:6));
values=[L;Ld;ldd0+lddG*F;sigma2;gaps;F;rate];
model=physical; model.scene=scene; model.profile=p; model.n=n; model.delayCount=nd;
model.fd=Function('ch5_fd',{z,u,w},{state,trace});
nominal=substitute(state,w,MX.zeros(6,1));
model.derivative=Function('ch5_derivative',{z,u},{jacobian(nominal,z),jacobian(nominal,u)});
model.values=Function('ch5_values',{z,u},{values,jacobian(values,z),jacobian(values,u)});
model.valueInputJacobian=Function('ch5_value_input',{z,u},{values,jacobian(values,u)});
model.valuesOnly=Function('ch5_values_only',{z,u},{values});
model.mechanicalValues=Function('ch5_mechanical_values',{z,u},{ad.mechanical(z(1:6),z(7:12),F)});
nominalTrace=substitute(trace,w,MX.zeros(6,1));gg={};
for h=1:numel(traceTime),gg{h}=model.valuesOnly(nominalTrace(:,h),u);end
interval=reshape(horzcat(gg{:}),34*numel(traceTime),1);
model.intervalValues=Function('ch5_interval_values',{z,u},{reshape(interval,34,numel(traceTime))});
model.intervalDerivative=Function('ch5_interval_derivative',{z,u}, ...
    {interval,jacobian(interval,z),jacobian(interval,u)});
model.combinedValues=Function('ch5_combined_values',{z,u},{[nominal;interval]});
model.force=Function('ch5_force',{z,u},{F,rate});
model.rhs=f; model.affineDynamics=ad.dynamics;
model.traceTime=traceTime; model.Ts=c.Ts; model.substeps=c.substeps;
model.strictConfig=strict;
model.constraintNames=[compose("length_%d",(1:6)');compose("speed_%d",(1:6)'); ...
    compose("acceleration_%d",(1:6)');"sigma_lower_squared"; ...
    "collision_roof";"collision_left";"collision_right"; ...
    compose("actual_force_%d",(1:6)');compose("actual_rate_%d",(1:6)')];
% Execute the same nonlinear preview entirely inside CasADi, without a MATLAB
% callback for each intermediate state. This changes overhead, not dynamics.
zz=MX.sym('preview_z',n);uu=MX.sym('preview_u',6,c.Np);cur=zz;
nodes=zz;previewValues={};previewTrace={};
for j=1:c.Np
    [cur,tr]=model.fd(cur,uu(:,j),zeros(6,1));nodes=[nodes,cur]; %#ok<AGROW>
    for h=1:numel(traceTime)
        previewValues{end+1}=model.mechanicalValues(tr(:,h),uu(:,j)); %#ok<AGROW>
    end
    previewTrace{end+1}=tr; %#ok<AGROW>
end
model.rollout=Function('ch5_preview',{zz,uu},{nodes,horzcat(previewValues{:}),horzcat(previewTrace{:})});
if c.integralGridTs>0
 dt=c.integralGridTs;ratio=round(c.Ts/dt);assert(abs(ratio*dt-c.Ts)<1e-12 && ratio>=1);
 iz=MX.sym('integral_z',n);iu=MX.sym('integral_u',6);iw=MX.sym('integral_w',6);current=iz;sumQ=MX.zeros(6,1);
 if ratio>1
  cc=c;cc.Ts=dt;cc.substeps=round(dt/(c.Ts/c.substeps));cc.integralGridTs=0;cc.Np=1;
  half=ch5PlantModel(physical,scene,p,cc);assert(half.n==n,'Tracking50:DelayState','Sensor/control delay memory needs explicit remapping');
 end
 for ii=1:ratio
  sumQ=sumQ+dt*current(1:6);
  if ii<ratio,current=half.fd(current,iu,iw);end
 end
 model.integralQuadrature=Function('ch5_integral',{iz,iu,iw},{sumQ,jacobian(sumQ,iw)});
end
end

function [slew,lag,F] = act(slew0,lag0,target,h,p)
import casadi.*
slope=sign(target-slew0).*p.slewRate;
ramp=abs(target-slew0)./p.slewRate; active=fmin(h,ramp);
atRamp=slew0+slope.*(active-p.tau)+(lag0-slew0+slope*p.tau).*exp(-active/p.tau);
slew=slew0+slope.*active;
lag=target+(atRamp-target).*exp(-(h-active)/p.tau);
F=fmin(fmax(lag,p.forceMin),p.forceMax);
end


