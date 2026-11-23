function step=buildAbfdPredictionStepAD(provider,profile,standardizedMean)
% Differentiable RK4 rigid/B step with exact slew-before-lag stage forces.
% z=[q;v;slew;lag], u=delayed command, r=centered standardized wrench error.
import casadi.*
if nargin<3, standardizedMean=zeros(6,1); end
ad=buildCasadiCompositeDynamics(provider);
% Keep RK4 stages as matrix/function calls. SX expansion duplicates millions
% of scalar derivative operations across state, command and noise Jacobians.
z=MX.sym('mean_state',24); u=MX.sym('delayed_command',6);
r=MX.sym('standardized_wrench',6); h=MX.sym('step_seconds');
target=sign(u).*fmax(abs(u)-profile.forceDeadzone,0);
limit=max(abs([provider.model.actuator.forceMin(:),provider.model.actuator.forceMax(:)]),[],2);
force=@(lag) fmin(fmax(lag,-limit),limit);
x=z(1:12); slew=z(13:18); lag=z(19:24);
[~,lagHalf]=actuator(slew,lag,target,h/2,profile);
[slewNext,lagNext]=actuator(slew,lag,target,h,profile);
k1=body(ad,x,force(lag),r+standardizedMean);
k2=body(ad,x+h*k1/2,force(lagHalf),r+standardizedMean);
k3=body(ad,x+h*k2/2,force(lagHalf),r+standardizedMean);
k4=body(ad,x+h*k3,force(lagNext),r+standardizedMean);
next=[x+h*(k1+2*k2+2*k3+k4)/6;slewNext;lagNext];
D=jacobian(next,[z;u;r]);
step=struct('advance',Function('abfd_mean_step',{z,u,r,h},{next}), ...
    'jacobian',Function('abfd_mean_step_jacobian',{z,u,r,h}, ...
    {D(:,1:24),D(:,25:30),D(:,31:36)}));
end
function xd=body(ad,x,force,standardized)
import casadi.*
[H,bias,Jv,~,~,Sigma]=ad.terms(x(1:6),x(7:12));
xd=[x(7:12);solve(H,Jv.'*force-bias-diag(sqrt(diag(Sigma)))*standardized)];
end
function [slewNext,lagNext]=actuator(slew,lag,target,h,profile)
import casadi.*
difference=target-slew; slope=sign(difference)*profile.forceRate;
ramp=abs(difference)/profile.forceRate; active=fmin(h,ramp); tau=profile.forceLag;
rampForce=slew+slope.*(active-tau)+(lag-slew+slope*tau).*exp(-active/tau);
lagNext=target+(rampForce-target).*exp(-(h-active)/tau);
% At a completed ramp the exact map is C1, with opposite one-sided second
% derivatives. Roundoff in two equal commands must not select a random side
% for covariance sensitivities. The linear limiting value differs by at most
% O(difference^2/(forceRate*tau)), below a force ulp in this narrow branch.
roundoff=64*eps*fmax(1,fmax(abs(target),abs(slew)));
lagNext=if_else(abs(difference)<=roundoff,target+(lag-target).*exp(-h/tau),lagNext);
% At target==slew, the completed-ramp branch must have d(slewNext)/d(target)=1.
slewNext=if_else(h==0,slew,if_else(h>=ramp,target,slew+slope.*h));
end
