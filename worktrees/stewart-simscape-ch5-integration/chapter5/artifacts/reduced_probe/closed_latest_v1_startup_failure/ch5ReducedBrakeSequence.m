function fun=ch5ReducedBrakeSequence(r)
% Physically conditioned numerical initializer. Its output must pass the
% complete nonlinear rollout and terminal gate before it can be executed.
persistent cached keySaved
key=char(r.mode+"|"+r.c.version);
if ~isempty(keySaved)&&strcmp(key,keySaved),fun=cached;return;end
import casadi.*
q=MX.sym('brake_q',6);v=MX.sym('brake_v',6);last=MX.sym('brake_last',6);
[drift,G,~,~,~,Jq,~,~,Ld,geometric]=r.ad.dynamics(q,v);
limit=.90*r.physical.actuator.lddotMax;
legDesired=fmin(fmax(-8*Ld+geometric,-limit),limit);
desired=solve(Jq,legDesired-geometric);target=solve(G,desired-drift);
command=fmin(fmax(target,fmax(r.strict.forceMin,last-r.strict.forceRateLimit*r.c.Ts)),fmin(r.strict.forceMax,last+r.strict.forceRateLimit*r.c.Ts));
policy=Function('ch5_leg_bounded_braking',{q,v,last},{command});
x=MX.sym('brake_initial',r.plant.n);previous=MX.sym('brake_previous',6);state=x;U=MX.zeros(6,r.c.H);
for j=1:r.c.H
    u=policy(state(1:6),state(7:12),previous);U(:,j)=u;
    state=r.plant.fd(state,u,MX.zeros(6,1));previous=u;
end
fun=Function('ch5_brake_initializer',{x,previous},{U},ch5ProjectionJitOptions());cached=fun;keySaved=key;
end
