function k=ch5ReducedKernels(r)
% Small stage derivatives, condensed numerically. No horizon-wide AD graph.
persistent cache keySaved
key=char(r.mode+"|"+r.c.version);
if ~isempty(keySaved)&&strcmp(key,keySaved),k=cache;return;end
import casadi.*
n=r.plant.n;x=MX.sym('reduced_x',n);u=MX.sym('reduced_u',6);
fd=ch5DerivativeMap(r);
if n==12,y=fd(x,u,zeros(6,1));else,y=fd(x,u);end
stage=Function('ch5_reduced_stage',{x,u},{y,jacobian(y,x),jacobian(y,u)},ch5ProjectionJitOptions());
value=r.plant.mechanicalValues(x,u);value=value(1:19);
mech=Function('ch5_reduced_mechanical',{x,u},{value,jacobian(value,x),jacobian(value,u)},ch5ProjectionJitOptions());
q=MX.sym('reduced_q',6);eq=r.ad.feedforward(q,MX.zeros(6,1),MX.zeros(6,1));
equilibrium=Function('ch5_reduced_equilibrium',{q},{eq,jacobian(eq,q)},ch5ProjectionJitOptions());
obstacle=MX.sym('reduced_obstacle',22);gap=r.collision.single(q,obstacle);
collision=Function('ch5_reduced_collision',{q,obstacle},{gap,jacobian(gap,q)},ch5ProjectionJitOptions());
k=struct('stage',stage,'stages',stage.map(r.c.H),'mech',mech,'mechanics',mech.map(2*r.c.H), ...
    'equilibrium',equilibrium,'collision',collision,'nodes',ch5NativeNodes(r),'preview',ch5NativePreview(r));
keySaved=key;cache=k;
end
