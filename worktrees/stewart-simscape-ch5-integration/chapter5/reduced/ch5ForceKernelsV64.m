function k=ch5ForceKernelsV64(r)
% Cache both actuator profiles. Never re-differentiate an ideal fd wrapper.
% The plant already owns exact ideal Jacobians constructed before JIT.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version+"|"+string(r.c.H));
if isKey(caches,key),k=caches(key);return;end
import casadi.*
n=r.plant.n;x=MX.sym('force64_x',n);u=MX.sym('force64_u',6);
if r.mode=="ideal-force"
    y=r.plant.fd(x,u,MX.zeros(6,1));[A,B]=r.plant.derivative(x,u);
    stage=Function('ch5_force64_stage',{x,u},{y,A,B},ch5ProjectionJitOptions());
else
    stage=ch5ReducedFactorStage(r);
end
value=r.plant.mechanicalValues(x,u);value=value(1:19);
mech=Function('ch5_force64_mechanical',{x,u},{value,jacobian(value,x),jacobian(value,u)},ch5ProjectionJitOptions());
q=MX.sym('force64_q',6);eq=r.ad.feedforward(q,MX.zeros(6,1),MX.zeros(6,1));
equilibrium=Function('ch5_force64_equilibrium',{q},{eq,jacobian(eq,q)},ch5ProjectionJitOptions());
o=MX.sym('force64_obstacle',22);gap=r.collision.single(q,o);
collision=Function('ch5_force64_collision',{q,o},{gap,jacobian(gap,q)},ch5ProjectionJitOptions());
k=struct('stage',stage,'stages',stage.map(r.c.H),'mech',mech,'mechanics',mech.map(2*r.c.H), ...
    'equilibrium',equilibrium,'collision',collision,'nodes',ch5NativeNodes(r),'preview',ch5NativePreview(r));
caches(key)=k;
end
