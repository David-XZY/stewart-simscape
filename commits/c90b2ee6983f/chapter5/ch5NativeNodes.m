function preview=ch5NativeNodes(r)
% The same fd, with state nodes only. Mechanical inner-grid auditing is a
% separate operation and is mandatory before accepting an executable plan.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);
if isKey(caches,key),preview=caches(key);return;end
import casadi.*
x=MX.sym('nodes_initial',r.plant.n);u=MX.sym('nodes_commands',6,r.c.H);
state=x;nodes=x;
for j=1:r.c.H
    state=r.plant.fd(state,u(:,j),zeros(6,1));nodes=[nodes,state]; %#ok<AGROW>
end
preview=Function('ch5_native_nodes',{x,u},{nodes},ch5ProjectionJitOptions());
caches(key)=preview;
end
