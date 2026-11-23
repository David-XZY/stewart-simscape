function fun=ch5ForceInnovationV73(r)
% Bijective shooting coordinates. No generated state reference or projection.
% Delta_u = v - L * ([x; previous] - xi), with x from the exact plant.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version+"|"+string(r.c.H));
if isKey(caches,key),fun=caches(key);return;end
import casadi.*
n=r.plant.n;H=r.c.H;x=MX.sym('innovation_x',n);last=MX.sym('innovation_previous',6);
V=MX.sym('innovation_values',6,H);L=MX.sym('innovation_gain',6,n+6);xi=MX.sym('innovation_anchor',n+6);
state=x;previous=last;U=MX.zeros(6,H);X=x;
for j=1:H
    u=previous+V(:,j)-L*([state;previous]-xi);U(:,j)=u;
    state=r.plant.fd(state,u,MX.zeros(6,1));X=[X,state];previous=u; %#ok<AGROW>
end
fun=Function('ch5_force_innovation_v73',{x,last,V,L,xi},{U,X},ch5ProjectionJitOptions());caches(key)=fun;
end
