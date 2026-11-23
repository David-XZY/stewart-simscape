function kernel=ch5ReducedHoldKernel(r)
% The unchanged physical integrator and all original points for one hold.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);
if isKey(caches,key),kernel=caches(key);return;end
import casadi.*
x=MX.sym('hold_initial',r.plant.n);u=MX.sym('hold_command',6);
[next,trace]=r.plant.fd(x,u,MX.zeros(6,1));values=cell(1,numel(r.plant.traceTime));
for j=1:numel(values),values{j}=r.plant.mechanicalValues(trace(:,j),u);end
kernel=Function('ch5_reduced_physical_hold',{x,u},{next,trace,horzcat(values{:})},ch5ProjectionJitOptions());
caches(key)=kernel;
end
