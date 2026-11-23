function preview=ch5NativePreview(r)
% Compile the same existing preview, not a replacement prediction model.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);
if isKey(caches,key),preview=caches(key);return;end
import casadi.*
x=MX.sym('preview_initial',r.plant.n);u=MX.sym('preview_commands',6,r.c.H);
[nodes,values,trace]=r.plant.rollout(x,u);
preview=Function('ch5_native_preview',{x,u},{nodes,values,trace},ch5ProjectionJitOptions());
caches(key)=preview;
end
