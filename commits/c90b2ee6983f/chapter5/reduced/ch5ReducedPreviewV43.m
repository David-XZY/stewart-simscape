function preview=ch5ReducedPreviewV43(r)
% Explicit short rollout. Do not rely on CasADi input-width broadcasting.
persistent cache
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);if isKey(cache,key),preview=cache(key);return;end
import casadi.*
x=MX.sym('short_initial',r.plant.n);U=MX.sym('short_commands',6,r.c.H);state=x;nodes=x;values={};trace={};
for j=1:r.c.H
    [state,tr]=r.plant.fd(state,U(:,j),MX.zeros(6,1));nodes=[nodes,state]; %#ok<AGROW>
    for h=1:numel(r.plant.traceTime),values{end+1}=r.plant.mechanicalValues(tr(:,h),U(:,j));end %#ok<AGROW>
    trace{end+1}=tr; %#ok<AGROW>
end
preview=Function('ch5_short_native_preview',{x,U},{nodes,horzcat(values{:}),horzcat(trace{:})},ch5ProjectionJitOptions());
assert(preview.size2_out(0)==r.c.H+1,'ch5:RolloutDimension','Preview output has the wrong horizon.');cache(key)=preview;
end
