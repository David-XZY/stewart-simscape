function k=ch5ReducedKernelsV45(r)
% Physical stage/constraint kernels do not depend on the prediction horizon.
% Reuse them; construct only the horizon-specific maps and explicit preview.
persistent cache
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);if isKey(cache,key),k=cache(key);return;end
[c,~]=ch5Init();fullRuntime=r;fullRuntime.c.H=c.H;fullRuntime.c.version=[c.version,'-speed-reduced-v1'];
assert(r.c.Ts==c.Ts&&r.plant.Ts==c.Ts,'ch5:PhysicsChanged','Kernel reuse requires identical physical steps.');
k=ch5ReducedKernelsV4(fullRuntime);
k.stages=k.stage.map(r.c.H);k.mechanics=k.mech.map(2*r.c.H);
k.nodes=ch5NativeNodes(r);k.preview=ch5ReducedPreviewV43(r);cache(key)=k;
end
