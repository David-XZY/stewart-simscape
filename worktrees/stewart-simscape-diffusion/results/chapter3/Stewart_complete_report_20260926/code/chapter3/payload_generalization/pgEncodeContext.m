function x=pgEncodeContext(task,mode)
if nargin<2, mode='basic'; end
x=dpContext(task); assert(numel(x)==93,'PG:Schema','Legacy slots changed.');
% Explicit scene flags replace the legacy family strings without changing slots.
x(14)=double(strcmp(task.family,'open')); x(15)=double(strcmp(task.family,'dual_fixture'));
p=task.payload; x=[x;double(strcmp(p.shape,'cylinder'));double(strcmp(p.shape,'box'));p.boundingDimensions(:);p.mass];
assert(numel(x)==99 && all(isfinite(x)),'PG:Schema','Invalid basic condition.');
if strcmp(mode,'signature'), x=[x;pgPhysicsSignature(task)]; assert(numel(x)==354,'PG:Schema','Invalid signature condition.');
else, assert(strcmp(mode,'basic'),'PG:Schema','Unknown context mode.'); end
end
