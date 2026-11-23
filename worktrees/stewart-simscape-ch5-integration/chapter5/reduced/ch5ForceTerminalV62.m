function t=ch5ForceTerminalV62(q,r,k,verifyRegion)
% Exact memoization restricted to one explicitly reset numerical planning call.
% No interpolation, pose quantization or cross-task reuse of a terminal region.
persistent cache
if nargin==0,cache=containers.Map('KeyType','char','ValueType','any');t=[];return;end
if nargin<4,verifyRegion=false;end
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
% Numerical callers reset before every planning operation and actuator change.
key=[char(r.mode),'|',r.c.version,'|',reshape(num2hex(double(q(:)))',1,[]),'|',num2str(verifyRegion)];
if isKey(cache,key),t=cache(key);return;end
t=ch5ForceTerminalV57(q,r,k,verifyRegion);cache(key)=t;
end
