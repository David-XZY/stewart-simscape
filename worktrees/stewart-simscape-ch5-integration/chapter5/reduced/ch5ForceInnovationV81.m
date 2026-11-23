function decoder=ch5ForceInnovationV81(r,candidateCount)
% Exact same triangular innovation map, using a compiled scan of one stage.
% This avoids the large unrolled native wrapper; no physics is approximated.
if nargin<2,candidateCount=1;end
persistent cache
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version+"|"+string(candidateCount));
if isKey(cache,key),decoder=cache(key);return;end
import casadi.*
n=r.plant.n;H=r.c.H;state=MX.sym('scan_state',n+6);v=MX.sym('scan_innovation',6);
L=MX.sym('scan_feedback',6,n+6);anchor=MX.sym('scan_anchor',n+6);
u=state(n+1:end)+v-L*(state-anchor);next=[r.plant.fd(state(1:n),u,MX.zeros(6,1));u];
stage=Function('ch5_innovation_scan_stage',{state,v,L,anchor},{next,u});scan=stage.mapaccum(H);
x=MX.sym('scan_initial',n);previous=MX.sym('scan_previous',6);V=MX.sym('scan_sequence',6,H);
[~,U]=scan([x;previous],V,repmat(L,1,H),repmat(anchor,1,H));
base=Function('ch5_innovation_scan',{x,previous,V,L,anchor},{U},ch5ProjectionJitOptions());
cache(key)=base.map(candidateCount);decoder=cache(key);
end
