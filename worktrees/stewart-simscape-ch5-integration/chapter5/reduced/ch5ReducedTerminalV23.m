function [U,info,nodes]=ch5ReducedTerminalV23(x,U,r,k,nodes)
% Identical Newton equations; reuse the fixed prefix and integrate ten tail steps.
H=r.c.H;first=H-9;info=struct('iterations',0,'residual',inf,'finite',true,'prefix_steps',first-1,'tail_steps',10);
if nargin<5,nodes=full(k.nodes(x,U));end
assert(isequal(size(nodes),[r.plant.n,H+1])&&norm(nodes(:,1)-x,inf)<1e-10,'ch5:TailPrefix','Invalid current rollout prefix.');
prefix=nodes(:,1:first-1);start=nodes(:,first);tail=tailKernel(r);
for iteration=1:3
    if ~all(isfinite(nodes),'all'),info.finite=false;return;end
    [ua,eqJac]=k.equilibrium(nodes(1:6,end));ua=full(ua);eqJac=full(eqJac);
    residual=[nodes(7:12,end);(U(:,end)-ua)/1000];info.residual=norm(residual,inf);
    if info.residual<1e-8,return;end
    mapped=k.stage.map(10);[~,aa,bb]=mapped(nodes(:,first:H),U(:,first:H));aa=full(aa);bb=full(bb);
    S=zeros(r.plant.n,12);
    for j=1:10
        S=aa(:,r.plant.n*(j-1)+(1:r.plant.n))*S;cols=1:6;if j>5,cols=7:12;end
        S(:,cols)=S(:,cols)+bb(:,6*(j-1)+(1:6));
    end
    F=[S(7:12,:);([zeros(6),eye(6)]-eqJac*S(1:6,:))/1000];
    if rcond(F)<1e-12,info.finite=false;return;end
    correction=-F\residual;
    if ~all(isfinite(correction))||norm(correction,inf)>200,info.finite=false;return;end
    U(:,first:H-5)=U(:,first:H-5)+correction(1:6);U(:,H-4:H)=U(:,H-4:H)+correction(7:12);
    info.iterations=iteration;nodes=[prefix,full(tail(start,U(:,first:H)))];
end
end
function fun=tailKernel(r)
persistent cache version
if isempty(version)||~strcmp(version,r.c.version),cache=containers.Map('KeyType','char','ValueType','any');version=r.c.version;end
key=char(r.mode);if isKey(cache,key),fun=cache(key);return;end
import casadi.*
x=MX.sym('tail_initial',r.plant.n);U=MX.sym('tail_input',6,10);state=x;nodes=x;
for j=1:10,state=r.plant.fd(state,U(:,j),MX.zeros(6,1));nodes=[nodes,state];end %#ok<AGROW>
fun=Function('ch5_terminal_tail',{x,U},{nodes},ch5ProjectionJitOptions());cache(key)=fun;
end
