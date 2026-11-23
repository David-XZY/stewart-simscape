function [value,grad,groups,cache]=pgPenalty(free,task,c)
% Shared seven-group physical penalty; exact singular-value chain derivative.
% Only the task-independent graph is cached; all task values are parameters.
import casadi.*
persistent graphs;
if isempty(graphs), graphs=containers.Map('KeyType','char','ValueType','any'); end
fixed=rmfield(task.model,'dynamics'); nob=numel(task.scene.hood.obstacles);
key=dpHash(struct('fixedRobot',fixed,'shape',task.payload.shape,'obstacles',nob,'controls',c.controlCount,'envelope',c.envelope,'version',2));
n=c.controlCount; points=33;
p=pgPhysics(task.model); numeric=[task.q0;task.qGoal;task.T;p.parameters;task.payload.boundingDimensions(:)];
for j=1:nob, o=task.scene.hood.obstacles(j); numeric=[numeric;o.center_S;o.halfSize;o.R_S(:)]; end
cache=struct('key',key,'hit',isKey(graphs,key));
if ~cache.hit
    parameters=MX.sym('task_parameters',29+15*nob); dyn=parameters(14:26); dims=parameters(27:29);
    z=MX.sym('controls',6*(n-6)); cp=[repmat(parameters(1:6),1,3),reshape(z,6,n-6),repmat(parameters(7:12),1,3)];
    [B,D,DD]=bdBasis(linspace(0,1,points),1,n); Q=cp*B; V=cp*D/parameters(13); A=cp*DD/parameters(13)^2;
    m=task.model; [L,Ld,Ldd,F,J]=p.parameterPoint(Q,V,A,dyn);
    groups0=[meanSquare([(repmat(m.lmin,1,points)-L)./repmat(m.lmax,1,points);(L-repmat(m.lmax,1,points))./repmat(m.lmax,1,points)]); ...
        meanSquare(abs(Ld)./repmat(m.actuator.ldotMax,1,points)-1); ...
        meanSquare(abs(Ldd)./repmat(m.actuator.lddotMax,1,points)-1); ...
        meanSquare([(repmat(m.actuator.forceMin,1,points)-F)/2000;(F-repmat(m.actuator.forceMax,1,points))/2000])];
    gaps={};
    for k=1:points
        R=p.pose(Q(:,k)); center=Q(1:3,k)+R*[0;0;.15];
        for j=1:numel(task.scene.hood.obstacles)
            offset=29+15*(j-1); o=struct('center_S',parameters(offset+(1:3)),'halfSize',parameters(offset+(4:6)),'R_S',reshape(parameters(offset+(7:15)),3,3)); delta=o.center_S-center;
            dirs=[MX.eye(3),R,MX(o.R_S),delta/sqrt(dot(delta,delta)+1e-24)]; dirs=[dirs,-dirs];
            local=R.'*dirs;
            if strcmp(task.payload.shape,'box'), support=dims.'/2*sqrt(local.^2+1e-12);
            else, support=dims(1)/2*sqrt(local(1,:).^2+1e-12)+dims(2)/2*sqrt(sum1(local(2:3,:).^2)+1e-12); end
            gaps{end+1}=mmax(delta.'*dirs-o.halfSize.'*sqrt((o.R_S.'*dirs).^2+1e-12)-support);
            gaps{end+1}=envelope(Q(:,k),o,R,m,c.envelope);
        end
    end
    if isempty(gaps), groups0=[groups0;MX(0)]; else, groups0=[groups0;meanSquare((.01-vertcat(gaps{:}))/.01)]; end
    base=sum1(groups0); direction=MX.sym('J_adjoint',numel(J));
    fun=Function('pg_penalty',{z,parameters},{base,gradient(base,z),J,groups0});
    pull=Function('pg_J_pullback',{z,parameters,direction},{jtimes(J(:),z,direction,true)}); graphs(key)=struct('fun',fun,'pull',pull);
else
    cached=graphs(key); fun=cached.fun; pull=cached.pull;
end
[value,grad,J,groups]=fun(free,numeric); value=full(value); grad=full(grad); J=full(J); groups=full(groups);
adj=zeros(size(J)); gs=0; gc=0; normScale=diag([1,1,1,2,2,2]);
for k=1:points
    cols=(k-1)*6+(1:6); [u,s,v]=svd(J(:,cols)*normScale); sv=diag(s); lo=sv(end); hi=sv(1);
    if lo<=1e-12 || ~isfinite(lo), value=inf; grad(:)=NaN; groups=[groups;inf;inf]; return; end
    a=max(0,(.1-lo)/.1); b=max(0,hi/lo/20-1); gs=gs+a^2/points; gc=gc+b^2/points;
    derivative=(-2*a/.1/points)*u(:,end)*v(:,end).';
    derivative=derivative+(2*b/20/points)*(u(:,1)*v(:,1).'/lo-hi/lo^2*u(:,end)*v(:,end).');
    adj(:,cols)=derivative*normScale;
end
value=value+gs+gc; grad=grad+full(pull(free,numeric,adj(:))); groups=[groups;gs;gc];
end
function y=meanSquare(x)
import casadi.*
y=sum1(fmax(x(:),0).^2)/numel(x);
end
function gaps=envelope(q,o,R,m,e)
import casadi.*
dirs=MX([eye(3),-eye(3),o.R_S,-o.R_S]); ab=@(x)sqrt(x.^2+1e-12);
box=o.halfSize.'*ab(o.R_S.'*dirs); ax=R(:,3).'*dirs; P=q(1:3)+R*m.B(:,m.legMap);
gaps=mmax((o.center_S-q(1:3)).'*dirs-box-e.platformHalfHeight*ab(ax)-e.platformRadius*sqrt(fmax(0,1-ax.^2)+1e-12));
ax=dirs(3,:); gaps=[gaps;mmax((o.center_S-e.baseCenter).'*dirs-box-e.baseHalfHeight*ab(ax)-e.baseRadius*sqrt(fmax(0,1-ax.^2)+1e-12))];
for i=1:6, center=(m.A(:,i)+P(:,i))/2; half=(P(:,i)-m.A(:,i))/2; gaps=[gaps;mmax((o.center_S-center).'*dirs-box-ab(half.'*dirs)-e.legRadius)]; end
end
