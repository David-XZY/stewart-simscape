function [d,normals]=dpClearance(q,task,refine)
% Conservative support-gap certificate. A positive gap certifies separation.
if nargin<3, refine=true; end
R=rpy2rotmZYX(q(4:6)); a=R*task.scene.objectCylinder.axis_P;
center=q(1:3)+R*task.scene.objectCylinder.center_P;
obs=task.scene.hood.obstacles; d=zeros(1,numel(obs)); normals=zeros(3,numel(obs));
for j=1:numel(obs)
    o=obs(j); delta=o.center_S-center;
    signs=[-1,-1,-1;1,-1,-1;-1,1,-1;1,1,-1;-1,-1,1;1,-1,1;-1,1,1;1,1,1].';
    vertexDelta=delta-o.R_S*(signs.*o.halfSize);
    % On n.axis=0 the support objective is piecewise sinusoidal. Its maxima
    % are vertex projections or box-edge boundaries; enumerate them exactly.
    side=vertexDelta-a*(a.'*vertexDelta); transverse=delta-a*(a.'*delta); edge=cross(repmat(a,1,3),o.R_S,1);
    raw=[o.R_S,-o.R_S,R,-R,delta,-delta,transverse,-transverse,edge,-edge,side,-side];
    len=sqrt(sum(raw.^2,1)); raw=raw(:,len>1e-12)./len(len>1e-12);
    axial=a.'*raw;
    vals=delta.'*raw-o.halfSize.'*abs(o.R_S.'*raw)- ...
        .5*task.scene.objectCylinder.length*abs(axial)-task.scene.objectCylinder.radius*sqrt(max(0,1-axial.^2));
    [d(j),i]=max(vals); normal=raw(:,i);
    % A >=10 mm certificate is sufficient; do not spend time maximizing an
    % already valid lower bound. Failed certificates get a separate refinement.
    if refine && d(j)<.01-1e-9
        angles=[atan2(normal(2),normal(1));asin(max(-1,min(1,normal(3))))];
        opt=optimset('Display','off','MaxIter',60,'MaxFunEvals',130,'TolX',1e-10,'TolFun',1e-11);
        ang=fminsearch(@(u)-gap(direction(u)),angles,opt); nn=direction(ang); value=gap(nn);
        if value>d(j), d(j)=value; normal=nn; end
    end
    normals(:,j)=normal;
end
    function value=gap(n)
        axial=n.'*a;
        value=n.'*delta-o.halfSize.'*abs(o.R_S.'*n)- ...
            .5*task.scene.objectCylinder.length*abs(axial)- ...
            task.scene.objectCylinder.radius*sqrt(max(0,1-axial^2));
    end
end
function n=direction(u)
n=[cos(u(1))*cos(u(2));sin(u(1))*cos(u(2));sin(u(2))];
end
