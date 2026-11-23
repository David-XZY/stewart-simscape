function [cp,report]=buildObstacleAwareSplineSeed(task,startIndex)
% Low-dimensional geometric warm start. Every final trajectory still needs
% the full dynamics/HS solve and independent dense acceptance.
import casadi.*
validateattributes(startIndex,{'double'},{'scalar','integer','positive','finite'});
s=task.scene; m=task.model; T=s.phase.durationApproach;
first=sgpIK(s.q0,m); last=sgpIK(s.qWaypoint,m);
lowerGuard=max(0,min(.001,.5*min(first.L-m.lmin,last.L-m.lmin)));
upperGuard=max(0,min(.001,.5*min(m.lmax-first.L,m.lmax-last.L)));
times=linspace(0,T,81); basis=zeros(10,numel(times));
for j=1:10
    unit=zeros(6,10); unit(1,j)=1;
    values=evaluateTrajectorySpline(unit,times,T); basis(j,:)=values(1,:);
end
cp0=zeros(6,10); cp0(:,1:3)=repmat(s.q0,1,3); cp0(:,8:10)=repmat(s.qWaypoint,1,3);
for j=4:7, cp0(:,j)=s.q0+(j-3)/5*(s.qWaypoint-s.q0); end
% Eight distinct transverse starts; all optimize the same geometry objective.
directions=[-1,1,0,0,-1,1,-1,1;0,0,-1,1,-1,1,1,-1];
localDirection=directions(:,1+mod(startIndex-1,8));
localDirection=localDirection/norm(localDirection);
direction=s.box.R_S(:,2:3)*localDirection;
amplitude=.12+.02*floor((startIndex-1)/8);
cp0(1:3,4:7)=cp0(1:3,4:7)+amplitude*direction;
delta=SX.sym('delta',6,4); scale=[.1;.1;.1;.2;.2;.2];
cpExpr=[repmat(s.q0,1,3),repmat(s.q0,1,4)+delta.*repmat(scale,1,4),repmat(s.qWaypoint,1,3)];
Q=cpExpr*basis; constraints={};
for k=1:size(Q,2)
    q=Q(:,k); R=rotation(q(4:6));
    for leg=1:6
        d=q(1:3)+R*m.B(:,m.legMap(leg))-m.A(:,leg); lengthSquared=d.'*d;
        constraints{end+1}=[lengthSquared-(m.lmax(leg)-upperGuard(leg))^2; ...
            (m.lmin(leg)+lowerGuard(leg))^2-lengthSquared]; %#ok<AGROW>
    end
    center=q(1:3)+R*s.objectCylinder.center_P; axis=R*s.objectCylinder.axis_P;
    for j=1:numel(s.hood.obstacles)
        obstacle=s.hood.obstacles(j); difference=obstacle.center_S-center;
        nCenter=difference/sqrt(difference.'*difference+1e-16);
        normals=[obstacle.R_S,-obstacle.R_S,R,-R,nCenter,-nCenter];
        if j==4 && isfield(s,'dividerPlacement')
            normals=[normals,s.dividerPlacement.startNormal,s.dividerPlacement.insertionNormal]; %#ok<AGROW>
        end
        gap=-1e6;
        for n=1:size(normals,2)
            normal=normals(:,n); axial=normal.'*axis;
            value=normal.'*difference-obstacle.halfSize.'*sqrt((obstacle.R_S.'*normal).^2+1e-16) ...
                -.5*s.objectCylinder.length*sqrt(axial^2+1e-16) ...
                -s.objectCylinder.radius*sqrt(sumsqr(normal-axis*axial)+1e-16);
            gap=fmax(gap,value);
        end
        cushion=.003*sin(pi*times(k)/T)^2;
        required=s.collision.safeDistance+cushion;
        if j==1, required=s.collision.stage1ConstraintDistance+cushion; end
        constraints{end+1}=required-gap; %#ok<AGROW>
    end
end
% One common objective for all starts; initial direction only changes x0.
baseline=zeros(6,4);
for j=1:4, baseline(:,j)=(j/5)*(s.qWaypoint-s.q0)./scale; end
objective=sumsqr(delta-baseline)+.1*sumsqr(delta(:,3:4)-2*delta(:,2:3)+delta(:,1:2));
g=vertcat(constraints{:}); z=delta(:); nlp=struct('x',z,'f',objective,'g',g);
opts=makeCommonIpoptOptions(struct('maxIter',500,'hessianApproximation','exact','printLevel',0));
opts.print_time=false; opts.ipopt.max_cpu_time=90;
solver=nlpsol('abfd_spline_seed','ipopt',nlp,opts);
z0=(cp0(:,4:7)-repmat(s.q0,1,4))./repmat(scale,1,4);
timer=tic; sol=solver('x0',z0(:),'lbg',-inf(numel(g),1),'ubg',zeros(numel(g),1));
stats=solver.stats(); evaluator=Function('seed_constraints',{z},{g}); violations=full(evaluator(sol.x));
[worst,which]=max(violations); violation=max(0,worst);
cp=cp0; cp(:,4:7)=repmat(s.q0,1,4)+reshape(full(sol.x),6,4).*repmat(scale,1,4);
report=struct('method','geometric spline warm start','start',startIndex,'status',stats.return_status, ...
    'startDirection',direction,'startAmplitude',amplitude,'initialControlPointsHash',abfdHash(cp0), ...
    'legLengthLowerGuard',lowerGuard,'legLengthUpperGuard',upperGuard, ...
    'iterations',stats.iter_count,'seconds',toc(timer),'maxViolation',violation, ...
    'worstSample',ceil(which/(12+numel(s.hood.obstacles))), ...
    'worstConstraintAtSample',mod(which-1,12+numel(s.hood.obstacles))+1, ...
    'passed',stats.success && violation<=1e-6,'fullTrajectoryAccepted',false);
end
function R=rotation(r)
cr=cos(r(1)); sr=sin(r(1)); cp=cos(r(2)); sp=sin(r(2)); cy=cos(r(3)); sy=sin(r(3));
R=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr; ...
   sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr;-sp,cp*sr,cp*cr];
end
