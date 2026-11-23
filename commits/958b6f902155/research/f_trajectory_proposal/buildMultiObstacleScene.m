function scene=buildMultiObstacleScene(scene,enabled)
if ~enabled, return; end
o=scene.hood.roof; o.name='approach-divider';
o.R_S=eye(3); o.axes_S=eye(3);
o.size=[.10;.08;.18]; o.halfSize=o.size/2;
margin=scene.collision.safeDistance+.005;
assert(norm(scene.qWaypoint(4:6)-scene.qGoal(4:6))<1e-12, ...
    'ABFD:InsertionOrientation','Divider certificate requires fixed insertion orientation.');
% Keep the obstacle inside an intermediate straight-path cylinder, but outside
% the start and the entire analytic insertion sweep. The same separating plane
% at both insertion endpoints certifies every translation between them.
for fraction=[.5,.4,.6,.3,.7,.2,.8]
    q=(1-fraction)*scene.q0+fraction*scene.qWaypoint;
    R=rpy2rotmZYX(q(4:6));
    for axial=[.45,-.45,.25,-.25,0]*scene.objectCylinder.length
        for angle=[0,pi/2,pi,3*pi/2]
            radial=.6*scene.objectCylinder.radius*[0;cos(angle);sin(angle)];
            o.center_S=q(1:3)+R*(scene.objectCylinder.center_P+ ...
                axial*scene.objectCylinder.axis_P+radial);
            normals=certificateNormals(o,scene);
            g0=gaps(normals,o,scene.q0,scene);
            gi=min(gaps(normals,o,scene.qWaypoint,scene),gaps(normals,o,scene.qGoal,scene));
            [startGap,i0]=max(g0); [insertionGap,ii]=max(gi);
            if startGap>=margin && insertionGap>=margin
                o.center_B=scene.box.R_S.'*(o.center_S-scene.box.center_S);
                scene.hood.obstacles(end+1)=o;
                scene.dividerPlacement=struct('version',2,'margin',margin, ...
                    'witnessPose',q,'startGap',startGap,'insertionGap',insertionGap, ...
                    'startNormal',normals(:,i0),'insertionNormal',normals(:,ii));
                return;
            end
        end
    end
end
error('ABFD:NoSafeDividerPlacement', ...
    'No approach divider clears the fixed start and complete insertion sweep.');
end
function n=certificateNormals(o,s)
n=[eye(3),-eye(3)];
for q=[s.q0,s.qWaypoint,s.qGoal]
    R=rpy2rotmZYX(q(4:6)); center=q(1:3)+R*s.objectCylinder.center_P;
    direction=o.center_S-center;
    n=[n,R,-R,direction,-direction]; %#ok<AGROW>
end
lengths=sqrt(sum(n.^2,1)); n=n(:,lengths>1e-12); lengths=lengths(lengths>1e-12);
n=n./lengths;
end
function value=gaps(n,o,q,s)
R=rpy2rotmZYX(q(4:6)); axis=R*s.objectCylinder.axis_P;
center=q(1:3)+R*s.objectCylinder.center_P; axial=axis.'*n;
value=(o.center_S-center).'*n-o.halfSize.'*abs(o.R_S.'*n)- ...
    .5*s.objectCylinder.length*abs(axial)- ...
    s.objectCylinder.radius*sqrt(max(0,1-axial.^2));
end
