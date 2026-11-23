function [linear,info]=ch5ReducedSupportQPv30(linear,q,S,o,r)
% Convex epigraph upper bound on cylinder support about a fixed separating plane.
% Orientation/center maps remain locally linearized; true nonlinear gates are mandatory.
nc=size(linear.Q,1);[C,axis,JC,JA]=geometry(r);C=full(C(q));axis=full(axis(q));JC=full(JC(q));JA=full(JA(q));
info=struct('planes',{{}},'auxiliary_variables',0,'continuous_safety_proven',false);
centers={r.scene.objectCylinder.center_P,[0;0;-.01]};axes={r.scene.objectCylinder.axis_P,[0;0;1]}; %#ok<NASGU>
radius=[r.scene.objectCylinder.radius,r.physical.rB];half=[r.scene.objectCylinder.length/2,.01];
for oi=find(o(22,:)>.5)
    obstacle=o(:,oi);obstacle(1:3)=obstacle(1:3)+obstacle(16:18)*(r.c.H*r.c.Ts);
    B=reshape(obstacle(7:15),3,3);gap=full(r.collision.single(q,obstacle));
    if gap>r.c.safe_distance+r.c.sample_hold_margin+.005,continue;end
    for shape=1:2
        ray=obstacle(1:3)-C(:,shape);normals=[eye(3),-eye(3),B,-B];if norm(ray)>1e-12,ray=ray/norm(ray);normals=[normals,ray,-ray];end;gaps=zeros(1,size(normals,2));
        a=axis(:,shape);
        for j=1:size(normals,2)
            normal=normals(:,j)/norm(normals(:,j));
            gaps(j)=normal'*(obstacle(1:3)-C(:,shape))-sum(obstacle(4:6).*abs(B'*normal))- ...
                half(shape)*abs(normal'*a)-radius(shape)*norm(cross(normal,a));
        end
        [~,selected]=max(gaps);normal=normals(:,selected)/norm(normals(:,selected));
        crossMatrix=[0,-normal(3),normal(2);normal(3),0,-normal(1);-normal(2),normal(1),0];
        rows=3*(shape-1)+(1:3);centerJac=JC(rows,:)*S;axisJac=JA(rows,:)*S;
        v=[normal'*a;crossMatrix*a];D=[normal'*axisJac;crossMatrix*axisJac];
        n=size(linear.Q,1);new=n+(1:4);
        linear.Q=blkdiag(linear.Q,1e-6*eye(4));linear.f=[linear.f;zeros(4,1)];
        linear.A=[linear.A,zeros(size(linear.A,1),4)];linear.E=[linear.E,zeros(size(linear.E,1),4)];
        A=zeros(9,n+4);A(1:4,1:nc)=D;A(1:4,new)=-eye(4);A(5:8,1:nc)=-D;A(5:8,new)=-eye(4);
        A(9,1:nc)=normal'*centerJac;A(9,new)=[half(shape),radius(shape)*ones(1,3)];
        rhs=normal'*(obstacle(1:3)-C(:,shape))-sum(obstacle(4:6).*abs(B'*normal))- ...
            r.c.safe_distance-r.c.sample_hold_margin-r.reduced.linearization_collision_reserve;
        linear.A=[linear.A;A];linear.b=[linear.b;-v;v;rhs];
        info.planes{end+1}=struct('obstacle',oi,'shape',shape,'normal',normal,'original_plane_gap',gaps(selected));
        info.auxiliary_variables=info.auxiliary_variables+4;
    end
end
end
function [C,axis,JC,JA]=geometry(r)
persistent saved version
if isempty(version)||~strcmp(version,r.c.version)
    import casadi.*
    q=SX.sym('support_pose',6);R=ch5Rotation(q(4:6));c=[q(1:3)+R*r.scene.objectCylinder.center_P,q(1:3)+R*[0;0;-.01]];
    a=[R*r.scene.objectCylinder.axis_P,R(:,3)];saved={Function('support_centers',{q},{c}),Function('support_axes',{q},{a}), ...
        Function('support_center_jacobian',{q},{jacobian(c(:),q)}),Function('support_axis_jacobian',{q},{jacobian(a(:),q)})};version=r.c.version;
end
C=saved{1};axis=saved{2};JC=saved{3};JA=saved{4};
end
