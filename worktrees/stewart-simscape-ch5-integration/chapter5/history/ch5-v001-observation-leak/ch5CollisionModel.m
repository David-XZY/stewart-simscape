function map = ch5CollisionModel(physical,scene,c)
% Every positive certificate is a separating support plane for BOTH solids.
% A negative certificate is not interpreted as proof of physical penetration.
import casadi.*
q=SX.sym('q',6);o=SX.sym('o',22,c.max_obstacles);v=SX.sym('qd',6);
R=ch5Rotation(q(4:6));centers={q(1:3)+R*scene.objectCylinder.center_P,q(1:3)+R*[0;0;-.01]};
axes={R*scene.objectCylinder.axis_P,R(:,3)};
radii=[scene.objectCylinder.radius,physical.rB];halves=[scene.objectCylinder.length/2,.01];
eps=1e-6;g=SX.zeros(c.max_obstacles,1);dq=SX.zeros(c.max_obstacles,6);
dt=SX.zeros(c.max_obstacles,1);dd=SX.zeros(c.max_obstacles,1);
for i=1:c.max_obstacles
    B=reshape(o(7:15,i),3,3);active=o(22,i);vel=o(16:18,i);
    shapes=cell(1,2);
    for s=1:2
        ray=o(1:3,i)-centers{s};ray=ray/sqrt(ray'*ray+eps^2);
        normals=[SX.eye(3),-SX.eye(3),B,-B,ray,-ray];a=axes{s};
        gaps=SX.zeros(size(normals,2),1);
        for j=1:size(normals,2)
            n=normals(:,j);dot=n'*a;
            support=halves(s)*sqrt(dot^2+eps^2)+radii(s)*sqrt(fmax(0,1-dot^2)+eps^2);
            gaps(j)=n'*(o(1:3,i)-centers{s})-sum1(o(4:6,i).*sqrt((B'*n).^2+eps^2))-support;
        end
        maximum=gaps(1);
        for j=2:numel(gaps)
            maximum=(maximum+gaps(j)+sqrt((maximum-gaps(j))^2+eps^2)-eps)/2;
        end
        shapes{s}=maximum;
    end
    lower=(shapes{1}+shapes{2}-sqrt((shapes{1}-shapes{2})^2+eps^2))/2;
    g(i)=if_else(active>.5,lower,10);
    gradient=jacobian(g(i),q);gc=jacobian(g(i),o(1:3,i));
    joint=[q;o(1:3,i)];jointv=[v;vel];hess=hessian(g(i),joint);
    dq(i,:)=gradient;dt(i)=gc*vel;dd(i)=jointv'*hess*jointv;
end
map=struct();map.value=Function('ch5_gap',{q,o},{g});
map.single=Function('ch5_single_gap',{q,o(:,1)},{g(1)});
map.cbf=Function('ch5_gap_derivatives',{q,v,o},{g,dq,dt,dd});
end
