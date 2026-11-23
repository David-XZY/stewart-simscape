function [distance,contact,converged]=ch5GjkDistance(q,obstacle,physical,scene)
% Distance between convex solids from their exact support maps (GJK).
R=ch5Rotation(q(4:6));distances=zeros(2,1);flags=false(2,1);conv=false(2,1);
for shape=1:2
    if shape==1
        center=q(1:3)+R*scene.objectCylinder.center_P;axis=R*scene.objectCylinder.axis_P;radius=scene.objectCylinder.radius;half=scene.objectCylinder.length/2;
    else,center=q(1:3)+R*[0;0;-.01];axis=R(:,3);radius=physical.rB;half=.01;end
    B=reshape(obstacle(7:15),3,3);bc=obstacle(1:3);bh=obstacle(4:6);
    d=bc-center;if norm(d)<1e-9,d=[1;0;0];end
    simplex=support(d);closest=simplex;success=false;
    for iter=1:50
        if norm(closest)<1e-8,success=true;break;end
        w=support(-closest);
        if closest'*closest-closest'*w<=1e-9*max(1,closest'*closest),success=true;break;end
        if any(vecnorm(simplex-w)<1e-12),success=true;break;end
        simplex=[simplex,w];[closest,simplex]=closestSimplex(simplex);
    end
    distances(shape)=norm(closest);flags(shape)=distances(shape)<1e-8;conv(shape)=success;
end
distance=min(distances);contact=any(flags);converged=all(conv);
    function w=support(d)
        axial=d'*axis;radial=d-axis*axial;rn=norm(radial);
        a=center+half*sign(axial)*axis;if rn>1e-14,a=a+radius*radial/rn;end
        b=bc+B*(bh.*sign(B'*(-d)));w=a-b;
    end
end
function [closest,kept]=closestSimplex(points)
n=size(points,2);best=inf;kept=points(:,1);closest=kept;
for mask=1:2^n-1
    selected=find(bitget(mask,1:n));if numel(selected)>4,continue;end
    p=points(:,selected);m=size(p,2);
    if m==1,weights=1;else
        K=[2*(p'*p),ones(m,1);ones(1,m),0];rhs=[zeros(m,1);1];
        solution=pinv(K)*rhs;weights=solution(1:m);
    end
    if any(weights< -1e-9)||abs(sum(weights)-1)>1e-7,continue;end
    candidate=p*weights;norm2=candidate'*candidate;
    if norm2<best,best=norm2;closest=candidate;kept=p(:,weights>1e-9);end
end
end
