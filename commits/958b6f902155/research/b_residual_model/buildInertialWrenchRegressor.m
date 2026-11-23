function Y=buildInertialWrenchRegressor(q,v,a,model)
% World wrench about platform origin, linear in [m; m*c; I_origin(6)].
% Inertia order: xx, yy, zz, xy, xz, yz, all expressed in the platform frame.
validateattributes([q(:);v(:);a(:)],{'double'},{'real','finite','numel',18});
R=rpy2rotmZYX(q(4:6)); [E,Ed]=rpyRateMapZYX(q(4:6),v(4:6));
w=E*v(4:6); alpha=Ed*v(4:6)+E*a(4:6);
acc=a(1:3); if model.dynamics.includeGravity, acc=acc-model.g; end
Y=zeros(6,10); Y(1:3,1)=acc;
Y(1:3,2:4)=(skew(alpha)+skew(w)*skew(w))*R;
Y(4:6,2:4)=-skew(acc)*R;
pairs=[1,1;2,2;3,3;1,2;1,3;2,3];
for k=1:6
    I=zeros(3); I(pairs(k,1),pairs(k,2))=1; I(pairs(k,2),pairs(k,1))=1;
    world=R*I*R.'; Y(4:6,4+k)=world*alpha+cross(w,world*w);
end
end
function S=skew(x)
S=[0,-x(3),x(2);x(3),0,-x(1);-x(2),x(1),0];
end
