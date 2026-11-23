function [R,E,Ed]=abfdRotationExpressions(r,v)
% Works for double, CasADi SX and MX. Angular velocity is world-expressed.
sr=sin(r(1)); cr=cos(r(1)); sp=sin(r(2)); cp=cos(r(2)); sy=sin(r(3)); cy=cos(r(3));
R=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr; ...
    sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr;-sp,cp*sr,cp*cr];
E=[cy*cp,-sy,0;sy*cp,cy,0;-sp,0,1];
Ed=[-cy*sp*v(2)-sy*cp*v(3),-cy*v(3),0; ...
    -sy*sp*v(2)+cy*cp*v(3),-sy*v(3),0;-cp*v(2),0,0];
end
