function out = ch5Rotation(in)
% ZYX Euler angles <-> active rotation from platform to world.
if isequal(size(in),[3,3])
    out=[atan2(in(3,2),in(3,3));atan2(-in(3,1),hypot(in(1,1),in(2,1)));atan2(in(2,1),in(1,1))];
else
    r=in(1);p=in(2);y=in(3);
    out=[cos(y)*cos(p),cos(y)*sin(p)*sin(r)-sin(y)*cos(r),cos(y)*sin(p)*cos(r)+sin(y)*sin(r); ...
         sin(y)*cos(p),sin(y)*sin(p)*sin(r)+cos(y)*cos(r),sin(y)*sin(p)*cos(r)-cos(y)*sin(r); ...
         -sin(p),cos(p)*sin(r),cos(p)*cos(r)];
end
end
