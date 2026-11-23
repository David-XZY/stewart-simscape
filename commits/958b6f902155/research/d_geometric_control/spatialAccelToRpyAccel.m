function accel=spatialAccelToRpyAccel(rpy,rates,alpha)
[~,E,Ed]=abfdRotationExpressions(rpy(:),rates(:));
assert(rcond(E)>1e-5,'ABFD:RpySingular','RPY acceleration map is singular.');
accel=E\(alpha(:)-Ed*rates(:));
end
