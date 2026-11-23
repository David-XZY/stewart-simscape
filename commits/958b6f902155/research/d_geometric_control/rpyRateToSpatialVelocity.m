function omega=rpyRateToSpatialVelocity(rpy,rates)
[~,E]=abfdRotationExpressions(rpy(:),rates(:));
assert(abs(cos(rpy(2)))>1e-5,'ABFD:RpySingular','RPY pitch is singular.');
omega=E*rates(:);
end
