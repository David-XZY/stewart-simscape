function log=readAbfdDirectAcceleration(simulation,log)
% Preserve the historical difference estimate alongside direct acceleration.
signal=simulation.get('abfdSpatialAcceleration'); t=signal.Time(:); values=squeeze(signal.Data);
if size(values,1)~=numel(t), values=values.'; end
[t,ids]=unique(t,'last'); values=values(ids,:);
assert(size(values,2)==6 && all(isfinite(values(:))) && ...
    log.t(1)>=t(1) && log.t(end)<=t(end),'ABFD:AccelerationCoverage','Direct acceleration must cover the observation clock.');
spatial=interp1(t,values,log.t).'; acceleration=spatial;
for k=1:numel(log.t)
    acceleration(4:6,k)=spatialAccelToRpyAccel(log.q(4:6,k),log.qd(4:6,k),spatial(4:6,k));
end
assert(all(isfinite(acceleration(:))),'ABFD:DirectAcceleration','Invalid direct generalized acceleration.');
log.differencedQdd=log.qdd; log.qdd=acceleration;
log.accelerationSource='direct Simscape world linear/angular acceleration, transformed to generalized coordinates';
end
