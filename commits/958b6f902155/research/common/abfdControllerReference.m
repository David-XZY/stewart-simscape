function sampled=abfdControllerReference(refs,provider,dt)
% HS knots certify/interpolate the reference; only actual controller ticks
% define the gain schedule. Include the true terminal time for the last cost.
times=0:dt:refs.t(end);
if refs.t(end)-times(end)>1e-10, times=[times,refs.t(end)]; end
[error,indices]=min(abs(refs.t(:)-times),[],1);
if any(error>1e-10)
    assert(isfield(refs,'hs'),'ABFD:MissingHsData','Resampling gains requires the complete HS reference.');
    refs=reconstructIhsidContinuousReference(refs.hs,provider,dt);
    [error,indices]=min(abs(refs.t(:)-times),[],1);
end
assert(all(error<=1e-10),'ABFD:ControllerClock','Reference must cover all controller update times.');
sampled=struct('t',times,'q',refs.q(:,indices),'qd',refs.qd(:,indices), ...
    'qdd',refs.qdd(:,indices),'Fleg',refs.Fleg(:,indices));
end
