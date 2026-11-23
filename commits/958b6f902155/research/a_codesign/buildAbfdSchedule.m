function design=buildAbfdSchedule(refs,provider,c)
if isfield(c,'predictionProfile') && any(strcmp(c.predictionProfile,{'sampled-hold-v2','actuator-var-v3'}))
    refs=abfdControllerReference(refs,provider,c.sampleTime);
end
if strcmp(c.controllerMode,'geometric-D')
    if strcmp(c.dVariant,'tvlqr')
        schedule=buildTransmissionWeightedTvlqr(refs,provider,c);
    else
        schedule=struct('K',repmat([25*eye(6),10*eye(6)],1,1,numel(refs.t)-1), ...
            't',refs.t,'variant',c.dVariant,'providerHash',provider.hash);
    end
    policy=buildGeometricControllerAD(provider,schedule.variant);
    kind='geometric';
else
    lin=linearizeTrajectoryAD(refs,provider);
    Q=repmat(c.Q,1,1,size(lin.A,3)); R=repmat(c.RF,1,1,size(lin.A,3));
    for k=1:size(Q,3), dt=diff(refs.t(k:k+1)); Q(:,:,k)=Q(:,:,k)*dt; R(:,:,k)=R(:,:,k)*dt; end
    schedule=buildFiniteHorizonFullStateLqr(lin.A,lin.B,Q,R,c.Q);
    schedule.t=refs.t; schedule.variant='fullstate-tvlqr'; policy=[]; kind='force';
end
design=struct('schedule',schedule,'policy',policy,'kind',kind);
end
