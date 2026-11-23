function provider=conditionResidualOnMeasuredInertia(base,bundle,profile,calibrationFile)
% Additional identification ablation from measured initial force responses.
% Neural weights are unchanged. This is a local rigid inertia approximation,
% followed by static balance; its uncertainty must not reuse the old VAR.
assert(strcmp(base.mode,'residual-B') && ~isfield(base,'biasCalibration') && ...
    ~isfield(base,'parameterCalibration'),'ABFD:InertiaCalibrationProvider');
assert(~isfield(base,'elasticity'),'ABFD:InertiaCalibrationProvider', ...
    'This legacy inertia-conditioning ablation does not preserve a structured elasticity asset.');
applyAbfdStaticPreload(profile,bundle,calibrationFile);
calibration=jsondecode(fileread(calibrationFile)); q=calibration.pose(:);
validateattributes(calibration.sensitivity,{'double'},{'size',[6,6],'real','finite'});
assert(rcond(calibration.sensitivity)>1e-8,'ABFD:InertiaSensitivity');
nominal=buildDynamicsProvider(base.nominalModel); o=nominal.evaluate(q,zeros(6,1),zeros(6,1));
% Sensitivity is world spatial acceleration per actual leg force.
spatialMass=o.Jv.'/calibration.sensitivity; [E,~]=rpyRateMapZYX(q(4:6),zeros(3,1));
T=blkdiag(eye(3),E); measuredH=spatialMass*T;
assert(norm(spatialMass-spatialMass.','fro')/norm(spatialMass,'fro')<1e-8 && ...
    min(eig((spatialMass+spatialMass.')/2))>0,'ABFD:MeasuredSpatialMass');
baseline=buildInertialWrenchRegressor(q,zeros(6,1),zeros(6,1),base.nominalModel); Y=zeros(36,10);
for k=1:6
    a=zeros(6,1); a(k)=1;
    Y(6*(k-1)+(1:6),:)=buildInertialWrenchRegressor(q,zeros(6,1),a,base.nominalModel)-baseline;
end
[~,R]=qr(Y,0); assert(rcond(R)>1e-8,'ABFD:InertiaIdentificationRank');
parameters=Y\measuredH(:); mass=parameters(1); com=parameters(2:4)/mass;
I=[parameters(5),parameters(8),parameters(9);parameters(8),parameters(6),parameters(10); ...
    parameters(9),parameters(10),parameters(7)];
inertia=I-mass*((com.'*com)*eye(3)-com*com.');
relativeResidual=norm(reshape(Y*parameters,6,6)-measuredH,'fro')/norm(measuredH,'fro');
assert(mass>0 && min(eig(inertia))>0 && min(eig(trace(inertia)/2*eye(3)-inertia))>=-1e-9 && ...
    norm(com)<.5 && relativeResidual<.01,'ABFD:InertiaCalibrationRejected', ...
    'Measured inertia is not accurately represented by a realizable rigid model.');
effective=base.model; effective.dynamics.totalMass=mass; effective.dynamics.comP=com;
effective.dynamics.inertiaAtCOM_P=inertia; rigid=buildDynamicsProvider(effective);
provider=base; provider.model=effective;
provider.parameterCalibration=struct('method','Least squares rigid inertia from independently measured spatial acceleration per actual force', ...
    'pose',q,'mass',mass,'com',com,'inertia',inertia,'relativeResidual',relativeResidual, ...
    'calibrationHash',abfdFileHash(calibrationFile),'baseProviderHash',base.hash,'uncertaintyValidated',false, ...
    'scope','One-pose inertia identification ablation; trained neural weights unchanged; no unseen-load or A/tube acceptance');
provider.hash=abfdHash(struct('base',base.hash,'parameterCalibration',provider.parameterCalibration));
provider.evaluate=@evaluate; provider.forward=@forward; provider.inverse=@inverse;
provider=conditionResidualMeanOnStaticBalance(provider,bundle,profile,calibrationFile);
    function out=evaluate(q,v,a)
        out=rigid.evaluate(q,v,a); [mu,Sigma]=evaluateResidualMlp([q(:);v(:)],base.network);
        out.bias=out.bias+mu; out.W=out.W+mu; out.Sigma=Sigma;
    end
    function derivative=forward(x,force)
        out=evaluate(x(1:6),x(7:12),zeros(6,1));
        derivative=[x(7:12);out.H\(out.Jv.'*force(:)-out.bias)];
    end
    function force=inverse(q,v,a)
        out=evaluate(q,v,a); force=out.Jv.'\out.W;
    end
end
