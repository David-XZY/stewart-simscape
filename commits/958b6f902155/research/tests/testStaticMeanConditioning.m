function report=testStaticMeanConditioning(asset,bundle,calibrationFile)
profile=makeHighFidelityPlantProfile('high-fidelity');
base=buildDynamicsProvider(bundle.model,'residual-B',asset,false,bundle.refs.q(:,1)+profile.initialPoseOffset);
provider=conditionResidualMeanOnStaticBalance(base,bundle,profile,calibrationFile);
q=provider.biasCalibration.pose; expected=provider.biasCalibration.staticForce;
force=provider.inverse(q,zeros(6,1),zeros(6,1)); equilibriumError=norm(force-expected,inf);
assert(equilibriumError<1e-9 && isequal(provider.network,base.network), ...
    'ABFD:StaticMeanConditioning','Equilibrium does not match or trained weights changed.');
ad=buildCasadiCompositeDynamics(provider); numericError=0; derivativeError=0;
for k=1:6
    x=[q;zeros(6,1)]+1e-3*sin((1:12).'+k); u=force+5*cos((1:6).'+k);
    numeric=provider.forward(x,u); symbolic=full(ad.f(x,u)); numericError=max(numericError,norm(numeric-symbolic,inf));
    [A,B]=ad.linearize(x,u); finite=zeros(12,18); h=1e-6;
    for j=1:18
        delta=zeros(18,1); delta(j)=h;
        finite(:,j)=(provider.forward(x+delta(1:12),u+delta(13:18))- ...
            provider.forward(x-delta(1:12),u-delta(13:18)))/(2*h);
    end
    derivativeError=max(derivativeError,norm([full(A),full(B)]-finite,'fro')/max(1,norm(finite,'fro')));
end
assert(numericError<1e-9 && derivativeError<1e-6,'ABFD:ConditionedAD','Conditioned numerical/AD dynamics differ.');
rejected=false;
try
    buildAbfdTrackingExpressions([],[],[],[],provider,[],[],[]);
catch e
    rejected=strcmp(e.identifier,'ABFD:UncalibratedConditionalUncertainty');
end
assert(rejected,'ABFD:ConditionedUncertaintyGuard','Unvalidated conditional covariance was accepted.');
report=struct('passed',true,'equilibriumForceError',equilibriumError,'numericError',numericError, ...
    'jacobianRelativeError',derivativeError,'uncalibratedTubeRejected',rejected, ...
    'baseStaticForce',base.inverse(q,zeros(6,1),zeros(6,1)), ...
    'measuredStaticForce',expected,'wrenchOffset',provider.biasCalibration.wrenchOffset, ...
    'scope','Static balance and dynamics/Jacobian contract; closed-loop benefit and conditional uncertainty not validated');
end
