function report=testMeasuredInertiaConditioning(asset,bundle,calibrationFile)
base=buildDynamicsProvider(bundle.model,'residual-B',asset); profile=makeHighFidelityPlantProfile('high-fidelity');
provider=conditionResidualOnMeasuredInertia(base,bundle,profile,calibrationFile);
assert(isequal(base.network,provider.network),'ABFD:CalibrationChangedNetwork');
q=provider.biasCalibration.pose; force=provider.biasCalibration.staticForce;
equilibriumError=norm(provider.inverse(q,zeros(6,1),zeros(6,1))-force,inf);
ad=buildCasadiCompositeDynamics(provider); maxValueError=0; maxDerivativeError=0;
for k=1:6
    x=[q;zeros(6,1)]+1e-3*sin((1:12).'+k); u=force+5*cos((1:6).'+k);
    maxValueError=max(maxValueError,norm(provider.forward(x,u)-full(ad.f(x,u)),inf));
    [A,B]=ad.linearize(x,u); finite=zeros(12,18); h=1e-6;
    for j=1:18
        delta=zeros(18,1); delta(j)=h;
        finite(:,j)=(provider.forward(x+delta(1:12),u+delta(13:18))- ...
            provider.forward(x-delta(1:12),u-delta(13:18)))/(2*h);
    end
    maxDerivativeError=max(maxDerivativeError,norm([full(A),full(B)]-finite,'fro')/max(1,norm(finite,'fro')));
end
rejected=false;
try
    buildAbfdTrackingExpressions([],[],[],[],provider,[],[],[]);
catch e
    rejected=strcmp(e.identifier,'ABFD:UncalibratedConditionalUncertainty');
end
assert(equilibriumError<1e-9 && maxValueError<1e-9 && maxDerivativeError<1e-6 && rejected,'ABFD:MeasuredInertiaContract');
report=struct('passed',true,'equilibriumForceError',equilibriumError,'numericValueError',maxValueError, ...
    'jacobianRelativeError',maxDerivativeError,'unvalidatedTubeRejected',rejected, ...
    'parameters',provider.parameterCalibration,'scope','Actual calibration numeric/AD contract only; closed-loop acceptance remains separate');
end
