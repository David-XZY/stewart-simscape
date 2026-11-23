function provider=conditionResidualMeanOnStaticBalance(base,bundle,profile,calibrationFile)
% Diagnostic inference-time offset from an independently measured equilibrium.
% The trained weights remain unchanged. This does not validate their covariance
% after conditioning. Propagation requires a matching explicit pilot temporal
% model and initial-observation contract; the legacy covariance is rejected.
assert(strcmp(base.mode,'residual-B') && ~isfield(base,'biasCalibration'), ...
    'ABFD:BiasCalibrationProvider','Use an unconditioned residual-B provider.');
assert(strcmp(abfdHash(base.nominalModel),abfdHash(bundle.model)), ...
    'ABFD:BiasCalibrationModel','Provider and calibration reference differ.');
calibrated=applyAbfdStaticPreload(profile,bundle,calibrationFile);
calibration=jsondecode(fileread(calibrationFile)); q=calibration.pose(:);
if isfield(base.network,'meanAnchoring')
    assert(isfield(base,'parameterCalibration') && ...
        strcmp(base.parameterCalibration.calibrationHash,abfdFileHash(calibrationFile)) && ...
        strcmp(base.parameterCalibration.plantHash,calibration.plantHash), ...
        'ABFD:StartupBalanceMismatch','Inertia and static balance must use the same independent startup observation.');
end
state=base.evaluate(q,zeros(6,1),zeros(6,1));
offset=state.Jv.'*calibrated.initialAppliedForce-state.W;
validateattributes(offset,{'double'},{'size',[6,1],'real','finite'});
provider=base;
provider.biasCalibration=struct('method','independently measured static equilibrium world-wrench offset', ...
    'wrenchOffset',offset,'pose',q,'staticForce',calibrated.initialAppliedForce, ...
    'calibrationFileHash',abfdFileHash(calibrationFile),'plantHash',calibration.plantHash, ...
    'baseProviderHash',base.hash,'uncertaintyValidated',false, ...
    'scope','Additional calibration ablation; trained weights unchanged; no A/tube acceptance');
provider.hash=abfdHash(struct('baseProviderHash',base.hash,'biasCalibration',provider.biasCalibration));
provider.evaluate=@evaluate; provider.forward=@forward; provider.inverse=@inverse;
    function out=evaluate(q,v,a)
        out=base.evaluate(q,v,a); out.bias=out.bias+offset; out.W=out.W+offset;
    end
    function derivative=forward(x,force)
        out=evaluate(x(1:6),x(7:12),zeros(6,1));
        derivative=[x(7:12);out.H\(out.Jv.'*force(:)-out.bias)];
    end
    function force=inverse(q,v,a)
        out=evaluate(q,v,a); force=out.Jv.'\out.W;
    end
end
