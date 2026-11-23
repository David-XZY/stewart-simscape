function test_87_dob_aware_strict_qp_contract
% DOB force must shift acceleration predictions and total-force bounds.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'strict_clf_cbf_qp'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
config = makeStrictClfCbfQpConfig(model);
q = model.qHome(:);
qd = zeros(6, 1);
equilibrium = inverseDynamicsCompositeRigidBody(q, qd, zeros(6, 1), model);
offset = [8; -7; 6; -5; 4; -3];
externalWrench = [15; -12; 9; 1.5; -1.2; 0.8];

base = evaluateStrictClfCbfAdModel(q, qd, model, scene, config);
shifted = applyKnownForceOffsetToStrictEvaluation( ...
    base, offset, externalWrench);
externalAcceleration = base.H\externalWrench;
assert(max(abs(shifted.drift-(base.drift+base.inputMap*offset+ ...
    externalAcceleration))) < 1e-12);
assert(max(abs(shifted.legAccelerationDrift- ...
    (base.legAccelerationDrift+base.legAccelerationMap*offset+ ...
    base.Jq*externalAcceleration))) < 1e-12);

[~, baseCbf] = buildStrictClfCbfConstraints(q, qd, q, qd, zeros(6, 1), ...
    model, scene, config, base);
[~, shiftedCbf] = buildStrictClfCbfConstraints(q, qd, q, qd, zeros(6, 1), ...
    model, scene, config, shifted);
knownLegAcceleration = base.legAccelerationMap*offset+ ...
    base.Jq*externalAcceleration;
for legIndex = 1:6
    upper = find(shiftedCbf.names == "acceleration_upper_"+legIndex);
    lower = find(shiftedCbf.names == "acceleration_lower_"+legIndex);
    assert(isscalar(upper) && isscalar(lower));
    assert(abs((shiftedCbf.b(upper)-baseCbf.b(upper))+ ...
        knownLegAcceleration(legIndex)) < 1e-11);
    assert(abs((shiftedCbf.b(lower)-baseCbf.b(lower))- ...
        knownLegAcceleration(legIndex)) < 1e-11);
end

previousTotal = equilibrium;
[totalForce, diagnostic] = stepStrictClfCbfQp(q, qd, q, qd, ...
    zeros(6, 1), equilibrium, previousTotal, model, scene, config, offset, ...
    externalWrench);
regulatedForce = diagnostic.regulatedForceCommand;
predictedQdd = shifted.drift+shifted.inputMap*regulatedForce;
physicalQdd = base.drift+base.inputMap*totalForce+externalAcceleration;
predictedLegAcceleration = shifted.legAccelerationDrift+ ...
    shifted.legAccelerationMap*regulatedForce;
physicalLegAcceleration = base.legAccelerationDrift+ ...
    base.legAccelerationMap*totalForce+base.Jq*externalAcceleration;
assert(max(abs(predictedQdd-physicalQdd)) < 1e-10);
assert(max(abs(predictedLegAcceleration-physicalLegAcceleration)) < 1e-10);
assert(max(abs(totalForce-(regulatedForce+offset))) < 1e-12);
assert(all(totalForce >= diagnostic.forceLower-1e-9));
assert(all(totalForce <= diagnostic.forceUpper+1e-9));
assert(all(abs(totalForce-previousTotal) <= config.forceRateLimit*config.dt+1e-9));

cancelWrench = -base.Jv.'*offset;
cancelled = applyKnownForceOffsetToStrictEvaluation( ...
    base, offset, cancelWrench);
assert(max(abs(cancelled.knownGeneralizedAcceleration)) < 1e-11);
assert(max(abs(cancelled.knownLegAcceleration)) < 1e-11);

% With a tight total-force slew bound, the legacy model rejects a large DOB
% command because it sees only its apparent acceleration. The paired DOB
% model recognizes that the estimated disturbance cancels that acceleration
% and keeps the same total command inside the hard CBF feasible set.
tight = config;
tight.forceRateLimit = 10*ones(6, 1);
largeOffset = 50*ones(6, 1);
previousDobTotal = equilibrium+largeOffset;
[~, legacyDobBlind] = stepStrictClfCbfQp(q, qd, q, qd, zeros(6, 1), ...
    previousDobTotal, previousDobTotal, model, scene, tight);
largeCancelWrench = -base.Jv.'*largeOffset;
[awareForce, dobAware] = stepStrictClfCbfQp(q, qd, q, qd, zeros(6, 1), ...
    equilibrium, previousDobTotal, model, scene, tight, largeOffset, ...
    largeCancelWrench);
assert(legacyDobBlind.usedFallback && ~legacyDobBlind.feasible);
assert(dobAware.feasible && ~dobAware.usedFallback);
assert(dobAware.minimumCbfResidual >= -1e-8);
assert(max(abs(awareForce-previousDobTotal)) < 1e-9);

[legacyForce, legacy] = stepStrictClfCbfQp(q, qd, q, qd, zeros(6, 1), ...
    equilibrium, previousTotal, model, scene, config);
[zeroForce, zero] = stepStrictClfCbfQp(q, qd, q, qd, zeros(6, 1), ...
    equilibrium, previousTotal, model, scene, config, zeros(6, 1));
assert(max(abs(legacyForce-zeroForce)) < 1e-12);
assert(max(abs(legacy.cbf.b-zero.cbf.b)) < 1e-12);
assert(max(abs(zero.knownForceOffset)) == 0);
fprintf('test_87_dob_aware_strict_qp_contract passed.\n');
end
