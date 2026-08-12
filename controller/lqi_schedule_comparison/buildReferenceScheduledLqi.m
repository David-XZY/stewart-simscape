function schedule = buildReferenceScheduledLqi(reference, model, config, options)
% buildReferenceScheduledLqi - Build fixed or reference-scheduled LQI data.
%
% The controller matrices are designed off line.  Runtime code only selects
% the precomputed discrete controller for the current reference sample.
arguments
    reference struct
    model struct
    config struct
    options.mode {mustBeMember(options.mode, {'fixed', 'scheduled'})} = 'scheduled'
    options.fixedPose double = []
    options.fixedVelocity double = []
    options.fixedForce double = []
    options.stateStep double = [1e-6 * ones(6, 1); 1e-5 * ones(6, 1)]
    options.inputStep (1, 1) double {mustBePositive} = 1
    options.checkConvergence (1, 1) logical = true
end

validateReference(reference);
time = reference.time(:).';
sampleCount = numel(time);
dt = median(diff(time));
if max(abs(diff(time)-dt)) > 1e-11
    error('buildReferenceScheduledLqi:NonuniformReference', ...
        'The reference time grid must be uniform.');
end

if strcmp(options.mode, 'fixed')
    fixedPose = valueOrDefault(options.fixedPose, model.qHome(:));
    fixedVelocity = valueOrDefault(options.fixedVelocity, zeros(6, 1));
    fixedForce = valueOrDefault(options.fixedForce, ...
        inverseDynamicsCompositeRigidBody(fixedPose, fixedVelocity, zeros(6, 1), model));
    validateattributes(fixedPose, {'double'}, {'real', 'finite', 'size', [6, 1]});
    validateattributes(fixedVelocity, {'double'}, {'real', 'finite', 'size', [6, 1]});
    validateattributes(fixedForce, {'double'}, {'real', 'finite', 'size', [6, 1]});
    item = designOne(fixedPose, fixedVelocity, fixedForce, model, config, dt, options);
    samples = repmat(item, sampleCount, 1);
    referenceIndex = ones(sampleCount, 1);
else
    samples = repmat(emptySample(), sampleCount, 1);
    referenceIndex = (1:sampleCount).';
    for sampleIndex = 1:sampleCount
        try
            samples(sampleIndex) = designOne(reference.q(:, sampleIndex), ...
                reference.qd(:, sampleIndex), reference.computedForce(:, sampleIndex), ...
                model, config, dt, options);
        catch exception
            error('buildReferenceScheduledLqi:DesignFailed', ...
                'LQI design failed at reference sample %d (t=%.9g s): %s', ...
                sampleIndex, time(sampleIndex), exception.message);
        end
    end
end

schedule = struct();
schedule.type = string(options.mode);
schedule.time = time;
schedule.dt = dt;
schedule.samples = samples;
schedule.referenceIndex = referenceIndex;
schedule.sampleCount = sampleCount;
schedule.allStable = all([samples.stable]);
schedule.maxInputMapRelativeError = max([samples.inputMapRelativeError]);
schedule.maxAConvergenceRelative = max([samples.aConvergenceRelative]);
schedule.minimumMassMatrixRcond = min([samples.massMatrixRcond]);
schedule.minimumForceMapRcond = min([samples.forceMapRcond]);
if ~schedule.allStable
    error('buildReferenceScheduledLqi:UnstableDesign', ...
        'At least one precomputed LQI design is not stable.');
end
end

function item = designOne(q, qd, force, model, config, dt, options)
linearization = linearizeCompositeRigidBodyLqi(q, qd, force, model, ...
    'stateStep', options.stateStep, 'inputStep', options.inputStep, ...
    'checkConvergence', options.checkConvergence);
lqiDesign = designLqiPoseForceController(linearization.plant, model.Lc, config);
discreteController = c2d(lqiDesign.Kx, dt, 'tustin');
[controllerA, controllerB, controllerC, controllerD] = ssdata(discreteController);

item = emptySample();
item.A = linearization.A;
item.B = linearization.B;
item.C = linearization.C;
item.D = linearization.D;
item.controllerA = controllerA;
item.controllerB = controllerB;
item.controllerC = controllerC;
item.controllerD = controllerD;
item.stateGain = lqiDesign.stateGain;
item.integralGain = lqiDesign.integralGain;
item.observerGain = lqiDesign.observerGain;
item.closedLoopPoles = lqiDesign.closedLoopPoles;
item.stable = lqiDesign.stable;
item.massMatrixRcond = linearization.massMatrixRcond;
item.forceMapRcond = linearization.forceMapRcond;
item.inputMapRelativeError = linearization.inputMapRelativeError;
item.aConvergenceRelative = linearization.aConvergenceRelative;
item.controllabilityRank = rank(ctrb([linearization.A, zeros(12, 6); ...
    -linearization.C, zeros(6, 6)], [linearization.B; zeros(6, 6)]));
item.observabilityRank = rank(obsv(linearization.A, linearization.C));
end

function item = emptySample()
item = struct('A', [], 'B', [], 'C', [], 'D', [], ...
    'controllerA', [], 'controllerB', [], 'controllerC', [], 'controllerD', [], ...
    'stateGain', [], 'integralGain', [], 'observerGain', [], ...
    'closedLoopPoles', [], 'stable', false, 'massMatrixRcond', NaN, ...
    'forceMapRcond', NaN, 'inputMapRelativeError', NaN, ...
    'aConvergenceRelative', NaN, 'controllabilityRank', NaN, ...
    'observabilityRank', NaN);
end

function value = valueOrDefault(value, fallback)
if isempty(value)
    value = fallback;
else
    value = value(:);
end
end

function validateReference(reference)
required = {'time', 'q', 'qd', 'computedForce'};
for index = 1:numel(required)
    if ~isfield(reference, required{index})
        error('buildReferenceScheduledLqi:MissingReferenceField', ...
            'reference.%s is required.', required{index});
    end
end
count = numel(reference.time);
if count < 2 || ~isequal(size(reference.q), [6, count]) || ...
        ~isequal(size(reference.qd), [6, count]) || ...
        ~isequal(size(reference.computedForce), [6, count])
    error('buildReferenceScheduledLqi:InvalidReference', ...
        'Reference q, qd, and computedForce must each be 6-by-N.');
end
if any(~isfinite([reference.q(:); reference.qd(:); reference.computedForce(:)])) || ...
        any(~isfinite(reference.time(:))) || any(diff(reference.time(:)) <= 0)
    error('buildReferenceScheduledLqi:InvalidReference', ...
        'Reference values must be finite and time must be strictly increasing.');
end
end
