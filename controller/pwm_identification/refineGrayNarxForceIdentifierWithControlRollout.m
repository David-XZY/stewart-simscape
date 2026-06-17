function refined = refineGrayNarxForceIdentifierWithControlRollout( ...
        dataset, teacher, initialIdentifier, evaluationRefs, overrides)
% refineGrayNarxForceIdentifierWithControlRollout - 使用独立闭环轨迹精炼力辨识模型
arguments
    dataset struct
    teacher struct
    initialIdentifier struct
    evaluationRefs struct
    overrides struct = struct()
end

options = struct('poseScale', [0.72; 0.83; 0.68; 0.75; 0.80; 0.70], ...
    'duration', inf, 'controlOverrides', struct());
fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('refineGrayNarxForceIdentifierWithControlRollout:UnknownOverride', ...
            '未知闭环精炼配置：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end

refinementRefs = buildDistinctReference(evaluationRefs, options.poseScale);
controlOverrides = options.controlOverrides;
controlOverrides.duration = options.duration;
rollout = comparePwmPoseForceControlLines(refinementRefs, teacher, initialIdentifier, ...
    controlOverrides);
augmentedDataset = appendControlRollout(dataset, rollout.identified, teacher);
refined = trainGrayNarxForceIdentifier(augmentedDataset, teacher);

initialReport = evaluateGrayNarxForceIdentifier(initialIdentifier, dataset);
refinedReport = evaluateGrayNarxForceIdentifier(refined, dataset);
refined.training.closedLoopRefinement = struct( ...
    'usedDistinctReference', max(abs(refinementRefs.q - evaluationRefs.q), [], 'all') > 1e-9, ...
    'usedUkfPoseFusion', rollout.identified.observability.usedUkfPoseFusion, ...
    'addedSampleCount', numel(rollout.identified.t), ...
    'poseScale', options.poseScale, ...
    'initialIndependentTestNrmse', initialReport.metrics.narxTestNrmse, ...
    'refinedIndependentTestNrmse', refinedReport.metrics.narxTestNrmse, ...
    'rolloutAlignedForceEstimateRms', rollout.metrics.identifiedAlignedForceEstimateRms);
end

function refs = buildDistinctReference(source, poseScale)
model = buildOptModelCustom();
refs = source;
refs.q = model.qHome + poseScale .* (source.q - model.qHome);
refs.qd = poseScale .* source.qd;
qdd = finiteDifference(refs.qd, refs.t);
refs.Fleg = zeros(size(source.Fleg));
for index = 1:numel(refs.t)
    refs.Fleg(:, index) = inverseDynamicsCompositeRigidBody( ...
        refs.q(:, index), refs.qd(:, index), qdd(:, index), model);
end
end

function derivative = finiteDifference(signal, time)
derivative = zeros(size(signal));
derivative(:, 1) = (signal(:, 2) - signal(:, 1)) / (time(2) - time(1));
derivative(:, end) = (signal(:, end) - signal(:, end - 1)) / ...
    (time(end) - time(end - 1));
for index = 2:numel(time) - 1
    derivative(:, index) = (signal(:, index + 1) - signal(:, index - 1)) / ...
        (time(index + 1) - time(index - 1));
end
end

function augmented = appendControlRollout(dataset, rollout, teacher)
augmented = dataset;
sampleCount = numel(rollout.t);
model = buildOptModelCustom();
encoderLength = zeros(sampleCount, 6);
for index = 1:sampleCount
    encoderLength(index, :) = sgpIK(rollout.qTrue(:, index), model).L.';
end
encoderLength = round(encoderLength * 16500) / 16500;
rolloutLegSpeedRows = transpose(rollout.legSpeed);
rolloutPwmRows = transpose(rollout.pwm);
deadzoneRows = transpose(teacher.deadzonePwm);
legAcceleration = [zeros(1, 6); diff(rolloutLegSpeedRows, 1, 1) / dataset.sampleTime];
appliedPwm = sign(rolloutPwmRows) .* max(abs(rolloutPwmRows) - deadzoneRows, 0);

augmented.pwm = [dataset.pwm; rolloutPwmRows];
augmented.appliedPwm = [dataset.appliedPwm; appliedPwm];
augmented.encoderLength = [dataset.encoderLength; encoderLength];
augmented.legSpeed = [dataset.legSpeed; rolloutLegSpeedRows];
augmented.trueLegSpeed = [dataset.trueLegSpeed; rollout.trueLegSpeed.'];
augmented.legAcceleration = [dataset.legAcceleration; legAcceleration];
augmented.measuredPose = [dataset.measuredPose; rollout.qFeedback.'];
augmented.platformPose = [dataset.platformPose; rollout.qTrue.'];
augmented.platformVelocity = [dataset.platformVelocity; rollout.qdTrue.'];
augmented.trueForce = [dataset.trueForce; rollout.trueForce.'];
augmented.hiddenCurrent = [dataset.hiddenCurrent; zeros(sampleCount, 6)];
augmented.time = [dataset.time; dataset.time(end) + dataset.sampleTime + ...
    (0:sampleCount - 1).' * dataset.sampleTime];
augmented.split.train = [dataset.split.train; true(sampleCount, 1)];
augmented.split.validation = [dataset.split.validation; false(sampleCount, 1)];
augmented.split.test = [dataset.split.test; false(sampleCount, 1)];
end
