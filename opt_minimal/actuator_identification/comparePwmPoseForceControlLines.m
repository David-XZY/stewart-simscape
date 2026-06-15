function comparison = comparePwmPoseForceControlLines(refs, teacher, identified, overrides)
% comparePwmPoseForceControlLines - 比较真值基准线与辨识反馈线
arguments
    refs struct
    teacher struct
    identified struct
    overrides struct = struct()
end

options = defaultOptions(teacher, overrides);
model = buildOptModelCustom();
reference = buildReference(refs, options);

comparison = struct();
comparison.oracle = simulateLine('oracle', reference, model, teacher, identified, options);
comparison.identified = simulateLine('identified', reference, model, teacher, identified, options);
comparison.metrics = buildMetrics(comparison, teacher);
comparison.options = options;
end

function options = defaultOptions(teacher, overrides)
options = struct();
options.duration = inf;
options.encoderResolution = 1 / 16500;
options.encoderVelocityFilterAlpha = 0.6;
options.useFusedPoseLegSpeed = true;
options.poseEstimatorMode = "ukf";
options.sensorNoiseEnabled = true;
options.sensorRandomSeed = 41;
options.encoderNoiseStd = 5e-4;
options.orientationNoiseStd = deg2rad(0.0055);
options.ukfOrientationNoiseStd = deg2rad([0.1; 0.1; 0.5]);
options.orientationBias = deg2rad([0.1; -0.1; 0.5]);
options.accelerationNoiseStd = 9.80665e-3;
options.accelerometerBias = 9.80665 * [20; -20; 40] * 1e-3;
options.angularVelocityNoiseStd = deg2rad(0.07);
options.gyroBias = deg2rad([0.5; -0.5; 1.0]);
options.homeCalibrationApplied = true;
options.orientationCalibrationResidual = zeros(3, 1);
options.accelerometerCalibrationResidual = 9.80665 * [15; -15; 35] * 1e-6;
options.gyroCalibrationResidual = deg2rad([8; -8; 8] / 3600);
options.ukfEncoderNoiseScale = 0.2;
options.ukfWarmupDuration = 0.2;
options.accelerationMode = "specificForce";
options.poseVelocityFilterAlpha = 0;
options.forceEstimateFilterAlpha = 0;
options.dynamicsForceObserverWeight = 0;
options.accelerationFilterAlpha = 0.85;
options.translationIterations = 4;
options.translationGain = [8.0e4; 8.0e4; 2.4e5];
options.rotationGain = [3.0e3; 3.0e3; 3.0e3];
options.translationRateGain = [2.4e4; 2.4e4; 6.0e4];
options.translationIntegralGain = [3.0e5; 3.0e5; 8.0e5];
options.translationIntegralLimit = [5e-3; 5e-3; 5e-3];
options.rotationRateGain = [8.0e2; 8.0e2; 8.0e2];
options.forceCorrectionLimit = 2000;
options.forceKpPwmPerNewton = 0.4;
options.forceKiPwmPerNewtonSecond = 5.0;
options.pwmRateLimit = 4000;
options.oracleForceKpPwmPerNewton = 1.1;
options.oracleForceKiPwmPerNewtonSecond = 12.0;
options.oraclePwmRateLimit = 4000;
options.sampleTime = teacher.sampleTime;

fields = fieldnames(overrides);
for index = 1:numel(fields)
    if ~isfield(options, fields{index})
        error('comparePwmPoseForceControlLines:UnknownOverride', ...
            '未知双线仿真参数：%s。', fields{index});
    end
    options.(fields{index}) = overrides.(fields{index});
end
end

function reference = buildReference(refs, options)
duration = min(options.duration, refs.t(end));
nodeMask = refs.t <= duration;
lastNode = find(nodeMask, 1, 'last');
if lastNode < numel(refs.t) && refs.t(lastNode) < duration
    lastNode = lastNode + 1;
end
nodeIndices = 1:lastNode;
poseReference = reconstructHermitePoseReference( ...
    refs.t(nodeIndices), refs.q(:, nodeIndices), refs.qd(:, nodeIndices), options.sampleTime);
timeMask = poseReference.t <= duration + 1e-12;

reference = struct();
reference.t = poseReference.t(timeMask);
reference.q = poseReference.q(:, timeMask);
reference.qd = poseReference.qd(:, timeMask);
reference.force = interp1(refs.t(:), refs.Fleg.', reference.t(:), 'linear').';
end

function result = simulateLine(mode, reference, model, teacher, identified, options)
sampleCount = numel(reference.t);
controllerModel = teacher;
if strcmp(mode, 'identified')
    controllerModel = identified;
end
controller = makePwmForceController(mode, controllerModel);
controller.kpPwmPerNewton = options.forceKpPwmPerNewton;
controller.kiPwmPerNewtonSecond = options.forceKiPwmPerNewtonSecond;
controller.pwmRateLimit = options.pwmRateLimit;
if strcmp(mode, 'oracle')
    controller.kpPwmPerNewton = options.oracleForceKpPwmPerNewton;
    controller.kiPwmPerNewtonSecond = options.oracleForceKiPwmPerNewtonSecond;
    controller.pwmRateLimit = options.oraclePwmRateLimit;
end
controllerState = initializePwmForceControllerState(controller);
plantState = initializeHighFidelityPwmActuatorState(teacher);
estimatorState = [];
if strcmp(mode, 'identified')
    estimatorState = initializeGrayNarxForceEstimatorState(identified);
end

x = [reference.q(:, 1); reference.qd(:, 1)];
previousEstimatedPose = reference.q(:, 1);
initialLegSpeed = sgpJacobian(x(1:6), model).Jq * x(7:12);
anchorPose = reference.q(:, 1);
anchorLength = sgpIK(anchorPose, model).L;
previousEncoderLength = zeros(6, 1);
previousFeedbackLegSpeed = initialLegSpeed;
[controllerState, plantState, initialPwm, initialForce] = initializeStaticLoad( ...
    controllerState, plantState, reference.force(:, 1), initialLegSpeed, controller, teacher);
previousFeedbackForce = initialForce;
poseIntegral = zeros(3, 1);
if strcmp(mode, 'identified')
    for warmupIndex = 1:20
        [estimatorState, previousFeedbackForce] = stepGrayNarxForceEstimator( ...
            estimatorState, initialPwm, initialLegSpeed, identified);
    end
end
previousFilteredForce = previousFeedbackForce;
previousFeedbackVelocity = reference.qd(:, 1);
previousFilteredPoseVelocity = reference.qd(:, 1);
filteredAcceleration = zeros(6, 1);
previousTrueAcceleration = zeros(6, 1);
poseUkf = [];
if strcmp(mode, 'identified')
    rng(options.sensorRandomSeed);
    if strcmp(options.poseEstimatorMode, "ukf")
        ukfOverrides = struct( ...
            'encoderNoiseStd', options.ukfEncoderNoiseScale * options.encoderNoiseStd, ...
            'orientationNoiseStd', options.ukfOrientationNoiseStd, ...
            'accelerationNoiseStd', options.accelerationNoiseStd, ...
            'angularVelocityNoiseStd', options.angularVelocityNoiseStd);
        poseUkf = initializePoseImuUkf(makePoseImuUkfConfig( ...
            model, options.sampleTime, anchorPose, ukfOverrides));
        poseUkf = warmupPoseImuUkf(poseUkf, anchorPose, model, options);
    end
end

result = initializeResult(mode, reference, teacher, sampleCount, options);
for sampleIndex = 1:sampleCount
    qTrue = x(1:6);
    qdTrue = x(7:12);
    jacobian = sgpJacobian(qTrue, model);
    trueLegSpeed = jacobian.Jq * qdTrue;

    if strcmp(mode, 'oracle')
        feedbackPose = qTrue;
        feedbackVelocity = qdTrue;
        feedbackForce = previousFeedbackForce;
        estimatedForce = previousFeedbackForce;
        feedbackLegSpeed = trueLegSpeed;
    else
        encoderLength = sgpIK(qTrue, model).L - anchorLength;
        orientationMeasurement = qTrue(4:6);
        worldAcceleration = previousTrueAcceleration(1:3);
        specificForce = rpy2rotmZYX(qTrue(4:6)).' * ...
            (worldAcceleration - model.g);
        angularVelocity = rpyRateMapZYX(qTrue(4:6)) * qdTrue(4:6);
        if options.sensorNoiseEnabled
            encoderLength = encoderLength + options.encoderNoiseStd * randn(6, 1);
            orientationBias = options.orientationBias;
            accelerometerBias = options.accelerometerBias;
            gyroBias = options.gyroBias;
            if options.homeCalibrationApplied
                orientationBias = options.orientationCalibrationResidual;
                accelerometerBias = options.accelerometerCalibrationResidual;
                gyroBias = options.gyroCalibrationResidual;
            end
            orientationMeasurement = orientationMeasurement + orientationBias + ...
                options.orientationNoiseStd .* randn(3, 1);
            worldAcceleration = worldAcceleration + accelerometerBias + ...
                options.accelerationNoiseStd * randn(3, 1);
            specificForce = specificForce + accelerometerBias + ...
                options.accelerationNoiseStd * randn(3, 1);
            angularVelocity = angularVelocity + gyroBias + ...
                options.angularVelocityNoiseStd * randn(3, 1);
        end
        encoderLength = quantizeSignal(encoderLength, options.encoderResolution);
        encoderDifferencedSpeed = (encoderLength - previousEncoderLength) / options.sampleTime;
        previousEncoderLength = encoderLength;
        if strcmp(options.poseEstimatorMode, "ukf")
            imuSample = struct( ...
                'relativeLength', encoderLength, ...
                'orientation', orientationMeasurement, ...
                'angularVelocity', angularVelocity, ...
                'accelerationMode', options.accelerationMode);
            if options.accelerationMode == "world"
                imuSample.acceleration = worldAcceleration;
            else
                imuSample.acceleration = specificForce;
            end
            [poseUkf, ukfOutput] = stepPoseImuUkf(poseUkf, imuSample);
            feedbackPose = ukfOutput.pose;
            feedbackVelocity = ukfOutput.velocity;
        else
            feedbackPose = estimatePoseFromEncoderImu( ...
                anchorLength + encoderLength, orientationMeasurement, ...
                previousEstimatedPose, model, options);
            rawPoseVelocity = (feedbackPose - previousEstimatedPose) / options.sampleTime;
            feedbackVelocity = options.poseVelocityFilterAlpha * previousFilteredPoseVelocity + ...
                (1 - options.poseVelocityFilterAlpha) * rawPoseVelocity;
        end
        previousFilteredPoseVelocity = feedbackVelocity;
        previousEstimatedPose = feedbackPose;
        rawAcceleration = (feedbackVelocity - previousFeedbackVelocity) / options.sampleTime;
        filteredAcceleration = options.accelerationFilterAlpha * filteredAcceleration + ...
            (1 - options.accelerationFilterAlpha) * rawAcceleration;
        previousFeedbackVelocity = feedbackVelocity;
        if options.useFusedPoseLegSpeed
            feedbackJacobian = sgpJacobian(feedbackPose, model);
            rawLegSpeed = feedbackJacobian.Jq * feedbackVelocity;
        else
            rawLegSpeed = encoderDifferencedSpeed;
        end
        feedbackLegSpeed = options.encoderVelocityFilterAlpha * previousFeedbackLegSpeed + ...
            (1 - options.encoderVelocityFilterAlpha) * rawLegSpeed;
        previousFeedbackLegSpeed = feedbackLegSpeed;
        [estimatorState, rawEstimatedForce] = stepGrayNarxForceEstimator( ...
            estimatorState, controllerState.previousPwm, feedbackLegSpeed, identified);
        feedbackForce = options.forceEstimateFilterAlpha * previousFilteredForce + ...
            (1 - options.forceEstimateFilterAlpha) * rawEstimatedForce;
        dynamicsEstimatedForce = inverseDynamicsCompositeRigidBody( ...
            feedbackPose, feedbackVelocity, filteredAcceleration, model);
        feedbackForce = (1 - options.dynamicsForceObserverWeight) * feedbackForce + ...
            options.dynamicsForceObserverWeight * dynamicsEstimatedForce;
        previousFilteredForce = feedbackForce;
        estimatedForce = feedbackForce;
    end

    [targetForce, poseIntegral] = addPoseFeedback(reference.force(:, sampleIndex), ...
        reference.q(:, sampleIndex), reference.qd(:, sampleIndex), ...
        feedbackPose, feedbackVelocity, poseIntegral, model, options);
    [controllerState, pwmCommand] = stepPwmForceController( ...
        controllerState, targetForce, feedbackForce, feedbackLegSpeed, controller);
    [xNext, plantState, plantOutput, previousTrueAcceleration] = advanceCoupledPlant( ...
        x, plantState, pwmCommand, model, teacher);
    previousFeedbackForce = plantOutput.force;

    result.qTrue(:, sampleIndex) = qTrue;
    result.qFeedback(:, sampleIndex) = feedbackPose;
    result.qdTrue(:, sampleIndex) = qdTrue;
    result.qdFeedback(:, sampleIndex) = feedbackVelocity;
    result.targetForce(:, sampleIndex) = targetForce;
    result.trueForce(:, sampleIndex) = plantOutput.force;
    result.estimatedForce(:, sampleIndex) = estimatedForce;
    if strcmp(mode, 'identified')
        result.rawEstimatedForce(:, sampleIndex) = rawEstimatedForce;
        result.dynamicsEstimatedForce(:, sampleIndex) = dynamicsEstimatedForce;
    else
        result.rawEstimatedForce(:, sampleIndex) = estimatedForce;
        result.dynamicsEstimatedForce(:, sampleIndex) = estimatedForce;
    end
    result.pwm(:, sampleIndex) = pwmCommand;
    result.legSpeed(:, sampleIndex) = feedbackLegSpeed;
    result.trueLegSpeed(:, sampleIndex) = trueLegSpeed;

    if sampleIndex < sampleCount
        x = xNext;
    end
end
end

function [controllerState, plantState, initialPwm, initialForce] = initializeStaticLoad( ...
        controllerState, plantState, targetForce, legSpeed, controller, teacher)
% 轨迹开始前先建立静态承载力，避免把上电瞬态误计为轨迹控制性能。
startupController = controller;
startupController.pwmRateLimit = inf;
[controllerState, initialPwm] = stepPwmForceController( ...
    controllerState, targetForce, targetForce, legSpeed, startupController);

forceGain = teacher.torqueConstant .* teacher.gearRatio .* ...
    (2 * pi ./ teacher.screwLead) .* teacher.efficiency;
friction = sign(targetForce) .* teacher.staticFriction + teacher.viscousFriction .* legSpeed;
plantState.current = (targetForce + friction) ./ forceGain;
[plantState, output] = stepHighFidelityPwmActuator(plantState, initialPwm, legSpeed, teacher);
initialForce = output.force;
end

function result = initializeResult(mode, reference, teacher, sampleCount, options)
result = struct();
result.mode = mode;
result.plantType = teacher.type;
result.t = reference.t;
result.qReference = reference.q;
result.qTrue = zeros(6, sampleCount);
result.qFeedback = zeros(6, sampleCount);
result.qdTrue = zeros(6, sampleCount);
result.qdFeedback = zeros(6, sampleCount);
result.targetForce = zeros(6, sampleCount);
result.trueForce = zeros(6, sampleCount);
result.estimatedForce = zeros(6, sampleCount);
result.rawEstimatedForce = zeros(6, sampleCount);
result.dynamicsEstimatedForce = zeros(6, sampleCount);
result.pwm = zeros(6, sampleCount);
result.legSpeed = zeros(6, sampleCount);
result.trueLegSpeed = zeros(6, sampleCount);
result.observability = struct( ...
    'usedTrueForceFeedback', strcmp(mode, 'oracle'), ...
    'usedTruePoseFeedback', strcmp(mode, 'oracle'), ...
    'usedEncoderImuFusion', strcmp(mode, 'identified'), ...
    'usedUkfPoseFusion', strcmp(mode, 'identified') && strcmp(options.poseEstimatorMode, "ukf"), ...
    'usedRelativeEncoder', strcmp(mode, 'identified'), ...
    'usedImuAcceleration', strcmp(mode, 'identified') && strcmp(options.poseEstimatorMode, "ukf"), ...
    'usedImuAngularVelocity', strcmp(mode, 'identified') && strcmp(options.poseEstimatorMode, "ukf"), ...
    'usedPositionMeasurement', false, ...
    'usedHomeCalibration', strcmp(mode, 'identified') && options.homeCalibrationApplied, ...
    'usedUkfWarmup', strcmp(mode, 'identified') && strcmp(options.poseEstimatorMode, "ukf") && ...
        options.ukfWarmupDuration > 0, ...
    'usedEncoderDifferencedLegSpeed', strcmp(mode, 'identified') && ...
        ~options.useFusedPoseLegSpeed, ...
    'usedFusedPoseLegSpeed', strcmp(mode, 'identified') && options.useFusedPoseLegSpeed, ...
    'usedDynamicsForceObserver', strcmp(mode, 'identified') && ...
        options.dynamicsForceObserverWeight > 0, ...
    'usedTrueLegSpeedFeedback', strcmp(mode, 'oracle'), ...
    'usedFeedbackPoseJacobian', strcmp(mode, 'identified'));
end

function estimator = warmupPoseImuUkf(estimator, anchorPose, model, options)
sampleCount = round(options.ukfWarmupDuration / options.sampleTime);
for sampleIndex = 1:sampleCount
    orientation = anchorPose(4:6);
    worldAcceleration = zeros(3, 1);
    specificForce = rpy2rotmZYX(anchorPose(4:6)).' * (-model.g);
    angularVelocity = zeros(3, 1);
    relativeLength = zeros(6, 1);
    if options.sensorNoiseEnabled
        relativeLength = relativeLength + options.encoderNoiseStd * randn(6, 1);
        orientation = orientation + options.orientationCalibrationResidual + ...
            options.orientationNoiseStd .* randn(3, 1);
        worldAcceleration = worldAcceleration + options.accelerometerCalibrationResidual + ...
            options.accelerationNoiseStd * randn(3, 1);
        specificForce = specificForce + options.accelerometerCalibrationResidual + ...
            options.accelerationNoiseStd * randn(3, 1);
        angularVelocity = angularVelocity + options.gyroCalibrationResidual + ...
            options.angularVelocityNoiseStd * randn(3, 1);
    end
    sample = struct('relativeLength', quantizeSignal(relativeLength, options.encoderResolution), ...
        'orientation', orientation, 'angularVelocity', angularVelocity, ...
        'accelerationMode', options.accelerationMode);
    if options.accelerationMode == "world"
        sample.acceleration = worldAcceleration;
    else
        sample.acceleration = specificForce;
    end
    [estimator, ~] = stepPoseImuUkf(estimator, sample);
end
end

function [targetForce, poseIntegral] = addPoseFeedback(feedforwardForce, referencePose, ...
        referenceVelocity, feedbackPose, feedbackVelocity, poseIntegral, model, options)
poseError = referencePose - feedbackPose;
velocityError = referenceVelocity - feedbackVelocity;
poseIntegral = poseIntegral + options.sampleTime * poseError(1:3);
poseIntegral = min(max(poseIntegral, -options.translationIntegralLimit), ...
    options.translationIntegralLimit);
wrenchCorrection = [options.translationGain .* poseError(1:3); ...
    options.rotationGain .* poseError(4:6)] + ...
    [options.translationRateGain .* velocityError(1:3); ...
    options.rotationRateGain .* velocityError(4:6)] + ...
    [options.translationIntegralGain .* poseIntegral; zeros(3, 1)];
jacobian = sgpJacobian(feedbackPose, model);
forceCorrection = jacobian.Jv.' \ wrenchCorrection;
forceCorrection = min(max(forceCorrection, -options.forceCorrectionLimit), ...
    options.forceCorrectionLimit);
targetForce = feedforwardForce + forceCorrection;
end

function pose = estimatePoseFromEncoderImu(lengthMeasurement, orientationMeasurement, ...
        previousPose, model, options)
pose = [previousPose(1:3); orientationMeasurement(:)];
for iteration = 1:options.translationIterations
    kin = sgpIK(pose, model);
    translationJacobian = kin.u.';
    translationDelta = translationJacobian \ (lengthMeasurement - kin.L);
    pose(1:3) = pose(1:3) + translationDelta;
end
end

function quantized = quantizeSignal(signal, resolution)
quantized = round(signal / resolution) * resolution;
end

function [state, actuatorState, output, acceleration] = advanceCoupledPlant( ...
        state, actuatorState, pwmCommand, model, teacher)
substepCount = teacher.integrationSubsteps;
substepTime = teacher.sampleTime / substepCount;
substepTeacher = teacher;
substepTeacher.sampleTime = substepTime;
for substepIndex = 1:substepCount
    jacobian = sgpJacobian(state(1:6), model);
    legSpeed = jacobian.Jq * state(7:12);
    [actuatorState, output] = stepHighFidelityPwmActuator( ...
        actuatorState, pwmCommand, legSpeed, substepTeacher);
    [~, auxiliary] = stateDynamicsCompositeRigidBody(state, output.force, model);
    acceleration = auxiliary.qdd;
    nextVelocity = state(7:12) + substepTime * auxiliary.qdd;
    state = [state(1:6) + substepTime * nextVelocity; nextVelocity];
end
if any(~isfinite(state))
    error('comparePwmPoseForceControlLines:NonfiniteState', '平台动力学状态出现非有限值。');
end
end

function metrics = buildMetrics(comparison, teacher)
forceSpan = 2 * max(teacher.forceLimit);
identifiedError = comparison.identified.trueForce - comparison.identified.targetForce;
oracleError = comparison.oracle.trueForce - comparison.oracle.targetForce;
sameSampleEstimateError = comparison.identified.estimatedForce - comparison.identified.trueForce;
alignedEstimateError = comparison.identified.estimatedForce(:, 2:end) - ...
    comparison.identified.trueForce(:, 1:end - 1);
metrics = struct();
metrics.identifiedForceTrackingNrmse = sqrt(mean(identifiedError.^2, 'all')) / forceSpan;
metrics.oracleForceTrackingNrmse = sqrt(mean(oracleError.^2, 'all')) / forceSpan;
metrics.identifiedForceEstimateAlignmentSamples = 1;
metrics.identifiedSameSampleForceEstimateRms = sqrt(mean(sameSampleEstimateError.^2, 'all'));
metrics.identifiedAlignedForceEstimateRms = sqrt(mean(alignedEstimateError.^2, 'all'));
metrics.identifiedAlignedForceEstimateNrmse = ...
    metrics.identifiedAlignedForceEstimateRms / forceSpan;
metrics.identifiedAlignedForceEstimateBias = max(abs(mean(alignedEstimateError, 2)));
metrics.identifiedPoseTranslationRms = sqrt(mean( ...
    (comparison.identified.qTrue(1:3, :) - comparison.identified.qReference(1:3, :)).^2, 'all'));
metrics.oraclePoseTranslationRms = sqrt(mean( ...
    (comparison.oracle.qTrue(1:3, :) - comparison.oracle.qReference(1:3, :)).^2, 'all'));
end
