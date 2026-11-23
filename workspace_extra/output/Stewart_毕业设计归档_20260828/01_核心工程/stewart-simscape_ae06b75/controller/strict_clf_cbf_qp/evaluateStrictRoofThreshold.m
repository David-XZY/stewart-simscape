function threshold = evaluateStrictRoofThreshold(time, scene, config)
% evaluateStrictRoofThreshold - C2 roof-clearance schedule for two phases.
arguments
    time double {mustBeScalarOrEmpty, mustBeFinite, mustBeNonnegative}
    scene struct
    config struct
end

if isfield(scene.collision, 'stage1ConstraintDistance')
    initialValue = scene.collision.stage1ConstraintDistance;
else
    initialValue = config.collision.roofStage1Distance;
end
if isfield(scene.collision, 'finalGap')
    finalValue = scene.collision.finalGap;
else
    finalValue = config.collision.roofFinalDistance;
end
if isfield(scene.phase, 'durationApproach')
    approachDuration = scene.phase.durationApproach;
else
    approachDuration = config.collision.approachDuration;
end
if isfield(scene.phase, 'durationInsertion')
    insertionDuration = scene.phase.durationInsertion;
else
    insertionDuration = config.collision.insertionDuration;
end

if time <= approachDuration
    value = initialValue;
    rate = 0;
    acceleration = 0;
elseif time >= approachDuration + insertionDuration
    value = finalValue;
    rate = 0;
    acceleration = 0;
else
    tau = (time-approachDuration) / insertionDuration;
    smooth = 10*tau^3 - 15*tau^4 + 6*tau^5;
    smoothRate = (30*tau^2 - 60*tau^3 + 30*tau^4) / insertionDuration;
    smoothAcceleration = (60*tau - 180*tau^2 + 120*tau^3) / insertionDuration^2;
    difference = finalValue - initialValue;
    value = initialValue + difference*smooth;
    rate = difference*smoothRate;
    acceleration = difference*smoothAcceleration;
end

threshold = struct('value', value, 'rate', rate, ...
    'acceleration', acceleration, 'initialValue', initialValue, ...
    'finalValue', finalValue);
end
