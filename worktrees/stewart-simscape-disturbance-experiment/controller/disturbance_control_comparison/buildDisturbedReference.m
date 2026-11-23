function reference = buildDisturbedReference(nominal, experimentCase, model, poseConfig, config)
% buildDisturbedReference - Install bump/noise without changing source data.
arguments
    nominal struct
    experimentCase struct
    model struct
    poseConfig struct
    config struct
end

reference = nominal;
time = nominal.time(:).';
sampleCount = numel(time);
reference.targetPerturbation = zeros(6, sampleCount);
reference.commandNoisePerturbation = zeros(6, sampleCount);
reference.feedforwardPolicy = "nominal";

if experimentCase.hasSmoothBump
    [shape, shapeRate, shapeAcceleration] = smoothBump(time, ...
        config.bumpWindow, config.bumpRiseTime);
    amplitude = experimentCase.scale*deg2rad(config.baseAttitudeBumpDeg(:));
    reference.targetPerturbation(4:6, :) = amplitude.*shape;
    reference.q(4:6, :) = nominal.q(4:6, :)+amplitude.*shape;
    reference.qd(4:6, :) = nominal.qd(4:6, :)+amplitude.*shapeRate;
    reference.qdd(4:6, :) = nominal.qdd(4:6, :)+amplitude.*shapeAcceleration;
    reference.feedforwardPolicy = "recomputed_for_smooth_target";
    reference = recomputeKinematicsAndFeedforward(reference, model, poseConfig);
end

if experimentCase.hasTargetNoise
    noise = generateBandLimitedAttitudeNoise(time, ...
        experimentCase.scale*deg2rad(config.baseAttitudeNoiseStdDeg), ...
        config.noiseCutoffHz, experimentCase.noiseSeed, config.noiseWindow);
    reference.targetPerturbation(4:6, :) = ...
        reference.targetPerturbation(4:6, :)+noise;
    reference.commandNoisePerturbation(4:6, :) = noise;
    reference.q(4:6, :) = reference.q(4:6, :)+noise;
    % Command noise deliberately affects only q.  qd/qdd and feedforward
    % remain those of the smooth or nominal trajectory assembled above.
    reference.feedforwardPolicy = reference.feedforwardPolicy+ ...
        "+pose_only_bandlimited_noise";
end
end

function [value, rate, acceleration] = smoothBump(time, window, riseTime)
t0 = window(1);
t1 = window(2);
value = zeros(1, numel(time));
rate = value;
acceleration = value;

rise = time >= t0 & time < t0+riseTime;
phase = (time(rise)-t0)/riseTime;
value(rise) = 0.5-0.5*cos(pi*phase);
rate(rise) = 0.5*pi/riseTime*sin(pi*phase);
acceleration(rise) = 0.5*(pi/riseTime)^2*cos(pi*phase);

holdRegion = time >= t0+riseTime & time <= t1-riseTime;
value(holdRegion) = 1;

fall = time > t1-riseTime & time <= t1;
phase = (time(fall)-(t1-riseTime))/riseTime;
value(fall) = 0.5+0.5*cos(pi*phase);
rate(fall) = -0.5*pi/riseTime*sin(pi*phase);
acceleration(fall) = -0.5*(pi/riseTime)^2*cos(pi*phase);
end

function reference = recomputeKinematicsAndFeedforward(reference, model, poseConfig)
sampleCount = numel(reference.time);
reference.legLength = zeros(6, sampleCount);
reference.legSpeed = zeros(6, sampleCount);
reference.computedForce = zeros(6, sampleCount);
for index = 1:sampleCount
    inverseKinematics = sgpIK(reference.q(:, index), model);
    jacobian = sgpJacobian(reference.q(:, index), model);
    reference.legLength(:, index) = inverseKinematics.L;
    reference.legSpeed(:, index) = jacobian.Jq*reference.qd(:, index);
    reference.computedForce(:, index) = computeComputedTorqueForce( ...
        reference.q(:, index), reference.qd(:, index), reference.qdd(:, index), ...
        reference.q(:, index), reference.qd(:, index), model, poseConfig);
end
end
