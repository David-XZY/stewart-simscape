function [state, estimatedForce, diagnostics] = stepGrayNarxForceEstimator( ...
    state, pwm, legSpeed, identified)
% stepGrayNarxForceEstimator - 仅使用 PWM 与编码器运动状态递推估计支链力
legSpeed = legSpeed(:);
acceleration = (legSpeed - state.previousSpeed) / identified.sampleTime;
[state.gray, grayOutput] = stepHighFidelityPwmActuator( ...
    state.gray, pwm, legSpeed, identified.gray);
grayForce = grayOutput.force;
residual = zeros(identified.axisCount, 1);

for axisIndex = 1:identified.axisCount
    feature = grayNarxFeatureVector(pwm(axisIndex), legSpeed(axisIndex), ...
        acceleration(axisIndex), grayForce(axisIndex), state.previousPwm(axisIndex), ...
        state.previousSpeed(axisIndex), state.previousGrayForce(axisIndex), ...
        state.previousResidual(axisIndex), state.previousResidual2(axisIndex));
    narx = identified.narx(axisIndex);
    indices = featureIndices(narx);
    residual(axisIndex) = (feature(indices) ./ narx.featureScale) * narx.coefficients;
end
estimatedForce = min(max(grayForce + residual, -2400), 2400);
state.previousResidual2 = state.previousResidual;
state.previousResidual = residual;
state.previousSpeed = legSpeed;
state.previousPwm = pwm;
state.previousGrayForce = grayForce;
diagnostics = struct('grayForce', grayForce, 'residualForce', residual);
end

function indices = featureIndices(model)
if isfield(model, 'featureIndices')
    indices = model.featureIndices;
else
    indices = 1:11;
end
end
