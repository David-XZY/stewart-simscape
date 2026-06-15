function report = evaluateGrayNarxForceIdentifier(identified, dataset)
% evaluateGrayNarxForceIdentifier - 自由运行评估灰箱与残差 NARX 力估计
grayForce = simulateGrayForceLocal(identified.gray, dataset.pwm, dataset.legSpeed);
estimatedForce = grayForce;
predictedResidual = zeros(size(grayForce));

for axisIndex = 1:identified.axisCount
    model = identified.narx(axisIndex);
    for sampleIndex = 2:size(grayForce, 1)
        feature = residualFeatureAt(dataset, grayForce, predictedResidual, sampleIndex, axisIndex);
        indices = featureIndices(model);
        predictedResidual(sampleIndex, axisIndex) = ...
            (feature(indices) ./ model.featureScale) * model.coefficients;
    end
end
estimatedForce = min(max(estimatedForce + predictedResidual, -2400), 2400);

testMask = dataset.split.test;
grayTestNrmse = nrmse(grayForce(testMask, :), dataset.trueForce(testMask, :));
narxTestNrmse = nrmse(estimatedForce(testMask, :), dataset.trueForce(testMask, :));
testBias = max(abs(mean(estimatedForce(testMask, :) - dataset.trueForce(testMask, :), 1))) / 4800;
freeRunStable = all(isfinite(estimatedForce), 'all') && max(abs(estimatedForce), [], 'all') <= 2400;

report = struct();
report.grayForce = grayForce;
report.estimatedForce = estimatedForce;
report.predictedResidual = predictedResidual;
report.metrics = struct('grayTestNrmse', grayTestNrmse, ...
    'narxTestNrmse', narxTestNrmse, 'testBias', testBias);
report.acceptance = struct('nrmsePassed', narxTestNrmse <= 0.05, ...
    'biasPassed', testBias <= 0.02, 'freeRunStable', freeRunStable);
report.passed = report.acceptance.nrmsePassed && report.acceptance.biasPassed && freeRunStable;
end

function force = simulateGrayForceLocal(gray, pwm, speed)
state = initializeHighFidelityPwmActuatorState(gray);
force = zeros(size(pwm));
for index = 1:size(pwm, 1)
    [state, output] = stepHighFidelityPwmActuator( ...
        state, pwm(index, :).', speed(index, :).', gray);
    force(index, :) = output.force.';
end
end

function feature = residualFeatureAt(dataset, grayForce, residual, index, axis)
previousIndex = max(index - 1, 1);
previous2Index = max(index - 2, 1);
feature = grayNarxFeatureVector(dataset.pwm(index, axis), dataset.legSpeed(index, axis), ...
    dataset.legAcceleration(index, axis), grayForce(index, axis), ...
    dataset.pwm(previousIndex, axis), dataset.legSpeed(previousIndex, axis), ...
    grayForce(previousIndex, axis), residual(previousIndex, axis), ...
    residual(previous2Index, axis));
end

function indices = featureIndices(model)
if isfield(model, 'featureIndices')
    indices = model.featureIndices;
else
    indices = 1:11;
end
end

function value = nrmse(estimate, truth)
value = sqrt(mean((estimate - truth).^2, 'all')) / 4800;
end
