function [gray, report] = fitGrayBoxFromData(initialGray, dataset)
% fitGrayBoxFromData - 使用可观测训练数据辨识灰箱中的可辨识参数组
trainMask = dataset.split.train;
lowerBound = [0.85; 0.80; 0.70; 0.70; 0.80; 0.60; 0.60; 0.80];
upperBound = [1.15; 1.20; 1.30; 1.50; 2.20; 1.50; 1.50; 1.20];
initialScale = ones(size(lowerBound));

objective = @(scale) weightedObjective(applyScale(initialGray, scale), dataset, trainMask);
objectiveBefore = objective(initialScale);
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
    'MaxIterations', 35, 'MaxFunctionEvaluations', 320, ...
    'OptimalityTolerance', 1e-6, 'StepTolerance', 1e-5);
[scale, objectiveAfter, exitFlag, output] = fmincon(objective, initialScale, ...
    [], [], [], [], lowerBound, upperBound, [], options);
gray = applyScale(initialGray, scale);

report = struct();
report.usedMeasuredData = true;
report.parameterNames = ["torqueConstant"; "backEmfConstant"; "inductance"; ...
    "coulombFriction"; "staticFriction"; "stribeckVelocity"; ...
    "viscousFriction"; "deadzonePwm"];
report.parameterCount = numel(scale);
report.scale = scale;
report.lowerBound = lowerBound;
report.upperBound = upperBound;
report.objectiveBefore = objectiveBefore;
report.objectiveAfter = objectiveAfter;
report.exitFlag = exitFlag;
report.iterations = output.iterations;
end

function value = weightedObjective(gray, dataset, trainMask)
force = simulateGrayForce(gray, dataset.pwm, dataset.legSpeed);
error = (force - dataset.trueForce) ./ max(gray.forceLimit);
lowSpeedWeight = 1 + 2 * exp(-(abs(dataset.legSpeed) / 0.03).^2);
selectedError = error(trainMask, :) .* sqrt(lowSpeedWeight(trainMask, :));
value = mean(selectedError.^2, 'all');
end

function gray = applyScale(initialGray, scale)
gray = initialGray;
gray.torqueConstant = scale(1) * initialGray.torqueConstant;
gray.backEmfConstant = scale(2) * initialGray.backEmfConstant;
gray.inductance = scale(3) * initialGray.inductance;
gray.coulombFriction = scale(4) * initialGray.coulombFriction;
gray.staticFriction = scale(5) * initialGray.staticFriction;
gray.stribeckVelocity = scale(6) * initialGray.stribeckVelocity;
gray.viscousFriction = scale(7) * initialGray.viscousFriction;
gray.deadzonePwm = scale(8) * initialGray.deadzonePwm;
end

function force = simulateGrayForce(gray, pwm, speed)
state = initializeHighFidelityPwmActuatorState(gray);
force = zeros(size(pwm));
for index = 1:size(pwm, 1)
    [state, output] = stepHighFidelityPwmActuator( ...
        state, pwm(index, :).', speed(index, :).', gray);
    force(index, :) = output.force.';
end
end
