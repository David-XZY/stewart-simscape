function identified = trainGrayNarxForceIdentifier(dataset, teacher)
% trainGrayNarxForceIdentifier - 训练每轴灰箱基础模型与残差多项式 NARX
initialGray = makeGrayBoxFromTeacher(teacher);
[gray, grayParameterFit] = fitGrayBoxFromData(initialGray, dataset);
grayForce = simulateGrayForce(gray, dataset.pwm, dataset.legSpeed);
residual = dataset.trueForce - grayForce;
trainMask = dataset.split.train;

axisModels = repmat(struct('coefficients', [], 'ridge', 1e-4, ...
    'featureIndices', [], 'featureScale', [], 'candidateFeatureCount', 0, ...
    'selectionCriterion', "forward-aic-mpo-diagnostic-with-robust-baseline-deployment", ...
    'sparseCandidateFeatureIndices', [], 'sparseCandidateValidationRms', NaN, ...
    'baselineValidationRms', NaN), 6, 1);
for axisIndex = 1:6
    features = buildResidualFeatures(dataset, grayForce, residual, axisIndex);
    axisModels(axisIndex) = selectSparseModel( ...
        features, residual(:, axisIndex), trainMask, dataset.split.validation, ...
        axisModels(axisIndex));
end

identified = struct();
identified.type = 'gray-box-residual-narx';
identified.axisCount = 6;
identified.sampleTime = dataset.sampleTime;
identified.gray = gray;
identified.narx = axisModels;
identified.training = struct('featureVersion', 2, 'usesTeacherForceOnlyDuringTraining', true, ...
    'grayParameterFit', grayParameterFit);
end

function gray = makeGrayBoxFromTeacher(teacher)
gray = teacher;
gray.type = 'gray-box-pwm-force-model';
gray.resistance = 1.025 * teacher.resistance;
gray.torqueConstant = 0.985 * teacher.torqueConstant;
gray.efficiency = 0.985 * teacher.efficiency;
gray.coulombFriction = 0.92 * teacher.coulombFriction;
gray.staticFriction = gray.coulombFriction;
gray.viscousFriction = 0.90 * teacher.viscousFriction;
gray.deadzonePwm = round(0.97 * teacher.deadzonePwm);
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

function features = buildResidualFeatures(dataset, grayForce, residual, axisIndex)
previousPwm = [dataset.pwm(1, axisIndex); dataset.pwm(1:end - 1, axisIndex)];
previousSpeed = [dataset.legSpeed(1, axisIndex); dataset.legSpeed(1:end - 1, axisIndex)];
previousGray = [grayForce(1, axisIndex); grayForce(1:end - 1, axisIndex)];
previousResidual = [0; residual(1:end - 1, axisIndex)];
previousResidual2 = [0; 0; residual(1:end - 2, axisIndex)];
features = grayNarxFeatureVector(dataset.pwm(:, axisIndex), dataset.legSpeed(:, axisIndex), ...
    dataset.legAcceleration(:, axisIndex), grayForce(:, axisIndex), ...
    previousPwm, previousSpeed, previousGray, previousResidual, previousResidual2);
end

function model = selectSparseModel(features, target, trainMask, validationMask, model)
validTrain = trainMask & all(isfinite(features), 2) & isfinite(target);
scale = std(features(validTrain, :), 0, 1);
scale(scale < 1e-6) = 1;
normalized = features ./ scale;
maxTerms = min(12, size(features, 2));
selected = 1;
remaining = 2:size(features, 2);
bestAic = inf;
for termCount = 2:maxTerms
    candidateAic = inf(size(remaining));
    for candidateIndex = 1:numel(remaining)
        indices = [selected, remaining(candidateIndex)];
        coefficients = ridgeFit(normalized(validTrain, indices), target(validTrain), model.ridge);
        error = target(validTrain) - normalized(validTrain, indices) * coefficients;
        candidateAic(candidateIndex) = numel(error) * log(mean(error.^2) + eps) + ...
            2 * numel(indices);
    end
    [nextAic, bestIndex] = min(candidateAic);
    if nextAic >= bestAic
        break;
    end
    bestAic = nextAic;
    selected(end + 1) = remaining(bestIndex); %#ok<AGROW>
    remaining(bestIndex) = [];
end

baseline = 1:11;
selectedModel = fitForIndices(normalized, target, validTrain, selected, scale, model);
baselineModel = fitForIndices(normalized, target, validTrain, baseline, scale, model);
selectedValidation = freeRunValidationError(features, target, validationMask, selectedModel);
baselineValidation = freeRunValidationError(features, target, validationMask, baselineModel);
model = baselineModel;
model.candidateFeatureCount = size(features, 2);
model.selectionCriterion = "forward-aic-mpo-diagnostic-with-robust-baseline-deployment";
model.sparseCandidateFeatureIndices = selectedModel.featureIndices;
model.sparseCandidateValidationRms = selectedValidation;
model.baselineValidationRms = baselineValidation;
end

function model = fitForIndices(normalized, target, validTrain, indices, scale, model)
model.featureIndices = indices;
model.featureScale = scale(indices);
model.coefficients = ridgeFit(normalized(validTrain, indices), target(validTrain), model.ridge);
end

function coefficients = ridgeFit(features, target, ridge)
coefficients = (features.' * features + ridge * eye(size(features, 2))) \ ...
    (features.' * target);
end

function value = freeRunValidationError(features, target, validationMask, model)
predicted = zeros(size(target));
for index = 2:numel(target)
    feature = features(index, :);
    feature(11) = predicted(index - 1) / 2400;
    if index > 2
        feature(20) = predicted(index - 2) / 2400;
    else
        feature(20) = 0;
    end
    predicted(index) = (feature(model.featureIndices) ./ model.featureScale) * model.coefficients;
end
error = predicted(validationMask) - target(validationMask);
value = sqrt(mean(error.^2));
end
