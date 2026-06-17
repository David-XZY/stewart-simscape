function test_25_pwm_force_identification_contract
% test_25_pwm_force_identification_contract - 验证灰箱与残差 NARX 训练/自由运行接口
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));
addpath(fullfile(optRoot, 'core'));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct( ...
    'sampleCount', 1800, 'randomSeed', 25));
assert(isequal(size(dataset.pwm), [1800, 6]));
assert(isequal(size(dataset.encoderLength), [1800, 6]));
assert(isequal(size(dataset.trueLegSpeed), [1800, 6]));
assert(any(abs(dataset.legSpeed - dataset.trueLegSpeed) > 1e-9, 'all'), ...
    '辨识训练必须使用编码器差分滤波腿速，而非仿真真腿速。');
assert(isequal(size(dataset.measuredPose), [1800, 6]));
assert(isequal(size(dataset.trueForce), [1800, 6]));
assert(all(dataset.split.train | dataset.split.validation | dataset.split.test));
assert(~any(dataset.split.train & dataset.split.test));

identified = trainGrayNarxForceIdentifier(dataset, teacher);
assert(strcmp(identified.type, 'gray-box-residual-narx'));
assert(identified.axisCount == 6);
assert(~isfield(identified, 'trueForce'), '部署模型不得携带教师真实力序列。');
assert(identified.training.grayParameterFit.usedMeasuredData);
assert(identified.training.grayParameterFit.objectiveAfter < ...
    identified.training.grayParameterFit.objectiveBefore);
assert(identified.training.grayParameterFit.parameterCount >= 6);
assert(identified.training.featureVersion == 2);
assert(all(arrayfun(@(model) numel(model.featureIndices) <= 12, identified.narx)));
assert(all(arrayfun(@(model) model.candidateFeatureCount >= 18, identified.narx)));

report = evaluateGrayNarxForceIdentifier(identified, dataset);
assert(all(isfinite(report.estimatedForce), 'all'));
assert(report.metrics.grayTestNrmse > report.metrics.narxTestNrmse);
assert(report.metrics.narxTestNrmse <= 0.05);
assert(report.metrics.testBias <= 0.02);
assert(report.acceptance.freeRunStable);
assert(report.passed);
end
