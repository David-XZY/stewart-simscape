function test_38_pwm_ukf_outer_loop_tuning_contract
% test_38_pwm_ukf_outer_loop_tuning_contract - 验证 UKF 与外环联合整定配置契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
addpath(fullfile(optRoot, 'actuator_identification'));

options = makePwmUkfOuterLoopTunedOptions();
assert(options.encoderNoiseStd == 5e-4);
assert(options.accelerationNoiseStd == 9.80665e-3);
assert(options.angularVelocityNoiseStd == deg2rad(0.07));
assert(isfield(options, 'ukfAccelerationNoiseStd'));
assert(isfield(options, 'ukfAngularVelocityNoiseStd'));
assert(isfield(options, 'ukfInitialPositionStd'));
assert(isfield(options, 'ukfInitialTranslationVelocityStd'));
assert(options.ukfAccelerationNoiseStd == 0.02);
assert(options.forceCorrectionLimit == 750);
assert(all(options.translationGain > 0));
assert(all(options.translationRateGain > 0));
assert(all(options.translationIntegralGain >= 0));

teacher = makeHighFidelityPwmActuator();
dataset = generatePwmIdentificationDataset(teacher, struct('sampleCount', 900, 'randomSeed', 38));
identified = trainGrayNarxForceIdentifier(dataset, teacher);
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');
comparison = comparePwmPoseForceControlLines(sample.refs, teacher, identified, ...
    mergeStruct(options, struct('duration', 0.3)));
assert(all(isfinite(comparison.identified.qFeedback), 'all'));
assert(comparison.options.encoderNoiseStd == 5e-4);
assert(comparison.options.ukfAccelerationNoiseStd == options.ukfAccelerationNoiseStd);
end

function merged = mergeStruct(base, override)
merged = base;
fields = fieldnames(override);
for index = 1:numel(fields)
    merged.(fields{index}) = override.(fields{index});
end
end
