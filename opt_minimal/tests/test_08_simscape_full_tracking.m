function test_08_simscape_full_tracking
% test_08_simscape_full_tracking - 验证默认 10 Hz 完整轨迹满足跟踪验收
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

sampleFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
setup = prepareSimscapeLengthControl(sampleFile, 10, true);
cleanup = onCleanup(@() closePreparedModel(setup.modelName));

simulationOutput = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapeLengthControl(simulationOutput.get('simout'), ...
    setup.refs, setup.model, setup.design);

assert(report.acceptance.lengthTrackingPassed);
assert(report.acceptance.translationTrackingPassed);
assert(report.acceptance.rotationTrackingPassed);
assert(report.acceptance.lengthPassed);
assert(report.acceptance.forcePassed);
assert(report.passed);
assert(report.metrics.maxTranslationPeak <= 10e-3);
assert(report.metrics.maxRotationPeak <= deg2rad(1));
assert(report.metrics.maxLengthTrackingPeak <= 5e-3);
assert(report.metrics.maxAbsLegSpeed < 1);
assert(report.metrics.maxAbsLegAcceleration < 10);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
