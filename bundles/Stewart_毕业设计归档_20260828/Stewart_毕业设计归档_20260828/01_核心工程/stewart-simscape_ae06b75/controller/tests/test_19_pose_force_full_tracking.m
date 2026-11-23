function test_19_pose_force_full_tracking
% test_19_pose_force_full_tracking - 验证力输入位姿控制完整轨迹硬验收
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'matlab', 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

sampleFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
setup = prepareSimscapePoseForceControl(sampleFile);
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
simulationOutput = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapePoseForceControl( ...
    simulationOutput.get('simout'), setup.refs, setup.model, setup.design, setup.config);

assert(report.passed);
assert(report.acceptance.forcePassed);
assert(report.acceptance.lengthPassed);
assert(report.acceptance.speedPassed);
assert(report.acceptance.accelerationPassed);
assert(report.acceptance.trackingPassed);
assert(report.metrics.maxAbsControlForce <= setup.config.forceLimit + 1e-6);
assert(report.metrics.maxAbsActualForce <= setup.config.forceLimit + 250);
assert(isfinite(report.metrics.targetActualForceRms));
assert(isfinite(report.metrics.targetActualForcePeak));
assert(report.metrics.targetActualForcePeak <= 2500);
assert(report.thresholds.maxLengthTrackingPeak == setup.config.nonidealMaxLengthTrackingPeak);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
