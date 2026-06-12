function test_19_pose_force_full_tracking
% test_19_pose_force_full_tracking - 验证力输入位姿控制完整轨迹硬验收
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

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
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
