function test_08_simscape_full_tracking
% test_08_simscape_full_tracking - 验证默认力输入位姿控制完整轨迹
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'matlab', 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

setup = prepareSimscapePoseForceControl("");
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
output = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapePoseForceControl( ...
    output.get('simout'), setup.refs, setup.model, setup.design, setup.config);

assert(report.passed);
assert(report.acceptance.lengthPassed);
assert(report.acceptance.forcePassed);
assert(report.acceptance.speedPassed);
assert(report.acceptance.accelerationPassed);
assert(report.acceptance.trackingPassed);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
