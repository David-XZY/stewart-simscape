function test_06_simscape_length_control_smoke
% test_06_simscape_length_control_smoke - 短时验证力输入位姿跟踪不会起步坠落
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

setup = prepareSimscapePoseForceControl("");
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
output = sim(setup.modelName, 'StopTime', '0.02', 'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapePoseForceControl( ...
    output.get('simout'), setup.refs, setup.model, setup.design, setup.config);

assert(setup.controller.type == 6);
assert(report.acceptance.finitePassed);
assert(report.acceptance.forcePassed);
assert(report.metrics.minAbsoluteLength > min(setup.refs.L(:, 1)) - 1e-3);
assert(report.metrics.maxAbsControlForce > 100);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
