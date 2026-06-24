function test_18_pose_force_preparation_contract
% test_18_pose_force_preparation_contract - 验证位姿力控制准备与整定接口
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
setup = prepareSimscapePoseForceControl(sampleFile, struct('bandwidthHz', 8));
cleanup = onCleanup(@() closePreparedModel(setup.modelName));

assert(setup.controller.type == 10);
assert(setup.simscapeData.stewart.actuators.type == 7);
assert(setup.config.gravityEnabled);
assert(setup.config.controlLaw == "computed-torque");
assert(setup.config.feedbackLaw == "lqi-output");
assert(setup.config.actuatorMode == "nonideal-force");
assert(setup.design.bandwidthHz == 8);
assert(setup.design.feedbackLaw == "lqi-output");
assert(setup.design.integratorCount == 6);
assert(setup.design.stable);
assert(isequal(size(setup.design.plant), [6, 6]));
assert(isequal(size(setup.design.cartesianPlant), [6, 6]));
assert(isequal(size(setup.design.K), [6, 12]));
assert(isa(setup.design.K, 'ss'));
assert(setup.design.feedbackInputCount == 12);
assert(strcmp(setup.refs.referenceInterpolation, 'cubic-hermite'));
assert(isequal(size(setup.refs.qdd), size(setup.refs.q)));
assert(max(abs(diff(setup.refs.t) - setup.config.derivativeSampleTime)) < 1e-12);
assert(evalin('base', 'exist(''K'', ''var'') == 1'));
assert(evalin('base', 'controller.type == 10'));
assert(evalin('base', 'isa(references.uFF, ''timeseries'')'));
assert(evalin('base', 'isa(references.rd, ''timeseries'')'));
assert(evalin('base', 'isa(references.rdd, ''timeseries'')'));
assert(evalin('base', 'exist(''computedTorqueConfig'', ''var'') == 1'));
assert(evalin('base', 'exist(''computedTorqueModel'', ''var'') == 1'));

legacySetup = prepareSimscapePoseForceControl(sampleFile, struct( ...
    'controlLaw', "linear-pose-force", 'actuatorMode', "ideal-force", ...
    'bandwidthHz', 8));
assert(legacySetup.controller.type == 6);
assert(legacySetup.simscapeData.stewart.actuators.type ~= 7);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
