function test_18_pose_force_preparation_contract
% test_18_pose_force_preparation_contract - 验证位姿力控制准备与整定接口
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

sampleFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
setup = prepareSimscapePoseForceControl(sampleFile, struct('bandwidthHz', 8));
cleanup = onCleanup(@() closePreparedModel(setup.modelName));

assert(setup.controller.type == 6);
assert(setup.config.gravityEnabled);
assert(setup.design.bandwidthHz == 8);
assert(setup.design.stable);
assert(isequal(size(setup.design.plant), [6, 6]));
assert(isequal(size(setup.design.cartesianPlant), [6, 6]));
assert(isequal(size(setup.design.K), [6, 6]));
assert(strcmp(setup.refs.referenceInterpolation, 'cubic-hermite'));
assert(max(abs(diff(setup.refs.t) - setup.config.derivativeSampleTime)) < 1e-12);
assert(evalin('base', 'exist(''K'', ''var'') == 1'));
assert(evalin('base', 'controller.type == 6'));
assert(evalin('base', 'isa(references.uFF, ''timeseries'')'));
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
