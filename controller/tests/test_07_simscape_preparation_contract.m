function test_07_simscape_preparation_contract
% test_07_simscape_preparation_contract - 验证力输入位姿跟踪统一准备接口
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

assert(setup.design.bandwidthHz == 15);
assert(setup.design.gainScale == 0.45);
assert(setup.design.rotationGainScale == 1.4);
assert(setup.config.gravityEnabled);
assert(setup.controller.type == 6);
assert(strcmp(setup.trajectoryFile, sampleFile));
assert(bdIsLoaded(setup.modelName));
assert(evalin('base', 'exist(''K'', ''var'') == 1'));
assert(evalin('base', 'controller.type == 6'));

invalidFile = [tempname, '.mat'];
invalidCleanup = onCleanup(@() deleteIfExists(invalidFile));
refs = rmfield(setup.refs, 'Fleg');
save(invalidFile, 'refs');
assertThrows(@() prepareSimscapePoseForceControl(invalidFile), ...
    'prepareSimscapePoseForceControl:InvalidReferences');
end

function assertThrows(action, expectedIdentifier)
try
    action();
catch exception
    assert(strcmp(exception.identifier, expectedIdentifier));
    return;
end
error('test_07_simscape_preparation_contract:ExpectedError', ...
    '预期抛出错误 %s。', expectedIdentifier);
end

function deleteIfExists(fileName)
if isfile(fileName)
    delete(fileName);
end
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
