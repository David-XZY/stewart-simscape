function test_07_simscape_preparation_contract
% test_07_simscape_preparation_contract - 验证统一准备接口、轨迹校验和手动模式
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

assert(setup.design.bandwidthHz == 10);
assert(setup.useForceFeedforward);
assert(strcmp(setup.trajectoryFile, sampleFile));
assert(bdIsLoaded(setup.modelName));
assert(abs(str2double(get_param(setup.modelName, 'StopTime')) - setup.refs.t(end)) < 1e-12);
assert(evalin('base', 'exist(''stewart'', ''var'') == 1'));
assert(evalin('base', 'exist(''references'', ''var'') == 1'));
assert(evalin('base', 'exist(''controller'', ''var'') == 1'));
assert(evalin('base', 'exist(''Kl'', ''var'') == 1'));

relativeFile = fullfile('opt_minimal', 'examples', ...
    'ihsid_40x20_limited_memory', 'simscape_references.mat');
originalFolder = pwd;
folderCleanup = onCleanup(@() cd(originalFolder));
cd(optRoot);
relativeSetup = prepareSimscapeLengthControl(relativeFile, 10, true);
assert(strcmp(relativeSetup.trajectoryFile, sampleFile));
clear folderCleanup;

invalidFile = [tempname, '.mat'];
invalidCleanup = onCleanup(@() deleteIfExists(invalidFile));
refs = rmfield(setup.refs, 'Fleg');
save(invalidFile, 'refs');
assertThrows(@() prepareSimscapeLengthControl(invalidFile, 10, true), ...
    'prepareSimscapeLengthControl:InvalidReferences');

unpublishedFile = [tempname, '.mat'];
unpublishedCleanup = onCleanup(@() deleteIfExists(unpublishedFile));
refs = setup.refs;
publishPassed = false;
save(unpublishedFile, 'refs', 'publishPassed');
assertThrows(@() prepareSimscapeLengthControl(unpublishedFile, 10, true), ...
    'prepareSimscapeLengthControl:TrajectoryNotPublished');

end

function assertThrows(action, expectedIdentifier)
try
    action();
catch exception
    assert(strcmp(exception.identifier, expectedIdentifier), ...
        '期望错误 %s，实际为 %s。', expectedIdentifier, exception.identifier);
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
