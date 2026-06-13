function test_11_length_cascade_preparation_contract
% test_11_length_cascade_preparation_contract - 验证长度模式仅依赖 q/qd
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

sampleFile = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat');
sample = load(sampleFile, 'refs');
refs = rmfield(sample.refs, {'L', 'Fleg'});
trajectoryFile = [tempname, '.mat'];
fileCleanup = onCleanup(@() deleteIfExists(trajectoryFile));
save(trajectoryFile, 'refs');

setup = prepareSimscapeLengthCascadeControl(trajectoryFile);
modelCleanup = onCleanup(@() closePreparedModel(setup.modelName));

assert(setup.controller.type == 8);
assert(setup.simscapeData.stewart.actuators.type == 5);
assert(~setup.config.gravityEnabled);
assert(~isfield(setup.references, 'uFF'));
assert(isfield(setup.references, 'rL'));
assert(isfield(setup.references, 'rLd'));
assert(strcmp(setup.refs.referenceInterpolation, 'cubic-hermite'));
assert(max(abs(diff(setup.refs.t) - setup.config.sampleTime)) < 1e-12);
assert(evalin('base', 'exist(''lengthCascadeConfig'', ''var'') == 1'));
assert(evalin('base', 'exist(''lengthCascadeDesign'', ''var'') == 1'));
assert(evalin('base', 'exist(''lengthServoConfig'', ''var'') == 1'));
assert(evalin('base', 'exist(''references'', ''var'') == 1'));
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
