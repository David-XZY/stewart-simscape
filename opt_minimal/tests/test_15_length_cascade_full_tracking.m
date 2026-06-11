function test_15_length_cascade_full_tracking
% test_15_length_cascade_full_tracking - 验证标准全轨迹纯长度硬验收
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'integration'));
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');
refs = rmfield(sample.refs, {'L', 'Fleg'});
trajectoryFile = [tempname, '.mat'];
fileCleanup = onCleanup(@() deleteIfExists(trajectoryFile));
save(trajectoryFile, 'refs');
setup = prepareSimscapeLengthCascadeControl(trajectoryFile);
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
output = sim(setup.modelName, ...
    'StopTime', num2str(setup.refs.t(end), 16), ...
    'ReturnWorkspaceOutputs', 'on');
report = evaluateSimscapeLengthCascadeControl(output.get('simout'), ...
    setup.refs, setup.model, setup.design, setup.config);
assert(report.passed, '标准全轨迹纯长度控制未通过硬验收。');
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
