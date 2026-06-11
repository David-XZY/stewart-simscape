function test_14_length_cascade_smoke
% test_14_length_cascade_smoke - 验证纯长度 Variant 可完成短时仿真
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'integration'));
setup = prepareSimscapeLengthCascadeControl();
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
output = sim(setup.modelName, 'StopTime', '0.2', 'ReturnWorkspaceOutputs', 'on');
simout = output.get('simout');
assert(all(isfinite(simout.y.dLm.Data), 'all'));
assert(all(isfinite(simout.uFeedback.Data), 'all'));
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
