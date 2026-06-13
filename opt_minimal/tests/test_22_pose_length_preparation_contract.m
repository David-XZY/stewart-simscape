function test_22_pose_length_preparation_contract
% test_22_pose_length_preparation_contract - 验证 Run04 准备链路与 SLX Variant
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'integration'));

setup = prepareSimscapePoseLengthControl();
cleanup = onCleanup(@() closePreparedModel(setup.modelName));

assert(setup.controller.type == 9);
assert(setup.simscapeData.stewart.actuators.type == 5);
assert(~setup.config.gravityEnabled);
assert(strcmp(setup.refs.referenceInterpolation, 'cubic-hermite'));
assert(isfield(setup.references, 'rJq'));
assert(isequal(size(setup.references.rJq.Data), [6, 6, numel(setup.refs.t)]));
assert(evalin('base', 'exist(''poseLengthConfig'', ''var'') == 1'));
assert(evalin('base', 'exist(''poseLengthDesign'', ''var'') == 1'));
assert(evalin('base', 'exist(''lengthServoConfig'', ''var'') == 1'));

variant = 'stewart_platform_model/Controller/Pose-Length-Cascade';
assert(getSimulinkBlockHandle(variant) ~= -1);
assert(strcmp(get_param(variant, 'VariantControl'), 'controller.type == 9'));
assert(getSimulinkBlockHandle([variant, '/Pose Error Lowpass']) ~= -1);
assert(getSimulinkBlockHandle([variant, '/Reference Jacobian Product']) ~= -1);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
