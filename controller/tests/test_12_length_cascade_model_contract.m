function test_12_length_cascade_model_contract
% test_12_length_cascade_model_contract - 验证 SLX 纯长度串级 Variant
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');

mainFile = fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx');
load_system(mainFile);
mainCleanup = onCleanup(@() close_system('stewart_platform_model', 0));
controllerVariants = find_system('stewart_platform_model/Controller', ...
    'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'SubSystem');
controllerControls = string(cellfun(@(block) get_param(block, 'VariantControl'), ...
    controllerVariants, 'UniformOutput', false));
assert(any(controllerControls == "controller.type == 8"), ...
    '顶层控制器缺少 controller.type == 8 长度串级分支。');
assert(any(controllerControls == "controller.type == 9"), ...
    '顶层控制器缺少 controller.type == 9 位姿反馈长度串级分支。');

strutFile = fullfile(projectRoot, 'matlab', 'simscape_subsystems', 'stewart_strut.slx');
load_system(strutFile);
strutCleanup = onCleanup(@() close_system('stewart_strut', 0));
actuatorVariants = find_system('stewart_strut/Actuator', ...
    'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'SubSystem');
actuatorControls = string(cellfun(@(block) get_param(block, 'VariantControl'), ...
    actuatorVariants, 'UniformOutput', false));
assert(any(actuatorControls == "stewart.actuators.type==5"), ...
    '腿执行器缺少 stewart.actuators.type==5 长度伺服分支。');

lengthServo = actuatorVariants{find(actuatorControls == "stewart.actuators.type==5", 1)};
prismatic = find_system(lengthServo, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'MaskType', 'Prismatic Joint');
assert(isscalar(prismatic), '长度伺服分支必须包含一个 Prismatic Joint。');
assert(strcmp(get_param(prismatic{1}, 'MotionActuationMode'), 'InputMotion'));
assert(strcmp(get_param(prismatic{1}, 'TorqueActuationMode'), 'ComputedTorque'));
assert(strcmp(get_param(prismatic{1}, 'SenseVelocity'), 'on'));
end
