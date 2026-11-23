function test_40_nonideal_force_actuator_contract
% test_40_nonideal_force_actuator_contract - 验证非理想力执行器 Simscape Variant
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
actuator = makeNonidealForceActuator(model);
assert(strcmp(actuator.type, 'nonideal-force-actuator'));
assert(isequal(size(actuator.forceLimit), [6, 1]));
assert(isequal(size(actuator.forceRateLimit), [6, 1]));
assert(isequal(size(actuator.initialForce), [6, 1]));
assert(all(actuator.forceLimit > 0));
assert(all(actuator.forceRateLimit > 0));
assert(actuator.timeConstant > 0);
assert(actuator.inputDelay >= 0);

strutFile = fullfile(projectRoot, 'matlab', 'simscape_subsystems', 'stewart_strut.slx');
load_system(strutFile);
cleanup = onCleanup(@() close_system('stewart_strut', 0));
variants = find_system('stewart_strut/Actuator', ...
    'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'SubSystem');
controls = string(cellfun(@(block) get_param(block, 'VariantControl'), ...
    variants, 'UniformOutput', false));
assert(any(controls == "stewart.actuators.type==7"), ...
    '腿执行器缺少 stewart.actuators.type==7 非理想力分支。');

nonidealVariant = variants{find(controls == "stewart.actuators.type==7", 1)};
requiredBlocks = {'Input Delay', 'Force Dead Zone', 'Force Rate Limit', ...
    'Force Lag', 'Force Saturation', 'Prismatic Joint'};
for index = 1:numel(requiredBlocks)
    assert(getSimulinkBlockHandle([nonidealVariant, '/', requiredBlocks{index}]) ~= -1, ...
        '非理想力分支缺少模块：%s。', requiredBlocks{index});
end

assert(strcmp(get_param([nonidealVariant, '/Gain'], 'Gain'), '1'), ...
    '非理想力分支输入方向必须与理想力源一致。');
assert(strcmp(get_param([nonidealVariant, '/Input Delay'], 'InitialOutput'), ...
    'stewart.nonidealForceActuator.initialForce(i)'));
assert(strcmp(get_param([nonidealVariant, '/Force Rate Limit'], 'InitialCondition'), ...
    'stewart.nonidealForceActuator.initialForce(i)'));
assert(strcmp(get_param([nonidealVariant, '/Force Lag'], 'X0'), ...
    'stewart.nonidealForceActuator.initialForce(i)'));
end
