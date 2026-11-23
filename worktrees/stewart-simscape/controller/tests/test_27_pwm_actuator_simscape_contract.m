function test_27_pwm_actuator_simscape_contract
% test_27_pwm_actuator_simscape_contract - 验证 PWM 物理执行器 Simscape Variant
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');

strutFile = fullfile(projectRoot, 'matlab', 'simscape_subsystems', 'stewart_strut.slx');
load_system(strutFile);
cleanup = onCleanup(@() close_system('stewart_strut', 0));
variants = find_system('stewart_strut/Actuator', ...
    'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'SubSystem');
controls = string(cellfun(@(block) get_param(block, 'VariantControl'), ...
    variants, 'UniformOutput', false));
assert(any(controls == "stewart.actuators.type==6"), ...
    '腿执行器缺少 stewart.actuators.type==6 PWM 物理分支。');

pwmVariant = variants{find(controls == "stewart.actuators.type==6", 1)};
requiredBlocks = {'PWM Dead Zone', 'PWM Voltage', 'Electrical Dynamics', ...
    'Force Saturation', 'Prismatic Joint'};
for index = 1:numel(requiredBlocks)
    assert(getSimulinkBlockHandle([pwmVariant, '/', requiredBlocks{index}]) ~= -1, ...
        'PWM 物理分支缺少模块：%s。', requiredBlocks{index});
end
end
