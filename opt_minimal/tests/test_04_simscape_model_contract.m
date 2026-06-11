function test_04_simscape_model_contract
% test_04_simscape_model_contract - 验证现有模型继续读取 references.r 和 references.rL
optRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(optRoot);
modelFile = fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx');
assert(isfile(modelFile), '未找到 stewart_platform_model.slx。');

load_system(modelFile);
cleanup = onCleanup(@() close_system('stewart_platform_model', 0));
blocks = find_system('stewart_platform_model', 'MatchFilter', @Simulink.match.allVariants, ...
    'BlockType', 'FromWorkspace');
variables = cellfun(@(block) get_param(block, 'VariableName'), blocks, 'UniformOutput', false);
assert(any(strcmp(variables, 'references.r')), '模型未读取 references.r。');
assert(any(strcmp(variables, 'references.rL')), '模型未读取 references.rL。');
assert(any(strcmp(variables, 'references.uFF')), '模型未读取 references.uFF。');
variantSource = find_system('stewart_platform_model', 'SearchDepth', 1, ...
    'BlockType', 'VariantSource');
assert(isscalar(variantSource), '顶层缺少执行器前馈 Variant Source。');

toWorkspace = find_system('stewart_platform_model', ...
    'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'ToWorkspace', ...
    'VariableName', 'simout');
assert(isscalar(toWorkspace), '模型必须只有一个 simout 输出。');
portHandles = get_param(toWorkspace{1}, 'PortHandles');
lineHandle = get_param(portHandles.Inport, 'Line');
sourceBlock = get_param(lineHandle, 'SrcBlockHandle');
inputSignals = string(get_param(sourceBlock, 'InputSignalNames'));
assert(any(inputSignals == "u"), 'simout 尚未记录控制器输出力 u。');
assert(any(inputSignals == "uFeedback"), 'simout 尚未记录长度反馈力 uFeedback。');
assert(any(inputSignals == "uFF"), 'simout 尚未记录前馈力 uFF。');
end
