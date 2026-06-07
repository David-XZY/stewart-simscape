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
end
