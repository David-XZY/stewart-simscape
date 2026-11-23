function test_11_fatrop_native_hs_contract()
% test_11_fatrop_native_hs_contract - 验证 FATROP-native HS 结构合同
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);
[z0, initialGuess] = buildInitialGuessFatropNativeHS(model, scene, disc);

solverOptions = struct('solverBackend', 'fatrop', 'fatropStructure', 'manual', ...
    'skipSolver', true, 'maxIter', 3);
nlpData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, solverOptions);

assert(strcmp(nlpData.method, 'FATROP_NATIVE_HS'), 'method 应标记为 FATROP_NATIVE_HS。');
assert(strcmp(nlpData.fatropStructure, 'manual'), 'FATROP-native HS 必须使用 manual 结构。');
assert(all(nlpData.manualStructure.nx == 12), '每个 stage 的状态维数必须是 12。');
assert(all(nlpData.manualStructure.nu(1:end-1) == 18), '每个区间控制必须是 [Fleft; Fmid; Fright] 共 18 维。');
assert(nlpData.manualStructure.nu(end) == 1, '终端 stage 只允许 1 维固定 dummy 控制以兼容 FATROP 接口。');
assert(nlpData.manualStructure.numFreeCollisionCertificates == 0, '不应保留自由 separator 碰撞证书变量。');
assert(numel(z0) == nlpData.sizes.numZ, 'native HS 初值长度必须与 NLP 变量长度一致。');
assert(numel(nlpData.equalityMask) == numel(nlpData.lbg), 'equalityMask 长度必须与约束长度一致。');
assert(any(~nlpData.equalityMask), 'native HS 必须保留路径/碰撞不等式。');

sourceText = fileread(fullfile(projectRoot, 'opt_minimal', 'buildCasadiFatropNativeHSNLP.m'));
assert(~contains(sourceText, 'dot3(n, n) - 1'), 'native HS 不应继续使用自由分离轴单位球等式。');
assert(~contains(sourceText, 'separator(:,') && ~contains(sourceText, 'sep('), ...
    'native HS 不应继续把 separator 作为优化变量。');

fprintf('TEST_11_OK fatrop native hs contract\n');
end

function disc = buildLocalDisc(scene, n1, n2)
disc = struct();
disc.numIntervalsApproach = n1;
disc.numIntervalsInsertion = n2;
disc.numIntervals = n1 + n2;
disc.numNodes = disc.numIntervals + 1;
disc.numMidpoints = disc.numIntervals;
disc.durationApproach = scene.phase.durationApproach;
disc.durationInsertion = scene.phase.durationInsertion;
disc.duration = disc.durationApproach + disc.durationInsertion;
disc.hApproach = disc.durationApproach / n1;
disc.hInsertion = disc.durationInsertion / n2;
if abs(disc.hApproach - disc.hInsertion) > 1e-12
    error('test_11_fatrop_native_hs_contract:NonUniformStep', '测试网格必须保持统一步长。');
end
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage1NodeIndices = 1:(n1 + 1);
disc.stage2NodeIndices = (n1 + 1):disc.numNodes;
disc.stage1MidIndices = 1:n1;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
