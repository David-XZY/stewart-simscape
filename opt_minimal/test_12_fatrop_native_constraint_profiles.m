function test_12_fatrop_native_constraint_profiles()
% test_12_fatrop_native_constraint_profiles - 验证 FATROP-native 约束分层诊断入口
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);
[~, initialGuess] = buildInitialGuessFatropNativeHS(model, scene, disc);

solverOptions = struct('solverBackend', 'fatrop', 'fatropStructure', 'manual', ...
    'skipSolver', true, 'maxIter', 3, 'constraintProfile', 'dynamics');
nlpData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, solverOptions);

assert(strcmp(nlpData.nativeConstraintProfile, 'dynamics'), ...
    'builder 应记录当前 FATROP-native 约束分层。');
assert(nlpData.sizes.numEq == 12 * disc.numIntervals, ...
    'dynamics profile 应只保留每段 12 维 shooting defect 等式。');
assert(nlpData.sizes.numIneq == 0, ...
    'dynamics profile 不应保留路径/碰撞/插入不等式。');
assert(all(nlpData.manualStructure.ng == 0), ...
    'dynamics profile 下 FATROP manual ng 应全部为 0。');
assert(all(nlpData.equalityMask), ...
    'dynamics profile 下所有约束都应是 shooting defect 等式。');

fullOptions = struct('solverBackend', 'fatrop', 'fatropStructure', 'manual', ...
    'skipSolver', true, 'maxIter', 3, 'constraintProfile', 'full');
fullData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, fullOptions);
hardOptions = fullOptions;
hardOptions.constraintProfile = 'full_hard';
hardData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, hardOptions);
softSideOptions = fullOptions;
softSideOptions.constraintProfile = 'full_soft_side';
softSideData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, softSideOptions);
smoothCollisionOptions = fullOptions;
smoothCollisionOptions.constraintProfile = 'full_smooth_collision';
smoothCollisionData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, smoothCollisionOptions);
stage2SideOptions = fullOptions;
stage2SideOptions.constraintProfile = 'full_stage2_side';
stage2SideData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, stage2SideOptions);
maskedSideOptions = fullOptions;
maskedSideOptions.constraintProfile = 'full_masked_side';
maskedSideData = buildCasadiFatropNativeHSNLP(model, scene, disc, initialGuess, maskedSideOptions);

assert(strcmp(fullData.nativeInsertionMode, 'soft'), ...
    'full profile 应默认使用 soft insertion 以避免 FATROP 硬等式退化。');
assert(strcmp(hardData.nativeInsertionMode, 'hard'), ...
    'full_hard profile 应保留硬插入等式用于诊断复现。');
assert(strcmp(softSideData.nativeSideCollisionMode, 'soft'), ...
    'full_soft_side profile 应把侧壁 gap 改为 soft penalty 诊断项。');
assert(strcmp(smoothCollisionData.nativeSideCollisionMode, 'smooth_finite'), ...
    'full_smooth_collision profile 应使用平滑有限障碍侧壁 gap。');
assert(strcmp(stage2SideData.nativeSideCollisionScope, 'stage2'), ...
    'full_stage2_side profile 应只在插入段保留硬侧壁 gap。');
assert(strcmp(maskedSideData.nativeSideCollisionScope, 'masked_nominal'), ...
    'full_masked_side profile 应使用名义轨迹可行性掩码筛选侧壁 gap。');
assert(strcmp(fullData.nativeSideCollisionScope, 'all'), ...
    'full profile 默认保留全段侧壁约束；stage2 收缩仅作为诊断开关。');
assert(strcmp(hardData.nativeSideCollisionScope, 'all'), ...
    'full_hard profile 应保留旧的全段侧壁约束用于诊断复现。');
assert(fullData.sizes.numEq < hardData.sizes.numEq, ...
    'soft insertion 应减少 stage2 几何硬等式数量。');
assert(fullData.sizes.numIneq == hardData.sizes.numIneq, ...
    'soft insertion 仍应保留原来的碰撞/单调不等式数量。');
assert(softSideData.sizes.numIneq < fullData.sizes.numIneq, ...
    'soft side profile 应减少侧壁硬不等式数量。');
assert(smoothCollisionData.sizes.numIneq == fullData.sizes.numIneq, ...
    'smooth finite side profile 应保留侧壁硬不等式数量，只替换 gap 形式。');
assert(maskedSideData.sizes.numIneq < fullData.sizes.numIneq, ...
    'masked side profile 应删除被固定法向近似误判为不可行的侧壁约束。');

fprintf('TEST_12_OK fatrop native constraint profiles\n');
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
    error('test_12_fatrop_native_constraint_profiles:NonUniformStep', ...
        '测试网格必须保持统一步长。');
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
