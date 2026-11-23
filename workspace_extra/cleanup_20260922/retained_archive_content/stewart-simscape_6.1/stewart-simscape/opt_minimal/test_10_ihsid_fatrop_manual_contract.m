%% test_10_ihsid_fatrop_manual_contract - IHSID-FATROP-manual 结构契约测试
% 本测试只检查最小 2x1 网格能构建 manual solver，并验证 pack/unpack、
% 终端阶段 nu=0、FATROP limited-memory 选项和 manual IPOPT 复用同一个 NLP。
clear; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'opt_minimal'));
addpath(fullfile(projectRoot, 'src'));
setupCasadiIpoptMa27(projectRoot);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
disc = buildLocalDisc(scene, 2, 1);
[zIHS, ihsGuess] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc);

zManual = packIHSFatropManualDecision(zIHS, scene, disc);
data = unpackIHSFatropManualDecision(zManual, scene, disc);
zRoundTrip = packIHSDecisionImplicit(data.Xinternal, data.Xmid, data.Anode, data.Amid, ...
    data.Fnode, data.Fmid, disc, data.separator);
assert(max(abs(zRoundTrip - zIHS)) < 1e-12, ...
    'manual pack/unpack 必须保持 IHSID 初值不变。');

terminalNuExpected = 16;
expectedManualLength = (disc.numIntervals + 1) * 24 + disc.numIntervals * 96 + terminalNuExpected;
assert(numel(zManual) == expectedManualLength, ...
    'manual 初值不能继续给终端节点附加完整 U_N，只允许终端碰撞证书的最小占位变量。');

solverOptions = struct('solverBackend', 'fatrop', 'fatropStructure', 'manual', 'skipSolver', true);
nlpData = buildCasadiImplicitIHSFatropManualNLP(model, scene, disc, ihsGuess, solverOptions);
assert(strcmp(nlpData.solverBackend, 'fatrop'), 'manual builder 必须记录 solverBackend=fatrop。');
assert(strcmp(nlpData.fatropStructure, 'manual'), 'manual builder 必须记录 fatropStructure=manual。');
assert(nlpData.sizes.numZ == numel(zManual), 'manual NLP 变量数必须等于 manual 初值长度。');
assert(nlpData.sizes.numZ == expectedManualLength, ...
    'manual NLP 不能继续给终端节点附加完整 U_N，只允许终端碰撞证书的最小占位变量。');
assert(numel(nlpData.opts.nx) == disc.numIntervals + 1, 'opts.nx 长度必须为 N+1。');
assert(numel(nlpData.opts.nu) == disc.numIntervals + 1, 'opts.nu 长度必须为 N+1。');
assert(double(nlpData.opts.nu{end}) == terminalNuExpected, ...
    'FATROP manual 终端阶段只能保留终端节点碰撞证书的最小占位变量。');
assert(numel(nlpData.opts.ng) == disc.numIntervals + 1, 'opts.ng 长度必须为 N+1。');
assert(numel(nlpData.opts.equality) == nlpData.sizes.numEq + nlpData.sizes.numIneq, ...
    'equality mask 长度必须等于全部约束行数。');
assert(strcmp(nlpData.opts.hessian_approximation, 'limited-memory'), ...
    'skipSolver 结构检查必须记录 FATROP limited-memory 意图。');

assert(exist('checkFatropManualStructure', 'file') == 2, ...
    '必须提供 checkFatropManualStructure.m，在求解前检查 manual 结构。');
assert(exist('checkIHSManualEquivalence', 'file') == 2, ...
    '必须提供 checkIHSManualEquivalence.m，在 FATROP 前检查 manual 与 standard 的公平性。');
assert(exist('probeFatropOptions', 'file') == 2, ...
    '必须提供 probeFatropOptions.m，逐项记录 FATROP 选项是否被接口接受。');

manualIpoptOptions = struct('solverBackend', 'ipopt', 'skipSolver', true);
manualIpoptData = buildCasadiImplicitIHSFatropManualNLP(model, scene, disc, ihsGuess, manualIpoptOptions);
assert(strcmp(manualIpoptData.solverBackend, 'ipopt'), ...
    'IHSID-MANUAL-IPOPT 必须复用 manual NLP，只允许 solverBackend 不同。');
assert(manualIpoptData.sizes.numZ == nlpData.sizes.numZ, ...
    'manual IPOPT 与 manual FATROP 必须共享相同 z 维度。');

run05Text = fileread(fullfile(projectRoot, 'opt_minimal', 'run_05_compare_IHSID_FATROP.m'));
assert(contains(run05Text, "solverOptions.hessianApproximation = 'exact'"), ...
    'run_05 对比入口必须把 IPOPT 切换为 exact Hessian。');
assert(~contains(run05Text, 'FATROP_LIMITED_MEMORY_UNAVAILABLE'), ...
    'exact Hessian 对比实验不能再因为 limited-memory 选项不可用而跳过 FATROP。');

fprintf('TEST_10_OK\n');

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
assert(abs(disc.hApproach - disc.hInsertion) < 1e-12, '测试网格必须保持统一步长。');
disc.h = disc.hApproach;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
disc.waypointNodeIndex = n1 + 1;
disc.stage2NodeIndices = disc.waypointNodeIndex:disc.numNodes;
disc.stage2MidIndices = (n1 + 1):disc.numIntervals;
disc.numStage1CollisionPoints = (n1 + 1) + n1;
disc.numAllCollisionPoints = disc.numNodes + disc.numMidpoints;
disc.numCollisionCertificates = disc.numStage1CollisionPoints + 2*disc.numAllCollisionPoints;
end
